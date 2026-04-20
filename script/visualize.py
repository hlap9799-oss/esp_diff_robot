#!/usr/bin/env python3
"""Visualize robot pose and kinematics from serial telemetry.

Serial format: TL,x,y,heading,linear,angular,leftCmPerSec,rightCmPerSec,leftTargetRps,rightTargetRps,leftMeasuredRps,rightMeasuredRps,deltaLeftCm,deltaRightCm
"""
import argparse
import math
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
from serial.tools import list_ports

FIELD_NAMES = [
    'x', 'y', 'heading', 'linear', 'angular',
    'leftCmPerSec', 'rightCmPerSec',
    'leftTargetRps', 'rightTargetRps', 'leftMeasuredRps', 'rightMeasuredRps',
    'deltaLeftCm', 'deltaRightCm', 'leftMotorPwm', 'rightMotorPwm'
]

DEFAULT_BAUD = 921600
BAUD_RATES = [9600, 115200, 230400, 460800, 921600]

# Default PID/PIDF coefficients
DEFAULT_SPEED_PID = {'kp': 2.0, 'ki': 0.3, 'kd': 0.05}
DEFAULT_HEADING_PID = {'kp': 1.5, 'ki': 0.2, 'kd': 0.1}
DEFAULT_POSITION_PID = {'kp': 0.5, 'ki': 0.05, 'kd': 0.1}
DEFAULT_PIDF = {'kp': 2.0, 'ki': 0.3, 'kd': 0.05, 'kf': 0.1}


class PoseVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title('Robot Pose Visualizer with PID/PIDF Control')
        self.root.geometry('1200x1000')
        
        self.ser = None
        self.connected = False
        self.anim = None
        self.latest = None
        self.path_points = []
        self.point_set = set()

        # ==== Top control frame ====
        control_frame = ttk.Frame(root)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)

        # Port selector
        ttk.Label(control_frame, text='Port:').pack(side=tk.LEFT, padx=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, state='readonly', width=20)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        self.refresh_ports()

        # Refresh ports button
        self.refresh_btn = ttk.Button(control_frame, text='Refresh', command=self.refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=2)

        # Baud rate selector
        ttk.Label(control_frame, text='Baud:').pack(side=tk.LEFT, padx=5)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.baud_combo = ttk.Combobox(control_frame, textvariable=self.baud_var, state='readonly', 
                                        values=[str(b) for b in BAUD_RATES], width=10)
        self.baud_combo.pack(side=tk.LEFT, padx=5)

        # Connect button
        self.connect_btn = ttk.Button(control_frame, text='Connect', command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        # Reset button
        self.reset_btn = ttk.Button(control_frame, text='Reset', command=self.reset, state=tk.DISABLED)
        self.reset_btn.pack(side=tk.LEFT, padx=5)

        # Status label
        self.status_var = tk.StringVar(value='Disconnected')
        status_label = ttk.Label(control_frame, textvariable=self.status_var, foreground='red')
        status_label.pack(side=tk.LEFT, padx=20)

        # ==== Control Buttons Frame ====
        button_frame = ttk.LabelFrame(root, text='Movement Control', padding=5)
        button_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)

        self.run_btn = ttk.Button(button_frame, text='RUN', command=self.cmd_run, state=tk.DISABLED, 
                                   width=15)
        self.run_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = ttk.Button(button_frame, text='STOP', command=self.cmd_stop, state=tk.DISABLED,
                                    width=15)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        # ==== Main content frame (left: PID controls, right: graph) ====
        main_frame = ttk.Frame(root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # ==== LEFT PANEL: PID/PIDF Coefficients (Vertical layout) ====
        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, padx=5)

        # Speed PID
        speed_frame = ttk.LabelFrame(left_panel, text='Speed PID (Motor Control)', padding=3)
        speed_frame.pack(fill=tk.X, pady=3)
        
        self.speed_pid = {}
        for i, key in enumerate(['kp', 'ki', 'kd']):
            ttk.Label(speed_frame, text=f'{key.upper()}:').grid(row=i, column=0, sticky=tk.W, padx=2, pady=2)
            var = tk.DoubleVar(value=DEFAULT_SPEED_PID[key])
            entry = ttk.Entry(speed_frame, textvariable=var, width=10)
            entry.grid(row=i, column=1, sticky=tk.E, padx=2, pady=2)
            self.speed_pid[key] = var

        # Heading PID
        heading_frame = ttk.LabelFrame(left_panel, text='Heading PID (Direction)', padding=3)
        heading_frame.pack(fill=tk.X, pady=3)
        
        self.heading_pid = {}
        for i, key in enumerate(['kp', 'ki', 'kd']):
            ttk.Label(heading_frame, text=f'{key.upper()}:').grid(row=i, column=0, sticky=tk.W, padx=2, pady=2)
            var = tk.DoubleVar(value=DEFAULT_HEADING_PID[key])
            entry = ttk.Entry(heading_frame, textvariable=var, width=10)
            entry.grid(row=i, column=1, sticky=tk.E, padx=2, pady=2)
            self.heading_pid[key] = var

        # Position PID
        position_frame = ttk.LabelFrame(left_panel, text='Position PID (Straight Line)', padding=3)
        position_frame.pack(fill=tk.X, pady=3)
        
        self.position_pid = {}
        for i, key in enumerate(['kp', 'ki', 'kd']):
            ttk.Label(position_frame, text=f'{key.upper()}:').grid(row=i, column=0, sticky=tk.W, padx=2, pady=2)
            var = tk.DoubleVar(value=DEFAULT_POSITION_PID[key])
            entry = ttk.Entry(position_frame, textvariable=var, width=10)
            entry.grid(row=i, column=1, sticky=tk.E, padx=2, pady=2)
            self.position_pid[key] = var

        # Movement PIDF
        move_frame = ttk.LabelFrame(left_panel, text='Movement PIDF', padding=3)
        move_frame.pack(fill=tk.X, pady=3)
        
        self.move_pidf = {}
        for i, key in enumerate(['kp', 'ki', 'kd', 'kf']):
            ttk.Label(move_frame, text=f'{key.upper()}:').grid(row=i, column=0, sticky=tk.W, padx=2, pady=2)
            var = tk.DoubleVar(value=DEFAULT_PIDF.get(key, 0.0))
            entry = ttk.Entry(move_frame, textvariable=var, width=10)
            entry.grid(row=i, column=1, sticky=tk.E, padx=2, pady=2)
            self.move_pidf[key] = var

        # Apply button
        ttk.Button(left_panel, text='Apply Coefficients', command=self.apply_coefficients, width=22).pack(fill=tk.X, pady=10)

        # ==== RIGHT PANEL: Graph Frame ====
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

        # Matplotlib figure embedded in tkinter
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.dot, = self.ax.plot([], [], 'o', color='tab:blue', markersize=8)
        self.heading_line, = self.ax.plot([], [], '-', color='tab:red', linewidth=2)
        self.path_scatter, = self.ax.plot([], [], '.', color='tab:gray', markersize=4, alpha=0.6)
        self.velocity_arrow = None
        self.text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontsize=10,
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='black'))

        self.ax.set_xlim(-100, 100)
        self.ax.set_ylim(-100, 100)
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_title('Robot Pose and Kinematics')
        self.ax.grid(True)
        self.ax.set_aspect('equal', 'box')

        # Start animation
        self.anim = animation.FuncAnimation(self.fig, self.update_graphics, interval=100, blit=False)
        self.canvas.draw()

    def refresh_ports(self):
        """Refresh available serial ports."""
        ports = [port.device for port in list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)

    def toggle_connection(self):
        """Connect or disconnect from serial port."""
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        """Establish serial connection."""
        port = self.port_var.get()
        if not port:
            self.status_var.set('No port selected')
            return

        try:
            baud = int(self.baud_var.get())
            self.ser = serial.Serial(port, baud, timeout=1)
            self.connected = True
            self.status_var.set(f'✓ Connected: {port}')
            self.connect_btn.config(text='Disconnect')
            self.reset_btn.config(state=tk.NORMAL)
            self.run_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.NORMAL)
        except Exception as e:
            self.status_var.set(f'Connection failed: {str(e)}')

    def disconnect(self):
        """Close serial connection."""
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
        self.connected = False
        self.status_var.set('✗ Disconnected')
        self.connect_btn.config(text='Connect')
        self.reset_btn.config(state=tk.DISABLED)
        self.run_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)

    def parse_line(self, line):
        try:
            parts = line.strip().split(',')
            if parts[0] != 'TL':
                return None
            
            # New format: 16 parts (with PWM)
            if len(parts) >= 16:
                values = [float(v) for v in parts[1:16]]
                return dict(zip(FIELD_NAMES, values))
            # Old format: 14 parts (without PWM) - for backward compatibility
            elif len(parts) == 14:
                old_field_names = FIELD_NAMES[:13]  # Without PWM fields
                values = [float(v) for v in parts[1:]]
                data = dict(zip(old_field_names, values))
                # Add placeholder PWM values
                data['leftMotorPwm'] = 0.0
                data['rightMotorPwm'] = 0.0
                return data
            else:
                return None
        except Exception as e:
            return None

    def read_serial(self):
        if not self.connected or not self.ser:
            return None
        try:
            raw = self.ser.readline().decode('utf-8', errors='ignore')
        except Exception:
            return None
        if not raw:
            return None
        
        # Try to parse as telemetry
        parsed = self.parse_line(raw)
        return parsed

    def send_command(self, cmd):
        """Send command to robot."""
        if not self.connected or not self.ser:
            return False
        try:
            self.ser.write((cmd + '\n').encode())
            self.ser.flush()
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

    def cmd_run(self):
        """Start movement sequence."""
        if self.send_command('RUN'):
            self.status_var.set('✓ Movement started')

    def cmd_stop(self):
        """Stop movement sequence."""
        if self.send_command('STOP'):
            self.status_var.set('✓ Movement stopped')

    def apply_coefficients(self):
        """Send PID/PIDF coefficients to robot."""
        try:
            speed_values = [self.speed_pid[k].get() for k in ['kp', 'ki', 'kd']]
            heading_values = [self.heading_pid[k].get() for k in ['kp', 'ki', 'kd']]
            position_values = [self.position_pid[k].get() for k in ['kp', 'ki', 'kd']]
            move_values = [self.move_pidf[k].get() for k in ['kp', 'ki', 'kd', 'kf']]
            
            # Send commands
            cmd_speed = f"PID_SPEED,{','.join(map(str, speed_values))}"
            cmd_heading = f"PID_HEADING,{','.join(map(str, heading_values))}"
            cmd_position = f"PID_POSITION,{','.join(map(str, position_values))}"
            cmd_move = f"PIDF_MOVE,{','.join(map(str, move_values))}"
            
            self.send_command(cmd_speed)
            self.send_command(cmd_heading)
            self.send_command(cmd_position)
            self.send_command(cmd_move)
            
            self.status_var.set('✓ Coefficients applied')
        except Exception as e:
            self.status_var.set(f'Error: {str(e)}')

    def reset(self):
        """Reset pose and encoder counts, clear trajectory."""
        if not self.connected or not self.ser:
            return
        try:
            self.ser.write(b'RESET\n')
            self.ser.flush()
        except Exception:
            pass
        self.path_points = []
        self.point_set = set()

    def update_graphics(self, frame):
        sample = self.read_serial()
        if sample is not None:
            self.latest = sample
            # Apply 90 degree counter-clockwise rotation for storage
            x_rot = -sample['y']
            y_rot = sample['x']
            rounded_point = (round(x_rot, 1), round(y_rot, 1))
            if rounded_point not in self.point_set:
                self.point_set.add(rounded_point)
                self.path_points.append(rounded_point)

        if self.latest is None:
            return self.path_scatter, self.dot, self.heading_line, self.text

        # Display rotated path points
        xs = [p[0] for p in self.path_points]
        ys = [p[1] for p in self.path_points]
        self.path_scatter.set_data(xs, ys)

        # Rotate 90 degrees counter-clockwise
        x_orig = self.latest['x']
        y_orig = self.latest['y']
        x = -y_orig
        y = x_orig
        heading = self.latest['heading'] + math.pi / 2
        speed = self.latest['linear']
        omega = self.latest['angular']

        self.dot.set_data([x], [y])

        heading_length = 15.0
        hx = x + heading_length * math.cos(heading)
        hy = y + heading_length * math.sin(heading)
        self.heading_line.set_data([x, hx], [y, hy])

        vel_angle = heading + (math.copysign(0.5, omega) if omega != 0 else 0.0)
        vel_length = max(5.0, min(35.0, abs(speed) * 0.3))
        vx = x + vel_length * math.cos(vel_angle)
        vy = y + vel_length * math.sin(vel_angle)

        if self.velocity_arrow is not None:
            self.velocity_arrow.remove()
        self.velocity_arrow = self.ax.arrow(
            x, y, vx - x, vy - y,
            head_width=3.0, head_length=5.0,
            fc='tab:green', ec='tab:green', length_includes_head=True, alpha=0.8)

        # Normalize heading to [0, 2π]
        heading_norm = self.latest['heading'] % (2 * math.pi)
        
        # Get PWM values (with fallback for old firmware)
        left_pwm = self.latest.get('leftMotorPwm', 0.0)
        right_pwm = self.latest.get('rightMotorPwm', 0.0)
        
        self.text.set_text(
            f"X={x:.1f} cm\nY={y:.1f} cm\nH={heading_norm:.3f} rad\n"
            f"V={speed:.2f} cm/s\nW={omega:.3f} rad/s\n"
            f"Lmeas={self.latest['leftCmPerSec']:.2f} cm/s\nRmeas={self.latest['rightCmPerSec']:.2f} cm/s\n"
            f"LPWM={left_pwm:.1f}% RPWM={right_pwm:.1f}%")

        return self.path_scatter, self.dot, self.heading_line, self.velocity_arrow, self.text

    def run(self):
        self.root.mainloop()


def main():
    root = tk.Tk()
    vis = PoseVisualizer(root)
    vis.run()


if __name__ == '__main__':
    main()