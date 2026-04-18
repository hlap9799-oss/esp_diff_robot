#!/usr/bin/env python3
"""Visualize robot pose and kinematics from serial telemetry.

Serial format: TL,x,y,heading,linear,angular,leftCmPerSec,rightCmPerSec,leftTargetRps,rightTargetRps,leftMeasuredRps,rightMeasuredRps,deltaLeftCm,deltaRightCm
"""
import argparse
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

FIELD_NAMES = [
    'x', 'y', 'heading', 'linear', 'angular',
    'leftCmPerSec', 'rightCmPerSec',
    'leftTargetRps', 'rightTargetRps', 'leftMeasuredRps', 'rightMeasuredRps',
    'deltaLeftCm', 'deltaRightCm'
]


class PoseVisualizer:
    def __init__(self, port, baud, max_points=100):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.latest = None
        self.path_points = []
        self.point_set = set()

        self.fig, self.ax = plt.subplots(figsize=(7, 7))
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

    def parse_line(self, line):
        try:
            parts = line.strip().split(',')
            if len(parts) != 14 or parts[0] != 'TL':
                return None
            values = [float(v) for v in parts[1:]]
            return dict(zip(FIELD_NAMES, values))
        except Exception:
            return None

    def read_serial(self):
        try:
            raw = self.ser.readline().decode('utf-8', errors='ignore')
        except Exception:
            return None
        if not raw:
            return None
        return self.parse_line(raw)

    def update_graphics(self, frame):
        sample = self.read_serial()
        if sample is not None:
            self.latest = sample
            rounded_point = (round(sample['x'], 1), round(sample['y'], 1))
            if rounded_point not in self.point_set:
                self.point_set.add(rounded_point)
                self.path_points.append(rounded_point)

        if self.latest is None:
            return self.path_scatter, self.dot, self.heading_line, self.text

        xs = [p[0] for p in self.path_points]
        ys = [p[1] for p in self.path_points]
        self.path_scatter.set_data(xs, ys)

        x = self.latest['x']
        y = self.latest['y']
        heading = self.latest['heading']
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

        self.text.set_text(
            f"X={x:.1f} cm\nY={y:.1f} cm\nH={heading:.3f} rad\n"
            f"V={speed:.2f} cm/s\nW={omega:.3f} rad/s\n"
            f"Lmeas={self.latest['leftCmPerSec']:.2f} cm/s\nRmeas={self.latest['rightCmPerSec']:.2f} cm/s")

        return self.path_scatter, self.dot, self.heading_line, self.velocity_arrow, self.text

    def run(self):
        animation.FuncAnimation(self.fig, self.update_graphics, interval=100, blit=False)
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visualize robot pose from serial telemetry.')
    parser.add_argument('--port', required=True, help='Serial port, e.g. COM3 or /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=921600, help='Serial baud rate')
    args = parser.parse_args()

    vis = PoseVisualizer(args.port, args.baud)
    vis.run()


if __name__ == '__main__':
    main()