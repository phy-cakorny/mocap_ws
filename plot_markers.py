#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import Markers
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

class MarkerPlotter(Node):
    def __init__(self):
        super().__init__('marker_plotter')
        self.sub = self.create_subscription(Markers, '/markers', self.cb, 10)
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        self.scatter = None

    def cb(self, msg: Markers):
        xs = [m.translation.x for m in msg.markers]
        ys = [m.translation.y for m in msg.markers]
        zs = [m.translation.z for m in msg.markers]
        self.ax.cla()
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        self.ax.scatter(xs, ys, zs, c='b', marker='o')
        self.ax.set_title(f'Frame {msg.frame_number} ({len(msg.markers)} markers)')
        plt.draw()
        plt.pause(0.001)

def main():
    rclpy.init()
    node = MarkerPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    plt.close('all')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()