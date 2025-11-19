# lidarPlotter.py
import sys
import numpy as np
import matplotlib.pyplot as plt

from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from dataSubscriber import lidar_buffer, start_subscriber_background


class LidarPlotter(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Lidar Viewer")

        # setup matplotlib figure
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="polar")
        self.canvas = FigureCanvas(self.fig)

        # setup layout
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        # start subscriber thread
        start_subscriber_background()

        # timer update every 50ms
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)

    def update_plot(self):
        points = lidar_buffer.get_latest()

        if points:
            angles = [p[0] for p in points]      # rad
            dists  = [p[1] for p in points]

            self.ax.clear()
            self.ax.scatter(angles, dists, s=5)
            self.ax.set_ylim(0, max(dists) + 100)

            self.canvas.draw()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = LidarPlotter()
    viewer.show()
    sys.exit(app.exec_())
