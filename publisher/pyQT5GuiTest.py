import sys
from typing import List, Tuple

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton
from PyQt5.QtCore import QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from lidarReader import LidarReader, LidarPoint


class LidarCanvas(FigureCanvas):
    def __init__(self, parent=None, max_distance: float = 4000.0):
        fig = Figure()
        super().__init__(fig)
        self.ax = fig.add_subplot(111, polar=True)
        self.max_distance = max_distance

        self._setup_axes()

    def _setup_axes(self):
        self.ax.set_theta_zero_location("N")   # 0° North
        self.ax.set_theta_direction(-1)        # angle increases clockwise
        self.ax.set_rmax(self.max_distance)
        self.ax.grid(True)

    def draw_points(self, points: List[LidarPoint]):
        self.ax.clear()
        self._setup_axes()

        if points:
            angles = [p[0] for p in points]
            dists = [p[1] for p in points]
            self.ax.scatter(angles, dists, s=5)

        self.draw()


class MainWindow(QMainWindow):
    def __init__(self, reader: LidarReader, update_ms: int = 100):
        super().__init__()
        self.setWindowTitle("LIDAR PyQt5 Viewer (Test)")
        self.reader = reader

        self.canvas = LidarCanvas(self)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.addWidget(self.canvas)

        # Add start/stop buttons
        self.start_button = QPushButton("Start Scanning", self)
        self.stop_button = QPushButton("Stop Scanning", self)

        self.start_button.clicked.connect(self.start_scanning)
        self.stop_button.clicked.connect(self.stop_scanning)

        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)

        self.setCentralWidget(central)

        # Timer to update data from LidarReader
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_view)
        self.timer.start(update_ms)

    def start_scanning(self):
        self.reader.start()  # Start scanning

    def stop_scanning(self):
        self.reader.stop()  # Stop scanning

    def update_view(self):
        points = self.reader.get_latest_points()
        self.canvas.draw_points(points)


def main():
    reader = LidarReader(port=None, reconnect_delay=1.0)
    reader.start()

    app = QApplication(sys.argv)
    win = MainWindow(reader)
    win.show()

    exit_code = app.exec_()

    # When closing the window => stop the reader
    reader.stop()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
