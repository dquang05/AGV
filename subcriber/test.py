import sys
from typing import cast

import numpy as np   
import matplotlib.pyplot as plt

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.projections.polar import PolarAxes

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QComboBox,
    QPushButton,
    QDoubleSpinBox,
)
from PyQt5.QtCore import Qt


class QuadrupGUI(QMainWindow):
    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle("Quadrup Control (PyQt5 + MQTT)")
        self.resize(800, 600)

        # ====== Central widget & main layout ======
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)

        # -------------------------------------------------
        # 1. Polar Plot (tương đương polaraxes trong MATLAB)
        # -------------------------------------------------
        self.fig = plt.figure()

        # cast để Pylance hiểu đây là PolarAxes, tránh báo lỗi attribute
        self.ax = cast(PolarAxes, self.fig.add_subplot(111, projection="polar"))
        self.ax.set_rlim(0, 5000)
        self.ax.set_theta_direction(-1)      # clockwise
        self.ax.set_theta_zero_location("E") # 0° sang phải
        self.ax.set_title("Robot Polar Map")

        self.canvas = FigureCanvas(self.fig)
        main_layout.addWidget(self.canvas, stretch=3)

        # -------------------------------------------------
        # 2. Vùng control phía dưới
        # -------------------------------------------------
        controls_layout = QVBoxLayout()
        main_layout.addLayout(controls_layout, stretch=2)

        # ====== 2.1. Hàng HC12  ======
        hc12_layout = QHBoxLayout()

        self.btn_hc12 = QPushButton("HC12")
        self.btn_hc12.setCheckable(True)  # dùng như toggle connect/disconnect sau này
        hc12_layout.addWidget(self.btn_hc12)

        hc12_layout.addStretch()
        controls_layout.addLayout(hc12_layout)

        # ====== 2.2. Hàng các ô nhập Rotate / Angular speed / Distance / Linear speed ======
        grid = QGridLayout()

        # --------- Direction (Rotate for) ----------
        lbl_dir = QLabel("Rotate for:")
        self.spin_dir = QDoubleSpinBox()
        self.spin_dir.setRange(-360.0, 360.0)
        self.spin_dir.setDecimals(1)
        self.spin_dir.setSingleStep(0.1)

        # --------- Speed (Linear speed) ----------
        lbl_speed = QLabel("Linear speed:")
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(-100.0, 100.0)
        self.spin_speed.setDecimals(1)
        self.spin_speed.setSingleStep(0.1)

        # --------- Angle (Angular speed) ----------
        lbl_angle = QLabel("Angular speed:")
        self.spin_angle = QDoubleSpinBox()
        self.spin_angle.setRange(-180.0, 180.0)
        self.spin_angle.setDecimals(1)
        self.spin_angle.setSingleStep(0.1)

        # --------- Distance (and move straight) ----------
        lbl_dist = QLabel("and move straight:")
        self.spin_distance = QDoubleSpinBox()
        self.spin_distance.setRange(-100.0, 100.0)
        self.spin_distance.setDecimals(1)
        self.spin_distance.setSingleStep(0.1)

        # Sắp vào grid giống MATLAB (2 cột trái/phải)
        # Row 0: Rotate for | Angular speed
        grid.addWidget(lbl_dir,         0, 0)
        grid.addWidget(self.spin_dir,   0, 1)
        grid.addWidget(lbl_angle,       0, 2)
        grid.addWidget(self.spin_angle, 0, 3)

        # Row 1: and move straight | Linear speed
        grid.addWidget(lbl_dist,           1, 0)
        grid.addWidget(self.spin_distance, 1, 1)
        grid.addWidget(lbl_speed,          1, 2)
        grid.addWidget(self.spin_speed,    1, 3)

        controls_layout.addLayout(grid)

        # ====== 2.3. Hàng Mode list + Run + LidarRun ======
        mode_layout = QHBoxLayout()

        # Mode dropdown
        mode_layout.addWidget(QLabel("Mode:"))
        self.mode_list = QComboBox()
        self.mode_list.addItems(["Manual", "Simple drive", "Auto Drive"])
        mode_layout.addWidget(self.mode_list)

        # Run button (toggle, bấm lần 2 để dừng)
        self.btn_run = QPushButton("Run")
        self.btn_run.setCheckable(True)
        mode_layout.addWidget(self.btn_run)

        # LidarRun button (toggle vẽ Lidar hoặc start/stop loop Lidar)
        self.btn_lidar_run = QPushButton("LidarRun")
        self.btn_lidar_run.setCheckable(True)
        mode_layout.addWidget(self.btn_lidar_run)

        mode_layout.addStretch()
        controls_layout.addLayout(mode_layout)

        # ====== 2.4. Kết nối signal (stub – chưa gắn logic MQTT) ======
        self.btn_hc12.toggled.connect(self.on_hc12_toggled)
        self.btn_run.toggled.connect(self.on_run_toggled)
        self.btn_lidar_run.toggled.connect(self.on_lidar_run_toggled)

    # -------------------------------------------------
    # Các slot stub – sau này bạn nhét logic MQTT / HC12 vào
    # -------------------------------------------------
    def on_hc12_toggled(self, checked: bool) -> None:
        if checked:
            self.btn_hc12.setText("HC12 (On)")
        else:
            self.btn_hc12.setText("HC12")
        print(f"[UI] HC12 toggled: {checked} (chưa gắn logic).")

    def on_run_toggled(self, checked: bool) -> None:
        if checked:
            self.btn_run.setText("Running")
        else:
            self.btn_run.setText("Run")
        print(f"[UI] Run toggled: {checked} – Mode = {self.mode_list.currentText()} (chưa gắn logic).")

    def on_lidar_run_toggled(self, checked: bool) -> None:
        if checked:
            self.btn_lidar_run.setText("LidarStop")
        else:
            self.btn_lidar_run.setText("LidarRun")
        print(f"[UI] LidarRun toggled: {checked} (chưa gắn logic).")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = QuadrupGUI()
    win.show()
    sys.exit(app.exec_())
