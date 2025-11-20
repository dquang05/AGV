import sys
import struct

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer, Qt

import serial
from serial.tools import list_ports

from test import QuadrupGUI
from dataSubscriber import lidar_buffer, start_subscriber_background


class QuadrupApp(QuadrupGUI):
    def __init__(self) -> None:
        super().__init__()

        # ====== Trạng thái HC12 / phím / mode ======
        self.hc12 = None  # type: ignore[assignment]  # serial.Serial hoặc None
        self.keys = {"w": 0, "a": 0, "s": 0, "d": 0}

        # Đảm bảo window nhận phím (cho W/A/S/D)
        self.setFocusPolicy(Qt.StrongFocus)

        # ====== MQTT Lidar subscriber + timer vẽ ======
        start_subscriber_background()

        self.lidar_timer = QTimer(self)
        self.lidar_timer.timeout.connect(self.update_lidar_plot)

        # ====== Timer Simple drive: gửi lệnh liên tục ======
        self.drive_timer = QTimer(self)
        self.drive_timer.timeout.connect(self.update_drive)
        self.drive_timer.start(50)  # 50 ms một lần

    # =====================================================
    # 1. LIDAR PLOT – lấy dữ liệu từ dataSubscriber
    # =====================================================
    def update_lidar_plot(self) -> None:
        """Lấy packet mới nhất từ lidar_buffer và vẽ lên polar plot."""
        points = lidar_buffer.get_latest()

        if not points:
            return

        angles = [p[0] for p in points]  # rad
        dists = [p[1] for p in points]

        if not dists:
            return

        self.ax.clear()
        self.ax.scatter(angles, dists, s=5)
        self.ax.set_rlim(0, max(dists) + 100)
        self.ax.set_theta_direction(-1)
        self.ax.set_theta_zero_location("E")
        self.ax.set_title("Robot Polar Map")

        self.canvas.draw()

    # =====================================================
    # 2. HC12 CONNECT / DISCONNECT
    # =====================================================
    def on_hc12_toggled(self, checked: bool) -> None:
        """
        Override hàm trong QuadrupGUI:
        - Nếu checked = True: tìm cổng USB-UART, connect.
        - Nếu checked = False: đóng cổng.
        """
        # Gọi super để đổi text nút (HC12 / HC12 (On))
        if checked:
            self.btn_hc12.setText("HC12 (On)")
        else:
            self.btn_hc12.setText("HC12")

        if checked:
            self.connect_hc12()
        else:
            self.disconnect_hc12()

    def connect_hc12(self) -> None:
        """Tìm cổng serial USB-UART và mở kết nối HC12."""
        # Ngắt kết nối cũ nếu có
        self.disconnect_hc12(silent=True)

        try:
            ports = list(list_ports.comports())
            if not ports:
                print("[HC12] Không tìm thấy cổng serial nào.")
                self.btn_hc12.setChecked(False)
                self.btn_hc12.setText("HC12")
                return

            # Cách đơn giản: chọn cổng "có vẻ USB" đầu tiên
            port_device = None
            for p in ports:
                desc = (p.description or "").lower()
                if "usb" in desc or "uart" in desc or "serial" in desc:
                    port_device = p.device
                    break

            # Nếu không tìm được theo mô tả thì lấy cổng đầu tiên
            if port_device is None:
                port_device = ports[0].device

            print(f"[HC12] Kết nối tới cổng: {port_device}")
            self.hc12 = serial.Serial(port_device, 9600, timeout=0.1)
            print("[HC12] Kết nối thành công.")

        except Exception as e:
            print(f"[HC12] Lỗi khi kết nối: {e}")
            self.hc12 = None
            self.btn_hc12.setChecked(False)
            self.btn_hc12.setText("HC12")

    def disconnect_hc12(self, silent: bool = False) -> None:
        """Đóng cổng HC12 nếu đang mở."""
        if self.hc12 is not None:
            try:
                if self.hc12.is_open:
                    self.hc12.close()
                    if not silent:
                        print("[HC12] Đã ngắt kết nối.")
            except Exception as e:
                if not silent:
                    print(f"[HC12] Lỗi khi ngắt kết nối: {e}")
        self.hc12 = None

    # =====================================================
    # 3. GỬI LỆNH QUA HC12 (send_data – giống MATLAB)
    # =====================================================
    def send_data(self, distance: float, direction: float, v: float, w: float) -> None:
        """
        Gửi gói 10 byte: [0xAA][dist(2)][dir(2)][w(2)][v(2)][0x55]
        - distance, direction: 0..65535 (uint16)
        - v, w: -100..100, scale *10, gửi int16
        """
        if self.hc12 is None or not self.hc12.is_open:
            print("[HC12] Chưa kết nối, không thể gửi dữ liệu.")
            return

        try:
            # Giới hạn
            v_clamp = max(min(v, 100.0), -100.0)
            w_clamp = max(min(w, 100.0), -100.0)
            dist_clamp = max(min(distance, 65535.0), 0.0)
            dir_clamp = max(min(direction, 65535.0), 0.0)

            dist_bytes = struct.pack("<H", int(round(dist_clamp)))
            dir_bytes = struct.pack("<H", int(round(dir_clamp)))

            v_int = int(round(v_clamp * 10))
            w_int = int(round(w_clamp * 10))
            v_bytes = struct.pack("<h", v_int)  # int16 little-endian
            w_bytes = struct.pack("<h", w_int)

            frame = bytes([0xAA]) + dist_bytes + dir_bytes + w_bytes + v_bytes + bytes([0x55])

            self.hc12.write(frame)
            # print(f"[HC12] Sent: {frame}")

        except Exception as e:
            print(f"[HC12] Lỗi gửi dữ liệu: {e}")

    # =====================================================
    # 4. NÚT RUN + LidarRun (override logic)
    # =====================================================
    def on_run_toggled(self, checked: bool) -> None:
        """
        - Manual mode: nếu checked True -> gửi 1 lệnh rồi thôi.
        - Simple drive: checked True -> bắt đầu dùng W/A/S/D; False -> dừng.
        - Auto Drive: tạm thời chưa làm logic, chỉ log ra.
        """
        if checked:
            self.btn_run.setText("Running")
        else:
            self.btn_run.setText("Run")

        mode = self.mode_list.currentText()
        print(f"[UI] Run toggled: {checked} – Mode = {mode}")

        if mode == "Manual":
            if checked:
                # Gửi 1 lệnh manual
                distance = self.spin_distance.value()
                direction = self.spin_dir.value()
                v = self.spin_speed.value()
                w = self.spin_angle.value()
                print(f"[Manual] Send once: dist={distance}, dir={direction}, v={v}, w={w}")
                self.send_data(distance, direction, v, w)
                # Sau khi gửi xong, tắt Run luôn (giống kiểu "one shot")
                self.btn_run.setChecked(False)
                self.btn_run.setText("Run")

        elif mode == "Simple drive":
            # Logic chính nằm ở update_drive() + keyPressEvent/Release
            if checked:
                print("[Simple drive] Bắt đầu nhận W/A/S/D.")
            else:
                print("[Simple drive] Dừng nhận W/A/S/D.")
                self.keys = {"w": 0, "a": 0, "s": 0, "d": 0}

        elif mode == "Auto Drive":
            # Tạm thời chỉ log
            if checked:
                print("[Auto Drive] Bắt đầu (logic chưa cài).")
            else:
                print("[Auto Drive] Dừng.")

    def on_lidar_run_toggled(self, checked: bool) -> None:
        """Start/stop timer vẽ Lidar."""
        if checked:
            self.btn_lidar_run.setText("LidarStop")
            self.lidar_timer.start(50)  # 50 ms
            print("[LIDAR] Bắt đầu vẽ.")
        else:
            self.btn_lidar_run.setText("LidarRun")
            self.lidar_timer.stop()
            print("[LIDAR] Dừng vẽ.")

    # =====================================================
    # 5. SIMPLE DRIVE: W/A/S/D + timer update_drive
    # =====================================================
    def keyPressEvent(self, event) -> None:  # type: ignore[override]
        """Nhấn W/A/S/D để điều khiển."""
        mode = self.mode_list.currentText()
        if mode == "Simple drive" and self.btn_run.isChecked():
            if event.key() == Qt.Key_W:
                self.keys["w"] = 1
            elif event.key() == Qt.Key_S:
                self.keys["s"] = 1
            elif event.key() == Qt.Key_A:
                self.keys["a"] = 1
            elif event.key() == Qt.Key_D:
                self.keys["d"] = 1

        # Gọi super để các phím khác vẫn xử lý bình thường
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:  # type: ignore[override]
        """Nhả W/A/S/D -> reset cờ."""
        mode = self.mode_list.currentText()
        if mode == "Simple drive" and self.btn_run.isChecked():
            if event.key() == Qt.Key_W:
                self.keys["w"] = 0
            elif event.key() == Qt.Key_S:
                self.keys["s"] = 0
            elif event.key() == Qt.Key_A:
                self.keys["a"] = 0
            elif event.key() == Qt.Key_D:
                self.keys["d"] = 0

        super().keyReleaseEvent(event)

    def update_drive(self) -> None:
        """
        Gọi mỗi 50 ms:
        - Nếu đang ở Simple drive + Run đang bật + HC12 connected:
          -> tính v, w từ W/A/S/D và gửi.
        """
        mode = self.mode_list.currentText()
        if mode != "Simple drive":
            return
        if not self.btn_run.isChecked():
            return
        if self.hc12 is None or not self.hc12.is_open:
            return

        up = self.keys["w"]
        down = self.keys["s"]
        left = self.keys["a"]
        right = self.keys["d"]

        v_base = self.spin_speed.value()   # độ lớn speed
        w_base = self.spin_angle.value()   # độ lớn angular

        v = (up - down) * v_base
        w = (right - left) * w_base

        if v != 0 or w != 0:
            # Simple drive: truyền distance & direction = 0 (chỉ điều khiển tốc độ)
            self.send_data(distance=0.0, direction=0.0, v=v, w=w)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = QuadrupApp()
    win.show()
    sys.exit(app.exec_())
