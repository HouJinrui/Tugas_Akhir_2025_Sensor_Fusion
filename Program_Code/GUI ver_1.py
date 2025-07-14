import sys
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QLabel,
                             QWidget, QHBoxLayout, QGroupBox)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QColor, QPalette
from pymodbus.client import ModbusTcpClient

class SensorDisplay(QMainWindow):
    def __init__(self):
        super().__init__()

        self.client = ModbusTcpClient('192.168.2.101', timeout=1)

        self.setWindowTitle("Sensor Monitor - Arduino Fusion System")
        self.setGeometry(100, 100, 600, 400)

        # Initialize previous values and timestamps for speed calculation
        self.previous_values = {}
        self.previous_time = None
        self.current_values = {}

        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor(240, 240, 240))
        self.setPalette(palette)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # Current Values Group
        current_group = QGroupBox("Current Sensor Values")
        main_layout.addWidget(current_group)
        current_layout = QVBoxLayout()
        current_group.setLayout(current_layout)

        # Speed Values Group
        speed_group = QGroupBox("Speed Values (Rate of Change)")
        main_layout.addWidget(speed_group)
        speed_layout = QVBoxLayout()
        speed_group.setLayout(speed_layout)

        value_font = QFont()
        value_font.setPointSize(16)
        value_font.setBold(True)

        label_font = QFont()
        label_font.setPointSize(12)

        # Register mapping matching Arduino code exactly
        self.register_names = {
            0: "MPU6050 Fusion (Accel+Gyro)",
            1: "Shaft Angle",
            2: "Final Fusion",
            3: "Response Time (ms)"
        }

        # Create current value labels
        self.current_labels = {}
        self.speed_labels = {}
        
        for reg, name in self.register_names.items():
            container = QWidget()
            h_layout = QHBoxLayout(container)
            label = QLabel(f"{name}:")
            label.setFont(label_font)
            h_layout.addWidget(label)
            value_label = QLabel("0.00")
            value_label.setFont(value_font)
            value_label.setAlignment(Qt.AlignRight)
            h_layout.addWidget(value_label)
            current_layout.addWidget(container)
            self.current_labels[reg] = value_label

        # Create speed value labels (skip Response Time for speed calculation)
        for reg, name in self.register_names.items():
            if reg == 3:  # Skip Response Time for speed calculation
                continue
            container = QWidget()
            h_layout = QHBoxLayout(container)
            label = QLabel(f"{name} Speed:")
            label.setFont(label_font)
            h_layout.addWidget(label)
            speed_label = QLabel("0.00 °/s")
            speed_label.setFont(value_font)
            speed_label.setAlignment(Qt.AlignRight)
            h_layout.addWidget(speed_label)
            speed_layout.addWidget(container)
            self.speed_labels[reg] = speed_label

        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setFont(label_font)
        self.status_label.setStyleSheet("padding: 8px;")
        self.update_status_color("disconnected")
        main_layout.addWidget(self.status_label)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_sensor_data)
        self.timer.start(100)  # 100ms = 10Hz update rate
        self.update_sensor_data()

    def update_status_color(self, state):
        palette = self.status_label.palette()
        if state == "connected":
            palette.setColor(QPalette.Window, QColor(200, 255, 200))
            palette.setColor(QPalette.WindowText, QColor(0, 100, 0))
        elif state == "error":
            palette.setColor(QPalette.Window, QColor(255, 200, 200))
            palette.setColor(QPalette.WindowText, QColor(100, 0, 0))
        else:
            palette.setColor(QPalette.Window, QColor(255, 255, 200))
            palette.setColor(QPalette.WindowText, QColor(100, 100, 0))
        self.status_label.setAutoFillBackground(True)
        self.status_label.setPalette(palette)

    def convert_to_signed(self, unsigned_value):
        if unsigned_value > 32767:
            return unsigned_value - 65536
        return unsigned_value

    def calculate_speed(self, current_value, previous_value, time_delta):
        """Calculate speed as change in value over time"""
        if time_delta > 0:
            return (current_value - previous_value) / time_delta
        return 0.0

    def update_sensor_data(self):
        try:
            if not self.client.connected:
                connection = self.client.connect()
                if not connection:
                    self.status_label.setText("Status: Connection failed")
                    self.update_status_color("error")
                    return

            # Read only 4 registers as sent by Arduino
            response = self.client.read_holding_registers(address=0, count=4, slave=1)

            if response.isError():
                self.status_label.setText("Status: Modbus error")
                self.update_status_color("error")
                return

            current_time = time.time()
            
            # Update current values
            for i in range(4):
                raw = self.convert_to_signed(response.registers[i])
                
                # Store current values for speed calculation
                if i == 3:  # Response Time (ms) - register 3
                    self.current_values[i] = raw
                    value = f"{raw} ms"
                else:
                    # Registers 0, 1, 2 are angles multiplied by 100 in Arduino
                    self.current_values[i] = raw / 100.0  # Convert back to degrees
                    value = f"{raw / 100.0:.2f}°"
                
                self.current_labels[i].setText(value)

            # Calculate and display speeds for angle registers only
            if self.previous_time is not None and self.previous_values:
                time_delta = current_time - self.previous_time
                
                for i in range(3):  # Only for registers 0, 1, 2 (angles)
                    if i in self.previous_values:
                        speed = self.calculate_speed(
                            self.current_values[i], 
                            self.previous_values[i], 
                            time_delta
                        )
                        
                        # Color code the speed based on magnitude
                        speed_text = f"{speed:.2f} °/s"
                        if abs(speed) > 50:  # High speed threshold
                            self.speed_labels[i].setStyleSheet("color: red;")
                        elif abs(speed) > 10:  # Medium speed threshold
                            self.speed_labels[i].setStyleSheet("color: orange;")
                        else:
                            self.speed_labels[i].setStyleSheet("color: green;")
                        
                        self.speed_labels[i].setText(speed_text)

            # Store current values as previous for next iteration
            self.previous_values = self.current_values.copy()
            self.previous_time = current_time

            self.status_label.setText("Status: Connected")
            self.update_status_color("connected")

        except Exception as e:
            self.status_label.setText(f"Status: Error - {str(e)}")
            self.update_status_color("error")
            if self.client.connected:
                self.client.close()

    def closeEvent(self, event):
        if self.client.connected:
            self.client.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SensorDisplay()
    window.show()
    sys.exit(app.exec_())