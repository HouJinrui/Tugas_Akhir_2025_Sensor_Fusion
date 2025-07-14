from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.clock import Clock
from kivy.properties import StringProperty, ListProperty
from pymodbus.client import ModbusTcpClient

class StatusLabel(Label):
    # Warna background bisa diubah berdasarkan status
    bg_color = ListProperty([1, 1, 0.8, 1])  # default kuning (disconnected)

    def on_bg_color(self, instance, value):
        self.canvas.before.clear()
        with self.canvas.before:
            from kivy.graphics import Color, Rectangle
            Color(*value)
            self.rect = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_rect, size=self.update_rect)

    def update_rect(self, *args):
        self.rect.pos = self.pos
        self.rect.size = self.size

class SensorMonitor(BoxLayout):
    roll_angle = StringProperty("0.00°")
    as5600_angle = StringProperty("0.00°")
    status_text = StringProperty("Disconnected")
    status_color = ListProperty([1, 1, 0.8, 1])  # default kuning

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.orientation = 'vertical'
        self.padding = 20
        self.spacing = 15

        # Modbus client
        self.client = ModbusTcpClient('192.168.2.101', timeout=1)

        # Sensor data group
        sensor_group = BoxLayout(orientation='vertical', spacing=20, size_hint_y=None)
        sensor_group.bind(minimum_height=sensor_group.setter('height'))

        # Roll Angle
        roll_layout = GridLayout(cols=2, size_hint_y=None, height=40)
        roll_layout.add_widget(Label(text="Roll Angle:", font_size='18sp', bold=True))
        self.roll_label = Label(text=self.roll_angle, font_size='28sp', halign='right')
        roll_layout.add_widget(self.roll_label)
        sensor_group.add_widget(roll_layout)

        # AS5600 Angle
        as5600_layout = GridLayout(cols=2, size_hint_y=None, height=40)
        as5600_layout.add_widget(Label(text="AS5600 Angle:", font_size='18sp', bold=True))
        self.as5600_label = Label(text=self.as5600_angle, font_size='28sp', halign='right')
        as5600_layout.add_widget(self.as5600_label)
        sensor_group.add_widget(as5600_layout)

        self.add_widget(sensor_group)

        # Status label with background color
        self.status_label = StatusLabel(text="Status: Disconnected",
                                        font_size='20sp',
                                        size_hint_y=None,
                                        height=40,
                                        bg_color=self.status_color)
        self.add_widget(self.status_label)

        # Button bar
        btn_bar = BoxLayout(size_hint_y=None, height=50, spacing=20)
        self.btn_start = Button(text="Start")
        self.btn_stop = Button(text="Stop")
        self.btn_reset = Button(text="Reset")

        self.btn_start.bind(on_press=self.on_start)
        self.btn_stop.bind(on_press=self.on_stop)
        self.btn_reset.bind(on_press=self.on_reset)

        btn_bar.add_widget(self.btn_start)
        btn_bar.add_widget(self.btn_stop)
        btn_bar.add_widget(self.btn_reset)

        self.add_widget(btn_bar)

        # Timer update setiap 0.1 detik
        Clock.schedule_interval(self.update_sensor_data, 0.1)

    def update_status(self, state):
        if state == "connected":
            self.status_text = "Status: Connected"
            self.status_color = [0.8, 1, 0.8, 1]  # light green
        elif state == "error":
            self.status_text = "Status: Error"
            self.status_color = [1, 0.8, 0.8, 1]  # light red
        else:
            self.status_text = "Status: Disconnected"
            self.status_color = [1, 1, 0.8, 1]  # light yellow

        self.status_label.text = self.status_text
        self.status_label.bg_color = self.status_color

    def convert_to_signed(self, unsigned_value):
        return unsigned_value - 65536 if unsigned_value > 32767 else unsigned_value

    def update_sensor_data(self, dt):
        try:
            if not self.client.connected:
                if not self.client.connect():
                    self.update_status("error")
                    return

            response = self.client.read_holding_registers(address=0, count=2, unit=1)
            if response.isError():
                self.update_status("error")
                return

            roll_raw = self.convert_to_signed(response.registers[0])
            as5600_raw = self.convert_to_signed(response.registers[1])

            roll_value = roll_raw / 100.0
            as5600_value = as5600_raw / 100.0

            self.roll_label.text = f"{roll_value:.2f}°"
            self.as5600_label.text = f"{as5600_value:.2f}°"
            self.update_status("connected")

        except Exception as e:
            self.update_status("error")
            if self.client.connected:
                self.client.close()

    # Tombol dummy (nanti kamu isi fungsi Modbus write / lainnya)
    def on_start(self, instance):
        print("Start button pressed")
        # Contoh: kirim perintah start ke device via Modbus write

    def on_stop(self, instance):
        print("Stop button pressed")
        # Contoh: kirim perintah stop ke device via Modbus write

    def on_reset(self, instance):
        print("Reset button pressed")
        # Contoh: kirim perintah reset ke device via Modbus write

    def on_stop_app(self):
        if self.client.connected:
            self.client.close()

class SensorApp(App):
    def build(self):
        return SensorMonitor()

    def on_stop(self):
        self.root.on_stop_app()

if __name__ == "__main__":
    SensorApp().run()
