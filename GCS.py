import datetime
import math
import sys
import threading
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor, QFont, QPen
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from flask import Flask, render_template, jsonify
import folium
import webview
from math import sin, cos, sqrt, atan2, radians


app = Flask(__name__)

# Bağlantıyı global olarak tanımlayın
vehicle = None

# Aracınıza bağlanın
def connect_vehicle():
    global vehicle
    connection_string = '127.0.0.1:14550'  # SITL bağlantı adresi
    print("Bağlanıyor...")
    vehicle = connect(connection_string, wait_ready=True)
    print("Bağlandı")

# Flask uygulamasını başlatın
@app.route('/')
def index():
    initial_location = vehicle.location.global_frame
    drone_map = folium.Map(location=[initial_location.lat, initial_location.lon], zoom_start=15)
    map_html = 'templates/map.html'
    drone_map.save(map_html)
    return render_template('index.html', initial_location=initial_location)

@app.route('/location')
def location():
    current_location = vehicle.location.global_frame
    return jsonify(lat=current_location.lat, lon=current_location.lon)

def start_flask_app():
    app.run(debug=True, use_reloader=False)

class AutotuneThread(QtCore.QThread):
    def __init__(self, vehicle):
        super().__init__()
        self.vehicle = vehicle

    def run(self):
        self.send_ned_velocity(5, 0, 0, 25)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # m/s
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0)  # yaw, yaw_rate (not used)

        for _ in range(0, duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

class DataFetcher(QtCore.QThread):
    dataFetched = QtCore.pyqtSignal(object)

    def __init__(self, vehicle):
        super().__init__()
        self.vehicle = vehicle

    def run(self):
        while True:
            data = {
                'attitude': self.vehicle.attitude,
                'location': self.vehicle.location.global_relative_frame,
                'battery': self.vehicle.battery,
                'groundspeed': self.vehicle.groundspeed,
                'airspeed': self.vehicle.airspeed,
                'gps': self.vehicle.gps_0,
                'mode': self.vehicle.mode,
                'armed': self.vehicle.armed,
                'home_location': self.vehicle.home_location,
                'current_location': self.vehicle.location.global_frame
            }
            self.dataFetched.emit(data)
            self.msleep(100)  # 100 ms delay

class HorizonIndicator(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.altitude = 0.0
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.battery_percent = 0.0

    def update_data(self, data):
        attitude = data.get('attitude')
        location = data.get('location')
        battery = data.get('battery')

        if attitude:
            self.pitch = math.degrees(attitude.pitch)
            self.roll = math.degrees(attitude.roll)
            self.yaw = math.degrees(attitude.yaw)

        if location:
            self.altitude = location.alt

        if battery:
            self.battery_voltage = battery.voltage
            self.battery_current = battery.current
            self.battery_percent = battery.level

        self.update()  # Refresh the display

    def paintEvent(self, event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)  # Enable anti-aliasing
        self.drawHorizon(qp)
        self.drawText(qp)
        self.drawRectangle(qp)
        self.drawGyroLines(qp)
        qp.end()  # Properly end the painting process

    def drawHorizon(self, qp):
        center_x = self.width() // 2
        center_y = self.height() // 2

        pitch_offset = (self.pitch / 90.0) * (self.height() / 2)

        # Draw the sky
        qp.save()
        qp.setBrush(QColor(0, 102, 204))  # Blue color
        qp.translate(0, pitch_offset)
        qp.drawRect(0, -self.height(), self.width(), center_y + self.height())
        qp.restore()

        # Draw the ground
        qp.save()
        qp.setBrush(QColor(255, 140, 45))  # Orange color
        qp.translate(0, pitch_offset)
        qp.drawRect(0, center_y, self.width(), self.height() - center_y + self.height())
        qp.restore()

        # Draw pitch lines and labels
        qp.save()
        qp.translate(center_x, center_y + pitch_offset)
        qp.rotate(-self.roll)

        qp.setPen(QPen(QColor(255, 255, 0), 2, Qt.SolidLine))  # Yellow color

        for p in range(-30, 31, 10):
            if p == 0:
                continue
            y = (p / 50.0) * (self.height() / 2) * 1.1
            qp.drawLine(-55, -int(y), 55, -int(y))
            qp.drawText(60, -int(y + 5), str(p))
            qp.drawText(-85, -int(y + 5), str(p))

        qp.restore()

    def drawText(self, qp):
        qp.setPen(QColor(0, 0, 0))
        qp.setFont(QFont('Arial', 15))

        # Bottom-left text
        text_x = 10
        text_y_start = self.height() - 120
        line_height = 20

        qp.drawText(text_x, text_y_start, f'Roll: {self.roll:.2f}')
        qp.drawText(text_x, text_y_start + line_height, f'Pitch: {self.pitch:.2f}')
        qp.drawText(text_x, text_y_start + 2 * line_height, f'Yaw: {self.yaw:.2f}')
        qp.drawText(text_x, text_y_start + 3 * line_height, f'Altitude: {self.altitude:.2f}')

        # Top-right text for battery
        text_x = self.width() - 200
        text_y_start = 20

    def drawRectangle(self, qp):
        # Rectangle for the gyroscope display
        margin = 0.40
        margin_x = (1 - margin) / 2
        rect_x = int(self.width() * margin_x)
        rect_width = int(self.width() * margin)

        margin_y = 0.05
        rect_y = int(self.height() * margin_y)
        rect_height = int(self.height() * (1 - 2 * margin_y))

        radius = int(self.width() * 0.10)

        qp.setPen(QPen(QColor(255, 255, 255), 2, Qt.SolidLine))
        qp.setBrush(QColor(0, 0, 0, 0))

        qp.drawRoundedRect(rect_x, rect_y, rect_width, rect_height, radius, radius)

    def drawGyroLines(self, qp):
        center_x = self.width() // 2
        center_y = self.height() // 2

        line_length = 25

        qp.setPen(QPen(QColor(0, 245, 0), 2, Qt.SolidLine))

        qp.drawLine(center_x, center_y, center_x + line_length, center_y + line_length)
        qp.drawLine(center_x, center_y, center_x - line_length, center_y + line_length)


class Ui_MainWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()
        self.vehicle = None
        self.message_log = []  # To store the log messages
        self.setupUi(self)

    def update_data(self, data):
        self.gyroscope.update_data(data)
        self.yukseklik(data)
        self.hava_hizi(data)
        self.gps_hizi(data)
        self.gps_sayisi(data)
        self.roll_acisi(data)
        self.pitch_acisi(data)
        self.yaw_acisi(data)
        self.batarya_durumu(data)
        self.saat()
        self.tarih()
        self.arm(data)
        self.mod_durumu(data)
        self.telemetri()
        self.uzaklik(data)


    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1920, 1094)
        MainWindow.setStyleSheet("background-color: rgb(0, 0, 0);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.camera = QtWidgets.QLabel(self.centralwidget)
        self.camera.setGeometry(QtCore.QRect(710, 50, 500, 600))
        self.camera.setStyleSheet("")
        self.camera.setText("")
        self.camera.setPixmap(QtGui.QPixmap(":/newPrefix/ucak1.png"))
        self.camera.setScaledContents(True)
        self.camera.setObjectName("camera")
        self.gyroscope = QtWidgets.QLabel(self.centralwidget)
        self.gyroscope.setGeometry(QtCore.QRect(1300, 380, 500, 600))
        self.gyroscope.setStyleSheet("")
        self.gyroscope.setText("")
        self.gyroscope.setPixmap(QtGui.QPixmap("OneDrive/Masaüstü/uçak2.png"))
        self.gyroscope.setScaledContents(True)
        self.gyroscope.setObjectName("gyroscope")

        self.gyroscope = HorizonIndicator(self.centralwidget)
        self.gyroscope.setGeometry(QtCore.QRect(1300, 380, 500, 600))  # Genişlik ve yükseklik değerlerini artırdım
        self.gyroscope.setObjectName("gyroscope")

        self.formLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.formLayoutWidget_2.setGeometry(QtCore.QRect(1650, 30, 141, 52))
        self.formLayoutWidget_2.setObjectName("formLayoutWidget_2")
        self.formLayout_2 = QtWidgets.QFormLayout(self.formLayoutWidget_2)
        self.formLayout_2.setContentsMargins(0, 0, 0, 0)
        self.formLayout_2.setObjectName("formLayout_2")
        self.saat_text = QtWidgets.QLabel(self.formLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.saat_text.sizePolicy().hasHeightForWidth())
        self.saat_text.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.saat_text.setFont(font)
        self.saat_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.saat_text.setObjectName("saat_text")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.saat_text)
        self.tarih_text = QtWidgets.QLabel(self.formLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tarih_text.sizePolicy().hasHeightForWidth())
        self.tarih_text.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.tarih_text.setFont(font)
        self.tarih_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.tarih_text.setObjectName("tarih_text")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.tarih_text)
        self.tarih_value = QtWidgets.QLabel(self.formLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.tarih_value.setFont(font)
        self.tarih_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.tarih_value.setObjectName("tarih_value")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.tarih_value)
        self.saat_value = QtWidgets.QLabel(self.formLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.saat_value.setFont(font)
        self.saat_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.saat_value.setObjectName("saat_value")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.saat_value)
        self.formLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.formLayoutWidget_3.setGeometry(QtCore.QRect(130, 50, 204, 266))
        self.formLayoutWidget_3.setObjectName("formLayoutWidget_3")
        self.formLayout_3 = QtWidgets.QFormLayout(self.formLayoutWidget_3)
        self.formLayout_3.setContentsMargins(0, 0, 0, 0)
        self.formLayout_3.setObjectName("formLayout_3")
        self.altitude_text = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.altitude_text.setFont(font)
        self.altitude_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.altitude_text.setObjectName("altitude_text")
        self.formLayout_3.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.altitude_text)
        self.altitude_value = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.altitude_value.setFont(font)
        self.altitude_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.altitude_value.setObjectName("altitude_value")
        self.formLayout_3.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.altitude_value)
        self.airspeed_text = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.airspeed_text.setFont(font)
        self.airspeed_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.airspeed_text.setObjectName("airspeed_text")
        self.formLayout_3.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.airspeed_text)
        self.airspeed_value = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.airspeed_value.setFont(font)
        self.airspeed_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.airspeed_value.setObjectName("airspeed_value")
        self.formLayout_3.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.airspeed_value)
        self.gpsspeed_text = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.gpsspeed_text.setFont(font)
        self.gpsspeed_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.gpsspeed_text.setObjectName("gpsspeed_text")
        self.formLayout_3.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.gpsspeed_text)
        self.gpsspeed_value = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.gpsspeed_value.setFont(font)
        self.gpsspeed_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.gpsspeed_value.setObjectName("gpsspeed_value")
        self.formLayout_3.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.gpsspeed_value)
        self.distance_text = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.distance_text.setFont(font)
        self.distance_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.distance_text.setObjectName("distance_text")
        self.formLayout_3.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.distance_text)
        self.distance_value = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.distance_value.setFont(font)
        self.distance_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.distance_value.setObjectName("distance_value")
        self.formLayout_3.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.distance_value)
        self.roll_text = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.roll_text.setFont(font)
        self.roll_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.roll_text.setObjectName("roll_text")
        self.formLayout_3.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.roll_text)
        self.roll_value = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.roll_value.setFont(font)
        self.roll_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.roll_value.setObjectName("roll_value")
        self.formLayout_3.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.roll_value)
        self.pitch_text = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pitch_text.setFont(font)
        self.pitch_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.pitch_text.setObjectName("pitch_text")
        self.formLayout_3.setWidget(5, QtWidgets.QFormLayout.LabelRole, self.pitch_text)
        self.pitch_value = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pitch_value.setFont(font)
        self.pitch_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.pitch_value.setObjectName("pitch_value")
        self.formLayout_3.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.pitch_value)
        self.yaw_text = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.yaw_text.setFont(font)
        self.yaw_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.yaw_text.setObjectName("yaw_text")
        self.formLayout_3.setWidget(6, QtWidgets.QFormLayout.LabelRole, self.yaw_text)
        self.yaw_value = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.yaw_value.setFont(font)
        self.yaw_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.yaw_value.setObjectName("yaw_value")
        self.formLayout_3.setWidget(6, QtWidgets.QFormLayout.FieldRole, self.yaw_value)
        self.throttle_text = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.throttle_text.setFont(font)
        self.throttle_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.throttle_text.setObjectName("throttle_text")
        self.formLayout_3.setWidget(7, QtWidgets.QFormLayout.LabelRole, self.throttle_text)
        self.throttle_value = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.throttle_value.setFont(font)
        self.throttle_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.throttle_value.setObjectName("throttle_value")
        self.formLayout_3.setWidget(7, QtWidgets.QFormLayout.FieldRole, self.throttle_value)
        self.gps_text = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.gps_text.setFont(font)
        self.gps_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.gps_text.setObjectName("gps_text")
        self.formLayout_3.setWidget(8, QtWidgets.QFormLayout.LabelRole, self.gps_text)
        self.gps_value = QtWidgets.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.gps_value.setFont(font)
        self.gps_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.gps_value.setObjectName("gps_value")
        self.formLayout_3.setWidget(8, QtWidgets.QFormLayout.FieldRole, self.gps_value)
        self.formLayoutWidget_4 = QtWidgets.QWidget(self.centralwidget)
        self.formLayoutWidget_4.setGeometry(QtCore.QRect(410, 140, 247, 116))
        self.formLayoutWidget_4.setObjectName("formLayoutWidget_4")
        self.formLayout_4 = QtWidgets.QFormLayout(self.formLayoutWidget_4)
        self.formLayout_4.setContentsMargins(0, 0, 0, 0)
        self.formLayout_4.setObjectName("formLayout_4")
        self.telemetry_value = QtWidgets.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(15)
        font.setBold(False)
        font.setWeight(50)
        self.telemetry_value.setFont(font)
        self.telemetry_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.telemetry_value.setObjectName("telemetry_value")
        self.formLayout_4.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.telemetry_value)
        self.arm_text = QtWidgets.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(15)
        font.setBold(False)
        font.setWeight(50)
        self.arm_text.setFont(font)
        self.arm_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.arm_text.setObjectName("arm_text")
        self.formLayout_4.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.arm_text)
        self.battery_value = QtWidgets.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(15)
        font.setBold(False)
        font.setWeight(50)
        self.battery_value.setFont(font)
        self.battery_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.battery_value.setObjectName("battery_value")
        self.formLayout_4.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.battery_value)
        self.battery_text = QtWidgets.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(15)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.battery_text.setFont(font)
        self.battery_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.battery_text.setObjectName("battery_text")
        self.formLayout_4.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.battery_text)
        self.mode_text = QtWidgets.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(15)
        font.setBold(False)
        font.setWeight(50)
        self.mode_text.setFont(font)
        self.mode_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.mode_text.setObjectName("mode_text")
        self.formLayout_4.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.mode_text)
        self.telemetry_text = QtWidgets.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(15)
        font.setBold(False)
        font.setWeight(50)
        self.telemetry_text.setFont(font)
        self.telemetry_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.telemetry_text.setObjectName("telemetry_text")
        self.formLayout_4.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.telemetry_text)
        self.arm_value = QtWidgets.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(15)
        font.setBold(False)
        font.setWeight(50)
        self.arm_value.setFont(font)
        self.arm_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.arm_value.setObjectName("arm_value")
        self.formLayout_4.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.arm_value)
        self.mode_value = QtWidgets.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(15)
        font.setBold(False)
        font.setWeight(50)
        self.mode_value.setFont(font)
        self.mode_value.setStyleSheet("color: rgb(0, 245, 0);")
        self.mode_value.setObjectName("mode_value")
        self.formLayout_4.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.mode_value)
        self.map = QtWidgets.QLabel(self.centralwidget)
        self.map.setGeometry(QtCore.QRect(120, 380, 500, 600))
        self.map.setStyleSheet("")
        self.map.setText("")
        self.map.setPixmap(QtGui.QPixmap(":/newPrefix/WhatsApp-Image-2022-02-14-at-03.52.04-1-1024x550.jpeg"))
        self.map.setScaledContents(True)
        self.map.setObjectName("map")
        self.logo = QtWidgets.QLabel(self.centralwidget)
        self.logo.setGeometry(QtCore.QRect(1320, 0, 441, 351))
        self.logo.setText("")
        self.logo.setPixmap(QtGui.QPixmap("SONYAKAMOZ1.png"))
        self.logo.setScaledContents(True)
        self.logo.setObjectName("logo")
        self.arm_button = QtWidgets.QPushButton(self.centralwidget)
        self.arm_button.setGeometry(QtCore.QRect(710, 710, 150, 50))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.arm_button.setFont(font)
        self.arm_button.setStyleSheet("color: rgb(0, 245, 0);\n"
                                      "background-color: rgb(60, 60, 60);")
        self.arm_button.setObjectName("arm_button")
        self.disarm_button = QtWidgets.QPushButton(self.centralwidget)
        self.disarm_button.setGeometry(QtCore.QRect(880, 710, 150, 50))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.disarm_button.setFont(font)
        self.disarm_button.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.disarm_button.setAutoFillBackground(False)
        self.disarm_button.setStyleSheet("color: rgb(0, 245, 0);\n"
                                         "background-color: rgb(60, 60, 60);")
        self.disarm_button.setAutoDefault(False)
        self.disarm_button.setObjectName("disarm_button")
        self.manual_button = QtWidgets.QPushButton(self.centralwidget)
        self.manual_button.setGeometry(QtCore.QRect(880, 790, 150, 50))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.manual_button.setFont(font)
        self.manual_button.setStyleSheet("color: rgb(0, 245, 0);\n"
                                         "background-color: rgb(60, 60, 60);")
        self.manual_button.setObjectName("manual_button")
        self.auto_button = QtWidgets.QPushButton(self.centralwidget)
        self.auto_button.setGeometry(QtCore.QRect(1050, 790, 150, 50))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.auto_button.setFont(font)
        self.auto_button.setStyleSheet("color: rgb(0, 245, 0);\n"
                                       "background-color: rgb(60, 60, 60);")
        self.auto_button.setObjectName("auto_button")
        self.fbwb_button = QtWidgets.QPushButton(self.centralwidget)
        self.fbwb_button.setGeometry(QtCore.QRect(880, 870, 150, 50))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.fbwb_button.setFont(font)
        self.fbwb_button.setStyleSheet("color: rgb(0, 245, 0);\n"
                                       "background-color: rgb(60, 60, 60);\n"
                                       "")
        self.fbwb_button.setObjectName("fbwb_button")
        self.autotune_button = QtWidgets.QPushButton(self.centralwidget)
        self.autotune_button.setGeometry(QtCore.QRect(1050, 870, 150, 50))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.autotune_button.setFont(font)
        self.autotune_button.setStyleSheet("color: rgb(0, 245, 0);\n"
                                           "background-color: rgb(60, 60, 60);")
        self.autotune_button.setObjectName("autotune_button")
        self.rtl_button = QtWidgets.QPushButton(self.centralwidget)
        self.rtl_button.setGeometry(QtCore.QRect(1050, 710, 150, 50))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.rtl_button.setFont(font)
        self.rtl_button.setStyleSheet("color: rgb(0, 245, 0);\n"
                                      "background-color: rgb(60, 60, 60);")
        self.rtl_button.setObjectName("rtl_button")
        self.loiter_button = QtWidgets.QPushButton(self.centralwidget)
        self.loiter_button.setGeometry(QtCore.QRect(710, 790, 150, 50))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.loiter_button.setFont(font)
        self.loiter_button.setStyleSheet("color: rgb(0, 245, 0);\n"
                                         "background-color: rgb(60, 60, 60);")
        self.loiter_button.setObjectName("loiter_button")
        self.fbwa_button = QtWidgets.QPushButton(self.centralwidget)
        self.fbwa_button.setGeometry(QtCore.QRect(710, 870, 150, 50))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.fbwa_button.setFont(font)
        self.fbwa_button.setStyleSheet("color: rgb(0, 245, 0);\n"
                                       "background-color: rgb(60, 60, 60);")
        self.fbwa_button.setObjectName("fbwa_button")
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(1320, 20, 321, 64))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.setObjectName("formLayout")
        self.ip_text = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.ip_text.setFont(font)
        self.ip_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.ip_text.setObjectName("ip_text")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.ip_text)
        self.ip_value = QtWidgets.QLineEdit(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ip_value.sizePolicy().hasHeightForWidth())
        self.ip_value.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.ip_value.setFont(font)
        self.ip_value.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.ip_value.setAutoFillBackground(False)
        self.ip_value.setStyleSheet("color: rgb(0, 245, 0);\n"
                                    "background-color: rgb(53, 53, 53);\n"
                                    "")
        self.ip_value.setObjectName("ip_value")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.ip_value)
        self.baud_text = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.baud_text.setFont(font)
        self.baud_text.setStyleSheet("color: rgb(0, 245, 0);")
        self.baud_text.setObjectName("baud_text")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.baud_text)
        self.baud_value = QtWidgets.QLineEdit(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.baud_value.sizePolicy().hasHeightForWidth())
        self.baud_value.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.baud_value.setFont(font)
        self.baud_value.setStyleSheet("color: rgb(0, 245, 0);\n"
                                      "background-color: rgb(53, 53, 53);\n"
                                      "")
        self.baud_value.setObjectName("baud_value")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.baud_value)
        self.horizontalLayout.addLayout(self.formLayout)
        self.connect_button = QtWidgets.QPushButton(self.layoutWidget)
        self.connect_button.setStyleSheet("color: rgb(0, 245, 0);\n"
                                          "background-color: rgb(60, 60, 60);")
        self.connect_button.setObjectName("connect_button")
        self.horizontalLayout.addWidget(self.connect_button)
        self.textBrowser = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QtCore.QRect(1330, 230, 421, 111))
        self.textBrowser.setStyleSheet("color: rgb(0, 245, 0);\n"
                                       "background-color: rgb(60, 60, 60);")
        self.textBrowser.setObjectName("textBrowser")
        self.logo.raise_()
        self.camera.raise_()
        self.gyroscope.raise_()
        self.formLayoutWidget_2.raise_()
        self.formLayoutWidget_3.raise_()
        self.formLayoutWidget_4.raise_()
        self.map.raise_()
        self.arm_button.raise_()
        self.disarm_button.raise_()
        self.manual_button.raise_()
        self.auto_button.raise_()
        self.fbwb_button.raise_()
        self.autotune_button.raise_()
        self.rtl_button.raise_()
        self.loiter_button.raise_()
        self.fbwa_button.raise_()
        self.layoutWidget.raise_()
        self.textBrowser.raise_()
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1920, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.arm_button.clicked.connect(self.arm_butonu)
        self.fbwa_button.clicked.connect(self.fbwa_butonu)
        self.auto_button.clicked.connect(self.auto_butonu)
        self.rtl_button.clicked.connect(self.rtl_butonu)
        self.fbwb_button.clicked.connect(self.fbwb_butonu)
        self.loiter_button.clicked.connect(self.loiter_butonu)
        self.manual_button.clicked.connect(self.manuel_butonu)
        self.disarm_button.clicked.connect(self.disarm_butonu)
        self.autotune_button.clicked.connect(self.start_autotune_thread)
        self.connect_button.clicked.connect(self.connect_vehicle)  # Connect button to the connect_vehicle function

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.saat_text.setText(_translate("MainWindow", "SAAT :"))
        self.tarih_text.setText(_translate("MainWindow", "TARİH :"))
        self.tarih_value.setText(_translate("MainWindow", ""))
        self.saat_value.setText(_translate("MainWindow", ""))
        self.altitude_text.setText(_translate("MainWindow", "YÜKSEKLİK:"))
        self.altitude_value.setText(_translate("MainWindow", ""))
        self.airspeed_text.setText(_translate("MainWindow", "HAVA HIZI:"))
        self.airspeed_value.setText(_translate("MainWindow", ""))
        self.gpsspeed_text.setText(_translate("MainWindow", "GPS HIZI:"))
        self.gpsspeed_value.setText(_translate("MainWindow", ""))
        self.distance_text.setText(_translate("MainWindow", "UZAKLIK:"))
        self.distance_value.setText(_translate("MainWindow", ""))
        self.roll_text.setText(_translate("MainWindow", "ROLL:"))
        self.roll_value.setText(_translate("MainWindow", ""))
        self.pitch_text.setText(_translate("MainWindow", "PITCH:"))
        self.pitch_value.setText(_translate("MainWindow", ""))
        self.yaw_text.setText(_translate("MainWindow", "YAW:"))
        self.yaw_value.setText(_translate("MainWindow", ""))
        self.throttle_text.setText(_translate("MainWindow", "GAZ:"))
        self.throttle_value.setText(_translate("MainWindow", ""))
        self.gps_text.setText(_translate("MainWindow", "GPS SAYISI:"))
        self.gps_value.setText(_translate("MainWindow", ""))
        self.telemetry_value.setText(_translate("MainWindow", ""))
        self.arm_text.setText(_translate("MainWindow", "ARM :"))
        self.battery_value.setText(_translate("MainWindow", ""))
        self.battery_text.setText(_translate("MainWindow", "BATARYA :"))
        self.mode_text.setText(_translate("MainWindow", "MOD DURUM :"))
        self.telemetry_text.setText(_translate("MainWindow", "TELEMETRİ :"))
        self.arm_value.setText(_translate("MainWindow", ""))
        self.mode_value.setText(_translate("MainWindow", ""))
        self.arm_button.setText(_translate("MainWindow", "ARM"))
        self.disarm_button.setText(_translate("MainWindow", "DISARM"))
        self.manual_button.setText(_translate("MainWindow", "GUIDED"))
        self.auto_button.setText(_translate("MainWindow", "STABILIZE"))
        self.fbwb_button.setText(_translate("MainWindow", "LAND"))
        self.autotune_button.setText(_translate("MainWindow", "GÖREV"))
        self.rtl_button.setText(_translate("MainWindow", "RTL"))
        self.loiter_button.setText(_translate("MainWindow", "LOITER"))
        self.fbwa_button.setText(_translate("MainWindow", "TAKEOFF"))
        self.ip_text.setText(_translate("MainWindow", "IP :"))
        self.baud_text.setText(_translate("MainWindow", "BAUD :"))
        self.connect_button.setText(_translate("MainWindow", "CONNECT"))
        self.textBrowser.setHtml(_translate("MainWindow",
                                            "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                                            "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
                                            "p, li { white-space: pre-wrap; }\n"
                                            "</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"


                                            "<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))

    def update_text_browser(self, message):
        self.message_log.append(message)
        self.textBrowser.setText("\n".join(self.message_log))
        self.textBrowser.moveCursor(QtGui.QTextCursor.End)

    def connect_vehicle(self):
        ip = self.ip_value.text()
        baud = int(self.baud_value.text())
        connection_string = f"{ip}:{baud}"
        global vehicle
        vehicle = connect(connection_string, wait_ready=False)
        self.data_fetcher = DataFetcher(vehicle)
        self.data_fetcher.dataFetched.connect(self.update_data)
        self.data_fetcher.start()
        self.update_text_browser(f"Bağlantı kuruldu: {connection_string}")

    def yukseklik(self, data):
        location = data.get('location')
        if location:
            self.altitude_value.setText(f"{location.alt:.2f}")

    def hava_hizi(self, data):
        airspeed = data.get('airspeed')
        self.airspeed_value.setText(f"{airspeed:.2f} m/s")

    def gps_hizi(self, data):
        groundspeed = data.get('groundspeed')
        self.gpsspeed_value.setText(f"{groundspeed:.2f} m/s")

    def gps_sayisi(self, data):
        gps = data.get('gps')
        if gps:
            self.gps_value.setText(str(gps.satellites_visible))

    def roll_acisi(self, data):
        attitude = data.get('attitude')
        if attitude:
            self.roll_value.setText(f"{math.degrees(attitude.roll):.2f}")

    def pitch_acisi(self, data):
        attitude = data.get('attitude')
        if attitude:
            self.pitch_value.setText(f"{math.degrees(attitude.pitch):.2f}")

    def yaw_acisi(self, data):
        attitude = data.get('attitude')
        if attitude:
            self.yaw_value.setText(f"{math.degrees(attitude.yaw):.2f}")

    def batarya_durumu(self, data):
        battery = data.get('battery')
        if battery:
            self.battery_value.setText(f"{battery.level}%")

    def mod_durumu(self, data):
        current_mode = data.get('mode')
        if current_mode:
            self.mode_value.setText(str(current_mode.name))

    def arm(self, data):
        armed = data.get('armed')
        self.arm_value.setText("TRUE" if armed else "FALSE")

    def telemetri(self):
        self.telemetry_value.setText(str("%99"))

    def arm_butonu(self):
        while not vehicle.is_armable:
            self.update_text_browser("ARM için bekleniyor...")

        vehicle.armed = True
        while not vehicle.armed:
            self.update_text_browser("ARM ediliyor...")

        if vehicle.armed:
            self.update_text_browser("Hava Aracı ARM Edildi.")
        else:
            self.update_text_browser("ARM edilemedi.")

    def manuel_butonu(self):
        if vehicle.mode.name != "GUIDED":
            vehicle.mode = VehicleMode("GUIDED")
            print("Araç GUIDED moduna geçti.")
        else:
            print("Araç zaten GUIDED modunda.")

    def fbwa_butonu(self):
        target_altitude = 25
        vehicle.simple_takeoff(target_altitude)
        if vehicle.location.global_relative_frame.alt > 1:
            self.update_text_browser("ARAÇ ZATEN HAVADA!")
        else:
            self.update_text_browser("ARAÇ KALKIŞ YAPIYOR")

    def auto_butonu(self):
        vehicle.mode = VehicleMode("STABILIZE")

    def disarm_butonu(self):
        if vehicle.armed:
            if vehicle.location.global_relative_frame.alt > 1:  # 1 metre üzerinde ise
                self.update_text_browser("ARAÇ HAVADA! İNİŞE GEÇİYOR")
                vehicle.mode = VehicleMode("LAND")
            else:
                vehicle.disarm()
                while vehicle.armed:  # Disarm işlemi gerçekleşene kadar bekle
                    self.update_text_browser("DISARM ediliyor...")

                if not vehicle.armed:
                    self.update_text_browser("HAVA ARACI DISARM EDİLDİ.")
                else:
                    self.update_text_browser("DISARM edilemedi.")

    def fbwb_butonu(self):
        if vehicle.mode.name == "LAND":
            print("Araç zaten LAND modunda.")
        else:
            vehicle.mode = VehicleMode("LAND")
            print("Araç LAND moduna geçirildi.")

    def rtl_butonu(self):
        if vehicle.mode.name == "RTL":
            self.update_text_browser("Araç zaten RTL modunda.")
        else:
            vehicle.mode = VehicleMode("RTL")
            self.update_text_browser("EVE DÖNÜYOR.")

    def loiter_butonu(self):
        if vehicle.mode.name != "LOITER":
            vehicle.mode = VehicleMode("LOITER")
            print("Araç LOITER moduna geçti.")
        else:
            print("Araç zaten LOITER modunda.")

    def start_autotune_thread(self):
        self.autotune_thread = AutotuneThread(vehicle)
        self.autotune_thread.start()

    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        # Dünya'nın yarıçapı (km)
        R = 6371.0

        lat1, lon1 = radians(lat1), radians(lon1)
        lat2, lon2 = radians(lat2), radians(lon2)

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        distance = R * c

        return distance


    def uzaklik(self, data):
        home_location = data.get('home_location')
        current_location = data.get('current_location')
        if home_location and current_location:
            distance = self.haversine_distance(home_location.lat, home_location.lon, current_location.lat, current_location.lon)
            self.distance_value.setText(f"{distance:.2f} km")

    def saat(self):
        saat1 = datetime.datetime.now().strftime('%H:%M:%S')
        self.saat_value.setText(saat1)

    def tarih(self):
        tarih1 = datetime.datetime.now().strftime("%d-%m-%Y")
        self.tarih_value.setText(tarih1)

if __name__ == "__main__":
    # Aracınıza bağlanın
    connect_vehicle()

    # Flask uygulamasını ayrı bir iş parçacığında başlatın
    flask_thread = threading.Thread(target=start_flask_app)
    flask_thread.daemon = True
    flask_thread.start()

    # PyQt5 uygulamasını başlatın
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()

    # Webview kullanarak haritayı pencere içinde gösterin
    webview.create_window("Harita Görüntüsü", "http://127.0.0.1:5000")
    webview.start()

    sys.exit(app.exec_())
