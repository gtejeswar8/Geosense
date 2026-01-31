import smbus
import time
import requests
import subprocess
import numpy as np

from math import atan2, degrees, sin

# ==== IP Location Config ====
API_TOKEN = "db010b4fca0623"

class EMAFilter:
    def __init__(self, alpha=0.2):    # Fixed constructor name
        self.alpha = alpha
        self.estimate = None
    def update(self, measurement):
        if self.estimate is None:
            self.estimate = measurement
        else:
            self.estimate = self.alpha * measurement + (1 - self.alpha) * self.estimate
        return self.estimate

# ==== Sensor Init ====
bus = smbus.SMBus(1)
MPU_ADDR = 0x68
bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # wake up MPU6050

QMC5883L_ADDR = 0x0d

# QMC5883L Initialization
bus.write_byte_data(QMC5883L_ADDR, 0x0B, 0x01)  # Reset
time.sleep(0.1)
bus.write_byte_data(QMC5883L_ADDR, 0x09, 0x1D)  # Control register: continuous mode, ODR=10Hz, RNG=2G

def read_word(addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return val - 65536
    else:
        return val

def read_word_qmc(addr, reg):
    low = bus.read_byte_data(addr, reg)
    high = bus.read_byte_data(addr, reg + 1)
    val = (high << 8) | low
    if val >= 0x8000:
        return val - 65536
    else:
        return val

def get_accel():
    accel_x = read_word(MPU_ADDR, 0x3B) / 16384.0
    accel_y = read_word(MPU_ADDR, 0x3D) / 16384.0
    accel_z = read_word(MPU_ADDR, 0x3F) / 16384.0
    return accel_x, accel_y, accel_z

def get_gyro():
    gyro_x = read_word(MPU_ADDR, 0x43) / 131.0
    gyro_y = read_word(MPU_ADDR, 0x45) / 131.0
    gyro_z = read_word(MPU_ADDR, 0x47) / 131.0
    return gyro_x, gyro_y, gyro_z

def get_mag_qmc():
    mag_x = read_word_qmc(QMC5883L_ADDR, 0x00)
    mag_y = read_word_qmc(QMC5883L_ADDR, 0x02)
    mag_z = read_word_qmc(QMC5883L_ADDR, 0x04)
    return mag_x, mag_y, mag_z

def get_heading():
    x = read_word_qmc(QMC5883L_ADDR, 0x00)
    y = read_word_qmc(QMC5883L_ADDR, 0x02)
    heading_rad = atan2(y, x)
    heading_deg = degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg

def get_actual_location():
    try:
        res = requests.get("https://ipinfo.io/json?token=" + API_TOKEN, timeout=5).json()
        loc = res.get("loc", "0,0").split(",")
        return float(loc[0]), float(loc[1])
    except:
        return 0.0, 0.0

def get_wifi_rssi():
    try:
        output = subprocess.check_output(["iwconfig", "wlan0"], stderr=subprocess.STDOUT).decode()
        for line in output.split("\n"):
            if "Signal level=" in line:
                parts = line.strip().split("Signal level=")
                if len(parts) > 1:
                    rssi_str = parts[1].split(" ")[0]
                    return int(rssi_str)
    except:
        pass
    return -70  # fallback

def normalize_rssi(rssi):
    rssi_clipped = max(min(rssi, -30), -100)
    return (rssi_clipped + 100) / 70.0

# Symbolic regression formula exactly as in your notebook:
def symbolic_prediction(accel_z, gyro_x, gyro_y, gyro_z, wifi_rssi_dbm):
    lat_scaled = (accel_z - gyro_z) * wifi_rssi_dbm * 0.023196703
    lon_scaled = sin(gyro_x * gyro_y) * -0.2512324
    lat = 13.0870 + lat_scaled * 0.0008
    lon = 80.2780 + lon_scaled * 0.0009
    return lat, lon

def get_lut_bins(norm_rssi, heading):
    rssi_bin = min(int(norm_rssi * 10), 9)
    heading_bin = int(heading / 45) % 8
    return rssi_bin, heading_bin

LUT = {}
MAX_VALUES_PER_BIN = 10

def update_lut(rssi_bin, heading_bin, new_delta_lat, new_delta_lon):
    key = (rssi_bin, heading_bin)
    if key not in LUT:
        LUT[key] = {'lat_vals': [], 'lon_vals': []}
    LUT[key]['lat_vals'].append(new_delta_lat)
    LUT[key]['lon_vals'].append(new_delta_lon)
    if len(LUT[key]['lat_vals']) > MAX_VALUES_PER_BIN:
        LUT[key]['lat_vals'].pop(0)
        LUT[key]['lon_vals'].pop(0)

def get_lut_correction(rssi_bin, heading_bin):
    key = (rssi_bin, heading_bin)
    if key in LUT and LUT[key]['lat_vals']:
        avg_lat = sum(LUT[key]['lat_vals']) / len(LUT[key]['lat_vals'])
        avg_lon = sum(LUT[key]['lon_vals']) / len(LUT[key]['lon_vals'])
        return avg_lat, avg_lon
    else:
        return 0.0, 0.0

# Filters
kf_gy = EMAFilter(alpha=0.2)
kf_gz = EMAFilter(alpha=0.2)
kf_gx = EMAFilter(alpha=0.2)
kf_az = EMAFilter(alpha=0.2)
kf_rssi = EMAFilter(alpha=0.2)

gyro_heading = 0.0
last_time = time.time()
pred_lat_list = []
pred_lon_list = []
corr_lat_list = []
corr_lon_list = []
act_lat_list = []
act_lon_list = []
lat_err_list = []
lon_err_list = []
total_err_list = []

for i in range(15):
    gyro_x, gyro_y, gyro_z = get_gyro()
    accel_x, accel_y, accel_z = get_accel()
    mag_x, mag_y, mag_z = get_mag_qmc()

    gyro_x_filt = kf_gx.update(gyro_x)
    gyro_y_filt = kf_gy.update(gyro_y)
    gyro_z_filt = kf_gz.update(gyro_z)
    accel_z_filt = kf_az.update(accel_z)

    raw_rssi = get_wifi_rssi()
    filtered_rssi = kf_rssi.update(raw_rssi)
    normalized_rssi = normalize_rssi(filtered_rssi)

    pred_lat, pred_lon = symbolic_prediction(
        accel_z_filt, gyro_x_filt, gyro_y_filt, gyro_z_filt, raw_rssi
    )

    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    gyro_heading += gyro_z_filt * dt
    gyro_heading %= 360

    compass_heading = get_heading()
    alpha_cf = 0.98
    fused_heading = alpha_cf * gyro_heading + (1 - alpha_cf) * compass_heading
    fused_heading %= 360

    rssi_bin, heading_bin = get_lut_bins(normalized_rssi, fused_heading)
    delta_lat, delta_lon = get_lut_correction(rssi_bin, heading_bin)

    lat_final = pred_lat + delta_lat
    lon_final = pred_lon + delta_lon

    actual_lat, actual_lon = get_actual_location()

    correction_delta_lat = actual_lat - lat_final
    correction_delta_lon = actual_lon - lon_final
    update_lut(rssi_bin, heading_bin, correction_delta_lat, correction_delta_lon)

    delta_lat_corr = abs(lat_final - actual_lat)
    delta_lon_corr = abs(lon_final - actual_lon)
    lat_error_corr = delta_lat_corr * 111320
    lon_error_corr = delta_lon_corr * 108435
    total_error_corr = (lat_error_corr ** 2 + lon_error_corr ** 2) ** 0.5

    print(f"----- Reading {i + 1} -----")
    print(f"RSSI        : {raw_rssi} dBm")
    print(f"Predicted   : Lat = {pred_lat:.6f}, Lon = {pred_lon:.6f}")
    print(f"Corrected   : Lat = {lat_final:.6f}, Lon = {lon_final:.6f}")
    print(f"Actual      : Lat = {actual_lat:.6f}, Lon = {actual_lon:.6f}")
    print(f"Error       : {round(lat_error_corr, 2)} m lat, {round(lon_error_corr, 2)} m lon")
    print(f"Total Error : {round(total_error_corr, 2)} meters\n")
    pred_lat_list.append(pred_lat)
    pred_lon_list.append(pred_lon)
    corr_lat_list.append(lat_final)
    corr_lon_list.append(lon_final)
    act_lat_list.append(actual_lat)
    act_lon_list.append(actual_lon)
    lat_err_list.append(lat_error_corr)
    lon_err_list.append(lon_error_corr)
    total_err_list.append(total_error_corr)


    time.sleep(2)
print("====== FINAL SUMMARY (After 15 Readings) ======")
print(f"Avg Predicted Location : Lat = {np.mean(pred_lat_list):.6f}, Lon = {np.mean(pred_lon_list):.6f}")
print(f"Avg Corrected Location : Lat = {np.mean(corr_lat_list):.6f}, Lon = {np.mean(corr_lon_list):.6f}")
print(f"Avg Actual Location    : Lat = {np.mean(act_lat_list):.6f}, Lon = {np.mean(act_lon_list):.6f}")
print(f"Avg Latitude Error     : {np.mean(lat_err_list):.2f} m")
print(f"Avg Longitude Error    : {np.mean(lon_err_list):.2f} m")
print(f"Avg Total Error        : {np.mean(total_err_list):.2f} meters")


