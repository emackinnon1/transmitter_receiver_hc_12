from machine import Pin, I2C, UART, lightsleep, reset
import time
import ustruct
from LC709203F_CR import LC709203F, PowerMode
import math
 
# Constants
ADXL345_ADDRESS = 0x53
ADXL345_POWER_CTL = 0x2D
ADXL345_DATA_FORMAT = 0x31
ADXL345_DATAX0 = 0x32
led_onboard = machine.Pin(25, machine.Pin.OUT)
fuel_check_interval = 10
acc_baseline = 40
orientation_baseline = 5

# Globals
last_x = None
last_y = None
last_z = None
last_pitch = None
last_roll = None
last_magnitude = None
last_batt = None
next_fuel_check = 0

# Initialize I2C
i2c_acc = I2C(1, sda=Pin(14), scl=Pin(15), freq=400000)
i2c_fuel_gauge = I2C(0, sda=Pin(0), scl=Pin(1))
print(i2c_fuel_gauge.scan())
# init uart
uart = UART(1, 9600, rx=Pin(5), tx=Pin(4) , bits=8, parity=None, stop=1, timeout=1)

sensor = LC709203F(i2c_fuel_gauge)
def flash_led():
    led_onboard.value(1)
    for i in range(3):
        led_onboard.toggle()
        time.sleep(0.1)
        led_onboard.toggle()
        time.sleep(0.1)
flash_led()

 
# Initialize ADXL345
def init_adxl345():
    i2c_acc.writeto_mem(ADXL345_ADDRESS, ADXL345_POWER_CTL, bytearray([0x08]))  # Set bit 3 to 1 to enable measurement mode
    i2c_acc.writeto_mem(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, bytearray([0x0B]))  # Set data format to full resolution, +/- 16g
 
# Read acceleration data
def read_accel_data():
    data = i2c_acc.readfrom_mem(ADXL345_ADDRESS, ADXL345_DATAX0, 6)
    x, y, z = ustruct.unpack('<3h', data)
    return x, y, z

# Calculate the magnitude of acceleration
def calc_accel_magnitude(x, y, z):
    return math.sqrt(x**2 + y**2 + z**2)
 
# Calculate roll angle in degrees
def calc_roll(x, y, z):
    return math.atan2(y, math.sqrt(x**2 + z**2)) * (180 / math.pi)
 
# Calculate pitch angle in degrees
def calc_pitch(x, y, z):
    return math.atan2(-x, math.sqrt(y**2 + z**2)) * (180 / math.pi)

init_adxl345()

def send_serial_msg(msg, sleep=None):
    uart.write(bytearray(msg, 'utf8'))
    if sleep:
        time.sleep(sleep)
    else:
        time.sleep(0.2)

def send_status(status):
    if status == "online":
        send_serial_msg('1,C,a', 0.5)
    else:
        send_serial_msg('1,C,b', 0.5)

def read_absolute_change(last, current, baseline):
    return last and abs(last - current) >= baseline

def detect_movement():
    global last_x
    global last_y
    global last_z
    global last_roll
    global last_pitch
    global last_magnitude

    led_onboard.value(1)
    x, y, z = read_accel_data()
    magnitude = calc_accel_magnitude(x, y, z)
    roll = calc_roll(x, y, z)
    pitch = calc_pitch(x, y, z)
    print("roll, pitch", roll, pitch)
    print("X: ", x, "Y: ", y, "Z: ", z)
    if read_absolute_change(last_x, x, acc_baseline) or read_absolute_change(last_y, y, acc_baseline) or read_absolute_change(last_z, z, acc_baseline):
        send_serial_msg('1,A,a')
        print("FUCK SOMETHIN MOVED")
        flash_led()
        send_serial_msg('1,D,b')
        last_x, last_y, last_z = x, y, z
        return
    if read_absolute_change(last_roll, roll, orientation_baseline) or read_absolute_change(last_pitch, pitch, orientation_baseline):
        send_serial_msg('1,A,a')
        print("FUCK ORIENTATION CHANGED")
        flash_led()
        send_serial_msg('1,D,a')
        last_roll, last_pitch = roll, pitch
        return
    if last_roll == None or last_pitch == None:
        last_roll, last_pitch = roll, pitch
    if last_x == None or last_y == None or last_z == None:
        last_x, last_y, last_z = x, y, z
    
def check_fuel():
    global last_batt
    global next_fuel_check
    new_reading = sensor.cell_percent
    print("new batt reading: " + str(new_reading), "new voltage reading: " + str(sensor.cell_voltage))
    if last_batt != new_reading:
        send_serial_msg('1,B,' + str(new_reading))
        last_batt = new_reading
    next_fuel_check = time.time() + fuel_check_interval

led_onboard.toggle()
send_status("online")
led_onboard.toggle()
while True:
    try:
        sensor.setOperateMode
        detect_movement()
        check_fuel()
        time.sleep_ms(100)
        sensor.setSleepMode
        lightsleep(3000)
    except Exception as e:
        print(e)
        send_status("error")
        time.sleep(1)
        reset()
