import pigpio
import os
from mpu6050 import mpu6050
import time
import math

os.system("sudo pigpiod")
time.sleep(1)
global elapsedTime

mpu = mpu6050(0x68)
start = time.time()


# pi = pigpio.pi()
# pi.set_servo_pulsewidth(escLeft,0)
# pi.set_servo_pulsewidth(escRight,0)
os.system("pigs s 27 0 s 22 0")

max_value = 2000
min_value = 700

global pid_p
global pid_i
global pid_d

global previousError
global desiredAngle

# PID constants
global kp
global ki
global kd

# Motor pwm signal
global pwmLeft
global pwmRight
global throttle

# Declaring variables for initial PID
pid_p = 0  # Faster response|More overshoot
pid_i = 0  # Faster response|More overshoot|No steady state error
pid_d = 0  # No change in response|Less overshoot|Destabilizes|10% of kp

previousError = 0
desiredAngle = 0

# PID constants
kp = 5.55
ki = 0.20
kd = 0.55

# Motor pwm signal
pwmLeft = 0
pwmRight = 0
throttle = 1000

pitch = 0


# PID code
def PID(gyro):
    global pid_p
    global pid_i
    global pid_d
    global previousError
    global desiredAngle
    global pwmLeft
    global pwmRight
    global throttle
    global elapsedTime
    error = gyro - desiredAngle

    # Proportional
    pid_p = kp * error

    # Integral
    if error > -3 and error < 3:
        pid_i = pid_i + (ki * error)

    # Derivative
    pid_d = ((error - previousError) / elapsedTime) * kd

    PID = pid_p + pid_i + pid_d

    # For max and min throttle of motors
    if PID < -min_value:
        PID = -min_value
    if PID > min_value:
        PID = min_value

    pwmLeft = throttle + PID
    pwmRight = throttle - PID

    # Making sure that the input to the ESC is within the range
    if pwmRight < min_value:
        pwmRight = min_value
    if pwmRight > max_value:
        pwmRight = max_value

    if pwmLeft < min_value:
        pwmLeft = min_value
    if pwmLeft > max_value:
        pwmLeft = max_value

    previousError = error

    return (pwmLeft, pwmRight)


# Function to get the gyro and accel errors in Y-direction
# Keep the MPU on flat surface during this time
def Offset():
    gyro_offset = 0
    accel_offset = 0
    for i in range(1000):
        gyro_data = mpu.get_gyro_data()
        accel_data = mpu.get_accel_data()
        gyro_offset += gyro_data["y"]
        accel_offset += accel_data["y"]
    return (gyro_offset / 1000, accel_offset / 1000)


y_offset, y_accel_offset = Offset()

while True:
    prevTime = start
    start = time.time()
    elapsedTime = start - prevTime

    accel_data = mpu.get_accel_data()
    accX = accel_data["x"]
    accY = accel_data["y"]
    accZ = accel_data["z"]
    gyro_data = mpu.get_gyro_data()
    y = gyro_data["y"] - y_offset

    # Converting degree/sec to degrees
    pitch += y * elapsedTime

    # Complementary filter
    accel_pitch = (
        math.atan(-1 * accX / math.sqrt(math.pow(accY, 2) + math.pow(accZ, 2)))
        * 180
        / math.pi
    ) + y_accel_offset
    pitch = 0.98 * pitch + 0.02 * accel_pitch

    pwmLeft, pwmRight = PID(pitch)
    # pi.set_servo_pulsewidth(escLeft,pwmLeft)
    # pi.set_servo_pulsewidth(escRight,pwmRight)
    print("PWM LEFT: ", pwmLeft)
    print("PWM RIGHT: ", pwmRight)
    os.system("pigs s 27 " + str(pwmLeft) + " s 22 " + str(pwmRight))

    print("Pitch: " + str(pitch))
    # time.sleep(.2)
