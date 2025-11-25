from gpiozero import PWMOutputDevice

# 좌우 바퀴 PWM 핀
left_motor = PWMOutputDevice(23)
right_motor = PWMOutputDevice(24)

def drive(x, y):
    f = max(min(y, 1), -1)
    t = max(min(x, 1), -1)

    left = f + t
    right = f - t

    left = max(min(left, 1), -1)
    right = max(min(right, 1), -1)

    left_motor.value = abs(left)
    right_motor.value = abs(right)

def stop():
    left_motor.value = 0
    right_motor.value = 0

def set_logger(func):
    pass
