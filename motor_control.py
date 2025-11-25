# motor_control.py
# MDDS60 또는 유사 드라이버 제어용 (RPi GPIO PWM)
# 필요: RPi.GPIO 설치 (pip install RPi.GPIO) - 라즈베리파이에서 실행

import RPi.GPIO as GPIO
import time
from typing import Tuple

# --- 사용자 설정 (배선에 맞게 변경) ---
# PWM 핀 (BCM 번호)
LEFT_PWM_PIN  = 18  # 예: GPIO18 (PWM0)
RIGHT_PWM_PIN = 13  # 예: GPIO13 (PWM1)

# 방향 제어 핀 (디지털)
LEFT_DIR_A = 23
LEFT_DIR_B = 24
RIGHT_DIR_A = 25
RIGHT_DIR_B = 8

# 비상 정지(스위치) 입력 핀 (pull-up)
KILL_SWITCH_PIN = 21

PWM_FREQ = 1000  # Hz

# --- 초기화 ---
GPIO.setmode(GPIO.BCM)
GPIO.setup([LEFT_DIR_A, LEFT_DIR_B, RIGHT_DIR_A, RIGHT_DIR_B], GPIO.OUT, initial=GPIO.LOW)
GPIO.setup([LEFT_PWM_PIN, RIGHT_PWM_PIN], GPIO.OUT)
GPIO.setup(KILL_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 스위치가 눌리면 LOW 라고 가정

left_pwm = GPIO.PWM(LEFT_PWM_PIN, PWM_FREQ)
right_pwm = GPIO.PWM(RIGHT_PWM_PIN, PWM_FREQ)
left_pwm.start(0)
right_pwm.start(0)

def kill_pressed() -> bool:
    # 스위치 눌림 => LOW (회로에 따라 반전 필요)
    return GPIO.input(KILL_SWITCH_PIN) == GPIO.LOW

def set_motor_speed(left_percent: float, right_percent: float):
    """
    left_percent, right_percent: -100..100 (음수 => 뒤로, 양수 => 앞으로)
    """
    if kill_pressed():
        stop()
        return

    # 클램프
    def clamp(v):
        if v > 100: return 100.0
        if v < -100: return -100.0
        return v

    l = clamp(left_percent)
    r = clamp(right_percent)

    # 방향 설정
    def apply_direction(a_pin, b_pin, val):
        if val >= 0:
            GPIO.output(a_pin, GPIO.HIGH)
            GPIO.output(b_pin, GPIO.LOW)
        else:
            GPIO.output(a_pin, GPIO.LOW)
            GPIO.output(b_pin, GPIO.HIGH)

    apply_direction(LEFT_DIR_A, LEFT_DIR_B, l)
    apply_direction(RIGHT_DIR_A, RIGHT_DIR_B, r)

    # PWM 값 (0..100)
    left_pwm.ChangeDutyCycle(abs(l))
    right_pwm.ChangeDutyCycle(abs(r))

def stop():
    left_pwm.ChangeDutyCycle(0)
    right_pwm.ChangeDutyCycle(0)
    GPIO.output([LEFT_DIR_A, LEFT_DIR_B, RIGHT_DIR_A, RIGHT_DIR_B], GPIO.LOW)

def cleanup():
    stop()
    left_pwm.stop()
    right_pwm.stop()
    GPIO.cleanup()
