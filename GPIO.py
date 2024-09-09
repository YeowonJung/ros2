import RPi.GPIO as GPIO
import time

# GPIO 핀 번호 설정
IN1 = 17  # 모터 드라이버의 IN1에 연결
IN2 = 18  # 모터 드라이버의 IN2에 연결
EN = 12   # 모터 드라이버의 Enable 핀에 연결 (PWM 신호)

# GPIO 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(EN, GPIO.OUT)

# PWM 설정 (주파수 100Hz)
pwm = GPIO.PWM(EN, 100)
pwm.start(0)  # 초기 속도 0

def motor_forward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)

def motor_backward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)

def motor_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        motor_forward(50)  # 모터를 전진시키고 50% 속도로 설정
        time.sleep(2)  # 2초 동안 전진
        motor_backward(50)  # 모터를 후진시키고 50% 속도로 설정
        time.sleep(2)  # 2초 동안 후진
        motor_stop()  # 모터 정지
        time.sleep(1)  # 1초 대기

except KeyboardInterrupt:
    pass

# 종료 시 GPIO 정리
GPIO.cleanup()
