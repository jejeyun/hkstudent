import RPi.GPIO as GPIO
import time
import smbus2
from RPLCD.i2c import CharLCD

# 스위치와 서보모터 핀 번호
SW_open = 20
SW_close = 21
SERVO_PIN = 16

bus = smbus2.SMBus(1)
address = 0x27

# I2C LCD 초기화
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, 
                port=1, cols=20, rows=4, 
                dotsize=8, charmap='A02', 
                auto_linebreaks=True, backlight_enabled=True)

# GPIO 핀 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(SW_open, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SW_close, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# PWM 객체 생성
pwm = GPIO.PWM(SERVO_PIN, 50)

# 서보모터 위치 조정 함수
def setAngle(angle):
    duty = angle / 18 + 2
    GPIO.output(SERVO_PIN, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(SERVO_PIN, False)
    pwm.ChangeDutyCycle(0)

# 인터럽트 이벤트 핸들러
def switch_callback(channel):
    if channel == SW_open:
        print("Open the Door!")
        lcd.clear()
        lcd.write_string("Open the Door!")
        for i in range(1, 180, 10):
            setAngle(i)
    
    elif channel == SW_close:
        print("Close the Door!")
        lcd.clear()
        lcd.write_string("Close the Door!")
        for i in range(180, 1, -10):
            setAngle(i)

# 인터럽트 이벤트 등록
GPIO.add_event_detect(SW_open, GPIO.FALLING, callback=switch_callback, bouncetime=200)
GPIO.add_event_detect(SW_close, GPIO.FALLING, callback=switch_callback, bouncetime=200)

# PWM 신호 시작
pwm.start(0)

try:
    while True:
        pass

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()