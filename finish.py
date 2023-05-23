#Importing Libraries

import cv2, dlib, time, math, smbus2, threading
import RPi.GPIO as GPIO
from RPLCD.i2c import CharLCD

#SW & Motor Setup
SW_open = 21
SW_close = 20
DoorMotor = 16

#TRM Setup
v = 0  # 속도
vmax = 0  # 최대 속도
Sigpin = 14  # 신호 입력 핀
Buzzer = 18  # 부저 핀

#LED Setup
LED_RED = 12
LED_GREEN = 6

#SW_Open Camera Trigger
Camera_Radar_on = False
w_detect = False
v_detect = False

#I2C Adresss
bus = smbus2.SMBus(1)
address = 0x27

#I2C LCD 초기화
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=20, rows=4, dotsize=8, 
                charmap='A02', auto_linebreaks=True, backlight_enabled=True)
# GPIO 핀 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(SW_open, GPIO.IN, pull_up_down = GPIO.PUD_UP)  #Open Switch
GPIO.setup(SW_close, GPIO.IN, pull_up_down = GPIO.PUD_UP) #Close Switch
GPIO.setup(DoorMotor, GPIO.OUT)    #Door Motor
GPIO.setup(Sigpin, GPIO.IN)        #Trm 
GPIO.setup(Buzzer, GPIO.OUT)       #Buzzer
GPIO.setup(LED_RED, GPIO.OUT)
GPIO.setup(LED_GREEN, GPIO.OUT)


# PWM 객체 생성
pwm = GPIO.PWM(DoorMotor, 50)

# 서보 모터 위치 조정 함수
def setAngle(angle):
    global v_detect, w_detect
    if v_detect == True and w_detect==True :
        pwm.stop()
    
    duty = angle / 18 + 2 
    GPIO.output(DoorMotor, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(DoorMotor, False)
    pwm.ChangeDutyCycle(0)
    
# 인터럽트 이벤트 핸들러
def switch_callback(channel):
    global Camera_Radar_on, v_detect, w_detect
    if channel == SW_open:
        time.sleep(1)
        Camera_Radar_on = True
        print("Open the Door!")
        lcd.clear()
        lcd.write_string("Open the Door!")
        #setAngle(180)
        for i in range(1, 180, 10):
            setAngle(i)
    elif channel == SW_close:
        time.sleep(1)
        Camera_Radar_on, v_detect, w_detect =False, False, False
        print("Close the Door!")
        lcd.clear()
        lcd.write_string("Close the Door!")
        setAngle(1)
        #for i in range(180, 1, -10):
        #    setAngle(i)
        
#Interrupt Calling
GPIO.add_event_detect(SW_open, GPIO.FALLING, callback=switch_callback, bouncetime=50)
GPIO.add_event_detect(SW_close, GPIO.FALLING, callback=switch_callback, bouncetime=50)

#Trm Operation
def trm121():
    while True:
        global v_detect, v, vmax
        t = time.time()  # 현재 시간을 초단위로 가져옴
        while GPIO.input(Sigpin):  # 신호가 HIGH인 동안 대기
            pass
        while not GPIO.input(Sigpin):  # 신호가 LOW인 동안 대기
            pass
    
        T = (time.time() - t) * 1e6  # 주기 측정
        f = 1 / T  # 주파수 측정
        v = int((f * 1e6) / 44.0)  # 속도 측정
        vmax = max(v, vmax)  # 속도의 최대값 측정
    
        if v > 0:
            vmax = v  # v가 0보다 큰 경우에만 vmax 업데이트
    
        print("{:3d} km/h".format(vmax))  # 출력
    
        if vmax >= 15:
            GPIO.output(Buzzer, GPIO.HIGH)  # 부저 켜기
            v_detect = True
        else:
            GPIO.output(Buzzer, GPIO.LOW)  # 부저 끄기
            v_detect = False
    
        time.sleep(0.5)

#Classifier File
carCascade = cv2.CascadeClassifier("/home/pi/Desktop/vehicle-speed-detection-using-opencv-python-master/vehicle-speed-detection-using-opencv-python-master/file.xml")

#Video file capture
'''

video = cv2.VideoCapture(0)
video = cv2.VideoCapture("/home/pi/Desktop/cars.mp4")
'''
video = cv2.VideoCapture("/home/pi/Desktop/vehicle-speed-detection-using-opencv-python-master/vehicle-speed-detection-using-opencv-python-master/carsVideo.mp4")

# Constant Declaration
WIDTH = 640
HEIGHT = 480

# trackMultipleObjects 함수 내부에서 사용자 정의 함수로서 호출
def control_LED(w):
    global w_detect
    if w > 140:  # w 값이 200보다 큰 경우 (빨간색)
        GPIO.output(LED_RED, GPIO.HIGH)  # 빨간색 LED 켜기
        w_detect = True
    else:  # w 값이 200 이하인 경우 (초록색)
        GPIO.output(LED_GREEN, GPIO.HIGH)  # 초록색 LED 켜기
        w_detect = False

#estimate speed function
def estimateSpeed(location1, location2):
    d_pixels = math.sqrt(math.pow(location2[0] - location1[0], 2) + math.pow(location2[1] - location1[1], 2))
    ppm = 8.8
    d_meters = d_pixels / ppm
    fps = 18
    speed = d_meters * fps * 3.6
    return speed

#tracking multiple objects
def trackMultipleObjects():
    global Camera_Radar_on
    
    rectangleColor = (0, 255, 255)
    frameCounter = 0
    currentCarID = 0
    fps = 0

    carTracker = {}
    carNumbers = {}
    carLocation1 = {}
    carLocation2 = {}
    speed = [None] * 1000

    out = cv2.VideoWriter('outTraffic.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 10, (WIDTH, HEIGHT))

    while True:
        start_time = time.time()
        rc, image = video.read()
        if type(image) == type(None):
            break

        image = cv2.resize(image, (WIDTH, HEIGHT))
        resultImage = image.copy()

        frameCounter = frameCounter + 1
        carIDtoDelete = []
        
        #추적 품질 계산 및 낮은 품질 ID 검출
        for carID in carTracker.keys():
            trackingQuality = carTracker[carID].update(image)

            if trackingQuality < 7:
                carIDtoDelete.append(carID)

        #낮은 품질 ID 삭제
        for carID in carIDtoDelete:
            print("Removing carID " + str(carID) + ' from list of trackers. ')
            print("Removing carID " + str(carID) + ' previous location. ')
            print("Removing carID " + str(carID) + ' current location. ')
            carTracker.pop(carID, None)
            carLocation1.pop(carID, None)
            carLocation2.pop(carID, None)

        #10frame마다 실행
        if not (frameCounter % 10):
            #이미지를 그레이스케일로 변환
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            cars = carCascade.detectMultiScale(gray, 1.1, 13, 18, (24, 24))

            for (_x, _y, _w, _h) in cars:
                x = int(_x)
                y = int(_y)
                w = int(_w)
                h = int(_h)
                
                x_bar = x + 0.5 * w
                y_bar = y + 0.5 * h
                
                matchCarID = None
                
                for carID in carTracker.keys():
                    trackedPosition = carTracker[carID].get_position()
                    
                    t_x = int(trackedPosition.left())
                    t_y = int(trackedPosition.top())
                    t_w = int(trackedPosition.width())
                    t_h = int(trackedPosition.height())
                    
                    t_x_bar = t_x + 0.5 * t_w
                    t_y_bar = t_y + 0.5 * t_h
                    
                    if ((t_x <= x_bar <= (t_x + t_w)) and (t_y <= y_bar <= (t_y + t_h)) and (x <= t_x_bar <= (x + w)) and (y <= t_y_bar <= (y + h))):
                        matchCarID = carID

                if matchCarID is None:
                    print(' Creating new tracker' + str(currentCarID))

                    tracker = dlib.correlation_tracker()
                    tracker.start_track(image, dlib.rectangle(x, y, x + w, y + h))

                    carTracker[currentCarID] = tracker
                    carLocation1[currentCarID] = [x, y, w, h]

                    currentCarID = currentCarID + 1

        for carID in carTracker.keys():
            trackedPosition = carTracker[carID].get_position()

            t_x = int(trackedPosition.left())
            t_y = int(trackedPosition.top())
            t_w = int(trackedPosition.width())
            t_h = int(trackedPosition.height())

            cv2.rectangle(resultImage, (t_x, t_y), (t_x + t_w, t_y + t_h), rectangleColor, 4)
            cv2.putText(resultImage, "car", (int(t_x + t_w/2), int(t_y-15)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 100) ,2)
            

            carLocation2[carID] = [t_x, t_y, t_w, t_h]
            control_LED(w)
        if len(carTracker) == 0:
            GPIO.output(LED_RED, GPIO.LOW)   # 빨간색 LED 끄기
            GPIO.output(LED_GREEN, GPIO.LOW)  # 초록색 LED 끄기    

        end_time = time.time()

        if not (end_time == start_time):
            fps = 1.0/(end_time - start_time)

        for i in carLocation1.keys():
            if frameCounter % 1 == 0:
                [x1, y1, w1, h1] = carLocation1[i]
                [x2, y2, w2, h2] = carLocation2[i]

                carLocation1[i] = [x2, y2, w2, h2]

                if [x1, y1, w1, h1] != [x2, y2, w2, h2]:
                    if (speed[i] == None or speed[i] == 0) and y1 >= 275 and y1 <= 285:
                        speed[i] = estimateSpeed([x1, y1, w1, h1], [x1, y2, w2, h2])

                    if speed[i] != None and y1 >= 180:
                        
                        cv2.putText(resultImage, "       " + str(int(speed[i])) + "km/h" + str(int(w)), (int(x1 + w1/2), int(y1-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 100) ,2)
                        
        cv2.imshow('result', resultImage)

        out.write(resultImage)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        elif Camera_Radar_on == False:
            break
    
    cv2.destroyAllWindows()
    out.release()
    
def main():
    try:
        while True:
            global Camera_Radar_on
            if Camera_Radar_on == True:
                Streaming = threading.Thread(target = trackMultipleObjects)
                Streaming.start()
                print("Streaming start")
                Radaring = threading.Thread(target = trm121)
                Radaring.start()
                print("Radaring start")
            elif Camera_Radar_on ==False:
                trm121(False)
        
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except:
        KeyboardInterrupt
        
if __name__ == '__main__':
    main()
        
