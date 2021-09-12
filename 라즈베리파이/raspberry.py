Python 3.7.8 (tags/v3.7.8:4b47a5b6ba, Jun 28 2020, 08:53:46) [MSC v.1916 64 bit (AMD64)] on win32
Type "help", "copyright", "credits" or "license()" for more information.
>>> 
import os
from threading import Thread
from keras.models import load_model
import numpy as np
import cv2
import Adafruit_PCA9685 
import wiringpi
import time
import sys
import socket
 
HOST =""
PORT = 8888
send = ''
stop_msg = 0
 
def funOne():
    frameWidth= 640         # 카메라 해상도
    frameHeight = 480
    brightness = 0
    threshold = 1     # 확률 임계값
    font = cv2.FONT_HERSHEY_SIMPLEX
 
    cap = cv2.VideoCapture(0) #비디오 캡쳐 시작
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)
    cap.set(10, brightness)
 
    model_save_path = '/home/pi/Desktop/test/model_trained_data2.h5'
    model = load_model(model_save_path)
 
    def grayscale(img):
        img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        return img

    def equalize(img):
        img =cv2.equalizeHist(img)
        return img

    def preprocessing(img):
        img = grayscale(img)
        img = equalize(img)
        img = img/255
        return img

    def getCalssName(classNo):
        if   classNo == 0: 
            return 'No Right Turn'
        elif classNo == 1: 
            return 'Speed Bump'
        elif classNo == 2: 
            return 'U-Turn'
 
    while 1:
        success, imgOrignal = cap.read()
        global send
        
    # PROCESS IMAGE
        img = np.asarray(imgOrignal)
        img = cv2.resize(img, (32, 32))
        img = preprocessing(img)
        #cv2.imshow("Processed Image", img)
        img = img.reshape(1, 32, 32, 1)
        cv2.putText(imgOrignal, "CLASS: " , (20, 35), font, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(imgOrignal, "PROBABILITY: ", (20, 75), font, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
        # PREDICT IMAGE
        predictions = model.predict(img)
        classIndex = model.predict_classes(img)
        probabilityValue =np.amax(predictions)
        if probabilityValue >= threshold:
        #print(getCalssName(classIndex))
            cv2.putText(imgOrignal,str(classIndex)+" "+str(getCalssName(classIndex)), (120, 35), font, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(imgOrignal, str(round(probabilityValue*100,2) )+"%", (180, 75), font, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
            send = str(getCalssName(classIndex)) 
        cv2.imshow("Result", imgOrignal)
        key = cv2.waitKey(1)
        if key == 32:
            break
 
def funTwo():
    global send
    # 모터 상태
    STOP  = 0
    FORWARD  = 1
    BACKWORD = 2

    # 모터 채널
    CH1 = 0
    CH2 = 1
    
    # PIN 입출력 설정
    OUTPUT = 1
    INPUT = 0

    # PIN 설정
    HIGH = 1
    LOW = 0
    
    # 실제 핀 정의
    #PWM PIN
    ENA = 25
    ENB = 30
    
    #GPIO PIN
    IN1 = 24
    IN2 = 23
    IN3 = 22
    IN4 = 21
    
    #PWM드라이버 사용을 위한 객체 생성
    robot_handle=Adafruit_PCA9685.PCA9685()
    servoMin=150 #서보모터 동작 최소 pulse값
    servoMax=550 #서보모터 동작 최대 pulse값

    # 핀 설정 함수
    def setPinConfig(EN, INA, INB):
        wiringpi.pinMode(EN, OUTPUT)
        wiringpi.pinMode(INA, OUTPUT)
        wiringpi.pinMode(INB, OUTPUT)
        wiringpi.softPwmCreate(EN, 0, 255)
        
        # 모터 제어 함수
    def setMotorContorl(PWM, INA, INB, speed, stat):
        #모터 속도 제어 PWM
        wiringpi.softPwmWrite(PWM, speed)
        #앞으로
        if stat == FORWARD:
            wiringpi.digitalWrite(INA, HIGH)
            wiringpi.digitalWrite(INB, LOW)
            #정지
        elif stat == STOP:
            wiringpi.digitalWrite(INA, LOW)
            wiringpi.digitalWrite(INB, LOW)
    # 모터 제어함수 간단하게 사용하기 위해 한번더 래핑(감쌈)
    def setMotor(ch, speed, stat):
        if ch == CH1:
            setMotorContorl(ENA, IN1, IN2, speed, stat)
        else:
            setMotorContorl(ENB, IN3, IN4, speed, stat)
            
    #서보모터 동작 각도와 pulse값을 맵핑시켜 주기위한 함수
    def map(value,min_angle, max_angle, min_pulse, max_pulse):
        angle_range=max_angle-min_angle
        pulse_range=max_pulse-min_pulse
        scale_factor=float(angle_range)/float(pulse_range)
        return min_pulse+(value/scale_factor)

    #서보모터 각도 이동을 위한 함수
    def set_angle(channel, angle):
        pulse=int(map(angle, 0, 180, servoMin, servoMax))
        robot_handle.set_pwm(channel, 0, pulse)
    #주파수를 50hz로 지정
    robot_handle.set_pwm_freq(50)
    wiringpi.wiringPiSetup()
    #모터 핀 설정
    setPinConfig(ENA, IN1, IN2)
    setPinConfig(ENB, IN3, IN4)
    a=96
    set_angle(3, a)
    while 1:
       #time.sleep(2)
       if(stop_msg == 2):
           setMotor(CH1, 130, FORWARD)
           setMotor(CH2, 130, FORWARD)
           if(send == 'Speed Bump'):
               setMotor(CH1, 80, FORWARD)
               setMotor(CH2, 80, FORWARD)
               time.sleep(2)
           elif(send == 'No Right Turn'):
                for i in range(23):
                    a -= 2
                    set_angle(3, a)
                    time.sleep(0.01)     
                time.sleep(1.2)
                for i in range(23):
                    a += 2
                    set_angle(3, a)
                    time.sleep(0.01)
                #time.sleep(2)
           elif(send == 'U-Turn'):
                for i in range(23):
                    a -= 2
                    set_angle(3, a)
                    time.sleep(0.01)
                time.sleep(2.7)
                for i in range(23):
                    a += 2
                    set_angle(3, a)
                    time.sleep(0.01)
                #time.sleep(5)
           send = ''
       elif(stop_msg == 1):
           setMotor(CH1, 50, STOP)
           setMotor(CH2, 50, STOP)
           send = ""
 
if __name__ == '__main__':
    proc = Thread(target=funOne, args=())
    proc2 = Thread(target=funTwo, args=())
    proc.start()
    proc2.start()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(1)
    
    while True:
        #접속승인
        conn, addr = s.accept()
        
        #데이터 수신
        data = conn.recv(1024)
        data = data.decode("utf8").strip()
        
        if not data:break
        print("Received: " + data)
        
        if(data == "Start"):
            stop_msg = 2
            data = ""
        elif(data == "Stop"):
            stop_msg = 1
            data = ""
            send = ""
        elif(data == "State"):
            sendmessage = send
            conn.sendall(sendmessage.encode())
        conn.close()
    s.close()