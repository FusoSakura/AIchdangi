#AI 차단기
(read Me FIle)

2025/05/20 프로잭트 진행도 약 20% 베이스 하드웨어 및 베이스 코드 제작완료.

V1.1 fixed codes

# 아두이노 코드

#include <Servo.h>

Servo myservo;

const int servoPin = 9;  // 서보 모터 핀 번호 (예시)
const int pin7 = 7;      // 디지털 7번 핀
const int pin8 = 8;      // 디지털 8번 핀

void setup() {
  Serial.begin(9600);
  while (!Serial); // 시리얼 연결 대기
  delay(2000);     // 초기화 대기

  myservo.attach(servoPin);
  pinMode(pin7, OUTPUT);
  pinMode(pin8, OUTPUT);

  myservo.write(160);        // 초기 서보 위치
  digitalWrite(pin7, LOW);
  digitalWrite(pin8, LOW);

  // 시리얼 버퍼 클리어
  while (Serial.available()) Serial.read();
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'C') {  // 차량 감지됨
      myservo.write(65);
      digitalWrite(pin7, HIGH);
      digitalWrite(pin8, LOW);
    }
    else if (command == 'N') {  // 차량 감지 안됨
      myservo.write(160);
      digitalWrite(pin7, LOW);
      digitalWrite(pin8, HIGH);
    }
  }
# 파이썬 코드드  

import cv2
import numpy as np
from ultralytics import YOLO
import serial
import time


arduino = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)  

det_model = YOLO("yolov8n.pt")

cam = cv2.VideoCapture(0)
if not cam.isOpened():
    print("카메라 열기 실패")
    exit()

prev_state = None 

while True:
    ret, frame = cam.read()
    if not ret:
        break

    scale = 1280 / max(frame.shape)
    if scale < 1:
        frame = cv2.resize(
            src=frame,
            dsize=None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_AREA
        )

    display_frame = frame.copy()

    frame_height, frame_width = frame.shape[:2]
    roi_height = int(frame_height * 0.3)
    roi_y = int((frame_height - roi_height) / 2)
    roi = (0, roi_y, frame_width, roi_height)

    cv2.rectangle(display_frame, (roi[0], roi[1]),
                  (roi[0] + roi[2], roi[1] + roi[3]), (255, 0, 0), 2)

    input_image = np.array(frame)
    results = det_model(input_image, verbose=False)
    boxes = results[0].boxes

    car_detected = False

    for box in boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        if cls_id == 2 and conf > 0.5:
            roi_x, roi_y, roi_w, roi_h = roi
            if (x1 < roi_x + roi_w and x2 > roi_x and
                y1 < roi_y + roi_h and y2 > roi_y):

                car_detected = True
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(display_frame, f"Car {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                print("감지됨")

    # 차량 감지 상태가 변경된 경우에만 아두이노에 신호 전송
    if car_detected != prev_state:
        if car_detected:
            arduino.write(b'C')  # 차량 감지 신호
            print("아두이노: 차량 감지 명령 전송")
        else:
            arduino.write(b'N')  # 차량 미감지 신호
            print("아두이노: 차량 미감지 명령 전송")
        prev_state = car_detected

    cv2.imshow('cam', display_frame)

    if cv2.waitKey(1) == ord('q'):
        break

arduino.close()
cam.release()
cv2.destroyAllWindows()



# 05/21 수정본
#아두이노코드

#include <Servo.h>

Servo myservo1;
Servo myservo2;

const int servoPin1 = 9;
const int servoPin2 = 10;

const int pin3 = 3;
const int pin7 = 7;
const int pin8 = 8;

unsigned long lastMoveTime = 0;
bool isAt180 = true;         // 현재 서보 위치 상태
bool carDetected = false;    // 차량 감지 상태
bool overrideByCar = false;  // 차량 감지에 의해 서보 제어가 발생했는지

void setup() {
  Serial.begin(9600);

  myservo1.attach(servoPin1);
  myservo2.attach(servoPin2);

  pinMode(pin3, OUTPUT);
  pinMode(pin7, OUTPUT);
  pinMode(pin8, OUTPUT);

  // 초기 서보 위치
  myservo1.write(180);
  myservo2.write(180);
  lastMoveTime = millis();

  digitalWrite(pin3, LOW);
  digitalWrite(pin7, LOW);
  digitalWrite(pin8, LOW);
}

void loop() {
  unsigned long currentTime = millis();

  // 시리얼 데이터 수신 처리
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'C') {
      carDetected = true;
    } else if (command == 'N') {
      carDetected = false;
    }
  }

  // 차량 감지 상태에 따라 우선 처리
  if (isAt180 && carDetected) {
    // 차량 감지 시 서보를 90도로 이동
    myservo1.write(90);
    myservo2.write(90);
    overrideByCar = true;

    digitalWrite(pin3, HIGH);
    digitalWrite(pin7, HIGH);
    digitalWrite(pin8, LOW);

    lastMoveTime = millis();  // 타이머 초기화
    isAt180 = false;
  }
  else if (overrideByCar && !carDetected) {
    // 차량이 더 이상 감지되지 않으면 복귀
    myservo1.write(180);
    myservo2.write(180);
    isAt180 = true;
    overrideByCar = false;

    digitalWrite(pin3, LOW);
    digitalWrite(pin7, LOW);
    digitalWrite(pin8, HIGH);

    lastMoveTime = millis();  // 타이머 초기화
  }
  else if (!overrideByCar) {
    // 일반 타이머 기반 제어 (차량 감지 우선 아님)
    if (isAt180 && currentTime - lastMoveTime >= 30000) {
      myservo1.write(90);
      myservo2.write(90);
      isAt180 = false;
      lastMoveTime = currentTime;
    }
    else if (!isAt180 && currentTime - lastMoveTime >= 10000) {
      myservo1.write(180);
      myservo2.write(180);
      isAt180 = true;
      lastMoveTime = currentTime;
    }

    // 90도 시점에서는 모든 핀 LOW
    if (!isAt180) {
      digitalWrite(pin3, LOW);
      digitalWrite(pin7, LOW);
      digitalWrite(pin8, LOW);
    }
  }
}



#파이썬코드(주피터랩)

import cv2
import numpy as np
from ultralytics import YOLO
import serial
import time

# 아두이노 시리얼 포트 설정 (필요 시 'COM3' 등으로 수정)
arduino = serial.Serial('COM5', 9600, timeout=1)
time.sleep(2)  # 아두이노와의 연결 대기

# YOLO 모델 로드
det_model = YOLO("yolov8n.pt")

# 외부 웹캠 열기 (1번 인덱스)
cam = cv2.VideoCapture(0)
if not cam.isOpened():
    print("카메라 열기 실패")
    exit()

# 차량 감지 상태 및 전송 시간 초기화
prev_state = None
last_send_time = time.time()

while True:
    ret, frame = cam.read()
    if not ret:
        break

    # 프레임 리사이즈 (너비 기준 1280 이하로 축소)
    scale = 1280 / max(frame.shape)
    if scale < 1:
        frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    display_frame = frame.copy()

    frame_height, frame_width = frame.shape[:2]
    roi_height = int(frame_height * 0.3)
    roi_y = int((frame_height - roi_height) / 2)
    roi = (0, roi_y, frame_width, roi_height)

    # ROI 시각화
    cv2.rectangle(display_frame, (roi[0], roi[1]),
                  (roi[0] + roi[2], roi[1] + roi[3]), (255, 0, 0), 2)

    # YOLO 객체 감지
    results = det_model(frame, verbose=False)
    boxes = results[0].boxes

    car_detected = False

    for box in boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        if cls_id == 2 and conf > 0.5:  # 클래스 ID 2 = 차량
            roi_x, roi_y, roi_w, roi_h = roi
            if x1 < roi_x + roi_w and x2 > roi_x and y1 < roi_y + roi_h and y2 > roi_y:
                car_detected = True
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(display_frame, f"Car {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                print("차량 감지됨")

    # 0.5초마다 아두이노로 차량 감지 여부 전송
    current_time = time.time()
    if current_time - last_send_time >= 0.5:
        if car_detected:
            arduino.write(b'C')
            print("아두이노: 차량 감지 명령 전송")
        else:
            arduino.write(b'N')
            print("아두이노: 차량 미감지 명령 전송")
        last_send_time = current_time

    # 결과 출력
    cv2.imshow('cam', display_frame)

    if cv2.waitKey(1) == ord('q'):
        break

# 종료 처리
arduino.close()
cam.release()
cv2.destroyAllWindows()




시리얼 통신 채택
}


