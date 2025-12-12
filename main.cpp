#include <Math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

#include <SoftwareSerial.h>
SoftwareSerial lineSerial(18, 19);    // RX, TX
SoftwareSerial OPENMV_Serial(50, 52);  // RX, TX

const int MOTOR_PIN[4][2] = { { 2, 3 }, { 4, 5 }, { 6, 7 }, { 8, 9 } };

const int IR_pins[16] = { 14, 48, 12, 11, 10, 29, 28, 27, 26, 25, 24, 23, 22, 17, 16, 15 };                                       //PIN
const float IR_data_angle[16] = { 0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5 };  //angle

float cosTab[361] = {};
float sinTab[361] = {};

float pi = 3.1415;
int max = 255,angularVelocity = 30;

void math() {
    for (int i=0;i<361;i++) {
        float r = i/180.0f * pi;
        cosTab[i] = cos(r);
        sinTab[i] = sin(r);
    }
}

void motor(int arr[]) {
    for (int i=0;i<4;i++) {
        if (arr[i] > 0) {
            analogWrite(MOTOR_PIN[i][0],arr[i]);
            analogWrite(MOTOR_PIN[i][1],LOW);
        }
        else {
            analogWrite(MOTOR_PIN[i][1],-1*arr[i]);
            analogWrite(MOTOR_PIN[i][0],LOW);
        }
    }
}

void motorSpeed(int moveAngle,int omega) {
    int convertAngle = -moveAngle + 45;
    if (convertAngle < 0) convertAngle += 360;
    float speedA = sinTab[convertAngle];
    float speedB = cosTab[convertAngle];
    float speedC = -sinTab[convertAngle];
    float speedD = -cosTab[convertAngle];
    int newMax = max;
    float ratio;
    if (omega == 30) newMax -= 30;
    else if (omega == -30) newMax += 30;
    if (fabs(speedA) > fabs(speedB)) ratio = newMax / fabs(speedA);
    else ratio = newMax / fabs(speedB);
    speedA *= ratio;
    speedB *= ratio;
    speedC *= ratio;
    speedD *= ratio;
    speedA += omega;
    speedB += omega;
    speedC += omega;
    speedD += omega;
    int speeds[4] = {int(speedA),int(speedB),int(speedC),int(speedD)};
    motor(speeds);
}

float getGoalAngle(int newFront) {
    String str2 = OPENMV_Serial.readStringUntil('\n');

    str2.trim();

    if (str2.length() > 0) {
        float x_value = str2.toInt();
        x_value *= (70.0f / 56.0f);
        float absAngle = newFront + x_value;
        if (absAngle < 0) absAngle += 360;
        if (absAngle >= 360) absAngle -= 360;
        return absAngle;
    }
    else return IRAngle();
}

// 角度の平均を返す関数
int midAngle(int arr[] ,int size) {
    float sumX = 0,sumY = 0;
    for (int i=0;i<size;i++) {
        sumX += cosTab[int(arr[i])];
        sumY += sinTab[int(arr[i])];
    }
    float r = atan2(sumY,sumX);
    float angle = r * 180.0f / pi;
    if (angle < 0) angle += 360;
    return int(angle);
}

int angles[100] = {};
int index = 0;

int IRAngle() {
    int angleList[16] = {};
    int count = 0;
    for (int i=0;i<16;i++) {
        if (digitalRead(IR_pins[i]) == LOW) {
            angleList[count] = IR_data_angle[i];
            count++;
        }
    }
    if (count > 0) {
        angles[index] = midAngle(angleList, count);
        index = (index + 1) % 100;
    }
    int angle = midAngle(angles, 100);
    return angle;
}

float front;
void setup() {
    lineSerial.begin(115200);
    lineSerial.setTimeout(20);
    OPENMV_Serial.begin(115200);

    if (!bno.begin()) {
        while (1);
    }

    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);

    pinMode(40, INPUT_PULLUP);

    math();
    while (digitalRead(40) == HIGH) {
        IRAngle();
        front = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();
    }
}

void loop() {
    String str;
    int moveAngle = IRAngle();
    float newFront = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();
    int omega = 0; // 半時計周りの角速度
    float diff = newFront - front;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    if (diff >= 15) {
        omega = angularVelocity;
    }
    else if (diff <= -15) {
        omega = -angularVelocity;
    }
    if (lineSerial.available()) {
        str = lineSerial.readStringUntil('\n');
    }
    str.trim();
    if (str.length() != 0 && str != "ERROR") {
        // ラインセンサー反応あり
        int lineAngle = str.toInt();
        moveAngle = lineAngle + 180;
    }
    else if (moveAngle <= 30 || 330 <= moveAngle) {
        while (!(120 <= moveAngle) && !(moveAngle <= 240)) {
            moveAngle = int(getGoalAngle(newFront));
        }
    }
    else if (30 < moveAngle && moveAngle <= 180) moveAngle += 30;
    else if (180 < moveAngle && moveAngle < 330) moveAngle -= 30;
    motorSpeed(moveAngle,omega);
}
