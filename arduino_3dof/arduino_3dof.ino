#define DirectionPin  (10u)
#define BaudRate      (1000000ul)
#define ID1            1
#define ID2            2
#define ID3            3

#include <AX12A.h>

String RCVdata;
String CMDdataDEG[3];
int CMDdataVal[3] = { 2047, 2047, 2047 };
float deg2rad = M_PI / 180.0;
float rad2deg = 180.0 / M_PI;
float b1[3], b2[3], b3[3], p1[3], p2[3], p3[3];
float l1[3], l2[3], l3[3];
float L1_a, L2_a, L3_a;
float bRp0[3], bRp1[3], bRp2[3], T[3];
float z_set = 105;
float angle, theta, phi;
float phi0 = 30;
float d = 50.0;
float e = 70.0;
float z0 = 26.0; //모터 좌표
float k; //모터 좌표 반지름
int angle_step;


float dot_product(float v[], float u[]) {
  float result = 0.0;
  int i;
  for (i = 0; i < 3; i++) {
    result += v[i] * u[i];
  }
  return result;
}

void BRP(float theta, float phi) {
  bRp0[0] = cos(theta * deg2rad);
  bRp0[1] = sin(theta * deg2rad) * sin(phi * deg2rad);
  bRp0[2] = cos(phi * deg2rad) * sin(theta * deg2rad);
  bRp1[0] = 0;
  bRp1[1] = cos(phi * deg2rad);
  bRp1[2] = -sin(phi * deg2rad);
  bRp2[0] = -sin(theta * deg2rad);
  bRp2[1] = cos(theta * deg2rad) * sin(phi * deg2rad);
  bRp2[2] = cos(phi * deg2rad) * cos(theta * deg2rad);
}

void create_l_vectors() {
  // effectively adding the T vector initially
  BRP(theta, phi);
  T[0] = 0.0;
  T[1] = 0.0;
  T[2] = z_set;
  l1[0] = T[0] + (dot_product(bRp0, p1)) - b1[0];
  l1[1] = T[1] + (dot_product(bRp1, p1)) - b1[1];
  l1[2] = T[2] + (dot_product(bRp2, p1)) - b1[2];
  l2[0] = T[0] + (dot_product(bRp0, p2)) - b2[0];
  l2[1] = T[1] + (dot_product(bRp1, p2)) - b2[1];
  l2[2] = T[2] + (dot_product(bRp2, p2)) - b2[2];
  l3[0] = T[0] + (dot_product(bRp0, p3)) - b3[0];
  l3[1] = T[1] + (dot_product(bRp1, p3)) - b3[1];
  l3[2] = T[2] + (dot_product(bRp2, p3)) - b3[2];
}
//length == 높이
int step_transform(float length) {
  if (length > 100.0) {
    length = 100.0;  //최대길이
  }
  if (length < 50.0) {
    length = 50.0;  //최소길이
  }
  float intermed = ((length * length) + (d * d) - (e * e)) / (2 * d * length);
  if (intermed >= 1) {
    intermed = 0.99;
  }
  angle = -rad2deg * acos(intermed) + 90.0 + phi0;
  angle_step = map(angle, 0, 360, 0, 4095);
  if (angle_step < 0) {
    angle_step = 0;
  }
  return angle_step;
  //각각의 길이를 0~4095로 변환후 반환
}



void setup() {
  //초기 위치 설정
  theta = 0.0;
  phi = 0.0;
  k = 100.0 / sin(60 * deg2rad);
  b1[0] = k;                       // body origin to motor 1 shaft center X position
  b1[1] = 0.0;                     // body origin to motor 1 shaft center Y position
  b1[2] = z0;                      // body origin to motor 1 shaft center Z position
  b2[0] = -k * sin(30 * deg2rad);  // same for motor 2
  b2[1] = k * cos(30 * deg2rad);
  b2[2] = z0;
  b3[0] = -k * sin(30 * deg2rad);  // same for motor 3
  b3[1] = -k * cos(30 * deg2rad);
  b3[2] = z0;
  p1[0] = k;                       // platform origin to effector pivot vector X position
  p1[1] = 0.0;                     // platform origin to effector pivot vector Y position
  p1[2] = 0.0;                     // platform origin to effector pivot vector Z position
  p2[0] = -k * sin(30 * deg2rad);  // same for second effector pivot point
  p2[1] = k * cos(30 * deg2rad);
  p2[2] = 0.0;
  p3[0] = -k * sin(30 * deg2rad);  // same for third effector pivot point
  p3[1] = -k * cos(30 * deg2rad);
  p3[2] = 0.0;

  // put your setup code here, to run once:
  Serial.begin(115200);  // 통신 속도
  ax12a.begin(BaudRate, DirectionPin, &Serial);
  delay(1000);
}

void loop() {

  // Serial.read()
  if (Serial.available() <= 0) {
    return;
  }
  char ch = Serial.read();
  RCVdata += ch;
  if (ch == '\n') {

    if (RCVdata.length() > 0) {
      int index;
      int tmpcnt = 0;
      String tmpString = RCVdata;
      tmpString.trim();
      Serial.print("command in rx, ry, z, rz  ");
      Serial.println(tmpString);

      while (tmpString.length() > 0) {
        index = tmpString.indexOf(",");
        if (index == -1) {
          CMDdataDEG[tmpcnt] = tmpString;
          CMDdataDEG[tmpcnt].trim();
          tmpcnt++;
          break;
        }

        CMDdataDEG[tmpcnt] = tmpString.substring(0, index);
        tmpString = tmpString.substring(index + 1);
        tmpString.trim();
        CMDdataDEG[tmpcnt].trim();
        tmpcnt++;
      }
    }

    // 파이썬으로 부터 데이터 받음.
    theta = CMDdataDEG[0].toInt(); //roll
    phi = CMDdataDEG[1].toInt(); //pitch
    z_set = CMDdataDEG[2].toInt(); //z-axis
    // 역기구학 계산
    create_l_vectors();                                                                // create the end-effector vectors
    L1_a = step_transform(sqrt((l1[0] * l1[0]) + (l1[1] * l1[1]) + (l1[2] * l1[2])));  // norm and
    L2_a = step_transform(sqrt((l2[0] * l2[0]) + (l2[1] * l2[1]) + (l2[2] * l2[2])));  // convert to steps
    L3_a = step_transform(sqrt((l3[0] * l3[0]) + (l3[1] * l3[1]) + (l3[2] * l3[2])));



    RCVdata = "";


    ax12a.move(ID1, L1_a);
    ax12a.move(ID1, L2_a);
    ax12a.move(ID1, L3_a);
  }
}
