#include <Wire.h>
#include <MPU6050.h>

const int trigPin = 5;     // ขา Trigger ของเซ็นเซอร์ HC-SR04
const int echoPin = 4;     // ขา Echo ของเซ็นเซอร์ HC-SR04

MPU6050 mpu;                // อ็อบเจ็กต์ MPU6050        

int VM1 = A1;  

unsigned long previousMillis = 0;   // เก็บเวลาครั้งล่าสุดที่พิมพ์ข้อความ
const long interval = 1000;         // ช่วงเวลาระหว่างข้อความในหน่วยมิลลิวินาที

// Buzzer setting
const int buzzerPin = 9; // กำหนดขา buzzer

int melody[] = {
  1000, 500, // โน้ต 1000 Hz, 500 ms
  1500, 500, // โน้ต 1500 Hz, 500 ms
  1000, 500, // โน้ต 1000 Hz, 500 ms
  1500, 500, // โน้ต 1500 Hz, 500 ms
  1000, 500, // โน้ต 1000 Hz, 500 ms
  1500, 500  // โน้ต 1500 Hz, 500 ms
};

int noteDuration = 10000; // ระยะเวลาในการเล่นโน้ต
int pauseDuration = 10;    // ระยะเวลาในการหยุดระหว่างโน้ต (200 ms)
// Buzzer setting end

void setup() {
  pinMode(VM1, OUTPUT);
  Serial.begin(9600);      // เริ่มการสื่อสารผ่านซีเรียลที่ 9600 bps
  Wire.begin();            // เริ่มการสื่อสาร I2C

  // กำหนดค่าเริ่มต้น MPU6050
  mpu.initialize();        // กำหนดค่าเริ่มต้นเซ็นเซอร์ MPU6050

  // ตรวจสอบว่าเซนเซอร์ทำงานปกติหรือไม่
  if(mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully");
  } else {
    Serial.println("MPU6050 connection failed");
    while(1); // หยุดการทำงานถ้าการเชื่อมต่อกับ MPU6050 ล้มเหลว
  }

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000); // ตั้งช่วงยิ่งเซนเซอร์ที่ 2000 องศาต่อวินาที

  pinMode(trigPin, OUTPUT);    // ตั้งโหมดขา Trigger เป็น Output
  pinMode(echoPin, INPUT);     // ตั้งโหมดขา Echo เป็น Input
  
  pinMode(buzzerPin, OUTPUT);  // กำหนดการส่งเสียงของ buzzer

  // สั่งให้มอเตอร์สั่น 2 ครั้งเมื่อเปิดเครื่อง
  vibrateMotorsTwice();
}

void loop() {
  long duration, cm;

  // วัดระยะทางโดยใช้เซ็นเซอร์ HC-SR04
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  cm = microsecondsToCentimeters(duration);

  // อ่านข้อมูลเซ็นเซอร์ MPU6050
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // คำนวณความเร่งแบบเวกเตอร์
  long acceleration = sqrt(sq(ax) + sq(ay) + sq(az));
  long gyro = sqrt(sq(gx) + sq(gy) + sq(gz));

  
  // คำนวณมุม pitch และ roll จากการเร่ง
  float pitch = atan2(ay, az) * 180 / PI;
  float roll = atan2(ax, az) * 180 / PI;

  Serial.print("ระยะทางจาก HC-SR04: ");
    Serial.print(cm);
    Serial.println(" เซนติเมตร");

  // แสดงค่า pitch และ roll ที่ Serial Monitor
  Serial.print("Pitch: "); Serial.print(pitch); Serial.print(" °\t");
  Serial.print("Roll: "); Serial.println(roll);

  // ตรวจสอบสถานะล้ม
  if (roll < 5) {
    Serial.println("Detecting a fall");
    playMelody();  // เรียกฟังก์ชันเล่นเมโลดี้เมื่อพบการล้ม
  }

  // ควบคุม Vibrator ตามระยะทาง
  if (cm <= 20) {
    Serial.println("high motors");
    digitalWrite(VM1, HIGH);
  } else if (cm <= 50) {
    Serial.println("medium motor");
    digitalWrite(VM1, HIGH);
    digitalWrite(VM1, LOW);
  } else {
    Serial.println("close 2 motors");
    digitalWrite(VM1, LOW);
  }

  // ความล่าช้าเพื่อควบคุมอัตราการประมวลผลของลูป
  delay(1000);
}

void vibrateMotorsTwice() {
  // สั่งให้มอเตอร์สั่น 2 ครั้งเมื่อเรียกใช้งาน
  for (int i = 0; i < 2; i++) {
    digitalWrite(VM1, HIGH);
    delay(500); // สั่น 500 มิลลิวินาที
    digitalWrite(VM1, LOW);
    delay(500); // หยุดสั่น 500 มิลลิวินาที
  }
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void playMelody() {
  unsigned long startMillis = millis();
  while (millis() - startMillis < 10000) {  // 10000 milliseconds = 10 seconds
    for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i += 2) {
      int frequency = melody[i];
      int duration = melody[i + 1];

      tone(buzzerPin, frequency, duration);
      delay(duration + pauseDuration);  // รอให้เสียงเล่นจบและหยุดระหว่างโน้ต
    }
  }
}
