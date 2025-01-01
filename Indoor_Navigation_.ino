#include <Wire.h>

// bu kod araba üçgen gitmesi için bir kod (servo ve lidar yok) çalışan bir kod
#define RADIUS 32.0  // Encoder'in dönen tekerlek yarıçapı (mm)
#define PPR 20       // Encoder'in her turdaki pulse sayısı

// MPU6050 ve QMC5883L için I2C adresleri
#define MPU6050_ADDR 0x68
#define QMC5883L_ADDR 0x0D

enum NavigationState {
  INITIAL_ROTATION,
  FIRST_STRAIGHT,
  SECOND_ROTATION,
  SECOND_STRAIGHT,
  THIRD_ROTATION,
  THIRD_STRAIGHT,
  COMPLETED
};

NavigationState currentState = INITIAL_ROTATION;
const float initialTargetYaw = 60;  // İlk rotasyon hedefi
const float secondTargetYaw = 180;  // İkinci rotasyon hedefi
const float thirdTargetYaw = 300;   // Üçüncü rotasyon hedefi
const float targetDistance = 800;   // mm cinsinden gidilecek ara mesafe
const float yawThreshold = 4;       // derece cinsinden sapma değeri



volatile unsigned long rightPulseCount = 0;
volatile unsigned long leftPulseCount = 0;
unsigned long previousRightPulseCount = 0;
unsigned long previousLeftPulseCount = 0;
float encoderDistance = 0;

const int rightEncoderPin = 18;  // Sağ interrupt encoder pini
const int leftEncoderPin = 19;   // Sol interrupt encoder pini

const int rightMotorPin1 = 33;
const int rightMotorPin2 = 32;
const int rightMotorPWM = 45;
const int leftMotorPin1 = 36;
const int leftMotorPin2 = 37;
const int leftMotorPWM = 44;



float yaw;
float targetYaw;



// Filtrelenmiş değer için değişkenler
float filtredMagX, filtredMagY, filtredMagZ;

uint32_t previousMillis;

// PD Controller
float pdError = 0.0, prevPdError = 0.0, prevPrevPdError = 0.0, integral = 0.0, derivative = 0.0;
const float Kp = 5.5, Kd = 1.0;
unsigned long pdTimeNow = 0, pdTimePrev = 0;
float deltaT = 0.0;
int outputPD = 0;


// Medyan filtresi için Buffer(geçici depo)
float magXBuffer[5] = { 0, 0, 0, 0, 0 };
float magYBuffer[5] = { 0, 0, 0, 0, 0 };
float magZBuffer[5] = { 0, 0, 0, 0, 0 };

float accelXBuffer[5] = { 0, 0, 0, 0, 0 };
float accelYBuffer[5] = { 0, 0, 0, 0, 0 };
float accelZBuffer[5] = { 0, 0, 0, 0, 0 };


void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);  // I2C hızını ayarla
  Wire.begin();
  delay(250);

  initMPU6050();  // MPU6050 Sensörünü Başlat (ivmeölçer)

  initQMC5883L();  // QMC5883L Sensörünü Başlat (manyetometre)

  initEncoder(); // Enkoder Pinleri Tanımlama ve interrupt'larını ayarlama

  initMotorPins();  // Motor Pinleri Tanımlama

  //previousMillis = millis();
}

void loop() {

  int16_t rawAccelX, rawAccelY, rawAccelZ;                     // Ham ivmeölçer verileri
  float calibratedAccelX, calibratedAccelY, calibratedAccelZ;  // Kalibre edilmiş ivmeölçer verileri

  int16_t rawMagX, rawMagY, rawMagZ;                     // Ham manyetometre verileri
  float calibratedMagX, calibratedMagY, calibratedMagZ;  // Kalibre edilmiş manyetometre verileri

  // MPU6050 verilerini oku
  readMPU6050(&rawAccelX, &rawAccelY, &rawAccelZ);
  // Kalibre edilmiş değerleri hesapla
  getCalibratedAccel(rawAccelX, rawAccelY, rawAccelZ,
                     &calibratedAccelX, &calibratedAccelY, &calibratedAccelZ);

  // QMC5883L manyetometre verilerini oku
  readQMC5883L(&rawMagX, &rawMagY, &rawMagZ);
  // Kalibre edilmiş değerleri hesapla
  getCalibratedMag(rawMagX, rawMagY, rawMagZ,
                   &calibratedMagX, &calibratedMagY, &calibratedMagZ);

  // Medyan filtresini uygula
  float medianAccelX = applyMedianFilter(calibratedAccelX, accelXBuffer);
  float medianAccelY = applyMedianFilter(calibratedAccelY, accelYBuffer);
  float medianAccelZ = applyMedianFilter(calibratedAccelZ, accelZBuffer);

  float medianMagX = applyMedianFilter(calibratedMagX, magXBuffer);
  float medianMagY = applyMedianFilter(calibratedMagY, magYBuffer);
  float medianMagZ = applyMedianFilter(calibratedMagZ, magZBuffer);

  filtredMagX = applyLowPassFilter(filtredMagX, medianMagX, 0.20);
  filtredMagY = applyLowPassFilter(filtredMagY, medianMagY, 0.20);
  filtredMagZ = applyLowPassFilter(filtredMagZ, medianMagZ, 0.20);

  // Yaw/heading/ baş açısını hesapla
  calculateYaw(medianAccelX, medianAccelY, medianAccelZ, filtredMagX, filtredMagY, filtredMagZ);

  // PD kontrolör
  pdTimeNow = micros();
  deltaT = (float)(pdTimeNow - pdTimePrev) / 1.0e6;

  pdError = targetYaw - yaw;                                                               // Oransal terimi                                                             // Integral term
  derivative = (float)(3 * pdError - 4 * prevPdError + prevPrevPdError) / (2.0 * deltaT);  // Türev terimi
  outputPD = (int)(Kp * pdError + Kd * derivative);

  //enkoderden mesafe hesaapla
  int16_t deltaRightPulseCount = rightPulseCount - previousRightPulseCount;
  previousRightPulseCount = rightPulseCount;
  int16_t deltaLeftPulseCount = leftPulseCount - previousLeftPulseCount;
  previousLeftPulseCount = leftPulseCount;

  float deltaRightDistance = (float(deltaRightPulseCount) / PPR) * (2 * 3.14159265 * RADIUS);
  float deltaLeftDistance = (float(deltaLeftPulseCount) / PPR) * (2 * 3.14159265 * RADIUS);
  float deltaDistance = (deltaRightDistance + deltaLeftDistance) / 2.0;
  encoderDistance = encoderDistance + deltaDistance;  // mm cinsinden

  //Navigasyonu yönet
  handleNavigation();
  prevPrevPdError = prevPdError;
  prevPdError = pdError;
  pdTimePrev = pdTimeNow;


  // Bilgileri seri monitöre yazdır
  Serial.print(" Control Input: ");
  Serial.print(outputPD);

  Serial.print(" rightPulseCount: ");
  Serial.print(rightPulseCount);
  Serial.print(" leftPulseCount: ");
  Serial.print(leftPulseCount);

  Serial.print(" deltaRightPulseCount: ");
  Serial.print(deltaRightPulseCount);
  Serial.print(" deltaLeftPulseCount: ");
  Serial.print(deltaLeftPulseCount);

  Serial.print(" encoderDistance: ");
  Serial.print(encoderDistance);

  Serial.print(" yaw: ");
  Serial.print(yaw);
  Serial.println("°");



  //while (millis() - previousMillis <= 100)
  //  ;  //Nan statment
  //previousMillis = millis();
}

void initEncoder() {
  // Encoder pinleri giriş ve pullup olarak ayarla
  pinMode(rightEncoderPin, INPUT_PULLUP);
  pinMode(leftEncoderPin, INPUT_PULLUP);

  // Encoder interrupt'larını ayarla
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, RISING);
}

// Sağ encoder için interrupt servis rutini
void rightEncoderISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTime > 1500) {  // 1500 µs debounce süresi
    rightPulseCount++;
    lastInterruptTime = currentTime;
  }
}

// Sol encoder için interrupt servis rutini
void leftEncoderISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTime > 1500) {  // 1500 µs debounce süresi
    leftPulseCount++;
    lastInterruptTime = currentTime;
  }
}


void initMotorPins() {
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
}

void moveForward() {
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  analogWrite(rightMotorPWM, 255);

  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(leftMotorPWM, 255);
}


void stopMotors() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  analogWrite(rightMotorPWM, 0);

  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(leftMotorPWM, 0);
}

void moveRight() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  analogWrite(rightMotorPWM, outputPD);

  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(leftMotorPWM, outputPD);
}

void moveLeft() {
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  analogWrite(rightMotorPWM, outputPD);

  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(leftMotorPWM, outputPD);
}


// Kalibre edilmiş ivmeölçer değerlerini hesapla
void getCalibratedAccel(int16_t rawAccelX, int16_t rawAccelY, int16_t rawAccelZ,
                        float *calibratedAccelX, float *calibratedAccelY, float *calibratedAccelZ) {

  const float offsetAccelX = 121.50;
  const float offsetAccelY = -24.500;
  const float offsetAccelZ = -104.500;
  const float scaleFactorCorrectionAccelX = 0.997012;
  const float scaleFactorCorrectionAccelY = 0.997954;
  const float scaleFactorCorrectionAccelZ = 1.005073;

  // Kalibre edilmiş değerleri hesapla
  *calibratedAccelX = (rawAccelX - offsetAccelX) * scaleFactorCorrectionAccelX;
  *calibratedAccelY = (rawAccelY - offsetAccelY) * scaleFactorCorrectionAccelY;
  *calibratedAccelZ = (rawAccelZ - offsetAccelZ) * scaleFactorCorrectionAccelZ;
}



// Kalibre edilmiş Manyetometre değerlerini hesapla
void getCalibratedMag(int16_t rawMagX, int16_t rawMagY, int16_t rawMagZ, float *calibratedMagX, float *calibratedMagY, float *calibratedMagZ) {

  const float offsetMagX = 2244.50;
  const float offsetMagY = 2735.50;
  const float offsetMagZ = 2595.00;
  const float scaleFactorCorrectionMagX = 0.971009;
  const float scaleFactorCorrectionMagY = 0.992861;
  const float scaleFactorCorrectionMagZ = 1.038471;

  // Kalibre edilmiş değerleri hesapla
  *calibratedMagX = (rawMagX - offsetMagX) * (scaleFactorCorrectionMagX);
  *calibratedMagY = (rawMagY - offsetMagY) * (scaleFactorCorrectionMagY);
  *calibratedMagZ = (rawMagZ - offsetMagZ) * (scaleFactorCorrectionMagZ);
}

// MPU6050 Sensörünü Başlat
void initMPU6050(void) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // Uyku modundan çık
  Wire.write(0x00);
  Wire.endTransmission();
}

// MPU6050'dan İvmeölçer verilerini oku
void readMPU6050(int16_t *accelX, int16_t *accelY, int16_t *accelZ) {
  Wire.beginTransmission(MPU6050_ADDR);  // bu dört satırı alçak geçiş filtresini aç
  Wire.write(0x1A);
  Wire.write(0x05);  //5 byte yaz
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);  //burada 8g'yi ayarlamak (çözünürlüğü ayarlamak)
  Wire.write(0x1C);                      //ivmeölçer çıkışlarını 1C adresinde(register) saklanır bu adresten gelen ölçüm aralığı AFS_SEL'e (JİROSKOP GÜRÜLTÜ PERFORMANSI) göre 2 yani 10 yani 16 bit aralığ o da bizim sonraki satır tanımı
  Wire.write(0x10);                      //0x10 adresi iki tabanlıda gösterimi 0001 0000 o da 16 bit eder //
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);  //ivmeölçer değerlerinden çekmesini başlamak
  Wire.write(0x3B);                      // ivmeölçer değerleri ilk saklanan değerin adresi AccX
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);  // bu fonkisyon 0x68 dresten iletilen verilerinden 6 baytı talep edeceğiz -- Normalde biz Master olarak biz veri göndeririz ama bu fonkisyonda Slave'i Master yaptık ve Master'ı Slave yaptık

  if (Wire.available() == 6) {
    *accelX = Wire.read() << 8 | Wire.read();  //Yüksek biti (MSB) 8 bit kaydırarak LSB bitinden veri çekmek
    *accelY = Wire.read() << 8 | Wire.read();
    *accelZ = Wire.read() << 8 | Wire.read();
  }
}
// QMC5883L Sensörünü Başlat
void initQMC5883L() {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x09);
  Wire.write(0x1D);  // Continuous mode, 200Hz output, 8G range, 512 OSR
  Wire.endTransmission();
}

// QMC5883L manyetometre verilerini oku
void readQMC5883L(int16_t *magX, int16_t *magY, int16_t *magZ) {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x00);  // Veri okuma başlangıç adresi
  Wire.endTransmission();
  Wire.requestFrom(0x0D, 6);

  if (Wire.available() == 6) {
    *magX = Wire.read() | (Wire.read() << 8);
    *magY = Wire.read() | (Wire.read() << 8);
    *magZ = Wire.read() | (Wire.read() << 8);
  }
}

// Medyan filtresi
float applyMedianFilter(float newData, float buffer[5]) {
  // 1. Yeni veriyi ekle (en eski veriyi güncelle)
  for (int i = 4; i > 0; i--) {
    buffer[i] = buffer[i - 1];
  }
  buffer[0] = newData;

  // 2. Dizi elemanlarını sıralamak için geçici bir kopya oluştur
  float sortedBuffer[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
  for (int i = 0; i < 5; i++) {
    sortedBuffer[i] = buffer[i];
  }

  // 3. Sıralama
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4 - i; j++) {
      if (sortedBuffer[j] > sortedBuffer[j + 1]) {
        float temp = sortedBuffer[j];
        sortedBuffer[j] = sortedBuffer[j + 1];
        sortedBuffer[j + 1] = temp;
      }
    }
  }

  if (isnan(sortedBuffer[2])) {
    Serial.println("NaN tespit edildi!");
    return 0.0;  // Hata durumunda varsayılan değer döndür
  } else {
    // 4. Ortanca (medyan) değeri döndür
    return sortedBuffer[2];
  }
}

// Low Pass filtresi uygulamak
float applyLowPassFilter(float currentValue, float newValue, float alpha) {
  return alpha * newValue + (1 - alpha) * currentValue;
}

// Tilt Compensation (eğem telafisi) ve Yaw hesaplama
void calculateYaw(float medianAccelX, float medianAccelY, float medianAccelZ, float filtredMagX, float filtredMagY, float filtredMagZ) {
  /// Pitch ve Roll açılarını hesapla (Radyan)
  float pitch = atan2(-medianAccelX, sqrt(medianAccelY * medianAccelY + medianAccelZ * medianAccelZ));  //radyan cinsinden
  float roll = atan2(medianAccelY, sqrt(medianAccelX * medianAccelX + medianAccelZ * medianAccelZ));

  //Tilt Combinsation (Eğim telafisi hesapla)
  float Xm = filtredMagX * cos(pitch) + filtredMagZ * sin(pitch);
  float Ym = filtredMagX * sin(roll) * sin(pitch) + filtredMagY * cos(roll) - filtredMagZ * sin(roll) * cos(pitch);

  // X ve Y eksenlerinden Yaw (heading/baş) açısını hesapla
  yaw = atan2(Ym, Xm) * 180.0 / 3.141592654;

  if (yaw < 0) yaw += 360;  // 0-360 derece aralığına taşı
}


void rotationCar() {
  if (outputPD > 0) {
    moveRight();
    if (outputPD > 255) {
      outputPD = 255;
    }
  } else {
    outputPD = -outputPD;
    moveLeft();
    if (outputPD > 255) {
      outputPD = 255;
    }
  }
}

void handleNavigation() {
  switch (currentState) {

    case INITIAL_ROTATION:
      targetYaw = initialTargetYaw;
      if (abs(targetYaw - yaw) <= yawThreshold) {
        currentState = FIRST_STRAIGHT;
        encoderDistance = 0;  // Düz çizgi hareketi için mesafeyi sıfırla
        moveForward();
      } else {
        rotationCar();
      }
      break;

    case FIRST_STRAIGHT:
      if (encoderDistance >= targetDistance) {
        currentState = SECOND_ROTATION;
        targetYaw = secondTargetYaw;
        stopMotors();
      } else {
        moveForward();
      }
      break;

    case SECOND_ROTATION:
      if (abs(targetYaw - yaw) <= yawThreshold) {
        currentState = SECOND_STRAIGHT;
        encoderDistance = 0;  // Düz çizgi hareketi için mesafeyi sıfırla
        moveForward();
      } else {
        rotationCar();
      }
      break;

    case SECOND_STRAIGHT:
      if (encoderDistance >= targetDistance) {
        currentState = THIRD_ROTATION;
        targetYaw = thirdTargetYaw;
        stopMotors();
      } else {
        moveForward();
      }
      break;

    case THIRD_ROTATION:
      if (abs(targetYaw - yaw) <= yawThreshold) {
        currentState = THIRD_STRAIGHT;
        encoderDistance = 0;  // Düz çizgi hareketi için mesafeyi sıfırla
        moveForward();
      } else {
        rotationCar();
      }
      break;

    case THIRD_STRAIGHT:
      if (encoderDistance >= targetDistance) {
        currentState = COMPLETED;
        stopMotors();
      } else {
        moveForward();
      }
      break;

    case COMPLETED:
      stopMotors();
      break;
  }
}

