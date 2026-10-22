#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include <Servo.h>

const int BMP_CS_PIN = 25;
const int MPU_CS_PIN = 4;

// Register Adress
const uint8_t MPU_WHO_AM_I = 0x75;
const uint8_t MPU_PWR_MGMT_1 = 0x6b;
const uint8_t MPU_DATA_START = 0x3b;

const uint8_t BMP_ID_REG = 0xD0;
const uint8_t BMP_CTRL_MEAS = 0xF4;
const uint8_t BMP_DATA_START = 0xf7;

int32_t accelOffsetX=0;
int32_t accelOffsetY=0;
int32_t accelOffsetZ=0;

int32_t gyroOffsetX=0;
int32_t gyroOffsetY=0;
int32_t gyroOffsetZ=0;

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

double bmp280_compensate_T(int32_t adc_t);
double bmp280_compensate_P(int32_t adc_p);

double filteredPitch = 0.0;
double filteredYaw = 0.0;
const double alpha = 0.92; // Filter constant

double pitch_offset = 0.0;
double yawCalibrationOffset = 0.0;
double gyyroX_bias = 0.0;
double gyroY_bias = 0.0;
unsigned long lastUpdateTime = 0;

// Dual axis controller (tune later)
double Kp = 1.7; // Proportional gain
double Kd = 0.15; // Derivative gain

//Servo limits
const double MAX_gimAngle = 15.0; // Maximum angle in degrees
const int servoPitch_CenterPWM = 1460; 
const int servoYaw_CenterPWM = 1520;
const double degToPWM = 11.1; // Conversion factor from degrees to PWM units


IntervalTimer flightTimer;
Servo Pitch;
Servo Yaw; 

const int Pitch_SERVO_PIN = 33;
const int Yaw_SERVO_PIN = 29;

const int red_ledPin = 3;
const int green_ledPin = 1;

const int PARACHUTE_PIN = 37;


//Angle history
double lastfiltered_Pitch = 0.0;
double lastfiltered_Yaw = 0.0;



// Heads up
void writeRegister(int csPin, uint8_t reg, uint8_t value);
uint8_t readRegister(int csPin, uint8_t reg);
void readBurst(int csPin, uint8_t startReg, uint8_t*buffer, uint8_t length);



void calibrateMPU() {
  int32_t sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 500;
  uint8_t calBuf[14];

  Serial.println("Calibrating Gyroscope...");
  delay(1000); // Wait for sensor to stabilize

  for (int i = 0; i < samples; i++) {

    readBurst(MPU_CS_PIN, MPU_DATA_START, calBuf, 14);

    sumX += (int32_t)((calBuf[8] << 8) | calBuf[9]);
    sumY += (int32_t)((calBuf[10] << 8) | calBuf[11]);
    sumZ += (int32_t)((calBuf[12] << 8) | calBuf[13]);
    
    delay(3);

  }

  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;

  Serial.printf("Calibrating Accelerometer...");
  delay(1000); // Wait for sensor to stabilize

  long ax_sum = 0, ay_sum = 0, az_sum = 0; 
  const int samples_2 = 2000;

  for (int i = 0; i < samples_2; i++) {

    readBurst(MPU_CS_PIN, MPU_DATA_START, calBuf, 14);

    ax_sum += (int32_t)((calBuf[0] << 8) | calBuf[1]);
    ay_sum += (int32_t)((calBuf[2] << 8) | calBuf[3]);
    az_sum += (int32_t)((calBuf[4] << 8) | calBuf[5]);
    delay(3);

    accelOffsetX = (ax_sum / samples_2);// Subtract 1g (16384) to account for gravity
    accelOffsetY = (ay_sum / samples_2)  - 16384;
    accelOffsetZ = (az_sum / samples_2); 

    Serial.printf("Calibrating Accelerometer Complete. Offsets: [%d, %d, %d]\n", accelOffsetX, accelOffsetY, accelOffsetZ);

    delay(10);

    double CalX_g = double(accelOffsetX + 16384) / 16384.0; // Convert to gravities (g
    double CalY_g = double(accelOffsetY) / 16384.0; // Convert to gravities (g
    
    double CalZ_g = double(accelOffsetZ) / 16384.0; // Convert to gravities (g)

    pitch_offset = atan2(CalX_g, sqrt(CalY_g * CalY_g + CalZ_g * CalZ_g)) * (180.0 / M_PI);
    yawCalibrationOffset = atan2(CalY_g, sqrt(CalX_g * CalX_g + CalZ_g * CalZ_g)) * (180.0 / M_PI);

    Serial.printf("Pitch Offset: %.2f | Yaw Offset: %.2f\n", pitch_offset, yawCalibrationOffset);
    Serial.printf("Gyroscope Offsets: [%d, %d, %d]\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  }

  Serial.printf("Gyroscope Calibration Complete. Offsets: [%d, %d, %d]\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
}

void BMP_Calibrate() {
  uint8_t caliBuf[24];
  readBurst(BMP_CS_PIN, 0x88, caliBuf, 24);

  dig_T1 = (uint16_t)((caliBuf[1] << 8) | caliBuf[0]);
  dig_T2 = (int16_t)((caliBuf[3] << 8) | caliBuf[2]);
  dig_T3 = (int16_t)((caliBuf[5] << 8) | caliBuf[4]);
  dig_P1 = (uint16_t)((caliBuf[7] << 8) | caliBuf[6]);
  dig_P2 = (int16_t)((caliBuf[9] << 8) | caliBuf[8]);
  dig_P3 = (int16_t)((caliBuf[11] << 8) | caliBuf[10]);
  dig_P4 = (int16_t)((caliBuf[13] << 8) | caliBuf[12]);
  dig_P5 = (int16_t)((caliBuf[15] << 8) | caliBuf[14]);
  dig_P6 = (int16_t)((caliBuf[17] << 8) | caliBuf[16]);
  dig_P7 = (int16_t)((caliBuf[19] << 8) | caliBuf[18]);
  dig_P8 = (int16_t)((caliBuf[21] << 8) | caliBuf[20]);
  dig_P9 = (int16_t)((caliBuf[23] << 8) | caliBuf[22]); 

  Serial.printf("-----BMP280 Calibration Registers-----:\n");
  Serial.printf("dig_T1: %d\n", dig_T1);
  Serial.printf("dig_T2: %d\n", dig_T2);
  Serial.printf("dig_T3: %d\n", dig_T3);

    Serial.printf("BMP280 Calobrated Successfully.");
}

int32_t t_fine;
  //DegC, Dpressision
  double bmp280_compensate_T(int32_t adc_T) {
    double var1 = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
    double var2 = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) * (((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
    t_fine = (int32_t)(var1 + var2);
    return (var1 + var2) / 5120.0;
  }

  //(Pa)
  double bmp280_compensate_P(int32_t adc_p) {
    double var1 = ((double)t_fine / 2.0) - 64000.0;
    double var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
    var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
    if (var1 == 0.0) {
      return 0; // avoid exception caused by division by zero
    }
    double p = 1048576.0 - (double)adc_p;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)dig_P8) / 32768.0;
    p = p + (var1 + var2 + (double)dig_P7) / 16.0;
    return p;
  }

void setup() {

    Serial.begin(115200);
  delay(2000);
  while (!Serial && millis() < 4000);

  pinMode(red_ledPin, OUTPUT);
  pinMode(green_ledPin, OUTPUT);

  digitalWrite(red_ledPin, HIGH);

  analogWriteResolution(12);

  analogWriteFrequency(Pitch_SERVO_PIN, 333);
  analogWriteFrequency(Yaw_SERVO_PIN, 333); 

  pinMode(Pitch_SERVO_PIN, OUTPUT);
  pinMode(Yaw_SERVO_PIN, OUTPUT);

  Pitch.attach(Pitch_SERVO_PIN);
  Yaw.attach(Yaw_SERVO_PIN);

  Pitch.write(1460);
  delay(2000);
  Yaw.write(1500);
  delay(2000);


  Serial.println("===(TESTING)SPI SENSOR RAW DATA READER===");

  // CS pins
  pinMode(BMP_CS_PIN, OUTPUT);
  pinMode(MPU_CS_PIN, OUTPUT);
  pinMode(PARACHUTE_PIN, OUTPUT);
  digitalWrite(BMP_CS_PIN, HIGH);
  digitalWrite(MPU_CS_PIN, HIGH);
  digitalWrite(PARACHUTE_PIN, LOW);

  SPI.begin();
  delay(100);

  //MPU9265 Startup
  uint8_t mpuID = readRegister(MPU_CS_PIN, MPU_WHO_AM_I);
  Serial.printf("MPU9250 ID: 0x%02X\n", mpuID);
  writeRegister(MPU_CS_PIN, 0x6b, 0x80); // Reset device
  delay(100);

  writeRegister(MPU_CS_PIN, 0x6b, 0x01); // Clock source
  delay(100);
  writeRegister(MPU_CS_PIN, 0x1b, 0x00); 
  delay(100);
  writeRegister(MPU_CS_PIN, 0x1a, 0x03); // DLPF_CFG = 3 (41Hz)

  writeRegister(BMP_CS_PIN, 0xF4, 0x27); // Set BMP280 to normal mode, oversampling x1 for both temperature and pressure

  delay(100);
  
  calibrateMPU();

  filteredPitch = 0.0;
  filteredYaw = 0.0;
  
  digitalWrite(red_ledPin, LOW); 
  delay(1000);
  digitalWrite(red_ledPin, HIGH);
  delay(3000);

  Serial.println("Begining Data Reading...\n");    
  delay(5000);


  uint8_t bmpID = readRegister(BMP_CS_PIN, BMP_ID_REG);
  Serial.printf("BMP280 ID: 0x%02X\n", bmpID);

  BMP_Calibrate();

  digitalWrite(red_ledPin, LOW);
  digitalWrite(green_ledPin, HIGH);
  delay(3000);
  digitalWrite(green_ledPin, LOW);
}

void loop() {



  //Reading MPU Data
  uint8_t mpuBuf[14];
  readBurst(MPU_CS_PIN, MPU_DATA_START, mpuBuf, 14);

  // 16-bit integers
  int16_t raw_ax = (mpuBuf[0] << 8) | mpuBuf[1];
  int16_t raw_ay = (mpuBuf[2] << 8) | mpuBuf[3];
  int16_t raw_az = (mpuBuf[4] << 8) | mpuBuf[5];

  int16_t raw_gx = (mpuBuf[8] << 8) | mpuBuf[9];
  int16_t raw_gy = (mpuBuf[10] << 8) | mpuBuf[11];
  int16_t raw_gz = (mpuBuf[12] << 8) | mpuBuf[13];
 
  int16_t calibrated_rawgx = raw_gx - gyroOffsetX;
  int16_t calibrated_rawgy = raw_gy - gyroOffsetY;
  int16_t calibrated_rawgz = raw_gz - gyroOffsetZ;

  int16_t calibrated_rawAX = raw_ax - accelOffsetX;
  int16_t calibrated_rawAY = raw_ay - accelOffsetY;
  int16_t calibrated_rawAZ = raw_az - accelOffsetZ;

  double accelPitchAxis_g = (double) calibrated_rawAX / 16384.0; // Convert to gravities (g)
  double accelLongitudinal_g = (double) calibrated_rawAY / 16384.0; // Convert to gravities (g)
  double accelYawAxis_g = (double) calibrated_rawAZ / 16384.0; // Convert to gravities (g)

  double gyroX_dps = (double)calibrated_rawgx / 131.0; // Convert to degrees per second (dps)
  double gyroY_dps = (double)calibrated_rawgy / 131.0; // Convert to degrees per second (dps)
  double gyroZ_dps = (double)calibrated_rawgz / 131.0; // Convert to degrees per second (dps)

  // Reading BMP Data
  uint8_t bmpbuf[6];
  digitalWrite(MPU_CS_PIN, HIGH); // Ensure MPU CS is high before reading BMP
  delayMicroseconds(5); // Short delay to ensure proper timing
  readBurst(BMP_CS_PIN, BMP_DATA_START, bmpbuf, 6);
  Serial.printf("BYTES: [%02X, %02X, %02X, %02X, %02X, %02X]\n", bmpbuf[0], bmpbuf[1], bmpbuf[2], bmpbuf[3], bmpbuf[4], bmpbuf[5]);

  // 20-bit
  int32_t adc_P = ((int32_t)bmpbuf[0] << 12) | ((int32_t)bmpbuf[1] << 4) | ((int32_t)(bmpbuf[2] & 0xF0) >> 4);
  int32_t adc_T = ((int32_t)bmpbuf[3] << 12) | ((int32_t)bmpbuf[4] << 4) | ((int32_t)(bmpbuf[5] & 0xF0) >> 4);

  double temperature_C = bmp280_compensate_T(adc_T);
  double pressure_Pa = bmp280_compensate_P(adc_P);
  double pressure_hPa = pressure_Pa / 100.0; // Convert to hPa

  double sealevelhPa = 1013.25;
  double altitude_m = 44330.0 * (1.0 - pow(pressure_hPa / sealevelhPa, 0.1903));
  
  static bool isCalibrated = false; 
  static int sampleCount = 0;
  static double baselineAltitude_m = 0.0;
  const int TOTAL_SAMPLES = 50;

  if(!isCalibrated) {
    baselineAltitude_m += altitude_m;
    sampleCount++;
    if (sampleCount >= TOTAL_SAMPLES) {
      baselineAltitude_m /= TOTAL_SAMPLES;
      isCalibrated = true;
    }
    altitude_m = baselineAltitude_m / (sampleCount > 0 ? sampleCount : 1);
  }

  double rawRelativeAltitude = altitude_m - baselineAltitude_m;

  // LOw pass FIlter
  static double filteredAltitude_m = 0.0;
  static bool filterInitialized = false;

  if (isCalibrated) {
    if (!filterInitialized) {
      filteredAltitude_m = rawRelativeAltitude;
      filterInitialized = true;
    } else {
      double alpha = 0.15;
      filteredAltitude_m = (alpha * rawRelativeAltitude) + ((1.0 - alpha) * filteredAltitude_m);
    
    }

  }
 double currentAltitude_m = filteredAltitude_m;

// Apogee detection
 enum FlightState {STATE_PAD, STATE_ASCENT, STATE_DESCENT };
 static FlightState rocketState = STATE_PAD;

 static double maxAltitude_m = 0.0;
 static int consecutiveDropCount = 0;
 static bool parachuteDeployed = false;
 
 const int PARACHUTE_PIN = 37;

 if (!isCalibrated) {
  switch (rocketState) {
    case STATE_PAD:
    if (currentAltitude_m > 15.0) {
      rocketState = STATE_ASCENT;
      Serial.println(F("---- LIFT-OFF DETECTED | ENTERING ASCENT PHASE ----"));
    }
    break;

    case STATE_ASCENT:
    if (currentAltitude_m > maxAltitude_m){
      maxAltitude_m = currentAltitude_m;
      consecutiveDropCount = 0;
    } else {
      consecutiveDropCount++;
    }
    if ((maxAltitude_m - currentAltitude_m > 2.5) && (consecutiveDropCount > 10)) {
      rocketState = STATE_DESCENT;
      parachuteDeployed = true;

      digitalWrite(PARACHUTE_PIN, HIGH);
      Serial.printf("### APOGEE DETECTED AT %.2fm ###\n", maxAltitude_m);
      Serial.println(F("### PARACHUTE CHARGE FIRED ###"));
    }
    break;

    case STATE_DESCENT:
    break;
  }
 }


   

  Serial.printf("ACCEL_G: [%.2f, %.2f, %.2f] | GYRO: [%.2f, %.2f, %.2f] | Press: %.2f |  Temp: %.2f C | ALT: %.2f\n", accelLongitudinal_g, accelPitchAxis_g, accelYawAxis_g, gyroX_dps, gyroY_dps, gyroZ_dps, pressure_hPa, temperature_C, currentAltitude_m);



  //time passed since last update
  unsigned long currentTime = micros();
  double dt = (currentTime - lastUpdateTime) / 1000000.0; // Convert to seconds
  lastUpdateTime = currentTime;

  //prevent dt spikes
  if (dt > 0.5) dt = 0.01;

  // Pitch and yaw from accelerometer
  double accelPitch = atan2(accelPitchAxis_g,  accelLongitudinal_g) * (180.0 / M_PI);
  double accelYaw= atan2(accelYawAxis_g, accelLongitudinal_g) * (180.0 / M_PI);

  double yawRate = gyroZ_dps;
  if (abs(yawRate) < 2.0) {
    yawRate = 0.0;
  }


  //Complementary filter
  filteredPitch = alpha * (filteredPitch + gyroX_dps * dt) + (1 - alpha) * accelPitch;
  filteredYaw = alpha * (filteredYaw + gyroZ_dps * dt) + (1 - alpha) * accelYaw;


  Serial.printf("Filtered Pitch: %.2f | Filtered Yaw: %.2f\n", filteredPitch, filteredYaw);
  

  //Error calculation
  double errorPitch = 0.00 - filteredPitch;
  double errorYaw = 0.00 - filteredYaw;

  //Derivative Term
  double derivativePitch = -(filteredPitch - lastfiltered_Pitch) / dt;
  double derivativeYaw = -(filteredYaw - lastfiltered_Yaw) / dt;

  //Save fot next
  lastfiltered_Pitch = filteredPitch;
  lastfiltered_Yaw = filteredYaw;

  //PD commands
  double u_pitch = (Kp * errorPitch) + (Kd * derivativePitch);
  double u_yaw = (Kp * errorYaw) + (Kd * derivativeYaw);

  // Safety limits
  double totalGimbal_Mag = sqrt((u_pitch * u_pitch) + (u_yaw * u_yaw));
  if (totalGimbal_Mag > MAX_gimAngle) {
    u_pitch = (u_pitch / totalGimbal_Mag) * MAX_gimAngle;
    u_yaw = (u_yaw / totalGimbal_Mag) * MAX_gimAngle;
  

  }

  int targetPitch_PWM =  servoPitch_CenterPWM + (u_pitch * degToPWM);
  int targetYaw_PWM = servoYaw_CenterPWM + (u_yaw * degToPWM);

  Pitch.write(targetPitch_PWM);
  Yaw.write(targetYaw_PWM);

  //Dignostic
  Serial.printf("PITCH: %.2f | YAW: %.2f || OUT => PitchPWM: %d | YAWPWM: %d\n", filteredPitch, filteredYaw, targetPitch_PWM, targetYaw_PWM);




  delay(10);



}

void writeRegister(int csPin, uint8_t reg, uint8_t value) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(csPin, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(value);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
}

  uint8_t readRegister(int csPin, uint8_t reg) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    uint8_t val;
    digitalWrite(csPin, LOW);
    SPI.transfer(reg | 0x80);
    val = SPI.transfer(0x00);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    return val;
  }

  void readBurst(int csPin, uint8_t startReg, uint8_t* buffer, uint8_t length) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(csPin, LOW);
    SPI.transfer(startReg | 0x80);
    for (uint8_t i = 0; i < length; i++) {
      buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
  }
