#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;

int pyro_H_2 = 16;
int pyro_H_1 = 17;
int pyro_S_2 = 8;
int pyro_S_1 = 9;
int pyro_2 = 21;
int pyro_1 = 20;
int buzzer = 22;
int led = 2;

float altt = 0;
float _alt = 0;
float a_x = 0;
float a_y = 0;
float _x = 0;
float _y = 0;

float filtre_deger_BME = 0;
float kalman_old_BME = 0;
float cov_old_BME = 0;

float filtre_deger_AY = 0;
float kalman_old_AX = 0;
float cov_old_AX = 0;

float filtre_deger_AX = 0;
float kalman_old_AY = 0;
float cov_old_AY = 0;

float MA_XY = 0;
float KMA_XY = 0;

int sayac = 0;
int sayac_2 = 0;

float old_altitude = 0;

void setup() {
  Serial.begin(9600);

  pinMode (led, OUTPUT);
  pinMode (buzzer, OUTPUT);
  pinMode (pyro_1, OUTPUT);
  pinMode (pyro_2, OUTPUT);
  pinMode (pyro_H_1, OUTPUT);
  pinMode (pyro_H_2, OUTPUT);
  pinMode (pyro_S_1, OUTPUT);
  pinMode (pyro_S_2, OUTPUT);
  delay(200);
  //******************Açılma Sinyalı************************
  for ( int j = 0; j < 5; j++) {
    digitalWrite(buzzer, HIGH);
    digitalWrite(led, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    digitalWrite(led, LOW);
    delay(100);
  }
  //********************************************
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    digitalWrite(buzzer, HIGH);
    while (1) {
      delay(10);
    }
  }
  unsigned status;
  status = bme.begin();
  if (!status) {
    digitalWrite(buzzer, HIGH);
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  for ( int i = 0; i < 5; i++) {
    _x = a.acceleration.x;
    _y = a.acceleration.y;
    _alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
    delay(200);
  }
  for ( int j = 0; j < 5; j++) {
    digitalWrite(buzzer, HIGH);
    digitalWrite(led, HIGH);
    delay(400);
    digitalWrite(buzzer, LOW);
    digitalWrite(led, LOW);
    delay(100);
  }
  delay(200);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  altt = bme.readAltitude(SEALEVELPRESSURE_HPA) - _alt;
  a_x = a.acceleration.x - _x;
  a_y = a.acceleration.y - _y;
  filtre_deger_BME = kalman_filter_BME(altt);
  filtre_deger_AY = kalman_filter_AY(a_y);
  filtre_deger_AX = kalman_filter_AY(a_x);
  MA_XY = abs(a_x) + abs(a_y);
  KMA_XY = abs(filtre_deger_AX) + abs(filtre_deger_AY);


  //  Serial.print("Kalman BME Yükseklik "); Serial.print(filtre_deger_BME); Serial.println(" m");
  //  Serial.print("Eski yükseklik "); Serial.print(old_altitude); Serial.println(" m");
  //  Serial.print("Acceleration X: "); Serial.print(a_x); Serial.print(", Y: "); Serial.println(a_y);
  //  Serial.print("kalman Acceleration X: "); Serial.print(filtre_deger_AX); Serial.print(", Y: "); Serial.println(filtre_deger_AY);
  //  Serial.print("Mutlak Toplam Acceleration: "); Serial.println(MA_XY);
  //  Serial.print("Mutlak Toplam Kalman Acceleration: "); Serial.println(KMA_XY);
  //  Serial.println("*****************xD******************");

  if (filtre_deger_BME >= 4000 && KMA_XY >= 5) {
    if (old_altitude > filtre_deger_BME ) {
      sayac++;
      if (sayac == 5)
      {
        digitalWrite(pyro_1, HIGH);
        digitalWrite(pyro_H_1, HIGH);
        digitalWrite(pyro_S_1, HIGH);
        delay(2000);
        digitalWrite(pyro_1, LOW);

      }
      else {
        sayac = 0;
      }
    }
  }

  else if (old_altitude > filtre_deger_BME && filtre_deger_BME < 4000) {
    if (filtre_deger_BME <= 500) {
      sayac_2++;
      if (sayac_2 == 5)
      {
        digitalWrite(pyro_2, HIGH);
        digitalWrite(pyro_H_2, HIGH);
        digitalWrite(pyro_S_2, HIGH);
        delay(2000);
        digitalWrite(pyro_2, LOW);

      }
      else {
        sayac_2 = 0;
      }
    }
  }

  old_altitude = filtre_deger_BME;
  digitalWrite(buzzer, HIGH);
  digitalWrite(led, HIGH);
  delay(25);
  digitalWrite(buzzer, LOW);
  digitalWrite(led, LOW);
  delay(75);
}


// Kalman Filtresi Fonksiyonları BME AX AY

float kalman_filter_BME (float input_BME)
{

  float kalman_new_BME = kalman_old_BME;
  float cov_new_BME = cov_old_BME + 0.80;

  float kalman_gain_BME = cov_new_BME / (cov_new_BME + 0.5);
  float kalman_calculated_BME = kalman_new_BME + (kalman_gain_BME * (input_BME - kalman_new_BME));

  cov_new_BME = (1 - kalman_gain_BME) * cov_old_BME;
  cov_old_BME = cov_new_BME;

  kalman_old_BME = kalman_calculated_BME;

  return kalman_calculated_BME;
}

float kalman_filter_AY (float input_AY)
{

  float kalman_new_AY = kalman_old_AY;
  float cov_new_AY = cov_old_AY + 0.50;

  float kalman_gain_AY = cov_new_AY / (cov_new_AY + 0.9);
  float kalman_calculated_AY = kalman_new_AY + (kalman_gain_AY * (input_AY - kalman_new_AY));

  cov_new_AY = (1 - kalman_gain_AY) * cov_old_AY;
  cov_old_AY = cov_new_AY;

  kalman_old_AY = kalman_calculated_AY;

  return kalman_calculated_AY;
}

float kalman_filter_AX (float input_AX)
{

  float kalman_new_AX = kalman_old_AX;
  float cov_new_AX = cov_old_AX + 0.50;

  float kalman_gain_AX = cov_new_AX / (cov_new_AX + 0.9);
  float kalman_calculated_AX = kalman_new_AX + (kalman_gain_AX * (input_AX - kalman_new_AX));

  cov_new_AX = (1 - kalman_gain_AX) * cov_old_AX;
  cov_old_AX = cov_new_AX;

  kalman_old_AX = kalman_calculated_AX;

  return kalman_calculated_AX;
}
