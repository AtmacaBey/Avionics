#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;

TinyGPS gps;
SoftwareSerial ss(0, 1);


int counter = 0;
int buzzer = 22;
int led = 23;
float altt = 0;
float _alt = 0;
int pyro_1 = 17;
int pyro_1_durum = 0;
int py_1 = 0;
int pyro_2 = 16;
int pyro_2_durum = 0;
int py_2 = 0;

void Durum1(void) {
  pyro_1_durum = digitalRead(pyro_1);
  if (pyro_1_durum == HIGH)
    py_1 = 1;
}

void Durum2(void) {
  pyro_2_durum = digitalRead(pyro_2);
  if (pyro_2_durum == HIGH)
    py_2 = 1;
}

void setup() {
  pinMode (led, OUTPUT);
  pinMode (buzzer, OUTPUT);
  pinMode (pyro_1, INPUT);
  pinMode (pyro_2, INPUT);

  for (int i = 0; i < 5; i++) {
    digitalWrite(led, HIGH);
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(led, LOW);
    digitalWrite(buzzer, LOW);
    delay(100);
  }
  delay(500);
  ss.begin(9600);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    digitalWrite(buzzer, HIGH);
    while (1);
  }
  for (int i = 0; i < 2; i++) {
    digitalWrite(led, HIGH);
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(led, LOW);
    digitalWrite(buzzer, LOW);
    delay(100);
  }
  delay(500);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    digitalWrite(buzzer, HIGH);
    while (1) {
      delay(10);
    }
  }
  for (int i = 0; i < 3; i++) {
    digitalWrite(led, HIGH);
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(led, LOW);
    digitalWrite(buzzer, LOW);
    delay(100);
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
  for ( int i = 0; i < 5; i++) {
    _alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
    delay(100);
  }
  delay(500);
}

void loop() {
  float flat, flon;
  unsigned long age;
  uint8_t sat = gps.satellites();
  float alt = gps.f_altitude();
  float spd = gps.f_speed_kmph();
  int dgr = gps.f_course();
  gps.f_get_position(&flat, &flon, &age);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  altt = bme.readAltitude(SEALEVELPRESSURE_HPA) - _alt;
  Durum1();
  Durum2();
  LoRa.beginPacket();
  LoRa.print(counter);
  LoRa.print("/"); 
  LoRa.print(sat);
  LoRa.print("/"); 
  LoRa.print(flat, 6); 
  LoRa.print("/"); 
  LoRa.print(flon, 6);
  LoRa.print("/"); 
  LoRa.print(spd);
  LoRa.print("/"); 
  LoRa.print(alt);
  LoRa.print("/"); 
  LoRa.print(altt);
  LoRa.print("/"); 
  LoRa.print(a.acceleration.x);
  LoRa.print("/"); 
  LoRa.print(a.acceleration.y);
  LoRa.print("/"); 
  LoRa.print(py_1);
  LoRa.print("/"); 
  LoRa.print(py_2);
  LoRa.endPacket();
  counter++;
  digitalWrite(led, LOW);
  digitalWrite(buzzer, LOW);
  smartdelay(100);
  digitalWrite(led, HIGH);
  digitalWrite(buzzer, HIGH);
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
