#include <Wire.h>
#include <SparkFun_ENS160.h>
#include <Adafruit_AHTX0.h>
#include <MQUnifiedsensor.h>

// ===============================
// ENS160 + AHT21 설정
// ===============================
SparkFun_ENS160 ens160;
Adafruit_AHTX0 aht;

// ===============================
// MQ-2 설정
// ===============================
#define Board "Arduino UNO"
#define Pin A0
#define Type "MQ-2"
#define Voltage_Resolution 5
#define ADC_Bit_Resolution 10
#define RatioMQ2CleanAir 9.83

MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

// ===============================
// SETUP
// ===============================
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Initializing ENS160 + AHT21 + MQ-2...");
  Wire.begin();

  if (!aht.begin()) {
    Serial.println("AHT21 not found!");
    while (1);
  }

  if (!ens160.begin()) {
    Serial.println("ENS160 not found!");
    while (1);
  }

  ens160.setOperatingMode(SFE_ENS160_STANDARD);
  Serial.println("ENS160 + AHT21 Ready!");

  MQ2.init();
  Serial.println("Heating MQ-2 for 2 minutes...");

  for (int i = 0; i < 120; i++) {
    MQ2.update();
    delay(1000);
  }

  MQ2.setRegressionMethod(1);
  MQ2.setA(574.25);
  MQ2.setB(-2.222);

  Serial.println("Calibrating MQ-2 in clean air...");
  float R0 = 0;
  for (int i = 0; i < 10; i++) {
    MQ2.update();
    R0 += MQ2.calibrate(RatioMQ2CleanAir);
    delay(500);
  }
  R0 = R0 / 10.0;
  MQ2.setR0(R0);

  Serial.print("MQ-2 R0 = ");
  Serial.println(R0, 2);
  Serial.println("All sensors ready!");
}

// ===============================
// LOOP (JSON 데이터 전송)
// ===============================
void loop() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  float eco2 = (float)ens160.getECO2();
  float tvoc = (float)ens160.getTVOC();

  MQ2.update();
  float LPG_PPM = MQ2.readSensor();

  // === JSON 형식으로 출력 (모든 값 소수점 2자리) ===
  Serial.print("{\"temp\": ");
  Serial.print(temp.temperature, 2);
  Serial.print(", \"hum\": ");
  Serial.print(humidity.relative_humidity, 2);
  Serial.print(", \"eco2\": ");
  Serial.print(eco2, 2);
  Serial.print(", \"tvoc\": ");
  Serial.print(tvoc, 2);
  Serial.print(", \"lpg\": ");
  Serial.print(LPG_PPM, 2);
  Serial.println("}");

  delay(15000); // 15초마다 전송
}
