//librerie generali
#include <String.h>
#include <math.h>

// librerie sender
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

//librerie bmp280
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;


// dichiarazione pin per il motore
const int motor1Pin = 32;
const int motor2Pin = 33;
const int enablePin = 4;
const int ledPin = 13;


//variabili per l'accelerometro
const int MPU = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int AcXcal, AcYcal, AcZcal, GyXcal, GyYcal, GyZcal, tcal;
double t, tx, tf, pitch, roll;
TwoWire I2CBMP = TwoWire(0);

void setup() {
  //qui chiamo le funzioni che ho creato per ogni sensore
  SetupTemperatura();
  SetupSender();
  SetupAccelerometro();
  SetupMotore();

}

void loop() {
  //creo un array dove metto i dati che vengono rilevati dall'accelerometro
  double data[9];
  LoopAccelerometro(data);
  //chiamo la funzione per far cambiare direzione al motore
  LoopMotore();
  //qui invece mando i dati tramite lora., inoltre leggo i dati dal bmp280 e mando anche i dati che ho raccolto con l'accelerometro
  LoRa.beginPacket();
  LoRa.print(F("Temperature = "));
  LoRa.print(bmp.readTemperature());
  LoRa.println(" *C");
  for (int i = 0; i < 9; i++) {
    LoRa.println(String(data[i]));
  }

  LoRa.print(F("Pressure = "));
  LoRa.print(bmp.readPressure());
  LoRa.println(" Pa");

  LoRa.print(F("Approx altitude = "));
  LoRa.print(bmp.readAltitude(1036.00)); 
  LoRa.println(" m");
  LoRa.endPacket();
  delay(100);
}

void SetupSender() {
  // cambio dei pin per adattare la libreria all'esp32
  LoRa.setPins(5, 34, 17);
  //questo comando serve per tetare che si può cambiare la frequenza
  LoRa.setSPIFrequency(900);
  while (!Serial);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

}
void SetupTemperatura() {
  //setup per avviare il sensore di temperatura
  Serial.begin(9600);
  while ( !Serial ) delay(100);   
  Serial.println(F("BMP280 test"));
  uint8_t status;
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,    
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500);
}
void SetupAccelerometro() {
  //setup per avviare l'accelerometro
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void  LoopAccelerometro(double data[]) {
  //leggo i dati dall'accelerometro e le metto nell'array dati
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  AcXcal = -950;
  AcYcal = -300;
  AcZcal = 0;
  tcal = -1600;
  GyXcal = 480;
  GyYcal = 170;
  GyZcal = 210;
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
  tx = Tmp + tcal;
  t = tx / 340 + 36.53;
  tf = (t * 9 / 5) + 32;
  data[0] = pitch;
  data[1] = roll;
  data[2] = AcX + AcXcal;
  data[3] = AcY + AcYcal;
  data[4] = AcZ + AcZcal;
  data[5] = t;
  data[6] = GyX + GyXcal;
  data[7] = GyY + GyYcal;
  data[8] = GyZ + GyZcal;
}
void SetupMotore() {
  //inizializzo i pin per il motore
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(enablePin, HIGH);
}

void LoopMotore() {
  //qui cambio il senso del motore, cambiano l'imput che arriva al ponte h
  digitalWrite(motor1Pin, LOW);
  digitalWrite(motor2Pin, HIGH);
  delay(5000);
  digitalWrite(motor1Pin, HIGH);
  digitalWrite(motor2Pin, LOW);
  delay(5000);
}
