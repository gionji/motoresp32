#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>

#include <arduinoFFT.h>

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

#define SCL_INDEX     0x00
#define SCL_TIME      0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT      0x03

const uint16_t BLOCK_SIZE = 4096; 

double signalFrequency    = 266;
double samplingFrequency  = 5000;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[BLOCK_SIZE];
double vImag[BLOCK_SIZE];


#define REAR_SUSPENSION_PIN  3
#define FRONT_SUSPENSION_PIN 4
#define REAR_TEMPERATURE_PIN 5
#define MIC_PIN A0

// https://iotassistant.io/esp32/smart-door-bell-noise-meter-using-fft-esp32/
// https://github.com/squix78/esp32-mic-fft/blob/master/esp32-mic-fft.ino
 
BLEServer* pServer                 = NULL;
BLECharacteristic* pCharacteristic = NULL;

BLECharacteristic* pPitchCharacteristic     = NULL;
BLECharacteristic* pRollCharacteristic      = NULL;
BLECharacteristic* pSuspFrontCharacteristic = NULL;
BLECharacteristic* pSuspRearCharacteristic  = NULL;
BLECharacteristic* pTempRearCharacteristic  = NULL;
BLECharacteristic* pTempExtCharacteristic   = NULL;
BLECharacteristic* pRpmCharacteristic       = NULL;
 
bool deviceConnected       = false;
bool oldDeviceConnected    = false;

uint32_t pitch_value       = 0;
uint32_t roll_value        = 0;
uint32_t susp_front_value  = 0;
uint32_t susp_rear_value   = 0;
uint32_t temp_ext_value    = 0;
uint32_t temp_rear_value   = 0;
uint32_t rpm_value         = 0;

uint32_t pitch = 0;
uint32_t roll = 0;
uint32_t susp_front = 0;
uint32_t susp_rear = 0;
uint32_t temp_ext = 0;
uint32_t temp_rear = 0;
uint32_t rpm = 0;

 
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
 
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"

#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define PITCH_CHARACTERISTIC_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a0"
#define ROLL_CHARACTERISTIC_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26a1"
#define SUSP_FRONT_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a2"
#define SUSP_REAR_CHARACTERISTIC_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a3"
#define TEMP_REAR_CHARACTERISTIC_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a4"
#define TEMP_FRONT_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a5"
#define TEMP_EXT_CHARACTERISTIC_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a6"
#define RPM_CHARACTERISTIC_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a7"


Adafruit_MPU6050 mpu;
 
 
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
 
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
 


 
void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("GionjiSP32");

  randomSeed(42);  

  samplingFrequency = calculateAdcSampligRate();
 
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
 
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
 
  // Create a BLE Characteristic
  pPitchCharacteristic = pService->createCharacteristic(
                      PITCH_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
 
  // Create a BLE Characteristic
  pRollCharacteristic = pService->createCharacteristic(
                      ROLL_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
 
  // Create a BLE Characteristic
  pSuspFrontCharacteristic = pService->createCharacteristic(
                      SUSP_FRONT_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
 
  // Create a BLE Characteristic
  pSuspRearCharacteristic = pService->createCharacteristic(
                      SUSP_REAR_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
 
  // Create a BLE Characteristic
  pTempRearCharacteristic = pService->createCharacteristic(
                      TEMP_REAR_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
 
  // Create a BLE Characteristic
  pTempExtCharacteristic = pService->createCharacteristic(
                      TEMP_EXT_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
 
  // Create a BLE Characteristic
  pRpmCharacteristic = pService->createCharacteristic(
                      RPM_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

 
  // Start the service
  pService->start();
 
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  // mpu init
  mpu_init();
  Serial.println("MPU initialized...");
}


 
void loop() {
    /* Get new sensor events with the readings */
    pitch      = get_pitch();
    roll       = get_roll();
    susp_front = get_susp_front();
    susp_rear  = get_susp_rear();
    temp_ext   = get_temp_ext();
    temp_rear  = get_temp_rear();
    rpm        = get_rpm();

    // notify changed value
    if (deviceConnected) {
        pPitchCharacteristic->setValue((uint8_t*)&pitch_value, pitch);
        pRollCharacteristic->setValue((uint8_t*)&roll_value, roll);
        pSuspFrontCharacteristic->setValue((uint8_t*)&susp_front_value, susp_front);
        pSuspRearCharacteristic->setValue((uint8_t*)&susp_rear_value, susp_rear);
        pTempExtCharacteristic->setValue((uint8_t*)&temp_ext_value, temp_ext);
        pTempRearCharacteristic->setValue((uint8_t*)&temp_rear_value, temp_rear);
        pRpmCharacteristic->setValue((uint8_t*)&rpm_value, rpm);
        
        pPitchCharacteristic->notify();
        pRollCharacteristic->notify();
        pSuspFrontCharacteristic->notify();
        pSuspRearCharacteristic->notify();
        pTempExtCharacteristic->notify();
        pTempRearCharacteristic->notify();
        pRpmCharacteristic->notify();
        delay(500);
    }
    
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}

int calculateAdcSampligRate(){
  long firstStamp = millis();
  
  for(int i=0; i<BLOCK_SIZE; i++){
    vReal[ i ] = analogRead(A0);
    vImag[ i ] = 0;
    }
  long secondStamp = millis();
  long elapsedTime = secondStamp - firstStamp;
  
  samplingFrequency = ((float)BLOCK_SIZE / (float)elapsedTime) * 1000;

  return samplingFrequency;
  }


int get_rpm(){

  // read audio
  long firstStamp = millis();
  
  for(int i=0; i<BLOCK_SIZE; i++){
    vReal[ i ] = analogRead(A0);
    vImag[ i ] = 0.0;
    }

  long secondStamp = millis();
  long elapsedTime = secondStamp - firstStamp;

  // fft
  FFT.Windowing(vReal, BLOCK_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, BLOCK_SIZE, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, BLOCK_SIZE);

  // ectract peaks
  double peak = FFT.MajorPeak(vReal, BLOCK_SIZE, samplingFrequency);
  Serial.println(peak, 6);
  
  return (int) peak;
}

int get_roll(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
    
  int roll  = functionsPitchRoll(a.acceleration.x, a.acceleration.y, a.acceleration.z);  
 
  return roll;
  }

int get_pitch(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  int pitch = functionsPitchRoll(a.acceleration.y, a.acceleration.x, a.acceleration.z);  //Calcolo angolo Pitch

  return roll;
  }

int get_temp_ext(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
    
  return temp.temperature;
  }
  

int get_temp_rear(){
  return (int) random(0,255);
  }

int get_susp_rear(){
  return (int) random(0,255);
  }
  
int get_susp_front(){
  return (int) random(0,255);
  }



int getDistance(int pingPin){
  // defines variables
  long duration;
  int distance;

  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);

  pinMode(pingPin, OUTPUT); 
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  
  pinMode(pingPin, INPUT); 
  duration = pulseIn(pingPin, HIGH);

  // Calculating the distance
  distance = duration*0.034/2;

  return distance;
  }



//Funzione per il calcolo degli angoli Pitch e Roll
double functionsPitchRoll(double A, double B, double C) {
  double DatoA, DatoB, Value;
  DatoA = A;
  DatoB = (B * B) + (C * C);
  DatoB = sqrt(DatoB);

  Value = atan2(DatoA, DatoB);
  Value = Value * 180 / 3.14;

  return (int)Value;
}


void mpu_init(){
  
  // Try to initialize MPU6050!
  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(1000);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  delay(100);
  Serial.println("");

}
  
