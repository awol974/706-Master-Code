#pragma once

// Library used for the mean and median filters, Sorts for you
#include "RunningMedian.h"
#include <FaBo9Axis_MPU9250.h>

FaBo9Axis fabo_9axis;

//RunningMedian LongIR_Samples = RunningMedian(10);
RunningMedian Sonar_Samples = RunningMedian(10);

// Sonar Sensor class
class Ultrasonic_Sensor {
  public:
    void readSensor();
    float cm;
    float GetMedian();
    float GetMean();

  private:
    unsigned long t1;
    unsigned long t2;
    unsigned long pulse_width;

    //used to mark which position of buffer the new reading goes into - circular buffer
    int count = 0;

    int buff[10];
};

void Ultrasonic_Sensor::readSensor(){
  // Hold the trigger pin high for at least 10 us
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(SONAR_ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      break;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(SONAR_ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      break;
    }
  }
  
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;

//Stroing in circular buffer
  buff[count] = cm;
  Sonar_Samples.add(cm);
  count++;
  if(count > 9){
    count = 0;
  }
  count++;
}

float Ultrasonic_Sensor::GetMedian() {
  float medianValue = Sonar_Samples.getMedian();
  return medianValue; 
}

float Ultrasonic_Sensor::GetMean() {
  float meanValue = Sonar_Samples.getAverage();
  return meanValue; 
}

 //2D120X 40mm - 300mm http://www.phidgets.com/products.php?product_id=3520
 //Distance = 2076.0 / (raw - 11) + 2;
class MediumIR_Sensor {
  public:
  void initialize();
    float readLeftSensor(int analogPin);
    float readRightSensor(int analogPin);
    float GetMedian();
    float GetMean();
    float GetRawValue(int analogPin);
  private:
    int count = 0;
    int buff[10];
    RunningMedian MedIR_Samples = RunningMedian(10);
   
    
  
};

void MediumIR_Sensor::initialize() {
    
}


float MediumIR_Sensor::readLeftSensor(int analogPin) {
  float raw = analogRead(analogPin);
  float value =  1291*pow(raw,-0.862);
  buff[count] = value;
  MedIR_Samples.add(buff[count]);
  
  if (count > 9) {
    count = 0;
  } 
  count++;
return value;
  
}

float MediumIR_Sensor::readRightSensor(int analogPin) {
  float raw = analogRead(analogPin);
  float value =2355*pow(raw,-0.975);
  buff[count] = value;
  MedIR_Samples.add(buff[count]);
  
  if (count > 9) {
    count = 0;
  } 
  count++;
return value;
  
}

float MediumIR_Sensor::GetMedian() {
  float medianValue = MedIR_Samples.getMedian();
  return medianValue; 
}

float MediumIR_Sensor::GetMean() {
  float meanValue = MedIR_Samples.getAverage();
  return meanValue; 
}

float MediumIR_Sensor::GetRawValue (int analogPin) {
  
    return analogRead(analogPin);
  
  
}


 //2Y0A21 100mm - 800mm http://www.phidgets.com/products.php?product_id=3521
 //Distance = 4800.0 / (raw - 20);
class LongIR_Sensor {
  public:
  void initialize();
    float readFrontSensor(int analogPin);
    float readRearSensor(int analogPin);
    float GetMedian();
    float GetMean();
    float GetRawValue(int analogPin);
  private:
    int count = 0;
    int buff[10];
    RunningMedian LongIR_Samples = RunningMedian(10);
  
};

void LongIR_Sensor::initialize() {
    
}

float LongIR_Sensor::readFrontSensor(int analogPin) {
  float raw = analogRead(analogPin);
  float value = 5819*pow(raw,-1.01);
  buff[count] = value;
  LongIR_Samples.add(buff[count]);
  
  if (count > 9) {
    count = 0;
  } 
  count ++;
  return value;
}

float LongIR_Sensor::readRearSensor(int analogPin) {
  float raw = analogRead(analogPin);
  float value = 2286.2*pow(raw,-0.841);
  buff[count] = value;
  LongIR_Samples.add(buff[count]);
  
  if (count > 9) {
    count = 0;
  } 
  count ++;
  return value;
}

float LongIR_Sensor::GetMedian() {
  float medianValue = LongIR_Samples.getMedian();
  return medianValue; 
}

float LongIR_Sensor::GetMean() {
  float meanValue = LongIR_Samples.getAverage();
  return meanValue; 
}

float LongIR_Sensor::GetRawValue (int analogPin) {

    return analogRead(analogPin);
 
  
}

class IMU {
  public: 
    void setupIMU();
    float getOrientation();
    void setOrientation(float o);
    void imuUpdate();

    float orientation;

  private:
        float gx, gy, gz;
        float delta;
        float prevRead;
};
void IMU::setupIMU(){

  fabo_9axis.begin();
 
}

float IMU::getOrientation(){
  return orientation;
}

void IMU::setOrientation(float o){
  this->orientation = o;
}


void IMU::imuUpdate(){
  fabo_9axis.readGyroXYZ(&gx, &gy, &gz);

  delta = micros() - prevRead;

  if(gz>gyroCutOff){
    orientation += gz*(delta/1000000);
  }
  prevRead = micros();

  delay(10);
}

