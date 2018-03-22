  
#include "FSM.h"
#include "Robot_Objects.h"
#include "Robot_Parameters.h"
#include "Sensors.h"

float ax, ay, az;

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  digitalWrite(SONAR_TRIG_PIN, LOW);

// Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

//If no MPU is attached then .begin will hang forever 
#ifndef NO_READ_MPU
  if (fabo_9axis.begin()) {
    SerialCom->println("configured FaBo 9Axis I2C Brick");
    axis_OK = true;
  } else {
    SerialCom->println("FaBo 9Axis device error");
    axis_OK = false;
  }
#endif

delay(1000); //settling time but not really needed

}

void loop(void) {
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case TRAVEL_FORWARD:
        
        //machine_state = travel_forward();
            
//      if(sonar.readSensor() < 10){
//        machine_state = stopped();
//      }
//      else{
//        machine_state =  travel_forward();
//      }
      break;

    case TURN_CCW:
        imu.imuUpdate();
        //timer = millis();
        machine_state = turn_ccw();
        break;
    case STOPPED:
    
        machine_state = stopped();
        
//      if(sonar.readSensor() > 10){
//        machine_state = travel_forward();
//      }
//      else{
//        machine_state =  stopped();
//      }
      break;
    case BATTERY_FLAT:
      machine_state = battery_flat();
      break;
  };

  fabo_9axis.readAccelXYZ(&ax, &ay, &az);
  SerialCom->print("MPU ax:");
  SerialCom->println(ax);

//Initialize 
//Right_IRSensor.initialize();
///Left_IRSensor.initialize();
//Front_IRSensor.initialize();
//Rear_IRSensor.initialize();

//Read the sensors
sonar.readSensor();
//Right_IRSensor.readSensor(MED_IR_RIGHT_PIN);
//Front_IRSensor.readSensor(LONG_IR_FRONT_PIN);
//Left_IRSensor.readSensor(MED_IR_LEFT_PIN);
//Rear_IRSensor.readSensor(LONG_IR_REAR_PIN);

//Print the median value from buffer
 SerialCom->print("Sonar:");
  SerialCom->println(sonar.GetMedian());
   SerialCom->print("Right Med IR :");
 SerialCom->println(Right_IRSensor.readRightSensor(MED_IR_RIGHT_PIN));
   SerialCom->print("Left Med IR: ");
  SerialCom->println(Left_IRSensor.readLeftSensor(MED_IR_LEFT_PIN));
   SerialCom->print("Front Long IR: ");
  SerialCom->println(Front_IRSensor.readFrontSensor(LONG_IR_FRONT_PIN));
   SerialCom->print("Rear Long IR: ");
  SerialCom->println(Rear_IRSensor.readRearSensor(LONG_IR_REAR_PIN));


  delay(500);

//Battery voltage stuff
   static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    SerialCom->print(" : Raw Lipo:");
    SerialCom->println(raw_lipo);
   
  }  
}



