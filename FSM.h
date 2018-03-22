#pragma once

#include "Robot_Objects.h"
//State machine states
enum STATE {
  INITIALISING,
  TRAVEL_FORWARD,
  TRAVEL_BACKWARD,
  TURN_CCW,
  TURN_CW,
  STRAFE_LEFT,
  STRAFE_RIGHT,
  STOPPED,
  BATTERY_FLAT
};

STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return TRAVEL_FORWARD;
}

STATE travel_forward() {
  
  enable_motors();
  
  left_front_motor.writeMicroseconds(1500 + (speed_val*left_front_gain));
  left_rear_motor.writeMicroseconds(1500 + (speed_val*left_rear_gain));
  right_rear_motor.writeMicroseconds(1500 - (speed_val*right_rear_gain));
  right_front_motor.writeMicroseconds(1500 - (speed_val*right_front_gain));

  #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return BATTERY_FLAT;
  #endif

  return TRAVEL_FORWARD;
}

STATE travel_backward() {

  enable_motors();
  
  left_front_motor.writeMicroseconds(1500 - speed_val*left_front_gain);
  left_rear_motor.writeMicroseconds(1500 - speed_val*left_rear_gain);
  right_rear_motor.writeMicroseconds(1500 + speed_val*right_rear_gain);
  right_front_motor.writeMicroseconds(1500 + speed_val*right_front_gain);

  #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return BATTERY_FLAT;
  #endif

  return TRAVEL_BACKWARD;
}

STATE turn_ccw() {

  enable_motors();
  
  left_front_motor.writeMicroseconds(1500 - speed_val*left_front_gain);
  left_rear_motor.writeMicroseconds(1500 - speed_val*left_rear_gain);
  right_rear_motor.writeMicroseconds(1500 - speed_val*right_rear_gain);
  right_front_motor.writeMicroseconds(1500 - speed_val*right_front_gain); 

   #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return BATTERY_FLAT;
   #endif
    if (imu.getOrientation() > turnAngle) {
      imu.setOrientation(0);
      return TRAVEL_FORWARD;
    } else {
      return TURN_CCW;
    }
}

STATE turn_cw() {

  enable_motors();
  
  left_front_motor.writeMicroseconds(1500 + speed_val*left_front_gain);
  left_rear_motor.writeMicroseconds(1500 + speed_val*left_rear_gain);
  right_rear_motor.writeMicroseconds(1500 + speed_val*right_rear_gain);
  right_front_motor.writeMicroseconds(1500 + speed_val*right_front_gain);  

  #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return BATTERY_FLAT;
  #endif

 
  return TURN_CW;
}

STATE strafe_left() {

  enable_motors();
  
  left_front_motor.writeMicroseconds(1500 - speed_val*left_front_gain);
  left_rear_motor.writeMicroseconds(1500 + speed_val*left_rear_gain);
  right_rear_motor.writeMicroseconds(1500 + speed_val*right_rear_gain);
  right_front_motor.writeMicroseconds(1500 - speed_val*right_front_gain);

  #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return BATTERY_FLAT;
  #endif

  return STRAFE_LEFT;
}

STATE strafe_right() {
  
  enable_motors();
  
  left_front_motor.writeMicroseconds(1500 + speed_val*left_front_gain);
  left_rear_motor.writeMicroseconds(1500 - speed_val*left_rear_gain);
  right_rear_motor.writeMicroseconds(1500 - speed_val*right_rear_gain);
  right_front_motor.writeMicroseconds(1500 + speed_val*right_front_gain);

  #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return BATTERY_FLAT;
  #endif

  return STRAFE_RIGHT;
}

STATE stopped() {
  
  disable_motors();

  #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return BATTERY_FLAT;
  #endif

  return STOPPED;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE battery_flat() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;

  disable_motors();
  //slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
    SerialCom->println("Please Re-charge Lipo");

    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK Counter:");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        //return RUNNING;
        
      }
    } else counter_lipo_voltage_ok = 0;
  }
  return STOPPED;
}





