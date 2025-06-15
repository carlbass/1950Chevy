/*
This works ONLY with the Seeed Studio library called CAN_BUS_Shield. This was compiled with version 2.3.3 of the library which was downloaded as a zip file from github
DO NOT USE other CAN bus libraries!!! The zip file is called Seeed_Arduino_CAN-master and is stored in the 1950 Chevy folder. The IDE currently used is 2.3.6
And make sure all the libraries and cached files on the PC are removed
The latest version of the library changed the number of arguments to sendMsgBuf which caused a lot of compilation errors
*/

#include "mcp_can.h"
#include "can-serial.h"
#include "mcp2515_can.h"

const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin


#define CONTROLLER_TEMPERATURE_PIN  3
#define CHIP_SELECT                 4
#define MOTOR_AMPERAGE_PIN          5
#define MOTOR_TEMPERATURE_PIN       6 
#define SPEEDBOX_PIN                7

#define MIN_MOTOR_TEMPERATURE 50.0
#define MAX_MOTOR_TEMPERATURE 350.0

#define MIN_CONTROLLER_TEMPERATURE 50.0
#define MAX_CONTROLLER_TEMPERATURE 200.0

#define MIN_MOTOR_AMPERAGE 0.0
#define MAX_MOTOR_AMPERAGE 500.0

unsigned char buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int vehicle_speed_kmph = 0;
int motor_1_temperature = 0;
int controller_1_temperature = 0;
int motor_2_temperature = 0;
int controller_2_temperature = 0;
int motor_1_amperage = 0;
int motor_2_amperage = 0;


void setup() {
    unsigned char len = 0;
    int           can_ID = 0x00;


    // get CAN bus communication starts
    while (CAN_OK != CAN.begin(CAN_500KBPS))           // set CANbus baudrate to 500kbps
    {
        delay(50);
    }    
    
    // turn on speedbox so we can see request for data 0x7E0; ; timing requires arduino up and running; otherwise speedbox times out after sending 0x7E0
    pinMode(SPEEDBOX_PIN, OUTPUT); 
    digitalWrite (SPEEDBOX_PIN, HIGH);

    int start_millis = millis();

    // start speedbox and wait for response and then set speedometer to 0 by sending appropriate CAN bus message; timeout after 5 seconds
    while ((millis() - start_millis < 5000) ) 
    {       
      if (CAN_MSGAVAIL == CAN.checkReceive())
      {
        CAN.readMsgBuf(&len, buf);    
        can_ID = CAN.getCanId();

        if (can_ID == 0x7E0)         
        {
          buf[0] = 0x3;  // number of additional data bytes
          buf[1] = 0x41; // show current data response
          buf[2] = 0xD;  // PID for vehicle speed in kmph
          buf[3] = 0;
          
          CAN.sendMsgBuf (0x7E8, 0, 0, 4, buf);  //  id assigned for OBD-II response
          ++i;
        }
        delay (25);
      }
    }

    // set up pin to write PWM signal to drive 3 gauges; the "fuel" gauge gets its info directly from the BMS
    pinMode(MOTOR_TEMPERATURE_PIN, OUTPUT);
    pinMode(CONTROLLER_TEMPERATURE_PIN, OUTPUT);
    pinMode(MOTOR_AMPERAGE_PIN, OUTPUT);
 
    // let the gauges climb all the way up and the back down
    for (int i = 0; i <= 255; ++i)
    {
      analogWrite(MOTOR_TEMPERATURE_PIN, i);            // when power is turned on, gauge should move all the way     
      analogWrite(CONTROLLER_TEMPERATURE_PIN, i);       // when power is turned on, gauge should move all the way clockwise
      analogWrite(MOTOR_AMPERAGE_PIN, i);               // when power is turned on, gauge should move all the way 
      // also let speedometer climb          
      buf[3] = i / 2.0;
      CAN.sendMsgBuf (0x7E8, 0, 0, 4, buf);  //  id assigned for OBD-II response
      delay (20);
    }
    
    for (int i = 255; i >= 0; --i)
    {
      analogWrite(MOTOR_TEMPERATURE_PIN, i); // and then move the needles back    
      analogWrite(CONTROLLER_TEMPERATURE_PIN, i);       
      analogWrite(MOTOR_AMPERAGE_PIN, i);               
      // also let speedometer slowly go back to 0          
      buf[3] = i / 2.0;
      CAN.sendMsgBuf (0x7E8, 0, 0, 4, buf);  //  id assigned for OBD-II response
      delay (20);
    }
}

void drive_electric_gauges()
{
    int PWM_out_level = 0;
    int max_motor_temperature = max(motor_1_temperature, motor_2_temperature);
    int max_controller_temperature = max(controller_1_temperature, controller_2_temperature);

    // output maximum motor temperature
    PWM_out_level = map (max_motor_temperature, MIN_MOTOR_TEMPERATURE, MAX_MOTOR_TEMPERATURE, 0, 255);
    PWM_out_level = constrain(PWM_out_level, 0, 255);
    analogWrite(MOTOR_TEMPERATURE_PIN, PWM_out_level);
    
    // output maximum contoller temperature
    PWM_out_level = map (max_controller_temperature, MIN_CONTROLLER_TEMPERATURE, MAX_CONTROLLER_TEMPERATURE, 0, 255);
    PWM_out_level = constrain(PWM_out_level, 0, 255);
    PWM_out_level = 255 - PWM_out_level;    
    analogWrite(CONTROLLER_TEMPERATURE_PIN, PWM_out_level);

    // output maximum motor current
    motor_1_amperage = constrain (motor_1_amperage, MIN_MOTOR_AMPERAGE, MAX_MOTOR_AMPERAGE/2.0);
    motor_2_amperage = constrain (motor_2_amperage, MIN_MOTOR_AMPERAGE, MAX_MOTOR_AMPERAGE/2.0);
    int combined_motor_amperage = motor_1_amperage + motor_2_amperage;

    // output combined ampergae through motors
    PWM_out_level = map (combined_motor_amperage, MIN_MOTOR_AMPERAGE, MAX_MOTOR_AMPERAGE, 0, 255);
    PWM_out_level = constrain(PWM_out_level, 0, 255);
    PWM_out_level = 255 - PWM_out_level;
    analogWrite(MOTOR_AMPERAGE_PIN, PWM_out_level);
}


/*
 * process CAN bus messages from controllers with following format
 * for 0x400 and 0x401
 * byte 0 + byte 1 = motor amperage
 * byte 3 + byte 4 = vehicle speed kmph
 * byte 5 + byte 6 = low word for odometer
 * byte 7 + byte 8 = high word for odometer
 *
 * for 0x402 and 0x403
 * byte 1 = inverter temperature
 * byte 2 = motor temperature
 * 
 * for 0x404 and 0x405 
 * byte 1 state of charge
 * byte 2 = fault level
 * byte 3 = fault code
*/

 
void loop() {
    unsigned char len = 0;
    int           can_ID = 0x00;

    if (CAN_MSGAVAIL == CAN.checkReceive()) {       
        CAN.readMsgBuf(&len, buf);    
        can_ID = CAN.getCanId();

        if (can_ID == 0x400)         
        {
          // extract current through motor
          motor_1_amperage = (buf[0] + (buf[1] << 8)) / 10.0; 

          // extract vehicle speed
          vehicle_speed_kmph = ((buf[2] + (buf[3] << 8)) / 10.0);
                      
          drive_electric_gauges();
       }
       else if (can_ID == 0x401)
       {        
          // extract current through motor
          motor_2_amperage = (buf[0] + (buf[1] << 8)) / 10.0; 

          // extract vehicle speed
          vehicle_speed_kmph = ((buf[2] + (buf[3] << 8)) / 10.0);
           
           drive_electric_gauges();
          
        }
        else if (can_ID == 0x402)
        {
          // extract motor and controller temperature
          controller_1_temperature = buf[0];
          motor_1_temperature = buf[1];

          // offset by 40 and convert temperature from Celsius to Fahrenheit
          controller_1_temperature += 40;        
          controller_1_temperature = (int) (((float) (controller_1_temperature) * 1.8)) + 32;

          motor_1_temperature += 40;        
          motor_1_temperature = (int) (((float) (motor_1_temperature) * 1.8)) + 32;
            
          drive_electric_gauges();
        }
        else if (can_ID == 0x403)
        {
          // extract motor and controller temperature
          controller_2_temperature = buf[0];
          motor_2_temperature = buf[1];

          // offset by 40 and convert temperature from Celsius to Fahrenheit
          controller_2_temperature += 40;        
          controller_2_temperature = (int) (((float) (controller_2_temperature) * 1.8)) + 32;

          motor_2_temperature += 40;        
          motor_2_temperature = (int) (((float) (motor_2_temperature) * 1.8)) + 32;

            
          drive_electric_gauges();
       }
       else if (can_ID == 0x7E0)
       {
          // send out the most recently recorded speed on CAN bus       
          can_ID = 0x7E8;  // specific OBD-II address for responses; used to send info to speedbox
          buf[0] = 0x3;  // number of additional data bytes
          buf[1] = 0x41; // show current data response
          buf[2] = 0xD;  // PID for vehicle speed in kmph
          buf[3] = vehicle_speed_kmph;

          CAN.sendMsgBuf (can_ID, 0, 0, 4, buf);  //  id assigned for OBD-II response
       }
    }
}
