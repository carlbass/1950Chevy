// 3/23/2021 rewrote to get rid of data logging and just drive gauges

#include <SPI.h>
#include "mcp2515_can.h"

/*
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SERIAL SerialUSB
#else
    #define SERIAL Serial
#endif
*/


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10

const int SPI_CS_PIN = 9;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

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
#define MAX_MOTOR_AMPERAGE 500

unsigned char buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

typedef struct {
    int vehicle_speed_kmph;
    int motor1_temperature;
    int controller1_temperature;
    int motor2_temperature;
    int controller2_temperature;
    int motor1_current;
    int motor2_current;
    int motor1_bus_current;
    int motor2_bus_current;  
} Operating_State;

Operating_State state;

int vehicle_speed_kmph = 0;
int motor1_temperature = 0;
int controller1_temperature = 0;
int motor2_temperature = 0;
int controller2_temperature = 0;
int motor1_current = 0;
int motor2_current = 0;
int motor1_bus_current = 0;
int motor2_bus_current = 0;

void setup() {
    unsigned char len = 0;
    int           can_ID = 0x00;
    int           i;

    Serial.begin(115200);
    delay (2000);

    state.vehicle_speed_kmph = 0;
    state.motor1_temperature = 0;
    state.controller1_temperature = 0;
    state.motor2_temperature = 0;
    state.controller2_temperature = 0;
    state.motor1_current = 0;
    state.motor2_current = 0;
    state.motor1_bus_current = 0;
    state.motor2_bus_current = 0;

    while (CAN_OK != CAN.begin(CAN_500KBPS))           // set CANbus baudrate to 500kbps
    {
        Serial.println("CAN init failed");
        delay(200);
    }    
    
   // turn on speedbox so we can see request for data 0x7E0; ; timing requires arduino up and running; otherwise speedbox times out after sending 0x7E0
    pinMode(SPEEDBOX_PIN, OUTPUT); 
    digitalWrite (SPEEDBOX_PIN, HIGH);

    int start_millis = millis();
 
    i = 0;
    while (i <= 95 && (millis() - start_millis < 15000) ) 
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
          buf[3] = i;
          
          if (i >= 95) buf[3] = 0;       // set speedometer back to 0
          
          if (i % 5 == 0) CAN.sendMsgBuf (0x7E8, 0, 4, buf);  //  id assigned for OBD-II response
          ++i;
        }
      }
    }

    // set up pin to write PWM signal to drive gauge
    pinMode(MOTOR_TEMPERATURE_PIN, OUTPUT);
    pinMode(CONTROLLER_TEMPERATURE_PIN, OUTPUT);
    pinMode(MOTOR_AMPERAGE_PIN, OUTPUT);
 
    // let the gauges climb to highest value
    for (int i = 0; i <= 255; ++i)
    {
      analogWrite(MOTOR_TEMPERATURE_PIN, i);  
      analogWrite(CONTROLLER_TEMPERATURE_PIN, 255-i); 
      analogWrite(MOTOR_AMPERAGE_PIN, 255-i); 
      
      delay (5);
    }

    
    // make temperature gauge come backto lowest value
    for (int i = 255; i >= 0; --i)
    {
      analogWrite(MOTOR_TEMPERATURE_PIN, i);   
      analogWrite(CONTROLLER_TEMPERATURE_PIN, 255-i);   
      analogWrite(MOTOR_AMPERAGE_PIN, 255-i); 

      delay (5);
    }
}

int map255(int value, float value_min, float value_max) 
{
   float value_range = value_max - value_min;
   float percent = ((float) (value - value_min)) / value_range;

   if (percent < 0.0) return 0;
   if (percent > 1.0) return 255;
   return ((int) (percent * 255.0));
}

void drive_electric_gauges()
{
  int PWM_out_level = 0;
  int max_motor_temperature = max(motor1_temperature, motor2_temperature);
  int max_controller_temperature = max(controller1_temperature, controller2_temperature);
  int i = 0;

  // analogWrite accepts values of 0 - 255; need to scale appropriately for all 3 gauges
  
  // output maximum motor temperature
  PWM_out_level = map255 (max_motor_temperature, MIN_MOTOR_TEMPERATURE, MAX_MOTOR_TEMPERATURE);
  analogWrite(MOTOR_TEMPERATURE_PIN, PWM_out_level);
  
  // output maximum contoller temperature
  PWM_out_level = map255 (max_controller_temperature, MIN_CONTROLLER_TEMPERATURE, MAX_CONTROLLER_TEMPERATURE);    
  analogWrite(CONTROLLER_TEMPERATURE_PIN, 255 - PWM_out_level);


  // output maximum motor current
  // one controller not sending amperage so let's display double of the one that works
  // alternative is to use bus current which seems to be working
  int motor_current = 2 * max(motor1_current, motor2_current);
  PWM_out_level = map255 (motor_current, MIN_MOTOR_AMPERAGE, MAX_MOTOR_AMPERAGE);            
  analogWrite(MOTOR_AMPERAGE_PIN, 255 - PWM_out_level);

}
 
void loop() {
    unsigned char len = 0;
    int           i;
    int           temperature_range;
    int           can_ID = 0x00;


    if (CAN_MSGAVAIL == CAN.checkReceive()) {       
        CAN.readMsgBuf(&len, buf);    
        can_ID = CAN.getCanId();

        if (can_ID == 0x400)         
        {        
           // extract kilometers per hour from first two bytes
           // just store speed for now; send message on bus when request from speedbox (0x7E0) appears
           vehicle_speed_kmph = buf[0] + (buf[1] << 8);
           vehicle_speed_kmph /= 10.0;  // speed from controller scaled up by 10

           // extract current through motor
           motor1_current = (buf[2] + (buf[3] << 8)) / 10.0;

           // extract bus current through motor
           motor1_bus_current = (buf[4] + (buf[5] << 8)) / 10.0;
            
           drive_electric_gauges();

       }
       else if (can_ID == 0x402)
       {
        
           // extract controller temperature; 0 - 255 maps to 40 -295 and  convert temperature from Celsius to Fahrenheit
           controller1_temperature = buf[0];
           controller1_temperature -= 40.0;                                 
           controller1_temperature = (int) (((float) (controller1_temperature) * 1.8)) + 32;        


           // extract motor temperature; 0 - 255 maps to 40 - 295 and convert from Celsius to Fahrenheit
           motor1_temperature = buf[1];
           motor1_temperature -= 40.0;                                 
           motor1_temperature = (int)((float) (motor1_temperature) * 1.8) + 32;         
       }
       else if (can_ID == 0x401)
       {
           // extract current through motor
           motor2_current = (buf[2] + (buf[3] << 8)) / 10.0;

           // extract bus current through motor
           motor2_bus_current = (buf[4] + (buf[5] << 8)) / 10.0;
           
           drive_electric_gauges();
       }
       else if (can_ID == 0x403)
       {
        
           // extract controller temperature; 0 - 255 maps to 40 -295 and  convert temperature from Celsius to Fahrenheit
           controller2_temperature = buf[0];
           controller2_temperature -= 40.0;                                 
           controller2_temperature = (int) (((float) (controller2_temperature) * 1.8)) + 32;        


           // extract motor temperature; 0 - 255 maps to 40 - 295 and convert from Celsius to Fahrenheit
           motor2_temperature = buf[1];
           motor2_temperature -= 40.0;                                 
           motor2_temperature = (int)((float) (motor2_temperature) * 1.8) + 32;         
       }
       else if (can_ID == 0x7E0)
       {
          // send out the most recently recorded speed on CAN bus       
          can_ID = 0x7E8;  // specific OBD-II address for responses; used to send info to speedbox
          buf[0] = 0x3;  // number of additional data bytes
          buf[1] = 0x41; // show current data response
          buf[2] = 0xD;  // PID for vehicle speed in kmph
          buf[3] = vehicle_speed_kmph;

          CAN.sendMsgBuf (can_ID, 0, 4, buf);  //  id assigned for OBD-II response

       }
       else // unrecognized CAN bus message that got through filters
       {
       }
    }
    else   // no message on bus right now
    {     
      
    }
}
