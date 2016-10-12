#include <Wire.h>
#include "../mavlink/include/airserver/mavlink.h"

#define SERIAL_PROBE Serial2

mavlink_message_t msg_probe;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

uint8_t system_type = 0;
uint8_t autopilot_type = 0;

int len = 0;

unsigned long int time_device = 0; 
unsigned long int date = 0;
double velocity = 0;
double aoa = 0; 
double aos = 0; 
double pa = 0; 
double ps = 0; 
double pt = 0; 
double tc = 0;
char checksum = 0;

char checker = 0;

char string_probe[1024];
int stringIndex_probe = 0;
char* ptr_probe;

int fieldNo_probe = 0;
char field_probe[16];
int fieldIndex_probe = 0;

uint8_t incomingByte_probe;
unsigned long time_probe;

//  GPS pps variables
elapsedMillis timer;
unsigned long time_decimal;

void setup() {

  Serial.begin(9600);
  SERIAL_PROBE.begin(115200);
  
  //Setup for GPS pulse
  pinMode(12, INPUT); //Want to RX interrupt signal on this pin
  interrupts();  // enable Interrupts
  attachInterrupt(12, syncTime, RISING);
}

void loop() {
  
  if(probe_read()) {
    
    Serial.print("time: ");
    Serial.print(time_device);
    Serial.print(" date: ");
    Serial.print(date);
    Serial.print(" vel: ");
    Serial.print(velocity);
    Serial.print(" aoa: ");
    Serial.print(aoa);
    Serial.print(" aos: ");
    Serial.print(aos);
    Serial.print(" pa: ");
    Serial.print(pa);
    Serial.print(" ps: ");
    Serial.print(ps);
    Serial.print(" pt: ");
    Serial.print(pt);
    Serial.print(" tc: ");
    Serial.print(tc);
    Serial.print(" check: ");
    Serial.print(checksum);
    Serial.println("");
   
  }  
  
  /**
  if(probe_read()) {
    mavlink_msg_aeroprobe_pack(system_type,
                               autopilot_type, 
                               &msg_probe, 
                               time_probe, 
                               time_device, 
                               date, 
                               velocity, 
                               aoa, 
                               aos, 
                               pa, 
                               ps, 
                               pt, 
                               tc);
    len = mavlink_msg_to_send_buffer(buf, &msg_probe);
    Serial.write(buf, len);
    **/
  //}
}

void syncTime(void)
{
  time_decimal = timer % 1000;
  if (time_decimal < 500) {
    timer -= time_decimal; 
  } else {
    timer += (1000 - time_decimal);
  }
}

int probe_read() { //Handle probe serial data
  if (SERIAL_PROBE.available() > 0) {
    incomingByte_probe = SERIAL_PROBE.read();
    
    if(fieldNo_probe < 9) {
      string_probe[stringIndex_probe] = incomingByte_probe;
    }

    //input tab-delimated fields
    switch (incomingByte_probe) {
      case ',': //end of field
        switch (fieldNo_probe) { //Depending on the field give to different variable
          case 0:
            time_device = atol(field_probe);
            cli();
            time_probe = timer;
            sei();
            break;
          case 1:
            date = atoll(field_probe);
            break;
          case 2:
            velocity = atof(field_probe);
            break;
          case 3:
            aoa = atof(field_probe);
            if(field_probe[0] == '-') aoa *= -1;
            break;
          case 4:
            aos = atof(field_probe);
            if(field_probe[0] == '-') aos *= -1;
            break;
          case 5:
            pa = atof(field_probe);
            if(field_probe[0] == '-') pa *= -1;
            break;
          case 6:
            ps = atof(field_probe);
            break;
          case 7:
            pt = atof(field_probe);
            break;
          case 8:
            tc = atof(field_probe);
            if(field_probe[0] == '-') tc *= -1;
            break;
        }
        
        fieldNo_probe++;
        memset(field_probe,0,strlen(field_probe));
        fieldIndex_probe = 0;
        field_probe[0] = '\0'; //empty for next variable read
        
        break;

      case '\n': //Handle end of input line

        checksum = field_probe[0]; //handle last variable
        
        checker = 0;
        for(ptr_probe = string_probe; *ptr_probe; ptr_probe++) { //Calculate checksum
          checker ^= *ptr_probe;
        }
        //reset everything for next line
        fieldNo_probe = 0;
        Serial.println(string_probe);
        memset(field_probe,0,strlen(field_probe));
        memset(string_probe,0,strlen(string_probe));
        stringIndex_probe = 0;
        fieldIndex_probe = 0;
        
        return 1;

      case 0://Handle NULL char
        break;

      default: //append lastest byte to field
        field_probe[fieldIndex_probe] = incomingByte_probe;
        fieldIndex_probe++;
        
    }
  }
  return 0;
}
