#include <Wire.h>
#include "../mavlink/include/airserver/mavlink.h"

#define SERIAL_PTH Serial1
#define SERIAL_NAV Serial3
#define SERIAL_PROBE Serial2

#define LED_PIN 13
bool blinkState = false;


//  mavlink variables
mavlink_message_t msg_pth;
mavlink_message_t msg_vector;
mavlink_message_t msg_probe;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

int system_type = 10;
int autopilot_type = 10;

uint16_t len = 0;

//  pth variables
int sampleNo_pth = 0;
float pressure = 0;
float externalTemp = 0;
float rh1 = 0;
float rh2 = 0;
float internalTemp = 0;

int fieldNo_pth = 0;
char field_pth[16];
int fieldIndex_pth = 0;

uint8_t incomingByte_pth;
unsigned long time_pth;

//  VectorNav variables
unsigned long time_vector;
uint8_t incomingByte_vector;

//uint64_t timeStartup;
unsigned long long int timeStartup;
unsigned long long int timeGPS;
unsigned long long int timeSyncln;
float roll;
float pitch;
float yaw;
float qtn[4];
float rate[3];
double lat;
double lon;
double alt;
float vel[3];
float accel[3];
float imu[6];
float magPres[5];
float deltaThetaVel[7];
uint16_t insStatus;
uint32_t syncLnCnt;
unsigned long long int timeGpsPps;
int8_t year;
uint8_t  month;
uint8_t  day;
uint8_t  hour;
uint8_t  minute;
uint8_t  second;
uint16_t  msecond;
                                             //0xFA 0x03 0xFF 0x7F 0x40 0x00
const uint64_t EXPECTED_HEADER = 274895078375424; //11111010 00000011 11111111 01111111 01000000 00000000


const uint8_t MESSAGE_SIZE = 218;
const uint8_t HEADER_SIZE = 6;

char message[MESSAGE_SIZE];
char header[HEADER_SIZE];
char test[8];
uint8_t byte_i = 0;

//  aeroprobe variables
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

char string[1024];
int stringIndex = 0;
char* ptr;

int fieldNo_probe = 0;
char field_probe[16];
int fieldIndex_probe = 0;

uint8_t incomingByte_probe;
unsigned long time_probe;

//  GPS pps variables
elapsedMillis timer;
unsigned long time_decimal;

void setup() {

  Serial.begin(115200);
  SERIAL_PTH.begin(19200, SERIAL_8N1); //ncar pth
  SERIAL_NAV.begin(115200); //vector nav
  SERIAL_PROBE.begin(115200); //aero probe

  //Setup for GPS pulse
  pinMode(12, INPUT); //Want to RX interrupt signal on this pin
  interrupts();  // enable Interrupts
  attachInterrupt(12, syncTime, RISING);

  pinMode(LED_PIN, OUTPUT);
  
}

void loop() {

    if(pth_read() && Serial.dtr()) {
      mavlink_msg_ncar_pth_pack(system_type, 
                                autopilot_type, 
                                &msg_pth, 
                                time_pth, 
                                sampleNo_pth, 
                                pressure, 
                                externalTemp, 
                                rh1, 
                                rh2, 
                                internalTemp);
      len = mavlink_msg_to_send_buffer(buf, &msg_pth);
      Serial.write(buf, len);
    }

    if(vector_read() && Serial.dtr()) {
      
      mavlink_msg_vector_nav_pack(system_type, 
                                  autopilot_type, 
                                  &msg_vector, 
                                  time_vector, 
                                  timeStartup, 
                                  timeGPS, 
                                  timeSyncln, 
                                  yaw, 
                                  pitch, 
                                  roll,
                                  qtn[0],
                                  qtn[1],
                                  qtn[2],
                                  qtn[3],
                                  rate[0],
                                  rate[1],
                                  rate[2],
                                  lat,
                                  lon,
                                  alt,
                                  vel[0],
                                  vel[1],
                                  vel[2],
                                  accel[0],
                                  accel[1],
                                  accel[2],
                                  imu[0],
                                  imu[1],
                                  imu[2],
                                  imu[3],
                                  imu[4],
                                  imu[5],
                                  magPres[0],
                                  magPres[1],
                                  magPres[2],
                                  magPres[3],
                                  magPres[4],
                                  deltaThetaVel[0],
                                  deltaThetaVel[1],
                                  deltaThetaVel[2],
                                  deltaThetaVel[3],
                                  deltaThetaVel[4],
                                  deltaThetaVel[5],
                                  deltaThetaVel[6],
                                  insStatus,
                                  syncLnCnt,
                                  timeGpsPps);
      len = mavlink_msg_to_send_buffer(buf, &msg_vector);
      Serial.write(buf, len);
      
    }
   
    if(probe_read() && Serial.dtr()) {
      
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
    }

  // blink LED to indicate activity
  
  if(Serial.dtr()) {
    blinkState = true;
  } else {
    blinkState = false;
  }
  digitalWrite(LED_PIN, blinkState);
  
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


int pth_read() { //Handle ncar pth serial data
  if (SERIAL_PTH.available() > 0) {
    incomingByte_pth = SERIAL_PTH.read();
    //input tab-delimated fields
    switch (incomingByte_pth) {
      case '\t': //end of field
        switch (fieldNo_pth) { //Depending on the field give to different variable
          case 0:
            sampleNo_pth = atoll(field_pth);
            cli();
            time_pth = timer;
            sei();
            break;
          case 1:
            pressure = atof(field_pth);
            break;
          case 2:
            externalTemp = atof(field_pth);
            break;
          case 3:
            rh1 = atof(field_pth);
            break;
          case 4:
            rh2 = atof(field_pth);
            break;
        }
        fieldNo_pth++;
        fieldIndex_pth = 0;
        field_pth[0] = '\0'; //empty for next variable read
        break;

      case '\n': //Handle end of input line
        internalTemp = atof(field_pth); //handle last variable

        //reset everything for next line
        fieldNo_pth = 0;
        field_pth[0] = '\0';
        fieldIndex_pth = 0;
        return 1;

      case 0://Handle NULL char
        break;

      default: //append lastest byte to field
        field_pth[fieldIndex_pth] = incomingByte_pth;
        fieldIndex_pth++;
    }
  }
  return 0;
}

int vector_read() {
  //Check to see if 4 next bytes are header. If so then start writing to message for parsing
  if(*(uint64_t*)header != EXPECTED_HEADER) {
    
    if(SERIAL_NAV.available() > 0) {
      if(SERIAL_NAV.read() != 0xFA) {
        return 0;
      } else {
        cli();
        time_vector = timer;
        sei();
        int i = 1;
        header[0] = 0xFA;
        for (i;i < HEADER_SIZE; i++) { // Read 4 bytes from serial to check agaisnt expected header
          while(SERIAL_NAV.available() <= 0) {
            delay(1);
          }
          if (SERIAL_NAV.available() > 0) {
            header[i] = SERIAL_NAV.read();
          }
        }
      }
    } 
    return 0;
    
  } else {
    if (SERIAL_NAV.available() > 0) { // Add bytes following header to message for parsing
        message[byte_i] = SERIAL_NAV.read();
        byte_i++;
    }
    if (byte_i == MESSAGE_SIZE) { //Once the message reaches the end
      memset(header,0,strlen(header)); //reset check for header
      byte_i = 0;
      return vector_parse();  
    }
    return 0;
  } 
  return 0;
}

int vector_parse() {
  timeStartup = *(uint64_t*)message;
  timeGPS = *(uint64_t*)(message + 8);
  timeSyncln = *(uint64_t*)(message + 16);
  yaw = *(float*)(message+24);
  pitch = *(float*)(message+28);
  roll = *(float*)(message+32);
  qtn[0] = *(float*)(message+36);
  qtn[1] = *(float*)(message+40);
  qtn[2] = *(float*)(message+44);
  qtn[3] = *(float*)(message+48);
  rate[0] = *(float*)(message+52);
  rate[1] = *(float*)(message+56);
  rate[2] = *(float*)(message+60);
  lat = *(double*)(message+64);
  lon = *(double*)(message+72);
  alt = *(double*)(message+80);
  vel[0] = *(float*)(message+88);
  vel[1] = *(float*)(message+92);
  vel[2] = *(float*)(message+96);
  accel[0] = *(float*)(message+100);
  accel[1] = *(float*)(message+104);
  accel[2] = *(float*)(message+108);
  imu[0] = *(float*)(message+112);
  imu[1] = *(float*)(message+116);
  imu[2] = *(float*)(message+120);
  imu[3] = *(float*)(message+124);
  imu[4] = *(float*)(message+128);
  imu[5] = *(float*)(message+132);
  magPres[0] = *(float*)(message+136);
  magPres[1] = *(float*)(message+140);
  magPres[2] = *(float*)(message+144);
  magPres[3] = *(float*)(message+148);
  magPres[4] = *(float*)(message+152);
  deltaThetaVel[0] = *(float*)(message+156);
  deltaThetaVel[1] = *(float*)(message+160);
  deltaThetaVel[2] = *(float*)(message+164);
  deltaThetaVel[3] = *(float*)(message+168);
  deltaThetaVel[4] = *(float*)(message+172);
  deltaThetaVel[5] = *(float*)(message+176);
  deltaThetaVel[6] = *(float*)(message+180);
  insStatus = *(uint16_t*)(message+184);
  syncLnCnt = *(uint32_t*)(message+186);
  timeGpsPps = *(unsigned long int*)(message+190);
  year = *(int8_t*)(message+191);
  month = *(uint8_t*)(message+192);
  day = *(*uint8_t)(message+193);
  hour = *(*uint8_t)(message+194);
  minute = *(*uint8_t)(message+195);
  second = *(*uint8_t)(message+196);
  msecond = *(*uint16_t)(message+198);
  return 1;
}

int probe_read() { //Handle probe serial data
  if (SERIAL_PROBE.available() > 0) {
    incomingByte_probe = SERIAL_PROBE.read();
    
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
        /** CHECKSUM
        checksum = field[0]; //handle last variable
        
        checker = 0;
        for(ptr = string; *ptr; ptr++) { //Calculate checksum
          checker ^= *ptr;
        }
        **/
        //reset everything for next line
        fieldNo_probe = 0;
        memset(field_probe,0,strlen(field_probe));
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
