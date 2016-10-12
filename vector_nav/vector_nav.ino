//  VectorNav variables
unsigned long time_vector;
uint8_t incomingByte_vector;

//uint64_t timeStartup;
unsigned long int timeStartup;
unsigned long int timeGPS;
unsigned long int timeSyncln;
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
unsigned long int timeGpsPps;
                                             //0xFA 0x01 0xFF 0x7F
const uint32_t EXPECTED_HEADER = 2147418618; //11111010 00000001 11111111 01111111


const uint8_t MESSAGE_SIZE = 218;
const uint8_t HEADER_SIZE = 4;

char message[MESSAGE_SIZE];
char header[HEADER_SIZE];
char test[8];
uint8_t byte_i = 0;



#define LED_PIN 13
bool blinkState = false;

void setup() {
    Serial.begin(9600);
    Serial3.begin(115200);
    //             Write,
    //             reg 75, 
    //             serial 2 output,
    //             800/8 = 100Hz,
    //             group 1,
    //             all fields, 
    //              XX = bypass checksum
    //Serial2.write("$VNWRG,75,2,8,01,255*XX\r\n");
    //Serial2.write("$VNWRG,75,1,32,01,7FFF*XX<CR><LF>");
    //Serial2.write("$VNWRG,75,1,32,00*XX<CR><LF>"); //erase that message type
    pinMode(LED_PIN, OUTPUT);
    
    header[0] = 0xFA;
}

void loop() {
  if(vector_read()) {
    /**
    Serial.print("time ");
    Serial.println(timeStartup);
    Serial.print("roll ");
    Serial.println(roll);
    Serial.print("pitch ");
    Serial.println(pitch);
    Serial.print("yaw ");
    Serial.println(yaw);
    Serial.print("insStatus ");
    Serial.println(insStatus);
    Serial.print("accel x ");
    Serial.println(accel[0]);
    Serial.print("accel y ");
    Serial.println(accel[1]);
    Serial.print("accel z ");
    Serial.println(accel[2]);
    **/
    Serial.print("time ");
    Serial.println((unsigned long int)timeStartup);
    /**
    Serial.print("lat ");
    Serial.println(lat);
    Serial.print("lon ");
    Serial.println(lon);
    Serial.print("alt ");
    Serial.println(alt);
    **/
    /**
    Serial.print("insStatus ");
    Serial.println(insStatus);
    Serial.print("timeGpsPps ");
    Serial.println(timeGpsPps);
    Serial.print("timeGPS ");
    Serial.println(timeGPS);
    Serial.print("timeGPS ");
    Serial.println(timeGPS);
    Serial.print("yaw ");
    Serial.println(yaw);
    **/
  }
  //delay(500);
  
  digitalWrite(LED_PIN, blinkState);
}

int vector_read() {
  //Check to see if 4 next bytes are header. If so then start writing to message for parsing
  if(*(uint32_t*)header != EXPECTED_HEADER) {
    
    if(Serial3.available() > 0) {
      if(Serial3.read() != 0xFA) {
        return 0;
      } else {
        int i = 1;
        header[0] = 0xFA;
        for (i;i < HEADER_SIZE; i++) { // Read 4 bytes from serial to check agaisnt expected header
          while(Serial3.available() <= 0) {
            delay(1);
          }
          if (Serial3.available() > 0) {
            header[i] = Serial3.read();
          }
        }
      }
    } 
    return 0;
    
  } else {
    if (Serial3.available() > 0) { // Add bytes following header to message for parsing
        message[byte_i] = Serial3.read();
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
  
  blinkState = !blinkState;
  
  Serial.print(message[0], HEX);
  Serial.print(" ");
  Serial.print(message[1], HEX);
  Serial.print(" ");
  Serial.print(message[2], HEX);
  Serial.print(" ");
  Serial.print(message[3], HEX);
  Serial.print(" ");
  Serial.print(message[4], HEX);
  Serial.print(" ");
  Serial.print(message[5], HEX);
  Serial.print(" ");
  Serial.print(message[6], HEX);
  Serial.print(" ");
  Serial.println(message[7], HEX);
  
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
  timeGpsPps = *(uint64_t*)(message+190);
  return 1;
}


