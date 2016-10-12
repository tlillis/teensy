#define SERIAL_PTH Serial1
#define SERIAL_NAV Serial3
#define SERIAL_PROBE Serial2



void setup() {
  Serial.begin(115200);
SERIAL_PTH.begin(19200, SERIAL_8N1); //ncar pth
SERIAL_NAV.begin(115200); //vector nav
SERIAL_PROBE.begin(115200); //aero probe


}

void loop() {
  if (SERIAL_PTH.available()) {
    Serial.print("GOT PTH\n");
  }
  if (SERIAL_NAV.available()) {
    Serial.print("GOT NAV\n");
  }
  if (SERIAL_PROBE.available()) {
    Serial.print("GOT PROBE\n");
  }
  delay(1000);
}
