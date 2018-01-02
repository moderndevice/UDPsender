

#define waitTime (30)   // Set this for delay period in ms. As written, this is the minimum now

#include <SPI.h>         
#include <Ethernet.h>
#include <EthernetUdp.h>  // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <Wire.h>

#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>


// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

#define NO_INTERRUPT 1
#define I2C_TIMEOUT 200

#define SDA_PORT PORTD
#define SDA_PIN 7
#define SCL_PORT PORTD
#define SCL_PIN 6

#include <avr/io.h>
#include <SoftI2CMaster.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define LIS3DH_DEFAULT_ADDRESS  (0x18)    // if SDO/SA0 is 3V, its 0x19
/*=========================================================================*/

#define LIS3DH_REG_STATUS1       0x07
#define LIS3DH_REG_OUTADC1_L     0x08
#define LIS3DH_REG_OUTADC1_H     0x09
#define LIS3DH_REG_OUTADC2_L     0x0A
#define LIS3DH_REG_OUTADC2_H     0x0B
#define LIS3DH_REG_OUTADC3_L     0x0C
#define LIS3DH_REG_OUTADC3_H     0x0D
#define LIS3DH_REG_INTCOUNT      0x0E
#define LIS3DH_REG_WHOAMI        0x0F
#define LIS3DH_REG_TEMPCFG       0x1F
#define LIS3DH_REG_CTRL1         0x20
#define LIS3DH_REG_CTRL2         0x21
#define LIS3DH_REG_CTRL3         0x22
#define LIS3DH_REG_CTRL4         0x23
#define LIS3DH_REG_CTRL5         0x24
#define LIS3DH_REG_CTRL6         0x25
#define LIS3DH_REG_REFERENCE     0x26
#define LIS3DH_REG_STATUS2       0x27
#define LIS3DH_REG_OUT_X_L       0x28
#define LIS3DH_REG_OUT_X_H       0x29
#define LIS3DH_REG_OUT_Y_L       0x2A
#define LIS3DH_REG_OUT_Y_H       0x2B
#define LIS3DH_REG_OUT_Z_L       0x2C
#define LIS3DH_REG_OUT_Z_H       0x2D
#define LIS3DH_REG_FIFOCTRL      0x2E
#define LIS3DH_REG_FIFOSRC       0x2F
#define LIS3DH_REG_INT1CFG       0x30
#define LIS3DH_REG_INT1SRC       0x31
#define LIS3DH_REG_INT1THS       0x32
#define LIS3DH_REG_INT1DUR       0x33
#define LIS3DH_REG_CLICKCFG      0x38
#define LIS3DH_REG_CLICKSRC      0x39
#define LIS3DH_REG_CLICKTHS      0x3A
#define LIS3DH_REG_TIMELIMIT     0x3B
#define LIS3DH_REG_TIMELATENCY   0x3C
#define LIS3DH_REG_TIMEWINDOW    0x3D
#define LIS3DH_REG_ACTTHS        0x3E
#define LIS3DH_REG_ACTDUR        0x3F

#define LIS3DH_ADDR 0x18 << 1

int16_t x, y, z;
float x_g, y_g, z_g;

// Use hardware I2C for sensor 1 and 2
Adafruit_LIS3DH lis1 = Adafruit_LIS3DH();
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH();

boolean sens1present = 0, sens2present = 0, sens3present = 0;

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

///////////////////////////////
// Change these values for local configuration

byte mac[] = { 0x90, 0xA2, 0xDA, 0x11, 0x0A, 0xBF };  // Mac address must match Ethernet Arduino, marked on bottom
IPAddress ip(192, 168, 254, 178);                     // IP address is hardcoded; change if on a different subnet or conflict  
unsigned int localPort = 80;                          // local port to listen on: 
                                                      // our local config blocked most UDP so we chose 80 (usually http)
/////////////////////////////////


// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];            //buffer to hold incoming packet,
char  ReplyBuffer[20] = "--------------------";       // a string to send back

String accelValue = "";
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
IPAddress remIP;
int remPort;
boolean start = false;

void setup() {
  Serial.begin(9600);
  // start the Ethernet and UDP:
  

  if (! lis1.begin(0x18)) {   // lis1
    Serial.println("Sensor1 not present.");
  }
  else {
    sens1present = true;
    Serial.println("Sensor1 found!");
  }

  if (! lis2.begin(0x19)) {   // lis2
    Serial.println("Sensor2 not present.");

  }
  else {
    sens2present = true;
    Serial.println("Sensor2 found!");
  }

  if (sens1present) {

    lis1.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!

    Serial.print("Sensor1 Range = ");
    Serial.print(2 << lis1.getRange());
    Serial.println("G");
  }

  if (sens2present) {
    lis2.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!

    Serial.print("Sensor2 Range = ");
    Serial.print(2 << lis2.getRange());
    Serial.println("G");
  }

 if (!LIS3DH_begin(LIS3DH_ADDR)) {
    Serial.println(F("LIS3DH INIT ERROR"));
   
  }
 setRange(LIS3DH_RANGE_16_G);  // or use LIS3DH_RANGE_8_G, LIS3DH_RANGE_4_G, e
 
  
  Serial.println("Set up ethernet... ");
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  printIPAddress();
  
  Serial.println("Waiting for any UDP message on port 80 to start... ");
}

void printIPAddress()
{
  Serial.print("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(".");
  }

  Serial.println();
}

void loop() {
 
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  
  if ((packetSize) && !(start)) {
    // Received a message over UDP; save the remoteIP and port
    start = true;
    Serial.print("Received start from ");
    remIP = Udp.remoteIP();
    for (int i = 0; i < 4; i++) {
      Serial.print(remIP[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    remPort = Udp.remotePort();
    Serial.print(", port ");
    Serial.println(remPort);
  }
  delay(10);

  if (start) {

    // Read sensor 1
    
    lis1.read();      // get X Y and Z data at once
    accelValue = "1X:";
    accelValue +=  lis1.x;
    accelValue +=  "Y:";
    accelValue += lis1.y;
    accelValue += "Z:";
    accelValue += lis1.z;

    // Convert the Arduino string to a char array
    accelValue.toCharArray(ReplyBuffer, 20);
    Serial.println(ReplyBuffer);

    // Send it
    Udp.beginPacket(remIP, remPort);
    Udp.write(ReplyBuffer);
    Udp.endPacket();
    delay(10);


    // Repeat for sensor 2
    lis2.read();      // get X Y and Z data at once

    accelValue = "2X:";
    accelValue +=  lis2.x;
    accelValue +=  "Y:";
    accelValue += lis2.y;
    accelValue += "Z:";
    accelValue += lis2.z;
    accelValue.toCharArray(ReplyBuffer, 20);
    Serial.println(ReplyBuffer);
    Udp.beginPacket(remIP, remPort);
    Udp.write(ReplyBuffer);
    Udp.endPacket();
    delay(10);

    LIS3DH_read();     // get X Y and Z data at once
    accelValue = "3X:";
    accelValue +=  x;
    accelValue +=  "Y:";
    accelValue += y;
    accelValue += "Z:";
    accelValue += z;
    accelValue.toCharArray(ReplyBuffer, 20);
    Serial.println(ReplyBuffer);
    Udp.beginPacket(remIP, remPort);
    Udp.write(ReplyBuffer);
    Udp.endPacket();
    delay(10);

    delay(waitTime-30);
  }
}

bool LIS3DH_begin(uint8_t i2caddr) {

  if (!i2c_init()) { // checks I2C lines - I2C
    Serial.println(F("I2C bus lockup or that the lines are not pulled up "));
    return false;
  }


  /* Check connection */
  uint8_t deviceid = readRegister8(LIS3DH_REG_WHOAMI);

  if (deviceid != 0x33)
  {
    Serial.println ("No LIS3DH detected");
    Serial.print ("deviceID = ");
    Serial.println (deviceid);
    Serial.println(deviceid, HEX);
    return false;
  }
  else {
    Serial.print ("deviceID = ");
    Serial.println(deviceid, HEX);
    Serial.println ("LIS3DH detected!");
  }

  // enable all axes, normal mode
  writeRegister8(LIS3DH_REG_CTRL1, 0x07);

  // 100Hz rate
  setDataRate(LIS3DH_DATARATE_100_HZ);

  // High res & BDU enabled
  writeRegister8(LIS3DH_REG_CTRL4, 0x88);

  // DRDY on INT1
  writeRegister8(LIS3DH_REG_CTRL3, 0x10);

  // Turn on orientation config - not implemented
  // writeRegister8(LIS3DH_REG_PL_CFG, 0x40);

  // enable adcs - not implemented
  // writeRegister8(LIS3DH_REG_TEMPCFG, 0x80);

  /*  debug all registers
    for (uint8_t i=0; i<0x30; i++) {
    Serial.print("$");
    Serial.print(i, HEX); Serial.print(" = 0x");
    Serial.println(readRegister8(i), HEX);
    }
  */

  return true;
}


uint8_t readRegister8(uint8_t reg) {
  uint8_t value;

  if (!i2c_start(LIS3DH_ADDR | I2C_WRITE)) {
    Serial.println("readRegister8 start-write failed");
    return false;
  }

  if (!i2c_write((uint8_t)reg)) {
    Serial.println("readRegister8 write failed");
    return false;
  }

  i2c_stop();

  if (!i2c_rep_start(LIS3DH_ADDR | I2C_READ)) {
    Serial.println("readRegister8 start-read failed");
    return false;
  }

  value =  i2c_read(true); // read one byte, true sends NAK, terminates read
  return value;
}

void writeRegister8(uint8_t reg, uint8_t value) {
  //  I2Cinterface->beginTransmission((uint8_t)_i2caddr);
  if (!i2c_rep_start(LIS3DH_ADDR | I2C_WRITE)) {
    Serial.println("writeRegister8 start-write failed");
    return false;
  }

  // I2Cinterface->write((uint8_t)reg);
  if (!i2c_write((uint8_t)reg)) {
    Serial.println("writeRegister8 write failed");
    return false;
  }

  // I2Cinterface->write((uint8_t)value);
  i2c_write((uint8_t)value);

  //I2Cinterface->endTransmission();
  i2c_stop();

}

void setDataRate(lis3dh_dataRate_t dataRate)
{
  uint8_t ctl1 = readRegister8(LIS3DH_REG_CTRL1);
  ctl1 &= ~(0xF0); // mask off bits
  ctl1 |= (dataRate << 4);
  writeRegister8(LIS3DH_REG_CTRL1, ctl1);
}

void LIS3DH_read(void) {
  // read x y z at once

  if (!i2c_rep_start(LIS3DH_ADDR | I2C_WRITE)) {
    Serial.println("LIS3DH_read start-write failed");
    return false;
  }

  i2c_write(LIS3DH_REG_OUT_X_L | 0x80); // 0x80 for autoincrement
  i2c_stop();
  if (!i2c_rep_start(LIS3DH_ADDR | I2C_READ)) {
    Serial.println("LIS3DH_read start-read failed");
    return false;
  }

  x = i2c_read(false);
  x |= ((uint16_t)i2c_read(false)) << 8;
  y = i2c_read(false);
  y |= ((uint16_t)i2c_read(false)) << 8;
  z = i2c_read(false);
  z |= ((uint16_t)i2c_read(true)) << 8;  // last read

  i2c_stop();

  /*  // not used in my sketch so opted for faster execution
    uint8_t range = getRange();
    uint16_t divider = 1;
    if (range == LIS3DH_RANGE_16_G) divider = 1365; // different sensitivity at 16g
    if (range == LIS3DH_RANGE_8_G) divider = 4096;
    if (range == LIS3DH_RANGE_4_G) divider = 8190;
    if (range == LIS3DH_RANGE_2_G) divider = 16380;

    x_g = (float)x / divider;
    y_g = (float)y / divider;
    z_g = (float)z / divider;
  */
}

uint8_t getRange(void)
{
  /* Read the data format register to preserve bits */
  return (uint8_t)((readRegister8(LIS3DH_REG_CTRL4) >> 4) & 0x03);
}

void setRange(lis3dh_range_t range)
{
  uint8_t r = readRegister8(LIS3DH_REG_CTRL4);
  r &= ~(0x30);
  r |= range << 4;
  writeRegister8(LIS3DH_REG_CTRL4, r);
}
