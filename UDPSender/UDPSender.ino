/* Dependencies:

     Adafruit LIS3DH
     Adafruit Unified Sensor

   Created By:

     Paul Badger 2017

   Description:

     Reads an LIS3DH accelerometer and sends UDP data out to 255.255.255.255:35791

     One datagram = one sample, data is text, formatted as:

       id:timestamp:x:y:z

     Or, if the sensor is missing:

       id:timestamp

     Where id is an integer, timestamp is an integer number of milliseconds, and x y z are
     decimals in g units. If sensor is missing, x y and z won't be sent.
*/

// determines unit id, ip address, and mac address; valid range 1-154. use 100+ for spares with switches.
// current spare unit is 100. second spare should be 101.
// In the case of spare, UNIT_ID will be decoupled from SENSOR_DEVICE_ID eg mac and IP will not necessarily be related to UNIT_ID
// that is sent with data
#define SENSOR_DEVICE_ID 1  // This should change with particular box - especially is not Unit_ID selection switch.
//                          // Reworked units (new connectors) all have unit selection switch        

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
//#include <avr/io.h>

int16_t x, y, z;
float x_g, y_g, z_g;

// Use hardware I2C for sensor 1 and 2
Adafruit_LIS3DH lis1 = Adafruit_LIS3DH();

boolean sens1present = 0;

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
#define Serial SerialUSB
#endif

///////////////////////////////
// Change these values for local configuration
// Change SENSOR_DEVICE_ID above to set mac, ip, and initial unit id (overwritten for units with switch)
// Spares take their UNIT_ID from a slide switch

byte UNIT_ID = SENSOR_DEVICE_ID;  // set this for various pieces of exercise equipment, 1, 2, 3 currently valid
byte mac[] = { 0x00, 0x01, 0x01, 0xAB, 0xCD, SENSOR_DEVICE_ID }; // 00:01:01:AB:CD:nn Mac address     - Spares n = 100+
IPAddress myIP(10, 0, 2, 100 + SENSOR_DEVICE_ID ); // 10.0.2.[100+n] (e.g. .101, .102, .103). - Spares n = 100+

unsigned int localPort = 80;   // not used - local port to listen on

// our local config blocked most UDP so we chose 80 (usually http)
/////////////////////////////////


// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];            //buffer to hold incoming packet,

String accelValue = "";
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

////////////////////////////////
// remote port configuration

IPAddress remIP(255, 255, 255, 255); // universal broadcast
int remPort = 35791;                 // agreed on email with Jason C

////////////////////////////////

boolean sensorStart = false;

void setup() {

  Serial.begin(57600);
  delay(1000);

  pinMode(3, INPUT_PULLUP);  // for three way switch on the spare
  pinMode(4, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(5, OUTPUT);

  if (!digitalRead(3)) UNIT_ID = 3; // if no switch present - no change in UNIT_ID
  else if (!digitalRead(4)) UNIT_ID = 2;
  else if (!digitalRead(7)) UNIT_ID = 1;

  IPAddress temp(10, 0, 2, 100 + UNIT_ID);
  mac[5] = UNIT_ID;  // set the last byte of mac according to the unit
  myIP = temp;

  Serial.print("ID = ");
  Serial.println(UNIT_ID);

  if (!lis1.begin(0x18)) {   // lis1
    Serial.println("Sensor not present.");
  }
  else {
    sens1present = true;
    sensorStart = true;
    Serial.println("Sensor found!");
  }

  lis1.setRange(LIS3DH_RANGE_4_G);  // or use LIS3DH_RANGE_8_G, LIS3DH_RANGE_16_G, e
#define SENSOR_RANGE_IN_G 4.0      // set this to match the above!

  // start the Ethernet and UDP:
  Serial.println("Set up ethernet... ");

  // start the Ethernet connection with DHCP:
  /* if (Ethernet.begin(mac) == 0) {
     // no point in carrying on, so do nothing forevermore:
     while (1) {
       Serial.println("Failed to configure Ethernet using DHCP");
       delay(1000);
     }
    }
  */

  Ethernet.begin(mac, myIP);
  Udp.begin(localPort);
  printIPAddress();
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


  if (sensorStart) {  // sensor is present, read sensor
    lis1.read();      // get X Y and Z data at once
    accelValue = UNIT_ID;
    accelValue += ":";
    accelValue +=  millis();
    accelValue += ":";
    accelValue += String(SENSOR_RANGE_IN_G * (double)lis1.x / 32767.0, 6);
    accelValue += ":";
    accelValue += String(SENSOR_RANGE_IN_G * (double)lis1.y / 32767.0, 6);
    accelValue += ":";
    accelValue += String(SENSOR_RANGE_IN_G * (double)lis1.z / 32767.0, 6);
  }
  else {   // no sensor found - send dummy −32,767's
    accelValue = UNIT_ID;
    accelValue += ":";
    accelValue += millis();
  }

  Serial.println(accelValue);

  // Send it
  Udp.beginPacket(remIP, remPort);
  Udp.write(accelValue.c_str());
  Udp.endPacket();
  delay(28);

}



