/* Reads an LIS3DH accelerometer and sends
    data out over 255.255.255.255
    Paul Badger 2017
    Data is formatted with an ID (one byte)
    Colon character
    Time Stamp (4 bytes)
    Colon character
    X data two bytes
    Colon character
    Y data two bytes
    Colon character
    Z data two bytes
    Colon character
*/

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
#include <avr/io.h>
// #include <SoftI2CMaster.h>



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

byte UNIT_ID = 1;  // set this for various pieces of exercise equipment, 1, 2, 3 currently valid
byte mac[] = { 0x00, 0x01, 0x01, 0xAB, 0xCD, 0xE1 }; // 00:01:01:AB:CD:En Mac address     - Spares n = 4 & 5
IPAddress myIP(192, 168, 2, 1 ); // 192.168.2.n  IP address is hardcoded - Spares n = 4 & 5
unsigned int localPort = 80;   // not used - local port to listen on

// our local config blocked most UDP so we chose 80 (usually http)
/////////////////////////////////


// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];            //buffer to hold incoming packet,
char  ReplyBuffer[30] = "------------------------------";       // a string to send back

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

  pinMode(3, INPUT_PULLUP);  // for three way switch on the spare
  pinMode(4, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(5, OUTPUT);

  if (!digitalRead(3)) UNIT_ID = 3; // if no switch present - no change in UNIT_ID
  else if (!digitalRead(4)) UNIT_ID = 2;
  else if (!digitalRead(7)) UNIT_ID = 1;

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

  lis1.setRange(LIS3DH_RANGE_16_G);  // or use LIS3DH_RANGE_8_G, LIS3DH_RANGE_4_G, e

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
    accelValue +=  lis1.x;
    accelValue +=  ":";
    accelValue += lis1.y;
    accelValue += ":";
    accelValue += lis1.z;
  }
  else {   // no sensor found - send dummy −32,768's
    accelValue = "1:";
    accelValue +=  millis();
    accelValue += ":";
    accelValue +=  -32767;
    accelValue +=  ":";
    accelValue += -32767;
    accelValue += ":";
    accelValue += -32767;
  }

  // Convert the Arduino string to a char array
  accelValue.toCharArray(ReplyBuffer, 30);
  Serial.println(ReplyBuffer);

  // Send it
  Udp.beginPacket(remIP, remPort);
  Udp.write(ReplyBuffer);
  Udp.endPacket();
  delay(28);

}



