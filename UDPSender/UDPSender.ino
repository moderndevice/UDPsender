/* Reads an LIS3DH accelerometer and sends  
 *  data out over 255.255.255.255
 *  Paul Badger 2017 
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

#define SDA_PORT PORTD
#define SDA_PIN 7
#define SCL_PORT PORTD
#define SCL_PIN 6

#include <avr/io.h>
#include <SoftI2CMaster.h>


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

byte mac[] = { 0x90, 0xA2, 0xDA, 0x11, 0x09, 0x62 };  // Mac address must match Ethernet Arduino, marked on bottom
IPAddress myIP(169, 254, 158, 178);                     // IP address is hardcoded; change if on a different subnet or conflict
unsigned int localPort = 80;                          // not used - local port to listen on:
// our local config blocked most UDP so we chose 80 (usually http)
/////////////////////////////////


// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];            //buffer to hold incoming packet,
char  ReplyBuffer[40] = "----------------------------------------";       // a string to send back

String accelValue = "";
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

////////////////////////////////
// remote port configuration

IPAddress remIP(255, 255, 255, 255); // universal broadcast
int remPort = 35791;                 // agreed on email with Jason C

////////////////////////////////

boolean start = false;

void setup() {
  Serial.begin(57600);


  if (!lis1.begin(0x18)) {   // lis1
    Serial.println("Sensor1 not present.");
  }
  else {
    sens1present = true;
    start = true;
    Serial.println("Sensor1 found!");
  }

  lis1.setRange(LIS3DH_RANGE_16_G);  // or use LIS3DH_RANGE_8_G, LIS3DH_RANGE_4_G, e

  // start the Ethernet and UDP:
  Serial.println("Set up ethernet... ");
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

  /*
      This code waits for an UDP packet, then sends to that address
      Inst

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

  */

  if (start) {  // sensor is present

    // Read sensor 1

    lis1.read();      // get X Y and Z data at once
    accelValue = "1X:";
    accelValue +=  lis1.x;
    accelValue +=  "Y:";
    accelValue += lis1.y;
    accelValue += "Z:";
    accelValue += lis1.z;

    // Convert the Arduino string to a char array
    accelValue.toCharArray(ReplyBuffer, 30);
    Serial.println(ReplyBuffer);

    // Send it
    Udp.beginPacket(remIP, remPort);
    Udp.write(ReplyBuffer);
    Udp.endPacket();
    delay(30);

  }
}



