Dependencies:
---

- Adafruit LIS3DH
- Adafruit Unified Sensor

Created By:
---
     
- Paul Badger 2017
   
Description:
---
     
Reads an LIS3DH accelerometer and sends UDP data out to 255.255.255.255:35791

One datagram = one sample, data is text, formatted as:

    id:timestamp:x:y:z

Or, if the sensor is missing:

    id:timestamp

Where id is an integer, timestamp is an integer number of milliseconds, and x y z are
decimals in g units. If sensor is missing, x y and z won't be sent.
