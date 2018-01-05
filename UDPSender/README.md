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
	
Example:

    1:32984224:0:-31:99

If sensor is missing, x y and z will be -32767.
