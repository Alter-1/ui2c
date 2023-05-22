# ui2c - Arduino-based UART-to-I2C adapter 

Features:
 
* 3 protocols over USB UART (115200 8N1)
    * RAW (data length + address (1 or 2 bytes) and data itself, all in I2C bus format)
    * Coptonix #020101-like (Master mode)
    * manual: local echo, console commands, data in HEX, built-in bus scaner 
* 7/10 bit I2C address support
* data length up to 255 bytes
* python adapter class, implementing i2c_msg from smbus2.SMBus to simplify integration with 
*   I2C/smbus applications and/or porting from Raspbery to Linux/Windows ui2c.py
    build-in logging feature in RAW mode, doesn't interfere with regular data flow 
    
Home: https://alter.org.ua/en/soft/arduino/ui2c/
