#include <stdint.h>
#include "WireZ.h"
#include <EEPROM.h>
#include "usb-i2c.h"

#ifndef UI2C_LOGGING
#define UI2C_LOGGING  1
#endif

#define Wire WireZ

#define LED 13

#define MAX_CMD_PREFIX    16
#define bufSz             (I2C_MAX_TRANSFER*2)+2    // for Coptonix we need CMD + double size (for ASCII representation) + <CR>
#define EOL "\n"
#define EOL2 "\n\r"            // for logging/local echo

// EEPROM layout

#define cfgProgrammed    0     // 1 byte
#define cfgLogging       1     // 1 byte
#define cfgMode          2     // 1 byte
                       //3
#define cfgI2CAddr       4     // 2 byts
                       //6-7
#define cfgUARTspd       8     // 4 byts
#define cfgCrcLen        (sizeof(uint32_t))
#define cfgCrc           (EEPROM.length()-cfgCrcLen)

// handled only by timeout 
char    CMD_RAW_TO_COPTONIX[] = UI2C_CMD_RAW_TO_COPTONIX;
char    CMD_RAW_TO_MANUAL[]   = UI2C_CMD_RAW_TO_MANUAL;
char    CMD_VERSION[]         = UI2C_CMD_VERSION;
char    VERSION_STR[]         = UI2C_VERSION_STR;

//uint8_t cmdbuffer[bufSz+1];               // UART input/output buffer
//uint8_t i2cbuffer[I2C_MAX_TRANSFER+1];    // I2C input buffer
// share UART and I2C buffers with short offset for non-command part to save some free memory
uint8_t iobuffer[bufSz+1+MAX_CMD_PREFIX];
#define i2cbuffer   iobuffer
uint8_t* cmdbuffer = &(iobuffer[MAX_CMD_PREFIX]);

int len = 0;                   // expected length of packet ot be sent
int pos = 0;                   // current position in input UART buffer
int req_len = 0;               // requested length to be read
int rpos = 0;                  // actually read from I2C bytes
int in_transaction = 0;        // transaction depth, don't release bus until reach zero, see also UI2C_RAW_CMD_BEGIN

char bProgrammed = 0;
char mode = MODE_RAW;
#if UI2C_LOGGING
char bLogging = 0;
#endif //UI2C_LOGGING

int i2c_addr = 0xfe;
uint32_t uart_speed = 115200;


void setup() {

  Wire.begin();
  Wire.setUserBuffer(&i2cbuffer[0], I2C_MAX_TRANSFER, true /*twi ZeroCopy and shared buffer with Wire*/);
  //Wire.onReceive(receiveEvent); // doesn't work as expected
  /*
     230400, 460800, 500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000
     ??? 2500000, 3000000, 3500000, 4000000
  */
  Serial.begin(uart_speed);  
  //Serial.begin(230400);  // max for ch341-uart, unstable
  // Define the LED pin as Output
  pinMode (LED, OUTPUT);
  digitalWrite(LED, HIGH);

  uint32_t crc = eeprom_crc();
  uint32_t crc0;
  EEPROM.get(cfgCrc, crc0);

  if(crc0 != crc) {
    //Serial.println("Init EEPROM");
    SetProgrammed(0);
  }

  bProgrammed = EEPROM.read(cfgProgrammed);
  if(bProgrammed) {
#if UI2C_LOGGING
    bLogging    = EEPROM.read(cfgLogging);
#endif //UI2C_LOGGING
    mode        = EEPROM.read(cfgMode);
    i2c_addr    = EEPROM.read(cfgI2CAddr);
  }
#if UI2C_LOGGING
  if(bLogging) {
    Serial.write(UI2C_RAW_LOG_PREFIX);
    Serial.print(VERSION_STR);
    Serial.write(EOL);
  }
#endif //UI2C_LOGGING
  digitalWrite(LED, LOW);

} // end setup()

void SetProgrammed(char bVal)
{
  bProgrammed = bVal;
  EEPROM.update(cfgProgrammed, bVal);
  if(bVal) {
#if UI2C_LOGGING
    EEPROM.update(cfgLogging, bLogging);
#endif //UI2C_LOGGING
    EEPROM.update(cfgMode,    mode);
    EEPROM.update(cfgI2CAddr, i2c_addr);
  }

  uint32_t crc = eeprom_crc();
  EEPROM.update(cfgCrc, crc);

} // end SetProgrammed()

void receiveEvent(int bytes) {

/*
#if UI2C_LOGGING
  if(bLogging) {
    Serial.write(UI2C_RAW_LOG_PREFIX);
    Serial.print("Recv req len ");
    Serial.print(req_len, HEX);
    Serial.write(EOL);
  }
#endif //UI2C_LOGGING
  while(Wire.available()) // loop through all but the last
  {
    if(rpos < req_len) {
      i2cbuffer[rpos++] = Wire.read(); // receive byte as a character
    } else {
      Wire.read();
    }
  }
  */
  digitalWrite(LED, HIGH);
#if UI2C_LOGGING
  if(bLogging) {
    Serial.write(UI2C_RAW_LOG_PREFIX);
    Serial.print("Recv ");
    Serial.print(bytes, HEX);
    Serial.write(EOL);
  }
#endif //UI2C_LOGGING
  switch(mode) {
  case MODE_RAW:
    if(bytes >= 0xf0)
    {
      // to handle special codes and long transfers correctly, see comment to UI2C_RAW_ERR_PREFIX
      Serial.write(UI2C_RAW_ERR_PREFIX);
    }
    Serial.write(bytes);
    break;
  case MODE_Coptonix:
    Serial.print("69"); // 'i'
    break;
  case MODE_Manual:
    Serial.print("<" EOL2);
    break;
  }
  for(int i=0; i<bytes; i++)
  {
    switch(mode) {
    case MODE_RAW:
      Serial.write(i2cbuffer[i]);
      break;
    case MODE_Coptonix:
    case MODE_Manual:
      // I2CDataAvail      | 0x69 | i d1 |             | ’69’+'XXYYZZ...'+<CR> 
      Serial.print((i2cbuffer[i] >> 4) & 0xf, HEX);
      Serial.print((i2cbuffer[i]     ) & 0xf, HEX);
     if(mode == MODE_Manual) {
       Serial.write(' ');
       if((i&0x7) == 7) {
         Serial.print(EOL2);
       }
     }
      break;
    }
  }
#if UI2C_LOGGING
  if(bLogging) {
    Serial.write(UI2C_RAW_LOG_PREFIX);
    for(int i=0; i<bytes; i++)
    {
      Serial.print((i2cbuffer[i] >> 4) & 0xf, HEX);
      Serial.print((i2cbuffer[i]     ) & 0xf, HEX);
    }
    Serial.write(EOL);
  }
#endif //UI2C_LOGGING
  if(mode == MODE_Coptonix) {
    Serial.print("\r");
  } else
  if(mode == MODE_Manual) {
    Serial.print(EOL2);
  }
  //rpos = 0;
  req_len = 0;
  digitalWrite(LED, LOW);
} // end receiveEvent()

void doScan() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    Wire.beginTransmission(address+1);

    if (error == 0 && Wire.endTransmission() != 0 ) // Special flag for SAMD Series
    {
        Serial.print("I2C device found at address 0x");
        if (address<16)
            Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("!");

        nDevices++;
    }
    else if (error==4) 
    {
        Serial.print("Unknown error at address 0x");
        if (address<16) 
            Serial.print("0");
        Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
      Serial.println("No I2C devices found\n");
  else
      Serial.println("done\n");

} // end doScan()

uint8_t incomingByte = 0;
uint32_t start_seq_t = 0;

uint8_t decode_hex4(char c) {
  if(c >= '0' && c <= '9') {
    return c-'0';
  }
  if(c >= 'A' && c <= 'F') {
    return 10+c-'A';
  }
  if(c >= 'a' && c <= 'f') {
    return 10+c-'a';
  }
  return 0xff;
} // end decode_hex4()

uint16_t decode_hex16(char* buff) {
  uint16_t v = 0;
  uint8_t v4;
  while(*buff) {
    v4 = decode_hex4(*buff);
    if(v4 == 0xff)
      return v;
    v = (v<<4) | v4;
    buff++;
  }
  return v;
} // end decode_hex16()

bool decode_hexbuff(char* buff, byte* dst) {
  uint8_t v = 0;
  char v4;
  int n;
  while(*buff) {
    v4 = decode_hex4(*buff);
    if(v4 < 0)
      return false;
    v = (v<<4) | v4;
    buff++;
    n++;
    if(n % 2 == 0) {
      *dst = v;
      v = 0;
      dst++;
    }
  }
  return (n % 2 == 0);
} // end decode_hex16()

void print_i2c_status(int error) {
  Serial.print(EOL2 "Status: ");
  Serial.print(error, HEX);
  switch(error) {
  case 0:
    Serial.print(" OK"); break;
  case 1:
    Serial.print(" data too long"); break;
  case 2:
    Serial.print(" NACK addr"); break;
  case 3:
    Serial.print(" NACK data"); break;
  case 4:
    Serial.print(" unknown"); break;
  case 5:
    Serial.print(" timeout"); break;
  }
  Serial.print(EOL2);
} // end print_i2c_status()

void loop() {

  // handle timeout
  uint16_t m = millis();
  bool do_read = false;
  byte error=0;
  int i;

  if((mode == MODE_RAW) && (start_seq_t!=0)) {
  /*****************
    RAW mode manual commands handled by timeout
   *****************/
    if((m - start_seq_t) > CMD_TIMEOUT) {
      
      if(len == CMD_RAW_TO_COPTONIX[0]+1 && 
         pos >= sizeof(CMD_RAW_TO_COPTONIX)-1 &&
         !memcmp(&cmdbuffer[0], &CMD_RAW_TO_COPTONIX[1], sizeof(CMD_RAW_TO_COPTONIX)-2 ))
      {
        mode = MODE_Coptonix;
        Serial.print(VERSION_STR);
        Serial.print(" Coptonix emulation" EOL2);
      } else
      if(len == CMD_VERSION[0]+1 && 
         pos >= sizeof(CMD_VERSION)-1 &&
         !memcmp(&cmdbuffer[0], &CMD_VERSION[1], sizeof(CMD_VERSION)-2 ))
      {
        Serial.print(VERSION_STR);
      } else
      if(len == CMD_RAW_TO_MANUAL[0]+1 && 
         pos >= sizeof(CMD_RAW_TO_MANUAL)-1 &&
         !memcmp(&cmdbuffer[0], &CMD_RAW_TO_MANUAL[1], sizeof(CMD_RAW_TO_MANUAL)-2 ))
      {
        mode = MODE_Manual;
        Serial.print(VERSION_STR);
        Serial.print(" Manual mode" EOL2);
      } else {
        //Serial.print("T" EOL2);
      }
      start_seq_t = 0;
      pos = 0;
      len = 0;
    }
  }

  /*****************
  read from serial as much as possible/as requested
   *****************/
  while (Serial.available() > 0)
  {
    digitalWrite(LED, HIGH);
    // read the incoming byte:
    start_seq_t = m;
    if(mode == MODE_RAW) {
      if(pos == 0 && len==0) {
        len = Serial.read(); // doesn't include len byte itself and includes addr/rdwr,  up to 0xff (I2C_MAX_TRANSFER)
#if UI2C_LOGGING
        if(bLogging) {
          Serial.write(UI2C_RAW_LOG_PREFIX);
          Serial.print("Raw CMD len ");
          Serial.print(len, HEX);
          Serial.write(EOL);
        }
#endif //UI2C_LOGGING
        if(len > I2C_MAX_TRANSFER) {
          len = 0;
        } else {
          len++;      // for addr
        }
        continue;
      }
    }
    cmdbuffer[pos++] = incomingByte = Serial.read();
    if(pos >= bufSz)
    {
      pos = 0;
      len = 0;
      continue;
    }

    /*
#if UI2C_LOGGING
    if(bLogging) {
      Serial.write(UI2C_RAW_LOG_PREFIX);
      Serial.print("Raw CHAR ");
      Serial.print(cmdbuffer[pos-1], HEX);
      Serial.print(" pos ");
      Serial.print(pos-1, HEX);
      Serial.print(" len ");
      Serial.print(len, HEX);
      Serial.write(EOL);
    }
#endif //UI2C_LOGGING
    */


  /*****************
  Manual & Coptonix
   *****************/

    if(mode == MODE_Coptonix ||
       mode == MODE_Manual) {
      // Coptonix I2C RS232 Adapter
      // Part No. #020101
      //Serial.write("Coptonix: ");
      if(
#if UI2C_LOGGING
         bLogging || 
#endif //UI2C_LOGGING
         (mode == MODE_Manual)) {
        Serial.write(incomingByte); // local echo
      }

      //Serial.write(incomingByte); // local echo
      if(pos && (incomingByte=='\n' || incomingByte=='\r'))
      {
        if(mode == MODE_Manual) {
          Serial.write('\r'); // local echo
        }
        //LogPrintln("Decode");
        cmdbuffer[pos-1] = 0;
        //LogPrintln(cmdbuffer);
        switch(cmdbuffer[0])
        {
        case 'a':
          // GetSlaveAddress   | 0x61 | a    | ’a’+<CR>      | ’61’+’XX’+<CR> 
          if(mode == MODE_Coptonix) {
            Serial.print("61");
          } else
          if(mode == MODE_Manual) {
            Serial.print(EOL2 "Dst Address: ");
          }
          Serial.print(i2c_addr, HEX);
          if(mode == MODE_Manual) {
            Serial.print("\n");
          }
          Serial.print("\r");
          break;
        case 'c':
          // ChangeSlaveAdress | 0x63 | c d1 | ’c’+’XX’+<CR> | ’63’+<CR> 
          i2c_addr = decode_hex16(&(cmdbuffer[1]));
          if(i2c_addr < 1023) {
            //Serial.print(EOL2);
            //Serial.print(i2c_addr, HEX);
            //Serial.print(EOL2);
            if(mode == MODE_Coptonix) {
              Serial.print("63\r");
            } else
            if(mode == MODE_Manual) {
              Serial.print(EOL2 "OK" EOL2);
            }
          } else {
            if(mode == MODE_Manual) {
              Serial.print(EOL2 "Wrong address" EOL2);
            }
          }
          break; 
        case 's':
          // ChangeSlaveAdress | 0x63 | c d1 | ’c’+’XX’+<CR> | ’63’+<CR> 
          SetProgrammed(1);
          if(mode == MODE_Coptonix) {
            Serial.print("73\r");
          } else
          if(mode == MODE_Manual) {
            Serial.print(EOL2 "OK" EOL2);
          }
          break; 
        case 'w':
          // WriteData         | 0x77 | w d..| ’w’+’XXYY..’+<CR> | ’77’+<CR> 
          len = (pos-2)/2;
          decode_hexbuff(&cmdbuffer[1], &i2cbuffer[0]);
          Wire.beginTransmission(i2c_addr);
          //len = Wire.write(&i2cbuffer[0], len);
          len = Wire.writeZeroCopy(len); // write from prepared i2cbuffer buffer
          error = Wire.endTransmission(true);
          if(mode == MODE_Coptonix) {
            if(error == 0) {
              Serial.print("77\r");
            }
          } else
          if(mode == MODE_Manual) {
            print_i2c_status(error);
          }
          break; 
        case 'x':
          // WriteDataInt1     | 0x78 | w d..| ’x’+’XXYY..’+<CR> | ’78’+<CR> + 69 ...
          cmdbuffer[pos-1] = 0;
          len = (pos-2)/2;
          i = 1;
          if(pos>4 && mode == MODE_Manual && (cmdbuffer[2] == ',' || cmdbuffer[3] == ',')) {
            // Slecial case - reply length extension
            // WriteDataInt1     | 0x78 | w l, d..| ’x'+ 'LL,’+’XXYY..’+<CR> | ’Status: XX’+<CR> + > XXYYZZ ...  // Manual mode
            for(i=2; cmdbuffer[i] != ','; i++) {
            }
            i++;
            /*
            Serial.print("i=");
            Serial.print(i, HEX);
            Serial.print(EOL2);
            */
            len = (pos-i-1)/2;
          }
          decode_hexbuff(&cmdbuffer[i], &i2cbuffer[0]);
          Wire.beginTransmission(i2c_addr);
          //len = Wire.write(&i2cbuffer[0], len);
          len = Wire.writeZeroCopy(len); // write from prepared i2cbuffer
          error = Wire.endTransmission(false);
          req_len = I2C_MAX_TRANSFER;
          if(mode == MODE_Coptonix) {
            if(error == 0) {
              Serial.print("78\r");
            }
          } else
          if(mode == MODE_Manual) {
            print_i2c_status(error);
            if(i>1) {
              req_len = decode_hex16(&cmdbuffer[1]);
            }
          }

          if(error == 0) {
            /*
            Serial.print("req_len=");
            Serial.print(req_len, HEX);
            Serial.print(EOL2);
            */
            Wire.requestFrom(i2c_addr, req_len, false);  // read directly to i2cbuffer
            rpos = Wire.readBytes(i2cbuffer, req_len); // actually do nothing, just update index
            /*
            Serial.print("rpos=");
            Serial.print(rpos, HEX);
            Serial.print(EOL2);
            */
            error = Wire.endTransmission(true);
            /*
            Serial.print(EOL2 "len = ");
            Serial.print(rpos, HEX);
            Serial.print(EOL2);
            */
            req_len = rpos;
            receiveEvent(rpos);
          }

          //Serial.print(EOL2);

          break; 

        // non-standard commands
        case 'r':
          // RawMode           | 0x72
          mode = MODE_RAW;
          //Serial.print("72\r");
          Serial.print(VERSION_STR);
          Serial.print(" RAW mode" EOL2);
          SetProgrammed(1);
          len = 0;
          break; 
        case 'm':
          // Manual Mode           
          mode = MODE_Manual;
          Serial.print(VERSION_STR);
          Serial.print(" Manual mode" EOL2);
          SetProgrammed(1);
          len = 0;
          break; 
        case 'l':
          // Logging          | 0x72
#if UI2C_LOGGING
          bLogging = decode_hex4(cmdbuffer[1]);
          if(bLogging) {
            Serial.print("Logging ON" EOL2);
          }
#endif //UI2C_LOGGING
          len = 0;
          break; 
        case '?':
          doScan();
          break;
        case 'v':
          Serial.print(VERSION_STR);
          Serial.print(EOL2);
          break;
        } // end cmdbuffer[0]
        pos = 0;
      } else
      if(incomingByte=='\n' || incomingByte=='\r') {
        pos = 0;
      }
      continue;
    } // end if(mode == MODE_Coptonix)

    if((pos == 1) && ((incomingByte & 0xf8) == 0xf0)) {
      len++; // expect 2 bytes for addr instead of 1
    }
    if(len>0 && pos==len) {
      break;
    }

  } // end while()

  digitalWrite(LED, LOW);

  if(mode == MODE_Coptonix ||
     mode == MODE_Manual) {
    return;
  }
  if(len==0 && pos==0) {
    start_seq_t = 0;
    return;
  }

  // MODE_RAW, wait for entire packet
  if(pos<len) {
    return;
  }

  start_seq_t = 0;
#if UI2C_LOGGING
  if(bLogging) {
    Serial.write(UI2C_RAW_LOG_PREFIX);
    Serial.print("Raw CMD char ");
    Serial.print(cmdbuffer[0], HEX);
    Serial.print(" pos ");
    Serial.print(pos, HEX);
    Serial.print(" len ");
    Serial.print(len, HEX);
    Serial.write(EOL);
  }
#endif //UI2C_LOGGING

  /*****************
  Handle RAW management commands
   *****************/
  if(len == 3 && cmdbuffer[0] == UI2C_RAW_CMD_PREFIX && cmdbuffer[1] == UI2C_RAW_CMD_MODE && cmdbuffer[2] == 1) {
    mode = MODE_Coptonix;
    pos = 0;
    len = 0;
    SetProgrammed(1);
    return;
  }
  if(len == 3 && cmdbuffer[0] == UI2C_RAW_CMD_PREFIX && cmdbuffer[1] == UI2C_RAW_CMD_BEGIN) {
    if(cmdbuffer[2] == 1) {

      if(in_transaction < 2)
        in_transaction++;

#if UI2C_LOGGING
      if(bLogging) {
        Serial.write(UI2C_RAW_LOG_PREFIX);
        Serial.print("beginTransmission ex ");
        Serial.print(in_transaction, HEX);
        Serial.write(EOL);
      }
#endif //UI2C_LOGGING
      Wire.beginTransmission(i2c_addr);

    } else {
      if(in_transaction > 0)
        in_transaction--;

#if UI2C_LOGGING
      if(bLogging) {
        Serial.write(UI2C_RAW_LOG_PREFIX);
        Serial.print(" end trans ex ");
        Serial.print(in_transaction, HEX);
        Serial.write(EOL);
      }
#endif //UI2C_LOGGING
      Wire.endTransmission(in_transaction==0);

    }
    pos = 0;
    len = 0;
    return;
  }
  if(len == 3 && cmdbuffer[0] == UI2C_RAW_CMD_PREFIX && cmdbuffer[1] == UI2C_RAW_CMD_LOG) {
#if UI2C_LOGGING
    bLogging = cmdbuffer[2];
#endif //UI2C_LOGGING
    pos = 0;
    len = 0;
    return;
  }

  /*****************
  Handle RAW data
   *****************/

  do_read = cmdbuffer[0] & 1;
  i2c_addr = ((uint16_t)cmdbuffer[0] & 0xfe) >> 1;

#if UI2C_LOGGING
  if(bLogging) {
    Serial.write(UI2C_RAW_LOG_PREFIX);
    Serial.print("Addr0 ");
    Serial.print(i2c_addr, HEX);
    Serial.write(EOL);
  }
#endif //UI2C_LOGGING

  pos = 0;
  if((i2c_addr & 0xf8) == 0xf0) {
    if(len >= 2) {
      i2c_addr = ((uint16_t)(i2c_addr & 0x06) << 7) | (uint16_t)cmdbuffer[1];
      pos = 1;
      len--;
    } else {
      pos = 0;
      len = 0;
      //Serial.write((byte)2);
      Serial.write(UI2C_RAW_ERR_PREFIX);
      Serial.write(0xff);
      return;
    }
  } else {
    //i2c_addr = (cmdbuffer[0] >> 1) & 0x7f;
  }

#if UI2C_LOGGING
  if(bLogging) {
    Serial.write(UI2C_RAW_LOG_PREFIX);
    Serial.print("ADDR ");
    Serial.print(i2c_addr, HEX);
    Serial.print(" pos ");
    Serial.print(pos+1, HEX);
    Serial.print(" len ");
    Serial.print(len-1, HEX);
    Serial.print(" RD ");
    Serial.print(do_read, HEX);
    Serial.write(EOL);
  }
#endif //UI2C_LOGGING

  if(in_transaction < 2) {
    in_transaction++;
  }

  digitalWrite(LED, HIGH);
  if(do_read) {
    in_transaction--;
    req_len = cmdbuffer[pos+1];

#if UI2C_LOGGING
    if(bLogging) {
      Serial.write(UI2C_RAW_LOG_PREFIX);
      Serial.print("requestFrom ");
      Serial.print(req_len, HEX);
      Serial.print(" end trans ");
      Serial.print(in_transaction==0, HEX);
      Serial.write(EOL);
    }
#endif //UI2C_LOGGING

    Wire.requestFrom(i2c_addr, req_len, in_transaction==0);
    rpos = Wire.readBytes(i2cbuffer, req_len);
    receiveEvent(rpos);

  } else {

#if UI2C_LOGGING
    if(bLogging) {
      Serial.write(UI2C_RAW_LOG_PREFIX);
      Serial.print("beginTransmission ");
      Serial.write(EOL);
    }
#endif //UI2C_LOGGING
    Wire.beginTransmission(i2c_addr);
    if(len > 1) {
      // Note: we assime that cmdbuffer and i2cbuffer are not overlapping because of setUserBuffer behavior
      len = Wire.write(&cmdbuffer[pos+1], len-1);
    }
    in_transaction--;
#if UI2C_LOGGING
    if(bLogging) {
      Serial.write(UI2C_RAW_LOG_PREFIX);
      Serial.print(" end trans ");
      Serial.print(in_transaction==0, HEX);
      Serial.write(EOL);
    }
#endif //UI2C_LOGGING
    error = Wire.endTransmission(in_transaction==0);
  }

  pos = 0;
  if(error > 0) {
    Serial.write(UI2C_RAW_ERR_PREFIX);
    Serial.write(error);
    len = 0;
    return;
  }
  if(!do_read) {
    if(len >= 0xf0) {
      // to handle special codes and long transfers correctly, see comment to UI2C_RAW_ERR_PREFIX
      Serial.write(UI2C_RAW_ERR_PREFIX);
    }
    Serial.write((byte)len);
  }
  len = 0;
  digitalWrite(LED, LOW);

}
