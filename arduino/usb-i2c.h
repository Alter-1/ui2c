#ifndef __usb_i2c_h__
#define __usb_i2c_h__

#define UI2C_CMD_RAW_TO_COPTONIX "mode=coptonix"
#define UI2C_CMD_RAW_TO_MANUAL   "mode=manual"
#define UI2C_CMD_VERSION         "version?"
#define UI2C_VERSION_STR         "UI2C v1.0"

#define I2C_MAX_TRANSFER  255

/*************************
    Operation Mode codes
 **************************/
#define MODE_RAW         0     // Raw mode, packet format:
                               // <length><I2C addr>[<i2C addr2>]<data bytes>
                               // <length><I2C addr + RD>[<i2C addr2>]<req length>
                               // <length><0xff><CMD><params>
                               // if reply data length is greater than 0xf0, it is passed as 0xff <length>
                               // values 0xf0-0xff are not used for error codes
                               // 1st byte <length> doesn't include itself and I2C addr, only data block (for RD=1 is always 1)

#define MODE_Coptonix    1     // simulate Coptonix #020101 I2C RS232 Adapter (Master mode), see https://coptonix.com/wp-content/uploads/2020/08/i2crs232slave.pdf
#define MODE_Manual      2     // similar to MODE_Coptonix, but with local echo and option for I2C device reply size limit: x<hex len>,<data...>
#define MODE_Sniffer     3     // sniffer, listen on I2C bus and reflect all traffic in RAW format to UART, uses other PINs


/*************************
// request (starting from I2C addr)
 **************************/
#define UI2C_RAW_CMD_PREFIX    0xff     // corresponds to invalid I2C address and used as special command prefix. Next byte is treated as command, see other UI2C_RAW_CMD_xxx below

#define UI2C_RAW_CMD_MODE      0xff     // switch between RAW (native) and Coptonix #020101 mode. Next byte is treated as mode, see MODE_xxx                                        
#define UI2C_RAW_CMD_BEGIN     0xfe     // acquire/release I2C bus for sequential transactions, next byte 0/1 treated as release(0)/acquire(1)                                      
#define UI2C_RAW_CMD_LOG       0xfd     // change logging level. Next byte: 0 - disabled, 1 - enabled, 2+ - reserved

/*************************
// reply
 **************************/
#define UI2C_RAW_ERR_PREFIX    0xff    // some error occured, next byte contains error code (see Wire.endTransmission())  
                                       // if reply data length is greater than 0xf0, it is passed as 0xff <length>
                                       // values 0xf0-0xff are not used for error codes

                                       // <length>                        // RAW write confirmation
                                       // <length><data bytes>            // RAW read reply
                                       // 0xff<F-length><data bytes>      // long RAW read reply
                                       // 0xff 0x80 <length><data bytes>  // read reply for proxy/sniffer
                                       // 0xff 0x00                       // write confirmation for proxy/sniffer
                                       // 0xff <error>                    // error report
                                       // 0xfe <log data> <CR>            // RAW log

#define UI2C_FF_LEN_THRESHOLD         0xf0  // length encoded with 2 bytes

#define UI2C_2W_STATUS_OK             0x00  // followed by <length>, for sniffer/proxy mode
#define UI2C_2W_ERR_TOO_LONG          0x01  // data too logs
#define UI2C_2W_ERR_ADDR_NACK         0x02  // addr NACK
#define UI2C_2W_ERR_DATA_NACK         0x03  // data NACK
#define UI2C_2W_ERR_UNKNOWN           0x04  // general error
#define UI2C_2W_ERR_TIMEOUT           0x05  // timeout
#define UI2C_2W_STATUS_RD_OK          0x80  // followed by <length><reply data bytes>, for sniffer/proxy mode

#define UI2C_RAW_LOG_PREFIX    0xfe     // UI2C log message, terminated with \n character, are sent when logging is enabled


//#define CMD_TIMEOUT       100  // 100 ms
#define CMD_TIMEOUT       1000  // 1000 ms


#endif //__usb_i2c_h__