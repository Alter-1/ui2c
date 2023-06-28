""" UI2C adapter module
(C) 2023 by Alexandr A. Telyatnikov aka Alter
"""
import serial
import sys
import time
#from smbus2 import i2c_msg
from ctypes import c_uint32, c_uint8, c_uint16, c_char, POINTER, Structure, Array, Union, create_string_buffer, string_at
#import traceback

verbose      = 0          # python logging
ui2c_logging = False      # request UI2C internal debug logs

# i2c_msg flags from uapi/linux/i2c.h
I2C_M_RD = 0x0001
I2C_MAX_TRANSFER = 255 

# communication mode
MODE_RAW         = 0            # Raw mode, packet format:
                                # <length><I2C addr>[<i2C addr2>]<data bytes>
                                # <length><I2C addr + RD>[<i2C addr2>]<req length>
                                # <length><0xff><CMD><params>
                                # if reply data length is greater than 0xf0, it is passed as 0xff <length>
                                # values 0xf0-0xff are not used for error codes
                                # 1st byte <length> doesn't include itself and I2C addr, only data block (for RD=1 is always 1)

MODE_Coptonix    = 1            # simulate Coptonix #020101 I2C RS232 Adapter, see https://coptonix.com/wp-content/uploads/2020/08/i2crs232slave.pdf
MODE_Manual      = 2            # similar to MODE_Coptonix, but with local echo and option for I2C device reply size limit

# reply
UI2C_RAW_ERR_PREFIX  =  0xff    # some error occured, next byte contains error code (see Wire.endTransmission())
                                # if reply data length is greater than 0xf0, it is passed as 0xff <length>
                                # values 0xf0-0xff are not used for error codes
                                # <length>                        // RAW write confirmation
                                # <length><data bytes>            // RAW read reply
                                # 0xff<F-length><data bytes>      // long RAW read reply
                                # 0xff 0x80 <length><data bytes>  // read reply for proxy/sniffer
                                # 0xff 0x00                       // write confirmation for proxy/sniffer
                                # 0xff <error>                    // error report
                                # 0xfe <log data> <CR>            // RAW log

UI2C_FF_LEN_THRESHOLD     =    0xf0  # length encoded with 2 bytes

UI2C_2W_STATUS_OK         =    0x00  # followed by <length>, for sniffer/proxy mode
UI2C_2W_ERR_TOO_LONG      =    0x01  # data too logs
UI2C_2W_ERR_ADDR_NACK     =    0x02  # addr NACK
UI2C_2W_ERR_DATA_NACK     =    0x03  # data NACK
UI2C_2W_ERR_UNKNOWN       =    0x04  # general error
UI2C_2W_ERR_TIMEOUT       =    0x05  # timeout
UI2C_2W_STATUS_RD_OK      =    0x80  # followed by <length><reply data bytes>, for sniffer/proxy mode

UI2C_RAW_LOG_PREFIX  =  0xfe    # UI2C log message, terminated with \n character, are sent when logging is enabled

# request (starting from I2C addr)
UI2C_RAW_CMD_PREFIX  =  0xff    # corresponds to invalid I2C address and used as special command prefix. Next byte is treated as command, see other UI2C_RAW_CMD_xxx below
UI2C_RAW_CMD_MODE    =  0xff    # switch between RAW (native) and Coptonix #020101 mode. Next byte is treated as mode, see MODE_xxx
UI2C_RAW_CMD_BEGIN   =  0xfe    # acquire/release I2C bus for sequential transactions, next byte 0/1 treated as release(0)/acquire(1)
UI2C_RAW_CMD_LOG     =  0xfd    # change logging level. Next byte: 0 - disabled, 1-3 used, 4+ - reserved

class i2c_msg(Structure):
    """
    As defined in ``i2c.h``.
    """
    _fields_ = [
        ('addr', c_uint16),
        ('flags', c_uint16),
        ('len', c_uint16),
        ('buf', POINTER(c_char))]

    def __iter__(self):
        """ Iterator / Generator
        :return: iterates over :py:attr:`buf`
        :rtype: :py:class:`generator` which returns int values
        """
        idx = 0
        while idx < self.len:
            yield ord(self.buf[idx])
            idx += 1

    def __len__(self):
        return self.len

    def __bytes__(self):
        return string_at(self.buf, self.len)

    def __repr__(self):
        return 'i2c_msg(%d,%d,%r)' % (self.addr, self.flags, self.__bytes__())

    def __str__(self):
        s = self.__bytes__()
        # Throw away non-decodable bytes
        s = s.decode(errors="ignore")
        return s

    @staticmethod
    def read(address, length):
        """
        Prepares an i2c read transaction.
        :param address: Slave address.
        :type: address: int
        :param length: Number of bytes to read.
        :type: length: int
        :return: New :py:class:`i2c_msg` instance for read operation.
        :rtype: :py:class:`i2c_msg`
        """
        arr = create_string_buffer(length)
        o = i2c_msg(
            addr=address, flags=I2C_M_RD, len=length,
            buf=arr)
        return o
    #end read()

    @staticmethod
    def write(address, buf):
        """
        Prepares an i2c write transaction.
        :param address: Slave address.
        :type address: int
        :param buf: Bytes to write. Either list of values or str.
        :type buf: list
        :return: New :py:class:`i2c_msg` instance for write operation.
        :rtype: :py:class:`i2c_msg`
        """
        if sys.version_info.major >= 3:
            if type(buf) is str:
                buf = bytes(map(ord, buf))
            else:
                buf = bytes(buf)
        else:
            if type(buf) is not str:
                buf = ''.join([chr(x) for x in buf])
        arr = create_string_buffer(buf, len(buf))
        return i2c_msg(
            addr=address, flags=0, len=len(arr),
            buf=arr)
    #end write()

#end class i2c_msg

def probe_ui2c_device(dev_name, speed=115200):
    try:
        #print(dev_name)
        # Note: open resets controller by default
        fd = serial.Serial(dev_name, speed,  bytesize=8, parity='N', stopbits=1, timeout=1.1, 
                        xonxoff=False, rtscts = False, dsrdtr = False )
        # Send the command 'version?'
        time.sleep(1.5) # wait for device to start up
        fd.reset_input_buffer();
        fd.reset_output_buffer();

        n=0;
        while(n<3):
            n = n+1
            fd.write(b'version?\n')
            time.sleep(1.0) # wait for command timeout

            # Read the response
            response = fd.readall().decode('ascii')

            if(len(response) == 0):
                continue

            # Check if 'UI2C' substring is found in the response
            if 'UI2C' in response:
                return True
        #end while

        return False

    except serial.SerialException as e:
        print('Serial port error:', str(e))
        return False

    finally:
        fd.close()

#end probe_ui2c_device()


class UartI2C(object):

    def __init__(self, dev_name=None, speed=115200):
        self.speed = speed
        if dev_name is not None:
            self.open(dev_name)
        self._pec = 0
        self.last_err = 0;
 
    def __enter__(self):
        """Enter handler."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Exit handler."""
        self.close()

    def open(self, dev_name):
        '''
        Open serial port with UART-I2C adapter connected, e.g. /dev/ttyUSB0 (linux) or COM11 (windows)
        '''
        self.fd = serial.Serial(dev_name, self.speed,  bytesize=8, parity='N', stopbits=1, timeout=2, 
                        xonxoff=False, rtscts = False, dsrdtr = False )
        #self.fd = serial.Serial(dev_name, self.speed, timeout=2) # 2 sec
        time.sleep(1.5) # wait for device to start up
        self.fd.reset_input_buffer();
        self.fd.reset_output_buffer();

        logstr = self.fd.readline()
        if(ui2c_logging):
            self._enable_logging(1)

    def close(self):
        """
        Close the i2c connection.
        """
        if self.fd:
            self.fd.close()
            self.fd = None
            self._pec = False

    def _get_pec(self):
        return self._pec

    def enable_pec(self, enable=True):
        self._pec = enable

    pec = property(_get_pec, enable_pec)  # Drop-in replacement for smbus member "pec"
    """Get and set SMBus PEC. 0 = disabled (default), 1 = enabled."""

    def _i2c_msg_to_raw(self, i2c_msg):
        if(i2c_msg.len > I2C_MAX_TRANSFER):
            return None
        length = i2c_msg.len + 1  # total packet length including addr
        addr = i2c_msg.addr
        bRead = 0
        if((i2c_msg.flags & I2C_M_RD) == I2C_M_RD):
            #print("  RD")
            bRead = 1

        if(addr > 0x7f):
            length = length+1
        b = [0] * (length+1)   # include length byte itself
        #b[0] = length
        b[0] = i2c_msg.len     # length does't include addr
        i = 0
        #print(addr)
        #print(b)
        #print(length)

        if(addr > 0x7f):
            b[1] = ((addr >> 7) & 0x06) | 0xf0 | bRead
            b[2] = addr & 0xff
            i = 3
        else:
            b[1] = ((addr << 1) & 0xfe) | bRead
            i = 2
        
        #print(i2c_msg.buf)
        n = 0
        if(bRead == 1):
            #b[0] = i              # packet length
            b[0] = 1            # 1 byte for expected reply length
            b[i] = i2c_msg.len  # requested length
            #length = i+2
        else:
            while(i<=length):
                #print(i)
                d = ord(i2c_msg.buf[n])
                #print(d)
                b[i] = d
                i = i+1
                n = n+1
            #end while()
        #end if(bRead)

        #print(b)
        #print("    try return")
        return b
    #end _i2c_msg_to_raw()

    def _start_stop(self, bStart):
        b = [2, UI2C_RAW_CMD_PREFIX, UI2C_RAW_CMD_BEGIN, bStart]
        self.fd.write(b)
    #end _start_stop()

    def _enable_logging(self, bEnable):
        b = [2, UI2C_RAW_CMD_PREFIX, UI2C_RAW_CMD_LOG, bEnable]
        self.fd.write(b)
    #end _enable_logging()

    def i2c_err_to_msg(self, err):
        if(err == UI2C_2W_STATUS_OK):  # 0
            pass
        elif(err>=UI2C_FF_LEN_THRESHOLD):  # 0xf0
            pass
        else:
            txt_err = "Unknown"
            if(err == 1):
                txt_err = "data too long"
            elif(err == 2):
                txt_err = "Addr NACK"
            elif(err == 3):
                txt_err = "Data NACK"
            elif(err == 5):
                txt_err = "Timeout"
            #print("I2C Error: "+str(err)+": "+txt_err)
            #raise IOError("I2C Error: "+str(err))
            #raise OSError(121,"no ACK")
            return txt_err
        # end if(err>=0xf0)
        return None
    #end i2c_err_to_msg()

    def i2c_rdwr(self, *i2c_msgs):
        """
        Combine a series of i2c read and write operations in a single
        transaction (with repeated start bits but no stop bits in between).
        This method takes i2c_msg instances as input, which must be created
        first with :py:meth:`i2c_msg.read` or :py:meth:`i2c_msg.write`.
        :param i2c_msgs: One or more i2c_msg class instances.
        :type i2c_msgs: i2c_msg
        :rtype: None
        """

        self.last_err = UI2C_2W_STATUS_OK

        # end previous transaction if any
        self._start_stop(0)

        # begin transaction
        self._start_stop(1)

        reply = None
        for i2c_msg in i2c_msgs:

            b = self._i2c_msg_to_raw(i2c_msg)
            #print("i2c_msg.flags: "+str(i2c_msg.flags))
            if(verbose>2):
                print("      try send")
                print(b)
            #for d in b:
            #  self.fd.write(d)
            send_len = b[0]+2
            if((b[0] & 0xf8) == 0xf0):
                send_len = send_len+1
            self.fd.write(b[0 : send_len])  # send only request part, not entire buffer 
            #if((i2c_msg.flags & I2C_M_RD) == I2C_M_RD):
            while(True):
                a = self.fd.read()
                if(verbose>2):
                    print("  reply:")
                    print(a)
                if(a == b''):
                    # timeout ?
                    raise IOError("UI2C communication timeout")

                length = a = ord(a)
                if(a == UI2C_RAW_LOG_PREFIX):
                    logstr = self.fd.readline()
                    print("  UI2C Log: ")
                    print("    "+str(logstr))
                    continue
                
                if(a == UI2C_RAW_ERR_PREFIX):
                    err = self.fd.read()
                    if(err == b''):
                        raise IOError("UI2C Error status timeout: "+str(err))
                    err = ord(err)
                    if(err == UI2C_2W_STATUS_OK):  # 0
                        pass
                    elif(err>=UI2C_FF_LEN_THRESHOLD):  # 0xf0
                        length=err
                    else:
                        self.last_err = err
                        txt_err = self.i2c_err_to_msg(err)
                        #print("I2C Error: "+str(err)+": "+txt_err)
                        #raise IOError("I2C Error: "+str(err))
                        raise OSError(121,"no ACK")
                    # end if(err>=0xf0)

                # end if(UI2C_RAW_ERR_PREFIX)

                if(verbose>1):
                    print("length: "+str(length))
                    print("i2c_msg.flags: "+str(i2c_msg.flags))

                if((i2c_msg.flags & I2C_M_RD) == I2C_M_RD):
                    if(verbose>1):
                        print("  sending: ")
                    reply = self.fd.read(length)
                    i2c_msg.len = length
                    #i2c_msg.buf = reply
                    i=0
                    while(i<length):
                        i2c_msg.buf[i] = reply[i]
                        i = i+1
                    #end while

                #end if I2C_M_RD

                break;
                
            # end while

        #end for()

        # end transaction
        self._start_stop(0)

    #end i2c_rdwr()

    def i2c_probe_dev(self, dev_addr):

        #verbose = 3
        #self._enable_logging(1)
        # end previous transaction if any
        self._start_stop(0)

        # begin transaction
        self._start_stop(1)

        #print("try write")
        try:
            part_read = i2c_msg.write(dev_addr, 2)
            self.i2c_rdwr(part_read)
            #part_write = i2c_msg.write(dev_addr, chr(0)+chr(0))
            #self.i2c_rdwr(part_write)
        except Exception as ex:
            #print('ERR: '+ str(ex))
            #log.error(e)
            #lines = traceback.format_exception(type(ex), ex, ex.__traceback__)
            #print(''.join(lines))
            pass
        # end try

        # end transaction
        self._start_stop(0)

        if(self.last_err == UI2C_2W_ERR_DATA_NACK or self.last_err == UI2C_2W_STATUS_OK):
            return True

        return False
    #end i2c_probe_dev()

# end class UartI2C