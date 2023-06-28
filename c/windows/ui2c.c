#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <Windows.h>
#include <winioctl.h>
#include <ui2c.h>
#include <usb-i2c.h>
#include <ui2c.h>

DLL_EXPORT
BOOL __stdcall i2c_msg_read(struct i2c_msg* msg, int address, int length) {
    msg->addr = (uint16_t)address;
    msg->flags = I2C_M_RD;
    msg->len = (uint16_t)length;
    msg->buf = (uint8_t*)malloc(length * sizeof(uint8_t));
    return msg->buf != NULL;
} // end i2c_msg_read()

DLL_EXPORT 
BOOL __stdcall i2c_msg_write(struct i2c_msg* msg, int address, char* data, int length) {
    msg->addr = (uint16_t)address;
    msg->flags = 0;
    msg->len = (uint16_t)length;
    msg->buf = (uint8_t*)malloc(length * sizeof(uint8_t));
    if(msg->buf == NULL)
        return FALSE;
    memcpy(msg->buf, data, length);
    return TRUE;
} // end i2c_msg_write()

DLL_EXPORT 
void __stdcall i2c_msg_free(struct i2c_msg* msg) {
    free(msg->buf);
    msg->buf = NULL;
} // end i2c_msg_free()

DWORD get_dcb_baudrate(int baudrate) {
    switch (baudrate) {
        case 9600:
            return CBR_9600;
        case 19200:
            return CBR_19200;
        case 38400:
            return CBR_38400;
        case 57600:
            return CBR_57600;
        case 115200:
            return CBR_115200;
/*
        case 230400:
            return 230400;
        case 460800:
            return 460800;
        case 500000:
            return 500000;
        case 576000:
            return 576000;
        case 921600:
            return 921600;
        case 1000000:
            return 1000000;
        case 1152000:
            return 1152000;
        case 1500000:
            return 1500000;
        case 2000000:
            return 2000000;
        case 2500000:
            return 2500000;
        case 3000000:
            return 3000000;
        case 3500000:
            return 3500000;
        case 4000000:
            return 4000000;
*/
        // Add more cases for different baud rates as needed
        default:
            printf("Unsupported baud rate\n");
            return (DWORD)-1;
    }
} // end get_dcb_baudrate()

void reset_input_buffer(HANDLE hSerial) {
    //PurgeComm(hSerial, PURGE_RXCLEAR);
    PurgeComm(hSerial, PURGE_TXCLEAR | PURGE_TXABORT | PURGE_RXCLEAR | PURGE_RXABORT);
}

HANDLE _ui2c_open(HANDLE hSerial, int speed) {

    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts;
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    SetupComm(hSerial, 4096, 4096);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        printf("Failed to get serial port state\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    dcbSerialParams.BaudRate = get_dcb_baudrate(speed);
    if(dcbSerialParams.BaudRate <= 0) {
        printf("Failed select serial port baudrate\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    dcbSerialParams.fOutxCtsFlow = FALSE;
    dcbSerialParams.fOutxDsrFlow = FALSE;
    dcbSerialParams.fOutX = FALSE;
    dcbSerialParams.fInX = FALSE;
    dcbSerialParams.fErrorChar = FALSE;
    dcbSerialParams.fBinary = TRUE;
    dcbSerialParams.fNull = FALSE;
    dcbSerialParams.fAbortOnError = FALSE;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        printf("Failed to set serial port state\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    // Configure the timeouts for read and write operations
    timeouts.ReadIntervalTimeout = 100; // MAXWORD ?
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        printf("Failed to set serial port timeouts\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    Sleep(1600); // Wait for device to start up + 1 sec before ReadFile

    reset_input_buffer(hSerial);

    return hSerial;
} // end _ui2c_open()

void _clear_com_error(HANDLE hSerial)
{
    DWORD dwErrorFlags;
   	COMSTAT ComStat;

   	ClearCommError( hSerial, &dwErrorFlags, &ComStat );
} // end _clear_com_error()

DLL_EXPORT 
HANDLE __stdcall ui2c_openA(const char* dev_name, int speed) {
    HANDLE hSerial;

    hSerial = CreateFileA(dev_name, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
    if (hSerial == INVALID_HANDLE_VALUE) {
        printf("Failed to open UART\n");
        return INVALID_HANDLE_VALUE;
    }
    return _ui2c_open(hSerial, speed);
} // end ui2c_openA()

DLL_EXPORT 
HANDLE __stdcall ui2c_openW(const WCHAR* dev_name, int speed) {
    HANDLE hSerial;

    hSerial = CreateFileW(dev_name, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
    if (hSerial == INVALID_HANDLE_VALUE) {
        printf("Failed to open UART\n");
        return INVALID_HANDLE_VALUE;
    }
    return _ui2c_open(hSerial, speed);
} // end ui2c_openW()

DLL_EXPORT 
void __stdcall ui2c_close(HANDLE hSerial) {
    CloseHandle(hSerial);
} // end ui2c_close()

DLL_EXPORT 
int __stdcall ui2c_probe(HANDLE hSerial) {
    char response[256] = {0};

    DWORD bytesWritten, bytesRead;

    for(int i=0; i<3; i++) {
        PurgeComm(hSerial, PURGE_TXCLEAR | PURGE_TXABORT | PURGE_RXCLEAR | PURGE_RXABORT);

        if (!WriteFile(hSerial, UI2C_CMD_VERSION "\n", (DWORD)strlen(UI2C_CMD_VERSION) + 1, &bytesWritten, NULL)) {
            printf("Failed to write to UART\n");
            return 0;
        }
        printf("  %d writen UART\n", bytesWritten);

        Sleep(CMD_TIMEOUT); // Wait for device to get timeout and reply

        DWORD dwErrorFlags;
       	COMSTAT ComStat;

       	ClearCommError( hSerial, &dwErrorFlags, &ComStat );
        reset_input_buffer(hSerial);

        if (!ReadFile(hSerial, response, sizeof(response) - 1, &bytesRead, NULL)) {
            printf("Failed to read from UART\n");
            return 0;
        }
        printf("  %d read from UART\n", bytesRead);
        if(bytesRead > 0) {
            break;
        }
    }
    response[bytesRead] = '\0';
    printf("%s\n", response);

    if (strstr(response, "UI2C") != NULL) {
        return 1;
    } else {
        return 0;
    }
} // end ui2c_probe()

DLL_EXPORT 
int __stdcall probe_ui2c_device(const char* dev_name, int speed) {
    HANDLE hSerial = ui2c_open(dev_name, speed);
    if (hSerial == INVALID_HANDLE_VALUE) {

        printf("Failed to open UART\n");
        return 0;
    }

    int result = ui2c_probe(hSerial);

    ui2c_close(hSerial);

    return result;
} // end probe_ui2c_device()

unsigned char *ui2c_msg_to_raw(struct i2c_msg *msg) {
    if (msg->len > I2C_MAX_TRANSFER) {
        return NULL;
    }

    unsigned int  length = msg->len + 1;
    uint16_t addr = msg->addr;
    unsigned char bRead = 0;

    if ((msg->flags & I2C_M_RD) == I2C_M_RD) {
        bRead = 1;
    }

    if (addr > 0x7F) {
        length += 1;
    }

    unsigned char *b = (uint8_t*)malloc((length + 1) * sizeof(uint8_t));
    b[0] = (uint8_t)msg->len;

    unsigned char i = 0;
    unsigned char n = 0;

    if (addr > 0x7F) {
        b[1] = ((addr >> 7) & 0x06) | 0xF0 | bRead;
        b[2] = addr & 0xFF;
        i = 3;
    } else {
        b[1] = ((addr << 1) & 0xFE) | bRead;
        i = 2;
    }

    if (bRead == 1) {
        b[0] = 1;
        b[i] = (uint8_t)msg->len;
    } else {
        while (i <= length) {
            unsigned char d = (unsigned char)msg->buf[n];
            b[i] = d;
            i += 1;
            n += 1;
        }
    }

    return b;
} // end ui2c_msg_to_raw()

DLL_EXPORT 
void __stdcall ui2c_start_stop(HANDLE hSerial, unsigned char bStart) {
    unsigned char b[4] = {2, UI2C_RAW_CMD_PREFIX, UI2C_RAW_CMD_BEGIN, bStart};
    DWORD bytesWritten;
    WriteFile(hSerial, b, sizeof(b), &bytesWritten, NULL);
} // end ui2c_start_stop()

DLL_EXPORT 
void __stdcall ui2c_enable_logging(HANDLE hSerial, unsigned char uLevel) {
    unsigned char b[4] = {2, UI2C_RAW_CMD_PREFIX, UI2C_RAW_CMD_LOG, uLevel};
    DWORD bytesWritten;
    WriteFile(hSerial, b, sizeof(b), &bytesWritten, NULL);
} // end ui2c_enable_logging()

DLL_EXPORT 
int __stdcall ui2c_rdwr(HANDLE hSerial, struct i2c_msg** msgs, int num_msgs) {
    // End previous transaction if any
    ui2c_start_stop(hSerial, 0);

    // Begin transaction
    ui2c_start_stop(hSerial, 1);

    unsigned char* reply = NULL;

    for (int i = 0; i < num_msgs; i++) {
        struct i2c_msg* msg = msgs[i];

        unsigned char* b = ui2c_msg_to_raw(msg);
        int send_len = b[0] + 2;
        // handle 10bit address
        if ((b[0] & 0xF8) == 0xF0) {
            send_len += 1;
        }

        DWORD bytesRead;
        DWORD bytesWritten;
        WriteFile(hSerial, b, send_len, &bytesWritten, NULL);
        free(b);

        while (1) {
            unsigned char a;
            if (!ReadFile(hSerial, &a, 1, &bytesRead, NULL)) {
                // Timeout
                printf("UI2C communication timeout\n");
                // Cleanup resources and handle the error
                return UI2C_2W_ERR_TIMEOUT;
            }

            unsigned char length = a;

            if (a == UI2C_RAW_LOG_PREFIX) {
                char logstr[256];
                bytesRead = 0;
                if (!ReadFile(hSerial, logstr, sizeof(logstr) - 1, &bytesRead, NULL)) {
                    //printf("UI2C communication timeout\n");
                    // Cleanup resources and handle the error
                    continue;
                }
                if (bytesRead > 0) {
                    logstr[bytesRead] = '\0';
                    printf("UI2C Log:\n    %s\n", logstr);
                }
                continue;
            }

            if (a == UI2C_RAW_ERR_PREFIX) {
                unsigned char err;
                bytesRead = 0;
                if (!ReadFile(hSerial, &err, 1, &bytesRead, NULL)) {
                    printf("UI2C communication timeout\n");
                    // Cleanup resources and handle the error
                    return UI2C_2W_ERR_TIMEOUT;
                }
                if (bytesRead == 0) {
                    printf("UI2C Error status timeout\n");
                    // Cleanup resources and handle the error
                    return UI2C_2W_ERR_TIMEOUT;
                }

                if (err == UI2C_2W_STATUS_OK) {
                    // 0 - Status OK
                    continue;
                } else if (err >= UI2C_FF_LEN_THRESHOLD) {
                    length = err;
                } else {
                    const char* txt_err = "Unknown";   // 4 and  > 5
                    switch (err) {
                        case UI2C_2W_ERR_TOO_LONG:     // 1
                            txt_err = "data too long";
                            break;
                        case UI2C_2W_ERR_ADDR_NACK:    // 2
                            txt_err = "Addr NACK";
                            break;
                        case UI2C_2W_ERR_DATA_NACK:    // 3
                            txt_err = "Data NACK";
                            break;
                        case UI2C_2W_ERR_TIMEOUT:      // 5
                            txt_err = "Timeout";
                            break;
                    }
                    printf("I2C Error: %d: %s\n", err, txt_err);
                    // Cleanup resources and handle the error
                    return err;
                }
            }

            if (msg->flags & I2C_M_RD) {
                reply = (uint8_t*)malloc(length);
                bytesRead = 0;
                if (!ReadFile(hSerial, reply, length, &bytesRead, NULL)) {
                    printf("UI2C communication timeout\n");
                    // Cleanup resources and handle the error
                    return UI2C_2W_ERR_TIMEOUT;
                }
                if (bytesRead > 0) {
                    msg->len = length;
                    memcpy(msg->buf, reply, length);
                }
                // Clean up allocated memory
                free(reply);
            }

            break;
        }
    }

    // End transaction
    ui2c_start_stop(hSerial, 0);

    return UI2C_2W_STATUS_OK;
} // end ui2c_rdwr()

DLL_EXPORT 
BOOL __stdcall i2c_probe_dev(HANDLE hSerial, int dev_addr)
{
    struct i2c_msg msg;
    int err;

    // End previous transaction if any
    ui2c_start_stop(hSerial, 0);

    // Begin transaction
    ui2c_start_stop(hSerial, 1);

    if(!i2c_msg_read(&msg, dev_addr, 2))        // read 2 bytes
    {
        return FALSE;
    }

    struct i2c_msg* msgs[1] = { &msg };
    err = ui2c_rdwr(hSerial, &msgs[0], 1);

    // End transaction
    ui2c_start_stop(hSerial, 0);

    i2c_msg_free(&msg);

    return err == UI2C_2W_ERR_DATA_NACK || err == UI2C_2W_STATUS_OK;

} // end i2c_probe_dev()
