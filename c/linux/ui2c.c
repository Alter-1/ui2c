#define __USE_MISC

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <usb-i2c.h>

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

void i2c_msg_read(struct i2c_msg* msg, int address, int length) {
    msg->addr = (uint16_t)address;
    msg->flags = I2C_M_RD;
    msg->len = (uint16_t)length;
    msg->buf = (uint8_t*)malloc(length * sizeof(uint8_t));
} // end i2c_msg_read()

void i2c_msg_write(struct i2c_msg* msg, int address, char* data, int length) {
    msg->addr = (uint16_t)address;
    msg->flags = 0;
    msg->len = (uint16_t)length;
    msg->buf = (uint8_t*)malloc(length * sizeof(uint8_t));
    memcpy(msg->buf, data, length);
} // end i2c_msg_write()

void i2c_msg_free(struct i2c_msg* msg) {
    if(!msg->buf)
        return;
    free(msg->buf);
    msg->buf = NULL;
} // end i2c_msg_free()

speed_t get_termios_baudrate(int baudrate) {
    switch (baudrate) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        case 2500000:
            return B2500000;
        case 3000000:
            return B3000000;
        case 3500000:
            return B3500000;
        case 4000000:
            return B4000000;
        // Add more cases for different baud rates as needed
        default:
            printf("Unsupported baud rate\n");
            return -1;
    }
} // end get_dcb_baudrate()

void reset_input_buffer(int fd) {
    tcflush(fd, TCIFLUSH);
}

int ui2c_open(const char *dev_name, int speed) {
    int fd;

    fd = open(dev_name, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("Failed to open UART");
        return -1;
    }

    struct termios options;
    if (tcgetattr(fd, &options) == -1) {
        perror("Failed to get serial port attributes");
        close(fd);
        return -1;
    }

    speed_t baudrate = get_termios_baudrate(speed);
    if (baudrate == (speed_t)-1) {
        close(fd);
        return -1;
    }

    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    // Set other UART options if needed
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    // Set the timeout value
    options.c_cc[VTIME] = 1; // 1 second timeout (10 * 0.1 seconds)

    if(tcsetattr(fd, TCSANOW, &options) == -1) {
        perror("Failed to set serial port attributes");
        close(fd);
        return -1;
    }

    usleep(1600000); // Wait for device to start up
    reset_input_buffer(fd);

    return fd;
}

void ui2c_close(int fd) {
    close(fd);
} // end ui2c_close()

ssize_t ui2c_read_timeout(int fd, void *buffer, size_t len, int timeout /* ms */) {
    ssize_t total_bytes_read = 0;
    int remaining_timeout = timeout;
    int to = max(min(timeout/10, 100), 1);

    while (remaining_timeout > 0) {
        // Check the number of currently available bytes
        int available_bytes;
        if (ioctl(fd, FIONREAD, &available_bytes) == -1) {
            perror("Failed to get available bytes");
            return -1;
        }

        if (available_bytes > 0) {
            // Read the available bytes up to the buffer length
            ssize_t bytes_read = read(fd, ((char*)(buffer))+total_bytes_read, len);
            if (bytes_read == -1) {
                perror("Failed to read from serial port");
                return -1;
            }

            total_bytes_read += bytes_read;
            remaining_timeout = timeout;

            // Check if there are more bytes available
            if (bytes_read < available_bytes) {
                // Sleep for a short delay before checking again
                usleep(to*1000); // 100 microseconds
            } else {
                // All available bytes have been read
                return total_bytes_read;
            }
        } else {
            // No bytes available, sleep for a short delay before checking again
            usleep(to*1000); // 100 microseconds
        }

        remaining_timeout -= to; // Subtract the delay time from the remaining timeout
    }

    // Timeout expired, return the total number of bytes read
    return total_bytes_read;
} // end ui2c_read_timeout()

int ui2c_probe(int fd) {
    char response[256] = {0};
    ssize_t bytes_read;

    //usleep(1500000); // Wait for device to start up

    for(int i=0; i<3; i++) {
        write(fd, UI2C_CMD_VERSION "\n", strlen(UI2C_CMD_VERSION)+1);

        usleep(1000000); // Wait 1s until device get timeout and send reply

            printf("try read\n");
        //bytes_read = read(fd, response, sizeof(response) - 1);
        bytes_read = ui2c_read_timeout(fd, response, sizeof(response) - 1, 500 /*ms*/);
        if (bytes_read < 0) {
            printf("Failed to read from UART\n");
            return 0;
        }
        if(bytes_read > 0) {
            break;
        }
    }

    response[bytes_read] = '\0';
    //printf("%s\n", response);

    if (strstr(response, "UI2C") != NULL) {
        return 1;
    } else {
        return 0;
    }
} // end ui2c_probe()

int probe_ui2c_device(const char *dev_name, int speed) {
    int fd = ui2c_open(dev_name, speed);
    if (fd < 0) {
        printf("Failed to open UART\n");
        return 0;
    }

    int result = ui2c_probe(fd);

    ui2c_close(fd);

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

void ui2c_start_stop(int fd, unsigned char bStart) {
    unsigned char b[4] = {2, UI2C_RAW_CMD_PREFIX, UI2C_RAW_CMD_BEGIN, bStart};
    write(fd, b, sizeof(b));
} // end ui2c_start_stop()

void ui2c_enable_logging(int fd, unsigned char uLevel) {
    unsigned char b[4] = {2, UI2C_RAW_CMD_PREFIX, UI2C_RAW_CMD_LOG, uLevel};
    write(fd, b, sizeof(b));
} // end ui2c_enable_logging()

int ui2c_rdwr(int fd, struct i2c_msg **msgs, int num_msgs) {
    // End previous transaction if any
    ui2c_start_stop(fd, 0);

    // Begin transaction
    ui2c_start_stop(fd, 1);

    unsigned char *reply = NULL;

    for (int i = 0; i < num_msgs; i++) {
        struct i2c_msg *msg = msgs[i];

        unsigned char *b = ui2c_msg_to_raw(msg);
        int send_len = b[0] + 2;
        // handle 10bit address
        if ((b[0] & 0xF8) == 0xF0) {
            send_len += 1;
        }

        write(fd, b, send_len);
        free(b);

        while (1) {
            unsigned char a;
            ssize_t bytes_read = read(fd, &a, 1);

            if (bytes_read == 0) {
                // Timeout
                printf("UI2C communication timeout\n");
                // Cleanup resources and handle the error
                return UI2C_2W_ERR_TIMEOUT;
            }

            unsigned char length = a;

            if (a == UI2C_RAW_LOG_PREFIX) {
                char logstr[256];
                ssize_t bytes_read = read(fd, logstr, sizeof(logstr) - 1);
                if (bytes_read > 0) {
                    logstr[bytes_read] = '\0';
                    printf("UI2C Log:\n    %s\n", logstr);
                }
                continue;
            }

            if (a == UI2C_RAW_ERR_PREFIX) {
                unsigned char err;
                ssize_t bytes_read = read(fd, &err, 1);
                if (bytes_read == 0) {
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
                    const char *txt_err = "Unknown";   // 4 and > 5
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

            if (msg->flags & I2C_M_RD && length) {
                reply = (uint8_t*)malloc(length);
                ssize_t bytes_read = read(fd, reply, length);
                if (bytes_read > 0) {
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
    ui2c_start_stop(fd, 0);

    return UI2C_2W_STATUS_OK;

} // end ui2c_rdwr()

