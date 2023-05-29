#include <stdio.h>
#include <stdint.h>
#include "ui2c.h"

int main(int argc, char** argv) {
    // Define the i2c_msgs
    struct i2c_msg msg1, msg2;
    int addr = 0x0b; // default DJI battery address
    char data[] = { 0x3f };
    i2c_msg_write(&msg1, addr, data, 1); // request hardware ID  0x3f command
    i2c_msg_read(&msg2, addr, 2);        // read hardware ID, 2 bytes

    // Initialize the UartI2C
    int uart_i2c;
    if(argc<2) {
        printf("Serial port with UI2C adapter not specified\n");
        return -1;
    }
    uart_i2c = ui2c_open(argv[1], 115200);
    if(uart_i2c <= 0)
        return -1;

    // Perform the i2c_rdwr operation
    struct i2c_msg* msgs[2] = { &msg1, &msg2 };
    ui2c_rdwr(uart_i2c, &msgs[0], 2);

    // Print the received data
    printf("Received ID: %x\n", *((uint16_t*)(msg2.buf)));

    // Cleanup
    i2c_msg_free(&msg1);
    i2c_msg_free(&msg2);
    ui2c_close(uart_i2c);

    return 0;
}
