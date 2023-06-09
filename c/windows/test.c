#include <windows.h>
#include <stdio.h>
#include <stdint.h>
#include <ui2c.h>

int main(int argc, char** argv) {
    // Define the i2c_msgs
    struct i2c_msg msg1, msg2;
    int addr = 0x0b; // default DJI battery address
    char data[] = { 0x3f };
    i2c_msg_write(&msg1, addr, data, 1); // request hardware ID  0x3f command
    i2c_msg_read(&msg2, addr, 2);        // read hardware ID, 2 bytes

    // Initialize the UartI2C
    F_HANDLE uart_i2c;
    if(argc<2) {
        printf("COM-port with UI2C adapter not specified\n");
        return -1;
    }
    printf("Try open %s\n", argv[1]);
    uart_i2c = ui2c_open(argv[1], 115200);
    if(uart_i2c == INVALID_HANDLE_VALUE)
        return -1;

    printf("Probe UI2C adapter...\n");
    if(!ui2c_probe(uart_i2c)) {
        printf("Probe UI2C failed\n");
        return -1;
    }

    printf("Probe device...\n");
    if(!i2c_probe_dev(uart_i2c, addr)) {
        printf("Probe I2C device @0x%x failed\n", addr);
        return -1;
    }

    // Perform the i2c_rdwr operation
    printf("Send/receive...\n");
    struct i2c_msg* msgs[2] = { &msg1, &msg2 };
    ui2c_rdwr(uart_i2c, &msgs[0], 2);

    // Print the received data
    printf("Received ID: %4.4x\n", *((uint16_t*)(msg2.buf)));

    // Cleanup
    i2c_msg_free(&msg1);
    i2c_msg_free(&msg2);
    ui2c_close(uart_i2c);

    return 0;
}
