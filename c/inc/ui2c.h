#ifndef __UI2C_LIB_H__
#define __UI2C_LIB_H__

#ifndef I2C_M_RD

#define I2C_M_RD          0x0001

struct i2c_msg {
    uint16_t addr;
    uint16_t flags;
    uint16_t len;
    uint8_t* buf;
};

#endif //I2C_M_RD

void i2c_msg_read(struct i2c_msg* msg, int address, int length);
void i2c_msg_write(struct i2c_msg* msg, int address, char* data, int length);
void i2c_msg_free(struct i2c_msg* msg);
int ui2c_open(const char *dev_name, int speed);
void ui2c_close(int fd);
int ui2c_probe(int fd, const char *command);
int probe_ui2c_device(const char *dev_name, int speed, const char *command);
void ui2c_enable_logging (int fd, unsigned char uLevel);
void ui2c_rdwr(int fd, struct i2c_msg **msgs, int num_msgs);


#endif // __UI2C_LIB_H__