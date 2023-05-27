#ifndef __UI2C_LIB_H__
#define __UI2C_LIB_H__

#ifndef I2C_M_RD

#define I2C_M_RD          0x0001

struct i2c_msg {
    uint16_t addr;
    uint16_t flags;
    uint16_t len;
    char* buf;
};

#endif //I2C_M_RD

#ifdef WIN32
 #define F_HANDLE    HANDLE
#else
 #define F_HANDLE    int
#endif //WIN32

void i2c_msg_read(struct i2c_msg* msg, int address, int length);
void i2c_msg_write(struct i2c_msg* msg, int address, char* data, int length);
void i2c_msg_free(struct i2c_msg* msg);
F_HANDLE ui2c_open(const char *dev_name, int speed);
void ui2c_close(F_HANDLE fd);
int ui2c_probe(F_HANDLE fd, const char *command);
int probe_ui2c_device(const char *dev_name, int speed, const char *command);
void ui2c_enable_logging (F_HANDLE fd, unsigned char uLevel);
void ui2c_rdwr(F_HANDLE fd, struct i2c_msg **msgs, int num_msgs);


#endif // __UI2C_LIB_H__