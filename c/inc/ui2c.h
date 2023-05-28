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

#ifdef WIN32
 #define F_HANDLE    HANDLE
 #define DLL_EXPORT __declspec(dllexport)
#else
 #define F_HANDLE    int
 #define DLL_EXPORT __attribute__((visibility("default")))
#endif //WIN32

#ifdef __cplusplus
extern "C" {
#endif

DLL_EXPORT void __stdcall i2c_msg_read(struct i2c_msg* msg, int address, int length);
DLL_EXPORT void __stdcall i2c_msg_write(struct i2c_msg* msg, int address, char* data, int length);
DLL_EXPORT void __stdcall i2c_msg_free(struct i2c_msg* msg);
#ifdef WIN32
  DLL_EXPORT F_HANDLE __stdcall ui2c_openA(const char *dev_name, int speed);
  DLL_EXPORT F_HANDLE __stdcall ui2c_openW(const WCHAR *dev_name, int speed);
  #ifdef _UNICODE
    #define ui2c_open  ui2c_openW
  #else
    #define ui2c_open  ui2c_openA
  #endif // _UNICODE
#else
  DLL_EXPORT F_HANDLE __stdcall ui2c_open(const char *dev_name, int speed);
#endif //WIN32
DLL_EXPORT void __stdcall ui2c_close(F_HANDLE fd);
DLL_EXPORT int  __stdcall ui2c_probe(F_HANDLE fd, const char *command);
DLL_EXPORT int  __stdcall probe_ui2c_device(const char *dev_name, int speed, const char *command);
DLL_EXPORT void __stdcall ui2c_enable_logging (F_HANDLE fd, unsigned char uLevel);
DLL_EXPORT void __stdcall ui2c_rdwr(F_HANDLE fd, struct i2c_msg **msgs, int num_msgs);


#ifdef __cplusplus
}
#endif

#endif // __UI2C_LIB_H__