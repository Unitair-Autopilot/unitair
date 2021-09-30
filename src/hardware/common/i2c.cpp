/** @file
    Class for I2C interface
*/

#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <cstring>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <hardware/common/i2c.h>

namespace unitair {
namespace hardware {


I2C::I2C(std::string i2c_bus_path, uint16_t device_address) {

    _bus_fd = open(i2c_bus_path.c_str(), O_RDWR);

    if(_bus_fd < 0){
        std::cerr << "I2C: Failed to open " <<
                      i2c_bus_path <<
                      std::endl;
        std::cerr << std::strerror(errno) <<
                     std::endl;
        exit(1);
    }
    if (ioctl(_bus_fd, I2C_SLAVE, device_address) < 0) {
        std::cerr << "I2C: Failed to set requested device address: " <<
                     device_address <<
                     std::endl;
        std::cerr << std::strerror(errno) <<
                     std::endl;
        exit(1);
    }
    _device_address = device_address;

    unsigned long funcs;
    if (ioctl(_bus_fd, I2C_FUNCS, &funcs) < 0) {
        std::cerr << "I2C: Failed to read I2C adapter functionality" << std::endl;
        exit(1);
    }
    if (!(funcs & I2C_FUNC_I2C)) {
        std::cerr << "I2C: Adapter does not support necessary functionality" << std::endl;
        exit(1);
    }
}


I2C::~I2C(){

    int ret = close(_bus_fd);
    if(ret < 0){
        std::cerr << "I2C: Failed to close I2C device" <<
                     std::endl;
        std::cerr << std::strerror(errno) <<
                     std::endl;
    }
}


bool I2C::read_data(uint8_t register_address, uint8_t* data_buffer, uint16_t num_bytes){

    struct i2c_rdwr_ioctl_data work_queue;
    struct i2c_msg msg[2];

    work_queue.nmsgs = 2;
    work_queue.msgs = msg;

    // write register address being read
    work_queue.msgs[0].addr = _device_address;
    work_queue.msgs[0].len = sizeof(register_address);
    work_queue.msgs[0].flags = 0;
    work_queue.msgs[0].buf = &register_address;

    // read register data
    work_queue.msgs[1].addr = _device_address;
    work_queue.msgs[1].len = num_bytes;
    work_queue.msgs[1].flags = I2C_M_RD;
    work_queue.msgs[1].buf = data_buffer;

    auto ret = ioctl(_bus_fd, I2C_RDWR, &work_queue);
    if(ret < 0){
        std::cerr << "I2C: Failed to read register data. Return value: " << ret <<
                     std::endl;
        std::cerr << std::strerror(errno) <<
                     std::endl;
        return false;
    }
    return true;
};


bool I2C::write_data(uint8_t register_address, uint8_t* data_buffer, uint16_t num_bytes){

     if(num_bytes > MAX_NUM_BYTES_WRITE){
        std::cerr << "I2C ERROR: " << __PRETTY_FUNCTION__ << " : " <<
                     "Cannot write more than " <<
                      MAX_NUM_BYTES_WRITE <<
                     " bytes" <<
                    std::endl;
        return false;
    }

    // ioctl call expects first byte to be register address
    _write_buffer[0] = register_address;
    for(int i = 0; i < num_bytes; i++){
        _write_buffer[i+1] = (data_buffer)[i];
    }

    struct i2c_rdwr_ioctl_data work_queue;
    struct i2c_msg msg[1];

    work_queue.nmsgs = 1;
    work_queue.msgs = msg;

    work_queue.msgs[0].addr = _device_address;
    work_queue.msgs[0].len = num_bytes + 1; // account for register address
    work_queue.msgs[0].flags = 0;
    work_queue.msgs[0].buf = _write_buffer;

    auto ret = ioctl(_bus_fd, I2C_RDWR, &work_queue);
    if(ret < 0){
        std::cerr << "I2C: Failed to write register data. Return value: " << ret <<
                     std::endl;
        std::cerr << std::strerror(errno) <<
                     std::endl;
        return false;
    }
    return true;
};


} // namespace hardware
} // namespace unitair
