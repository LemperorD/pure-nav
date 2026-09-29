#include "common/shm.hpp"
#include "driver/livox_driver/include/livox_driver.hpp"

int main() {
    pure::common::SharedMemory shm("test_shm", 1024);
    shm.write("Hello, shared memory!");
    std::string message = shm.read();
    std::cout << "Read from shared memory: " << message << std::endl;
    return 0;
}