#include "auto_aim_system.h"
#include <iostream>

int main() {
    AutoAimSystem system;

    if (!system.initialize("/home/lqy/s27_homework/src/armors/avi.mp4",
                           "/home/lqy/s27_homework/src/auto_aim/config/calibration.yaml")) {
        std::cerr << "Initialization failed." << std::endl;
        return -1;
    }

    system.run();
    return 0;
}
