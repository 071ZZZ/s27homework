#include "auto_aim_system.h"
#include <iostream>

int main() {
    AutoAimSystem system;

    if (!system.initialize("/home/lqy/s27_homework/src/auto_aim/videos/11.mp4",
                           "/home/lqy/s27_homework/src/auto_aim/config/calibration.yaml")) {
        std::cerr << "Initialization failed." << std::endl;
        return -1;
    }

    system.run();
    return 0;
}
