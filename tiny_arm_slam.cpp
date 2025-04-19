#include <iostream>
#include "tiny_arm_slam.hpp"

int main(int argc, char **argv){
    std::cout << "Starting Tiny ARM SLAM\n";
    tiny_arm_slam::process_folder_cli(argc, argv);
    std::cout << "Tiny ARM SLAM run Complete\n";
    return 0;
}
