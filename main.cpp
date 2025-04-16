#include <iostream>
#include "baby_vSLAM.cpp"

int main(int argc, char **argv){
    std::cout << "Starting iSAM SLAM\n";
    baby_vSLAM::process_folder_cli(argc, argv);
    std::cout << "Slam Algorithm run Complete\n";
    return 0;
}
