#include <iostream>
#include "slamProto3.cpp"

int main()
{
    std::cout << "Starting iSAM SLAM" << std::endl;
    prototype3::slam();
    std::cout << "Slam Algorithm run Complete\n";
    return 0;
}
