#include <iostream>
#include "rerunLogPoseDetection.cpp"
#include "slamProto2.cpp"

int main()
{
    detectPose();
    std::cout << "Pose Detection Complete\n";
    prototype2::slam();
    std::cout << "Slam Algorithm run Complete\n";
    return 0;
}
