#ifndef INTRINSICS_H
#define INTRINSICS_H
#include "cli.hpp"
#include "json.hpp"
#include "logger.hpp"
using json = nlohmann::json;
#include <string>
#include <fstream>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#define STRING(x) #x
#define XSTRING(x) STRING(x)

namespace tiny_arm_slam {
    //variables from the cli start
    int frame_first=45, frame_last=280;
    std::vector<std::string> imgList;
    std::string imgFolder = "datasets/roomMap";
    std::string calibrationFile = "datasets/roomMap/calibration.json";
    float markerLength = 0.111; // 1.0 == 1 meter
    cv::Mat intrinsicsMatrix = (cv::Mat_<double>(3, 3) <<
        1025.26513671875, 0.0, 642.6650390625,
        0.0, 1025.26513671875, 359.37811279296875,
        0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 14) << 2.0888574, -82.303825,
        -0.00071347022, 0.0020022474, 315.66144, 1.8588818,
        -80.083954, 308.98071, 0, 0, 0, 0, 0, 0);
    //variables from the cli stop
    // using these results in earlier failure...supposed to be more accurate?
    /*inline cv::Mat distCoeffs = (cv::Mat_<double>(1, 14) << 2.088857412338257,
        -82.30382537841797, -0.0007134702173061669, 0.0020022473763674498, 315.66143798828125, 1.8588818311691284,
        -80.08395385742188, 308.980712890625, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);*/


    /*const boost::shared_ptr<gtsam::Cal3DS2> K(
        new gtsam::Cal3DS2(
           intrinsicsMatrix.at<double>(0, 0),
           intrinsicsMatrix.at<double>(1, 1),
           0.0,
           intrinsicsMatrix.at<double>(0, 2),
           intrinsicsMatrix.at<double>(1, 2),
           distCoeffs.at<double>(0),
           distCoeffs.at<double>(1),
           distCoeffs.at<double>(2),
           distCoeffs.at<double>(3)
           ));*/
    inline void load_calibration_file(){
        int rows,cols,i,j;
        std::ifstream file(calibrationFile.c_str());
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << calibrationFile << std::endl;
            exit(1);
        }
        json config;
        file>>config;
        markerLength = config["markerLength"];
        frame_first = config["frame_first"];
        frame_last = config["frame_last"];
        rows = config["intrinsics"].size();
        cols = config["intrinsics"][0].size();
        intrinsicsMatrix.create(rows,cols,CV_64F);
        for(i=0; i<rows; i++)
            for(j=0; j<cols; j++) intrinsicsMatrix.at<double>(i,j) = config["intrinsics"][i][j];
        cols = config["distCoeffs"].size();
        distCoeffs.create(1,cols,CV_64F);
        for(j=0; j<cols; j++) distCoeffs.at<double>(0,j) = config["distCoeffs"][j];
        std::cout<<"Processing "<<(frame_last-frame_first)<<"images at "<<imgFolder<<" with calibration file "<<calibrationFile<<std::endl;
        if(tiny_arm_slam::logs){
            std::cout<<"Logging Enabled to text log file "<<tiny_arm_slam::logFile
            <<" and rerun log file "<<tiny_arm_slam::logFile<<".rrd"<<std::endl;
        }
        else std::cout<<"Logging Disabled"<<std::endl;
    }
    inline void load_cli_options(int argc, char **argv){
        std::vector<Command> commands = tiny_arm_slam::parseCommands(argc,argv);
        tiny_arm_slam::Command *calibration = tiny_arm_slam::findCommand(commands,"calibration");
        tiny_arm_slam::Command *images = tiny_arm_slam::findCommand(commands,"imageroot");
        tiny_arm_slam::Command *log = tiny_arm_slam::findCommand(commands,"logfile");

        if(images!=nullptr) imgFolder=images->val;
        calibrationFile = calibration!=nullptr? calibration->val: imgFolder+"/calibration.json";
        if(log!=nullptr) tiny_arm_slam::logFile=log->val;
        tiny_arm_slam::logs = tiny_arm_slam::findCommand(commands,"logs") != nullptr;
        load_calibration_file();
    }
    inline void load_manual_options(std::string imageroot, std::string calibration, std::string logfile, bool does_log){
        imgFolder = imageroot;
        calibrationFile = calibration;
        tiny_arm_slam::logFile = logfile;
        tiny_arm_slam::logs = does_log;
        load_calibration_file();
    }
    
    inline boost::shared_ptr<gtsam::Cal3DS2> K(
        new gtsam::Cal3DS2(
            intrinsicsMatrix.at<double>(0, 0),
            intrinsicsMatrix.at<double>(1, 1),
            0.0,
            intrinsicsMatrix.at<double>(0, 2),
            intrinsicsMatrix.at<double>(1, 2),
            0, 0));

    // Define the camera observation noise model
    inline auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v
    inline auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1),gtsam::Vector3::Constant(0.1)).finished());


    inline cv::Mat objPoints(4, 1, CV_32FC3);

    inline void initObjPoints(cv::Mat objPoints) {
        // x down, y across, z up (to facilitate placing world in corner)
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(0, 0, 0); // top left
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(0, markerLength, 0); // top right
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength, markerLength, 0); // bottom right
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(markerLength, 0, 0); // bottom left
    }

    inline void getCorners(const cv::Mat& image, std::vector<int>& ids,
                           std::vector<std::vector<cv::Point2f>>& corners) {
        cv::aruco::detectMarkers(
            image, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),
            corners, ids);
    }

    inline cv::Mat getCVImage(int frameNum) {
        const std::string imgPath = imgFolder + "/image" + std::to_string(frameNum) + std::string(".png");
        return cv::imread(imgPath, cv::IMREAD_COLOR);
    }

    struct Tag {
        gtsam::Pose3 pose{};
        int count{};
    };

    inline long getCurrentRSS() {
        std::ifstream status_file("/proc/self/status");
        std::string line;
        while (std::getline(status_file, line)) {
            if (line.substr(0, 6) == "VmRSS:") {
                long rss = 0;
                std::sscanf(line.c_str(), "VmRSS: %ld", &rss);
                return rss; // in KB
            }
        }
        return 0; // fallback
    }
}
#endif // INTRINSICS_H
