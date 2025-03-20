#ifndef EXPEIMENTDATA_H
#define EXPEIMENTDATA_H
#include <string>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#define STRING(x) #x
#define XSTRING(x) STRING(x)

namespace experiments {
    // inline std::string dirPath = std::string(XSTRING(SOURCE_ROOT)).append("/original order/vid_APRILTAG_36h11_720p/");
    inline std::string dirPath = std::string(XSTRING(SOURCE_ROOT)).append("/roomMap");
    inline float markerLength = 0.111;

    inline cv::Mat intrinsicsMatrix = (cv::Mat_<double>(3, 3) <<
        1025.26513671875, 0.0, 642.6650390625,
        0.0, 1025.26513671875, 359.37811279296875,
        0, 0, 1);
    inline cv::Mat distCoeffs = (cv::Mat_<double>(1, 14) << 2.0888574, -82.303825,
        -0.00071347022, 0.0020022474, 315.66144, 1.8588818,
        -80.083954, 308.98071, 0, 0, 0, 0, 0, 0);


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
           distCoeffs.at<double>(3)));*/
    const boost::shared_ptr<gtsam::Cal3DS2> K(
        new gtsam::Cal3DS2(
            intrinsicsMatrix.at<double>(0, 0),
            intrinsicsMatrix.at<double>(1, 1),
            0.0,
            intrinsicsMatrix.at<double>(0, 2),
            intrinsicsMatrix.at<double>(1, 2),
            0, 0));

    // Define the camera observation noise model
    inline auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

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

    inline cv::Mat getCVImage(const int frameNum) {
        const std::string imgPath = dirPath + "image" + std::to_string(frameNum) + std::string(".png");
        return cv::imread(imgPath, cv::IMREAD_COLOR);
    }

    struct Tag {
        gtsam::Pose3 pose{};
        int count{};
    };
}
#endif // EXPEIMENTDATA_H
