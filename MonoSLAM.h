#ifndef SLAM_H
#define SLAM_H

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>


#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

/**
 * Pose detection and vSLAM from video feed
 * Process:
 *      name of dir passed as cmd arg
 *      info.json found and parsed
 *      MonoSLAM obj created and passed the camInts, distCoeffs, tag family
 *      each frame of the vid asscced, transformed to cv::Mat/cvFrame? and passed to MonoSLAM obj
 *      for each frame passed, VLSAM obj:
 *          detects markers
 *          sets world (if first) or check if at least one of the tags were seen before
 *          detects pose of tags
 *          for redetected tags:
 *              passes (u,v) to factor graph
 *          for new tags:
 *              calcs pose wrt to world and passes to factor graph
 *          calcs camera pose wrt world and passes to factor graph
 *          updates factor graph (perhaps more than once?)
 *
 *  (REMEMBER: data validation)
 *
 */

namespace armslam {
    struct Tag {
        gtsam::Pose3 pose{};
        int count{};
    };

    /**
     * Class-based implementation of slamProto3.cpp
     */
    class MonoSLAM {
        // TODO: implement conditional logging
        cv::aruco::PREDEFINED_DICTIONARY_NAME tagDictName;
        float markerLength;
        cv::Mat intrinsicsMatrix;
        cv::Mat distCoeffs;
        cv::Mat objPoints;

        // TODO: consider declaring the following 3 as a more generic type allowing adjusting the specific initializations via params
        boost::shared_ptr<gtsam::Cal3DS2> K;
        boost::shared_ptr<gtsam::noiseModel::Isotropic> camNoise;
        boost::shared_ptr<gtsam::noiseModel::Diagonal> poseNoise;

        gtsam::ISAM2 isam;
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initialEstimates;
        gtsam::Values currentEstimate;

        int worldTagID; // add option to use fist camera pose as origin
        int currentPose;
        int frameCount;
        gtsam::Pose3 prev_wTc;
        std::map<int, Tag> observedTags;


    public:
        /**
         * @param tagFamily
         * @param tagLength
         * @param camIntrinsics
         * @param camDistCoeffs
         */
        MonoSLAM(const std::string& tagFamily, const float tagLength, const cv::Mat& camIntrinsics, const cv::Mat& camDistCoeffs) {
            setTagFamily(tagFamily);

            markerLength = tagLength;
            intrinsicsMatrix = camIntrinsics;
            distCoeffs = camDistCoeffs;

            initTagObjPoints();

            K = createCameraCalibrationIgnoreDistortion(intrinsicsMatrix);
            camNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v
            poseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1),gtsam::Vector3::Constant(0.1)).finished());

            isam = createISAM2Obj(); // TODO: allow passing custom iSAM2params obj through MonoSLAM constructor

            currentPose = 0;
            frameCount = 0;
        }

        void setTagFamily(const std::string& tagFamily) {
            if (tagFamily == "DICT_APRILTAG_36h11") {
                tagDictName = cv::aruco::DICT_APRILTAG_36h11;
            }
        }

        void initTagObjPoints() {
            objPoints = cv::Mat(4, 1, CV_32FC3);

            // x down, y across, z up (to facilitate placing world in corner)
            objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(0, 0, 0); // top left
            objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(0, markerLength, 0); // top right
            objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength, markerLength, 0); // bottom right
            objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(markerLength, 0, 0); // bottom left
        }

        // TODO: consider allowing different types of cam calibs to be created
        static boost::shared_ptr<gtsam::Cal3DS2> createCameraCalibration(cv::Mat intrinsics, cv::Mat distortion) {
            return
                boost::make_shared<gtsam::Cal3DS2>(
                    intrinsics.at<double>(0, 0),
                    intrinsics.at<double>(1, 1),
                    0.0,
                    intrinsics.at<double>(0, 2),
                    intrinsics.at<double>(1, 2),
                    distortion.at<double>(0),
                    distortion.at<double>(1),
                    distortion.at<double>(2),
                    distortion.at<double>(3));
        }

        static boost::shared_ptr<gtsam::Cal3DS2> createCameraCalibrationIgnoreDistortion(cv::Mat intrinsics) {
            return
                boost::make_shared<gtsam::Cal3DS2>(
                    intrinsics.at<double>(0, 0),
                    intrinsics.at<double>(1, 1),
                    0.0,
                    intrinsics.at<double>(0, 2),
                    intrinsics.at<double>(1, 2),
                    0, 0);
        }

        static gtsam::ISAM2 createISAM2Obj() {
            gtsam::ISAM2Params parameters;
            parameters.relinearizeThreshold = 0.01;
            parameters.relinearizeSkip = 1;
            parameters.enableDetailedResults = true;
            parameters.factorization = gtsam::ISAM2Params::QR;

            return gtsam::ISAM2(parameters);
        }

        bool update(cv::Mat& imageFrame);

        void getCorners(const cv::Mat& image, std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners);
    };
}


#endif //SLAM_H
