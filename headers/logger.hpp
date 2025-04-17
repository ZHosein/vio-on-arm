//
// Created by zakareeyah on 8/4/24.
//

#ifndef LOGGER_H
#define LOGGER_H

#include <rerun.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/ProjectionFactor.h>

namespace tiny_arm_slam{
    std::string logFile = "debug.log";
    inline rerun::RecordingStream startLogger(const std::string& stream="Logger") {
        auto rec = rerun::RecordingStream(stream);
        //rec.spawn().exit_on_failure();
        rec.save(tiny_arm_slam::logFile+".rrd").exit_on_failure();
        rec.set_time_sequence("Frame", 0);
        return rec;
    }
    
    inline void logCVImage(const rerun::RecordingStream& rec, const cv::Mat& image, const int frameNum, const std::string& entity="world/camera/image") {
        rec.set_time_sequence("Frame", frameNum);
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        rec.log(entity, rerun::Image(image.data, {static_cast<uint32_t>(image.cols), static_cast<uint32_t>(image.rows)}, rerun::datatypes::ColorModel::RGB));
    }
    
    inline void log2DPoint(const rerun::RecordingStream& rec, const gtsam::Point2& point, const int frameNum, const std::string& entity, int color=1) {
        rec.set_time_sequence("Frame", frameNum);
        rerun::Color c{};
        switch (color) {
        case 0:
            c = rerun::Color(0, 0, 0);
            break;
        case 1:
                c = rerun::Color(255, 0, 0); break;
        case 2:
                c = rerun::Color(0, 255, 0); break;
        case 3:
                c = rerun::Color(0, 0, 255); break;
        default:
                c = rerun::Color(255, 255, 255); break;
        }
    
        std::vector<rerun::Position2D> points = {rerun::Position2D(static_cast<float>(point.x()), static_cast<float>(point.y()))};
        rec.log(entity, rerun::Points2D(points).with_colors(c).with_radii(5.0f));
    }
    
    inline void log3DPoint(const rerun::RecordingStream& rec, const gtsam::Point3& point, const int frameNum, const std::string& entity, const int color=1) {
        rec.set_time_sequence("Frame", frameNum);
        rerun::Color c{};
        switch (color) {
        case 0:
            c = rerun::Color(0, 0, 0);break; // black
        case 1:
            c = rerun::Color(255, 0, 0); break; // red
        case 2:
            c = rerun::Color(0, 255, 0); break; // green
        case 3:
            c = rerun::Color(0, 0, 255); break; // blue
        case 4:
            c = rerun::Color(188, 77, 185); break; // purple
        default:
            c = rerun::Color(255, 255, 255); break; // white
        }
        std::vector<rerun::Position3D> points = {rerun::Position3D(static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z()))};
        rec.log(entity, rerun::Points3D(points).with_colors(c));
    }
    
    inline void logTextTrace(const rerun::RecordingStream& rec, const std::string& text, const int frameNum) {
        rec.set_time_sequence("Frame", frameNum);
        rec.log("logs", rerun::TextLog(text).with_level(rerun::TextLogLevel::Trace));
    }
    
    inline void logPose(const rerun::RecordingStream& rec, gtsam::Pose3 pose, const int frameNum, const std::string& entity) {
        rec.set_time_sequence("Frame", frameNum);
        const rerun::Mat3x3 rot(static_cast<Eigen::Matrix3f>(
                                        pose.rotation().matrix().cast<float>())
                                    .data());
        const rerun::Vec3D trans(static_cast<Eigen::Vector3f>(
                pose.translation().matrix().cast<float>())
            .data());
        rec.log(entity, rerun::Transform3D(trans, rot));
    }
    
    inline void logPinholeCamera(const rerun::RecordingStream& rec, const cv::Mat& intrinsicsMatrix, const int frameNum, const std::string& entity) {
        rec.set_time_sequence("Frame", frameNum);
        const Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> intrinsics_Eigen(
                const_cast<Eigen::Map<Eigen::Matrix<double, -1, -1, 1>>::PointerArgType>(intrinsicsMatrix.ptr<double>()), intrinsicsMatrix.rows, intrinsicsMatrix.cols);
        rec.log(
            entity,
            rerun::Pinhole(rerun::components::PinholeProjection(
                rerun::Mat3x3(static_cast<Eigen::Matrix3f>(intrinsics_Eigen.cast<float>()).data())))
        );
    }
}


#endif //LOGGER_H
