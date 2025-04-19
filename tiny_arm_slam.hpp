// main hpp file
// modified by Paul, adapted by Zakareeyah
// Experimenting with iSAM (and visual odometry); minimally commented code
// Taking first POSE as WORLD
//
#ifndef TINYARMSLAM_H
#define TINYARMSLAM_H

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <sys/resource.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <rerun.hpp>
#include <gtsam/slam/BetweenFactor.h>
using namespace std; //wow, chrono lives INSIDE of std

#include "headers/logger.hpp"
#include "headers/intrinsics.hpp"

namespace tiny_arm_slam {

    int poseNum;
    double cpu_usage;
    gtsam::Pose3 prev_wTc;
    vector<long long> update_times, slam_times, memory_usage;
    std::unique_ptr<rerun::RecordingStream> rec = nullptr;
    gtsam::ISAM2 isam;
    std::map<int,Tag> observedTags;
    gtsam::NonlinearFactorGraph factorGraph; // projFactors for new landmarks waiting to be observed twice
    gtsam::Values valueEstimates;

    inline void process_image(cv::Mat image, int frame){
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        tiny_arm_slam::getCorners(image, ids, corners);
        cv::aruco::drawDetectedMarkers(image, corners, ids);


        gtsam::Pose3 wTc; // camera pose wrt wrld

        if (ids.size() > 2 && !(frame > 359 && frame < 420)) {
            // if first pose
            if (observedTags.empty()) {
                int worldTagID = ids[0]; // let the first tag detected be world
                observedTags.insert({worldTagID, Tag{}});

                // get cTw - pose of world wrt camera
                cv::Vec3d rvec, tvec;
                cv::solvePnP(tiny_arm_slam::objPoints, corners[0], intrinsicsMatrix, distCoeffs, rvec,tvec);
                gtsam::Pose3 cTw(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)),gtsam::Point3(tvec.val));
                wTc = cTw.inverse(); // pose of camera wrt world
                prev_wTc = wTc;

                // Add a prior on pose x0, with 30cm std on x,y,z 0.1 rad on roll,pitch,yaw (see experimentData for noise model)
                factorGraph.addPrior(gtsam::Symbol('x', poseNum), wTc, poseNoise);

                // Add a prior on landmark l0
                auto pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.01);
                factorGraph.addPrior(gtsam::Symbol('l', ids[0]), gtsam::Point3(0, 0, 0),pointNoise);

                // add estimate for landmark l0
                valueEstimates.insert(gtsam::Symbol('l', ids[0]), gtsam::Point3(0, 0, 0));
            }
            else {
                bool continuous = false;
                for (int j = 0; j < ids.size(); ++j) {
                    // calc wTc (camera pose wrt world)
                    if (observedTags.count(ids[j]) == 1) { // tag was seen before and we know its pose wrt world
                        gtsam::Pose3 wTt = observedTags[ids[j]].pose;
                        // pose of tag wrt wrld (transforms pts in tag crdnt frm to world)
                        cv::Vec3d rvec, tvec;
                        cv::solvePnP(tiny_arm_slam::objPoints, corners[j], intrinsicsMatrix, distCoeffs, rvec, tvec);
                        gtsam::Pose3 cTt(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)), gtsam::Point3(tvec.val));
                        wTc = wTt.compose(cTt.inverse()); // pose of cam wrt world

                        // calc odometry (pTc: curr cam pose wrt prev cam pose); add to graph
                        auto wTp = prev_wTc;
                        gtsam::Pose3 pTc = wTp.inverse().compose(wTc); // pTw * wTc

                        factorGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                            gtsam::Symbol('x', poseNum - 1), gtsam::Symbol('x', poseNum), pTc, poseNoise));
                        prev_wTc = wTc;

                        continuous = true;
                        break;
                    }
                }
                // no reobserved tags in curr frame
                if (!continuous) {
                    fprintf(stderr, "Loss of Continuity at frame: %d\n", frame);
                    return; // skip this frame
                }
            }

            // Add an initial guess for the current camera pose
            valueEstimates.insert(gtsam::Symbol('x', poseNum), wTc);
            logPose(*rec, wTc, frame, "world/camera");

            // Add initial guesses to all newly observed landmarks
            for (size_t j = 0; j < ids.size(); ++j) {
                if (observedTags.count(ids[j]) == 0) {
                    // get cam pose wrt world; get pts wrt world; add to graph
                    cv::Vec3d rvec, tvec;
                    cv::solvePnP(tiny_arm_slam::objPoints, corners[j], intrinsicsMatrix, distCoeffs,rvec, tvec);
                    cv::drawFrameAxes(image, intrinsicsMatrix, distCoeffs, rvec, tvec, markerLength);

                    gtsam::Pose3 cTt(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)),gtsam::Point3(tvec.val));
                    auto wTt = wTc.compose(cTt);
                    logPose(*rec, wTt, frame, "world/markers/tag" + std::to_string(ids[j]));

                    observedTags.insert({ids[j], {wTt, 0}});
                    gtsam::Point3 initial_lj = wTt.transformFrom(gtsam::Point3(0, 0, 0)); // coordinates of top left corner of tag

                    valueEstimates.insert(gtsam::Symbol('l', ids[j]), initial_lj);

                    gtsam::PinholeCamera<gtsam::Cal3DS2> camera(wTc, *K);
                    gtsam::Point2 projectedPoint = camera.project(initial_lj);
                    log2DPoint(*rec, projectedPoint, frame, "world/camera/image/point");

                }
            }

            // Add factors for each landmark observation (reobserved and new)
            for (size_t j = 0; j < ids.size(); ++j) {
                observedTags[ids[j]].count += 1;
                gtsam::Point2 measurement = gtsam::Point2(corners[j][0].x, corners[j][0].y); // top left corner
                factorGraph.emplace_shared<gtsam::GenericProjectionFactor<
                    gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
                    measurement, noise, gtsam::Symbol('x', poseNum),
                    gtsam::Symbol('l', ids[j]), K);
            }

            tiny_arm_slam::logCVImage(*rec, image, frame, "world/camera/image");

            // Update iSAM with the new factors and values ONLY if all landmarks in this frame have been observed at least x2.
            bool update = true;
            for (int id : ids) if (observedTags[id].count < 2) update = false;
            if(update) {
                // newLandmarksGraph.print("***********************************GRAPH***********************************");
                // std::cout << "Updating iSAM...\n";
                // isam.print();
                // isam.printStats();
                try {
                    auto update_start = chrono::high_resolution_clock::now();
                    isam.update(factorGraph, valueEstimates);
                    isam.update();
                    auto update_stop = chrono::high_resolution_clock::now();
                    auto update_duration = chrono::duration_cast<chrono::nanoseconds>(update_stop - update_start);
                    update_times.push_back(static_cast<long long>(update_duration.count())); //measurement 4 in nanoseconds


                    // isam.saveGraph("graph.txt");

                    // get estimates
                    // gtsam::Values currentEstimate = isam.calculateEstimate();
                    gtsam::Values currentEstimate = isam.calculateBestEstimate();
                    // consider using .filter() to get a separate filtered views for landmarks and poses
                    auto landmarks = currentEstimate.filter<gtsam::Point3>();
                    //std::cout << landmarks.size() << std::endl;
                    // log tag estimates as one point cloud
                    for (const auto& landmark : landmarks) {
                        gtsam::Symbol key(landmark.key);
                        gtsam::Point3 point3 = landmark.value;
                        // std::cout << "Key: " << key.chr() << key.index()  << ", Point3: (" << point3.x() << ", " << point3.y() << ", " << point3.z() << ")" << std::endl;
                        log3DPoint(*rec, point3, frame, "world/landmarks/" + key.string(), 4);
                    }
                    // log pose estimates as another point cloud
                    auto poses = currentEstimate.filter<gtsam::Pose3>();
                    for (const auto& pose: poses) {
                        gtsam::Symbol key(pose.key);
                        gtsam::Pose3 pose3 = pose.value;
                        gtsam::Point3 point3 = pose3.translation();
                        log3DPoint(*rec, point3, frame, "world/poses/" + key.string(), -1);
                    }
                    // Clear the factor graph and values for the next iteration
                    factorGraph.resize(0);
                    valueEstimates.clear();
                }
                catch (gtsam::IndeterminantLinearSystemException &e) {
                    fprintf(stderr, "Indeterminate linear system exception: %s\n", e.what());
                    gtsam::Values currentEstimate = isam.calculateEstimate();

                    std::cerr << "****************************************************" << std::endl;
                    std::cerr << "Error at frame " << frame << std::endl;
                    // isam.getDelta().print();
                    // currentEstimate.print("Current estimate: ");

                    isam.print();
                    exit(1);
                }
            }
            poseNum++;
        }
    }

    inline void write_logs(){
        int i;
        std::ofstream the_log(tiny_arm_slam::logFile.c_str());
        if(!the_log.is_open()){
            std::cerr << "Failed to write to log file: " << tiny_arm_slam::logFile << std::endl;
            exit(1);
        }
        gtsam::Values currentEstimate = isam.calculateBestEstimate();
        auto landmarks = currentEstimate.filter<gtsam::Point3>();
        the_log << "Landmarks Discovered: "<<landmarks.size();
        the_log << "\n\nCPU Usage(%): " << cpu_usage << "%";
        the_log << "\n\nMemory Usage(kb):\n";
        for(i=0;i<memory_usage.size();i++) the_log<<memory_usage[i]<<'\t';
        the_log << "\n\nSlam Loop Times(ns):\n";
        for(i=0;i<slam_times.size();i++) the_log<<slam_times[i]<<'\t';
        the_log << "\n\nUpdate Times(ns):\n";
        for(i=0;i<update_times.size();i++) the_log<<update_times[i]<<'\t';

        auto linearized = isam.getFactorsUnsafe().linearize(isam.calculateEstimate());
        auto [denseHessian, infoVec] = linearized->hessian();
        the_log << "\n\nJacHess stuff now\n";
        the_log << "Hessian Eigen Values: \n" << std::endl;
        the_log << denseHessian.eigenvalues();
        the_log << infoVec;
        the_log << "Jacobian: \n" << std::endl;
        auto [denseJacobian, rhs_b] = linearized->jacobian();
        the_log << denseJacobian;
        the_log << rhs_b;
        the_log.close();
        // auto jac = isam.getFactorsUnsafe().linearize(isam.calculateEstimate())->jacobian();
        /*isam.update();
        isam.update();
        isam.update();
        isam.update();
        isam.print();
        isam.getDelta().print();
        isam.calculateEstimate().print();*/
    }

    inline void process_folder_cli(int argc, char **argv) { //using cli for setup
        tiny_arm_slam::load_cli_options(argc,argv);
        //reset values start
        update_times.clear();
        slam_times.clear();
        memory_usage.clear();
        observedTags.clear();
        poseNum = 0;
        rec = std::make_unique<rerun::RecordingStream>(tiny_arm_slam::startLogger("tiny_arm_slam"));
        tiny_arm_slam::logPinholeCamera(*rec, tiny_arm_slam::intrinsicsMatrix, 0, "world/camera/image");
        tiny_arm_slam::initObjPoints(tiny_arm_slam::objPoints);

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        parameters.enableDetailedResults = true;
        parameters.factorization = gtsam::ISAM2Params::QR;
        isam = gtsam::ISAM2(parameters);
        factorGraph.resize(0);
        valueEstimates.clear();
        //reset values stop
        
        // 3 tags alone: 45-53
        // 45-59 no crash
        // 45-629: iSAM fails...somewhere: sus f80
        // skip f359-420: bad init of tag 9
        // 45-552: good for 1 .update(); fails on 2
        // 45-280: good for 2 .update(); fails on 3
        //for (int frame = 45; frame <= 63; frame++) {
        struct rusage usage_start, usage_stop;
        auto cpu_start = chrono::high_resolution_clock::now();
        getrusage(RUSAGE_SELF, &usage_start);
        //video processing loop start
        for (int frame = tiny_arm_slam::frame_first; frame <= tiny_arm_slam::frame_last; frame++) {
            long memory_usage_start = tiny_arm_slam::getCurrentRSS();
            auto slam_start = chrono::high_resolution_clock::now();

            cv::Mat image = tiny_arm_slam::getCVImage(static_cast<int>(frame));
            process_image(image,frame);

            auto slam_stop = chrono::high_resolution_clock::now();
            auto slam_duration = chrono::duration_cast<chrono::nanoseconds>(slam_stop - slam_start);
            slam_times.push_back(static_cast<long long>(slam_duration.count())); //measurement 3 in nanoseconds
            long memory_usage_stop = tiny_arm_slam::getCurrentRSS();
            memory_usage.push_back(static_cast<long long>(memory_usage_stop-memory_usage_start)); //measurement 2 in kb
        }
        //video processing loop stop

        getrusage(RUSAGE_SELF, &usage_stop);
        auto cpu_stop = chrono::high_resolution_clock::now();
        double user_time = (usage_stop.ru_utime.tv_sec - usage_start.ru_utime.tv_sec)
                + (usage_stop.ru_utime.tv_usec - usage_start.ru_utime.tv_usec) / 1e6;
        double system_time = (usage_stop.ru_stime.tv_sec - usage_start.ru_stime.tv_sec)
                + (usage_stop.ru_stime.tv_usec - usage_start.ru_stime.tv_usec) / 1e6;
        double real_time = chrono::duration_cast<chrono::duration<double>>(cpu_stop - cpu_start).count();
        cpu_usage = ((system_time+user_time)/real_time)*100; //measurement 1 in percent


        gtsam::ISAM2Result results = isam.update();
        results.print();
        // std::cout << frame << std::endl;


        if(tiny_arm_slam::logs){ //idk what JacHess is but I'm leaving the word here because it was here
            write_logs();
        }


        
    }

    inline void process_folder_manual(std::string imageroot, std::string calibration, std::string logfile, bool does_log) { //using manual setup
        tiny_arm_slam::load_manual_options(imageroot,calibration,logfile,does_log);
        //reset values start
        update_times.clear();
        slam_times.clear();
        memory_usage.clear();
        observedTags.clear();
        poseNum = 0;
        rec = std::make_unique<rerun::RecordingStream>(tiny_arm_slam::startLogger("tiny_arm_slam"));
        tiny_arm_slam::logPinholeCamera(*rec, tiny_arm_slam::intrinsicsMatrix, 0, "world/camera/image");
        tiny_arm_slam::initObjPoints(tiny_arm_slam::objPoints);

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        parameters.enableDetailedResults = true;
        parameters.factorization = gtsam::ISAM2Params::QR;
        isam = gtsam::ISAM2(parameters);
        factorGraph.resize(0);
        valueEstimates.clear();
        //reset values stop
        
        // 3 tags alone: 45-53
        // 45-59 no crash
        // 45-629: iSAM fails...somewhere: sus f80
        // skip f359-420: bad init of tag 9
        // 45-552: good for 1 .update(); fails on 2
        // 45-280: good for 2 .update(); fails on 3
        //for (int frame = 45; frame <= 63; frame++) {
        struct rusage usage_start, usage_stop;
        auto cpu_start = chrono::high_resolution_clock::now();
        getrusage(RUSAGE_SELF, &usage_start);
        //video processing loop start
        for (int frame = tiny_arm_slam::frame_first; frame <= tiny_arm_slam::frame_last; frame++) {
            long memory_usage_start = tiny_arm_slam::getCurrentRSS();
            auto slam_start = chrono::high_resolution_clock::now();

            cv::Mat image = tiny_arm_slam::getCVImage(static_cast<int>(frame));
            process_image(image,frame);

            auto slam_stop = chrono::high_resolution_clock::now();
            auto slam_duration = chrono::duration_cast<chrono::nanoseconds>(slam_stop - slam_start);
            slam_times.push_back(static_cast<long long>(slam_duration.count())); //measurement 3 in nanoseconds
            long memory_usage_stop = tiny_arm_slam::getCurrentRSS();
            memory_usage.push_back(static_cast<long long>(memory_usage_stop-memory_usage_start)); //measurement 2 in kb
        }
        //video processing loop stop

        getrusage(RUSAGE_SELF, &usage_stop);
        auto cpu_stop = chrono::high_resolution_clock::now();
        double user_time = (usage_stop.ru_utime.tv_sec - usage_start.ru_utime.tv_sec)
                + (usage_stop.ru_utime.tv_usec - usage_start.ru_utime.tv_usec) / 1e6;
        double system_time = (usage_stop.ru_stime.tv_sec - usage_start.ru_stime.tv_sec)
                + (usage_stop.ru_stime.tv_usec - usage_start.ru_stime.tv_usec) / 1e6;
        double real_time = chrono::duration_cast<chrono::duration<double>>(cpu_stop - cpu_start).count();
        cpu_usage = ((system_time+user_time)/real_time)*100; //measurement 1 in percent


        gtsam::ISAM2Result results = isam.update();
        results.print();
        // std::cout << frame << std::endl;


        if(tiny_arm_slam::logs) write_logs();


        
    }
}
#endif //TINYARMSLAM_H
