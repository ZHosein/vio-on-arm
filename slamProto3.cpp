//
// Created by Zakareeyah on 3/19/25.
// Experimenting with iSAM (and visual odometry); minimally commented code
// Taking first POSE as WORLD
//

#include <iostream>
#include <fstream>
#include <vector>

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

#include "rerunLogger.h"
#include "expeimentData.h"

namespace prototype3 {
    using namespace experiments;

    void slam() {
        auto rec = startLogger("Prototype3");
        logPinholeCamera(rec, intrinsicsMatrix, 0, "world/camera/image");

        initObjPoints(objPoints);

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        parameters.enableDetailedResults = true;
        parameters.factorization = gtsam::ISAM2Params::QR;
        gtsam::ISAM2 isam(parameters);

        gtsam::NonlinearFactorGraph factorGraph; // projFactors for new landmarks waiting to be observed twice
        gtsam::Values valueEstimates;

        int worldTagID;
        int poseNum = 0;
        int landmarkNum = 0;
        gtsam::Pose3 prev_wTc;
        std::map<int, Tag> observedTags;
        // 3 tags alone: 45-53
        // 45-59 no crash
        // 45-629: iSAM fails...somewhere: sus f80
        // skip f359-420: bad init of tag 9
        // 45-552: good for 1 .update(); fails on 2
        // 45-280: good for 2 .update(); fails on 3
        //for (int frame = 45; frame <= 63; frame++) {
        for (int frame = 45; frame <= 280; frame++) {
            // rec.set_time_sequence("Frame", static_cast<int64_t>(frame));

            cv::Mat image = getCVImage(static_cast<int>(frame));

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            getCorners(image, ids, corners);
            cv::aruco::drawDetectedMarkers(image, corners, ids);


            gtsam::Pose3 wTc; // camera pose wrt wrld

            if (ids.size() > 2 && !(frame > 359 && frame < 420)) {
                // if first pose
                if (observedTags.empty()) {
                    worldTagID = ids[0]; // let the first tag detected be world
                    observedTags.insert({worldTagID, Tag{}});

                    // get cTw - pose of world wrt camera
                    cv::Vec3d rvec, tvec;
                    cv::solvePnP(objPoints, corners[0], intrinsicsMatrix, distCoeffs, rvec,tvec);
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
                } else {
                    bool continuous = false;
                    for (int j = 0; j < ids.size(); ++j) {
                        // calc wTc (camera pose wrt world)
                        if (observedTags.count(ids[j]) == 1) { // tag was seen before and we know its pose wrt world
                            gtsam::Pose3 wTt = observedTags[ids[j]].pose;
                            // pose of tag wrt wrld (transforms pts in tag crdnt frm to world)
                            cv::Vec3d rvec, tvec;
                            cv::solvePnP(objPoints, corners[j], intrinsicsMatrix, distCoeffs, rvec, tvec);
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
                        continue; // skip this frame
                    }
                }

                 // Add an initial guess for the current camera pose
                valueEstimates.insert(gtsam::Symbol('x', poseNum), wTc);
                logPose(rec, wTc, frame, "world/camera");

                // Add initial guesses to all newly observed landmarks
                for (size_t j = 0; j < ids.size(); ++j) {
                    if (observedTags.count(ids[j]) == 0) {
                        // get cam pose wrt world; get pts wrt world; add to graph
                        cv::Vec3d rvec, tvec;
                        cv::solvePnP(objPoints, corners[j], intrinsicsMatrix, distCoeffs,rvec, tvec);
                        cv::drawFrameAxes(image, intrinsicsMatrix, distCoeffs, rvec, tvec, markerLength);

                        gtsam::Pose3 cTt(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)),gtsam::Point3(tvec.val));
                        auto wTt = wTc.compose(cTt);
                        logPose(rec, wTt, frame, "world/markers/tag" + std::to_string(ids[j]));

                        observedTags.insert({ids[j], {wTt, 0}});
                        gtsam::Point3 initial_lj = wTt.transformFrom(gtsam::Point3(0, 0, 0)); // coordinates of top left corner of tag

                        valueEstimates.insert(gtsam::Symbol('l', ids[j]), initial_lj);

                        // add prior to all tags - failed
                        /*auto pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.01);
                        factorGraph.addPrior(gtsam::Symbol('l', ids[j]), initial_lj,pointNoise);*/

                        gtsam::PinholeCamera<gtsam::Cal3DS2> camera(wTc, *K);
                        gtsam::Point2 projectedPoint = camera.project(initial_lj);
                        log2DPoint(rec, projectedPoint, frame, "world/camera/image/point");

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

                logCVImage(rec, image, frame, "world/camera/image");

                // Update iSAM with the new factors and values ONLY if all landmarks in this frame have been observed at least x2.
                bool update = true;
                for (int id : ids) if (observedTags[id].count < 2) update = false;
                if(update) {
                    // newLandmarksGraph.print("***********************************GRAPH***********************************");
                    // std::cout << "Updating iSAM...\n";
                    // isam.print();
                    // isam.printStats();
                    try {
                        isam.update(factorGraph, valueEstimates);
                        isam.update();


                        // isam.saveGraph("graph.txt");

                        // get estimates
                        // gtsam::Values currentEstimate = isam.calculateEstimate();
                        gtsam::Values currentEstimate = isam.calculateBestEstimate();
                        // consider using .filter() to get a separate filtered views for landmarks and poses
                        auto landmarks = currentEstimate.filter<gtsam::Point3>();
                        std::cout << landmarks.size() << std::endl;
                        // log tag estimates as one point cloud
                        for (const auto& landmark : landmarks) {
                            gtsam::Symbol key(landmark.key);
                            gtsam::Point3 point3 = landmark.value;
                            // std::cout << "Key: " << key.chr() << key.index()  << ", Point3: (" << point3.x() << ", " << point3.y() << ", " << point3.z() << ")" << std::endl;
                            log3DPoint(rec, point3, frame, "world/landmarks/" + key.string(), 4);
                        }
                        // log pose estimates as another point cloud
                        auto poses = currentEstimate.filter<gtsam::Pose3>();
                        for (const auto& pose: poses) {
                            gtsam::Symbol key(pose.key);
                            gtsam::Pose3 pose3 = pose.value;
                            gtsam::Point3 point3 = pose3.translation();
                            log3DPoint(rec, point3, frame, "world/poses/" + key.string(), -1);
                        }
                        // Clear the factor graph and values for the next iteration
                        factorGraph.resize(0);
                        valueEstimates.clear();
                    }
                    catch (gtsam::IndeterminantLinearSystemException &e) {
                        fprintf(stderr, "Indeterminate linear system exception: %s\n", e.what());
                        gtsam::Values currentEstimate = isam.calculateEstimate();

                        std::cout << "****************************************************" << std::endl;
                        std::cout << "Frame " << frame << ": " << std::endl;
                        // isam.getDelta().print();
                        // currentEstimate.print("Current estimate: ");

                        isam.print();
                        exit(1);
                    }
                }
                poseNum++;
            }
        } // end video processing loop
        gtsam::ISAM2Result results = isam.update();
        results.print();
        // std::cout << frame << std::endl;

        if (bool logMatrices = false) {
            std::ofstream Jac_Hess_Out("JacHess.txt");
            auto linearized = isam.getFactorsUnsafe().linearize(isam.calculateEstimate());
            auto [denseHessian, infoVec] = linearized->hessian();
            Jac_Hess_Out << "Hessian Eigen Values: \n" << std::endl;
            Jac_Hess_Out << denseHessian.eigenvalues();
            Jac_Hess_Out << infoVec;
            Jac_Hess_Out << "Jacobian: \n" << std::endl;
            auto [denseJacobian, rhs_b] = linearized->jacobian();
            Jac_Hess_Out << denseJacobian;
            Jac_Hess_Out << rhs_b;
            Jac_Hess_Out.close();
        }


        // auto jac = isam.getFactorsUnsafe().linearize(isam.calculateEstimate())->jacobian();
        /*isam.update();
        isam.update();
        isam.update();
        isam.update();
        isam.print();
        isam.getDelta().print();
        isam.calculateEstimate().print();*/
    }
}


