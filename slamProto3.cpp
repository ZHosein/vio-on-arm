//
// Created by Zakareeyah on 3/19/25.
// Experimenting with iSAM (and visual odometry); minimally commented code
// Taking first POSE as WORLD
//

#include <iostream>
#include <vector>

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

#include <rerun.hpp>
#include <gtsam/slam/BetweenFactor.h>

#include "rerunLogger.h"
#include "expeimentData.h"

namespace prototype3 {
    using namespace experiments;

    void slam() {
        auto rec = startLogger("Prototype3");

        initObjPoints(objPoints);

        gtsam::NonlinearISAM isam{};

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
        for (int frame = 45; frame <= 629; frame++) {
            // rec.set_time_sequence("Frame", static_cast<int64_t>(frame));

            cv::Mat image = getCVImage(static_cast<int>(frame));

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            getCorners(image, ids, corners);
            cv::aruco::drawDetectedMarkers(image, corners, ids);


            gtsam::Pose3 wTc; // camera pose wrt wrld

            if (ids.size() > 2) {
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

                            break;
                        }
                    }
                }

                 // Add an initial guess for the current camera pose
                valueEstimates.insert(gtsam::Symbol('x', poseNum), wTc);

                // Add initial guesses to all newly observed landmarks
                for (size_t j = 0; j < ids.size(); ++j) {
                    if (observedTags.count(ids[j]) == 0) {
                        // get cam pose wrt world; get pts wrt world; add to graph
                        cv::Vec3d rvec, tvec;
                        cv::solvePnP(objPoints, corners[j], intrinsicsMatrix, distCoeffs,rvec, tvec);
                        cv::drawFrameAxes(image, intrinsicsMatrix, distCoeffs, rvec, tvec, markerLength);

                        gtsam::Pose3 cTt(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)),gtsam::Point3(tvec.val));
                        auto wTt = wTc.compose(cTt);
                        logPose(rec, wTt, frame, "world/marker" + std::to_string(ids[j]));

                        observedTags.insert({ids[j], {wTt, 0}});
                        gtsam::Point3 initial_lj = wTt.transformFrom(gtsam::Point3(0, 0, 0)); // coordinates of top left corner of tag

                        valueEstimates.insert(gtsam::Symbol('l', ids[j]), initial_lj);

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
                        // isam.saveGraph("graph.txt");
                        gtsam::Values currentEstimate = isam.estimate();
                        std::cout << "****************************************************" << std::endl;
                        std::cout << "Frame " << frame << ": " << std::endl;
                        currentEstimate.print("Current estimate: ");
                        // Clear the factor graph and values for the next iteration
                        factorGraph.resize(0);
                        valueEstimates.clear();
                    }
                    catch (gtsam::IndeterminantLinearSystemException &e) {
                        fprintf(stderr, "Indeterminate linear system exception: %s\n", e.what());

                        exit(1);
                    }
                }
                poseNum++;
            }
        }
    }
}


