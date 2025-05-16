#include "MonoSLAM.h"

void armslam::MonoSLAM::getCorners(const cv::Mat& image, std::vector<int>& ids,
                                   std::vector<std::vector<cv::Point2f>>& corners) {
    cv::aruco::detectMarkers(
        image, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),
        corners, ids);
}

/**
 * uses the data in the frame (if it is valid) to update iSAM (if all tags in the current frame have an observation count >=2)
 *  - adds estimates for curr pose and any newly observed landmarks
 *  - adds a visual odometry factor between curr pose and prev pose
 *  - adds a projection factor between curr pose and all landmarks observed in curr frame
 * @param imageFrame curr frame from video to be processed
 * @return false if frame fails criteria; true if passes - data in frame used to update iSAM
 */
bool armslam::MonoSLAM::update(cv::Mat& imageFrame) {

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    getCorners(imageFrame, ids, corners);
    cv::aruco::drawDetectedMarkers(imageFrame, corners, ids); // for logging/debuggin TODO: wrap in conditional; don't do if not debugging

    gtsam::Pose3 wTc; // curr cam pose wrt world
    // test frame valid; TODO: consider other possible test and abstracting into separate func, e.g.: asserting contrast ratio above a certain threshold before passing to getCorners(...);
    if (ids.size() > 2) { // at least 2 tags detected in image
        // if first pose
        if (observedTags.empty()) {
            worldTagID = ids[0]; // let the first tag detected be world
            observedTags.insert({worldTagID, Tag{}});

            // get cTw - pose of world wrt camera
            cv::Vec3d rvec, tvec;
            cv::solvePnP(objPoints, corners[0], intrinsicsMatrix, distCoeffs, rvec,tvec);
            const gtsam::Pose3 cTw(gtsam::Rot3::Rodrigues(gtsam::Vector3(rvec.val)),gtsam::Point3(tvec.val));
            wTc = cTw.inverse(); // pose of camera wrt world
            prev_wTc = wTc;

            // Add a prior on pose x0, with 30cm std on x,y,z 0.1 rad on roll,pitch,yaw (see experimentData for noise model)
            graph.addPrior(gtsam::Symbol('x', currentPose), wTc, poseNoise);

            // Add a prior on landmark l0
            const auto pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.01); // TODO: make a class var and allow adjusting via params
            graph.addPrior(gtsam::Symbol('l', ids[0]), gtsam::Point3(0, 0, 0),pointNoise);

            // add estimate for landmark l0
            initialEstimates.insert(gtsam::Symbol('l', ids[0]), gtsam::Point3(0, 0, 0));
        } else {
            // continuity fail - return false: frame data not used to update iSAM
            // consider changing this (perhaps throw an error/set err flag instead) if visual odometry implemented in a diff way or if the data is preserved until a loop closure
        }
        return true;
    } return false; // less than only 1 or 0 tags in curr frame
}
