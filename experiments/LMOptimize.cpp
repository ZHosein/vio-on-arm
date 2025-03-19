#include "expeimentData.cpp"

namespace experiments {
    int numImages = 30;

    void LMSLAM() {
        std::cout << std::endl << "Slam started..." << std::endl;
        initObjPoints(objPoints);

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initialEstimate;

        std::map<int, Tag> observedTags; // stores all already observed tags
        int worldTagID;
        int poseNum = 0;

        for (size_t frame_num = 0; frame_num <= numImages; frame_num += 1) {
            cv::Mat image = getCVImage(static_cast<int>(frame_num));
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            getCorners(image, ids, corners);

            if (ids.size() > 2) {
                if (observedTags.empty()) {
                    worldTagID = ids[0];


                }
            }
        }
    }
}