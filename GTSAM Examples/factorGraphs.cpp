#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

int main () {
    gtsam::NonlinearFactorGraph graph;

    const gtsam::Pose2 priorMean(0.0,0.0,0.0);
    auto noiseModel = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));
    graph.addPrior(1, priorMean, noiseModel);

    gtsam::Pose2 odometry(2.0,0.0,0.0);
    auto odometryNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(1, 2, odometry, odometryNoise);
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(2, 3, odometry, odometryNoise);

    graph.print("\nFactor Graph\n");

    return 0;
}
