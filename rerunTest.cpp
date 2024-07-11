#include <iostream>
#include <rerun.hpp>

void logTrans() {
    auto rec = rerun::RecordingStream("rerun_example_dna_abacus");
    auto result = rec.connect("10.0.0.2:9876");
    if (result.is_err()) {
        std::cout << "Failed to connect to rerun viewer";
        exit(1);
    }
    rec.log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);

    rec.log(
        "world/camera",
        rerun::Transform3D(rerun::RotationAxisAngle({0.0f, 0.0f, 0.0f}, rerun::Angle::radians(0)), 1.0f)
    );


    std::cout << "hello world!";
}