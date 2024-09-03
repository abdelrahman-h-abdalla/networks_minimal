// Created by Siddhant Gangapurwala


#include <iostream>
#include <experimental/filesystem>
#include <random>

#include "FeasibilityMargin.hpp"
#include "Transform.hpp"
#include "networks_minimal/Activation.hpp"


int main() {
    std::string currentPath(std::experimental::filesystem::current_path());
    std::string parametersDirectory(currentPath + "/../examples/parameters/");

    FeasibilityMargin margin(parametersDirectory + "hyqreal",
                            {40, 256, 256, 128, 1},
                            activation.relu);
    margin.loadDataScalersFromFile(parametersDirectory + "hyqreal/dataScalers.txt");
    Transform inputTransform;

    /// Prepare input in right frames
    // Compute rotation matrix (from base to world) from roll/pitch/yaw - Intrinsic rotations.
    Eigen::Matrix3d w_R_b = inputTransform.eulerToRotationMatrix(0.0, 0.0, 0.0);
    Eigen::Matrix3d b_R_w = w_R_b.transpose();

    margin.states.gravityRot = w_R_b.row(2);
    margin.states.linAcc = b_R_w * Eigen::Vector3d(0., 0., 0.);
    margin.states.angAcc = b_R_w * Eigen::Vector3d(0., 0., 0.);
    margin.states.extForce = b_R_w * Eigen::Vector3d(0., 0., 0.);
    margin.states.extTorque = b_R_w * Eigen::Vector3d(0., 0., 0.);
    // Convert to the base frame
    margin.states.feetPos.resize(4);
    margin.states.feetPos[0] = b_R_w * Eigen::Vector3d(0.44, 0.34, -0.55); // LF Foot Position
    margin.states.feetPos[1] = b_R_w * Eigen::Vector3d(0.44, -0.34, -0.55); // RF Foot Position
    margin.states.feetPos[2] = b_R_w * Eigen::Vector3d(-0.44, 0.34, -0.55); // LH Foot Position
    margin.states.feetPos[3] = b_R_w * Eigen::Vector3d(-0.44, -0.34, -0.55); // RH Foot Position.
    // Set friction
    margin.states.friction = 0.5;
    // Set feet normals
    margin.states.normals.resize(4);
    margin.states.normals[0] = Eigen::Vector3d(0., 0., 1.);
    margin.states.normals[1] = Eigen::Vector3d(0., 0., 1.);
    margin.states.normals[2] = Eigen::Vector3d(0., 0., 1.);
    margin.states.normals[3] = Eigen::Vector3d(0., 0., 1.);

    // Eigen::MatrixXd networkInput;
    // networkInput.setZero(40, 1);
    // networkInput.col(0) << margin.states.gravityRot,
    //                         margin.states.linAcc,
    //                         margin.states.angAcc,
    //                         margin.states.extForce,
    //                         margin.states.extTorque,
    //                         margin.states.feetPos[0],
    //                         margin.states.feetPos[1],
    //                         margin.states.feetPos[2],
    //                         margin.states.feetPos[3],
    //                         margin.states.friction,
    //                         margin.states.normals[0],
    //                         margin.states.normals[1],
    //                         margin.states.normals[2],
    //                         margin.states.normals[3];
    // std::cout << "networkInput_: " << networkInput << std::endl;


    /// Test the feasibility margin
    
    double deltaPosRange = 0.6;
    double deltaAccRange = 8;
    int numOfTests = 25;
    std::vector<double> deltaPosRangeVec(numOfTests);
    std::vector<double> deltaAccRangeVec(numOfTests);
    std::vector<double> posMarginX(numOfTests);
    std::vector<double> posMarginY(numOfTests);
    std::vector<double> posMarginAccX(numOfTests);
    double posStep = deltaPosRange / (numOfTests - 1);
    double posStart = -deltaPosRange / 2.0;
    double accStep = deltaAccRange / (numOfTests - 1);
    double accStart = -deltaAccRange / 2.0;
    // std::cout << "deltaPosRangeVec: " << std::endl;
    for (int i = 0; i < numOfTests; ++i) {
        deltaPosRangeVec[i] = posStart + i * posStep;
        deltaAccRangeVec[i] = accStart + i * accStep;
    }

    // Loop over CoM x
    std::cout << "posMarginX: " << std::endl;
    for (auto i = 0; i < numOfTests; ++i)
    {
        margin.states.feetPos[0] = b_R_w * Eigen::Vector3d(0.44, 0.34, -0.55); // LF Foot Position
        margin.states.feetPos[1] = b_R_w * Eigen::Vector3d(0.44, -0.34, -0.55); // RF Foot Position
        margin.states.feetPos[2] = b_R_w * Eigen::Vector3d(-0.44, 0.34, -0.55); // LH Foot Position
        margin.states.feetPos[3] = b_R_w * Eigen::Vector3d(-0.44, -0.34, -0.55); // RH Foot Position.
        margin.states.feetPos[0](0) -= deltaPosRangeVec[i];
        margin.states.feetPos[1](0) -= deltaPosRangeVec[i];
        margin.states.feetPos[2](0) -= deltaPosRangeVec[i];
        margin.states.feetPos[3](0) -= deltaPosRangeVec[i];

        posMarginX[i] = margin.getFeasibilityMargin(margin.states);
        // std::cout << "grad " << -1 * (margin.getFeasibilityGradients(margin.states)(15) +
        //                                     margin.getFeasibilityGradients(margin.states)(18) +
        //                                     margin.getFeasibilityGradients(margin.states)(21) +
        //                                     margin.getFeasibilityGradients(margin.states)(24)) << std::endl;
        std::cout << "marg " << posMarginX[i] << std::endl;
    }


    margin.states.feetPos[0] = b_R_w * Eigen::Vector3d(0.44, 0.34, -0.55); // LF Foot Position
    margin.states.feetPos[1] = b_R_w * Eigen::Vector3d(0.44, -0.34, -0.55); // RF Foot Position
    margin.states.feetPos[2] = b_R_w * Eigen::Vector3d(-0.44, 0.34, -0.55); // LH Foot Position
    margin.states.feetPos[3] = b_R_w * Eigen::Vector3d(-0.44, -0.34, -0.55); // RH Foot Position.
    // Loop over CoM y
    std::cout << std::endl;
    std::cout << "posMarginY: " << std::endl;
    for (auto i = 0; i < numOfTests; ++i)
    {
        margin.states.feetPos[0] = b_R_w * Eigen::Vector3d(0.44, 0.34, -0.55); // LF Foot Position
        margin.states.feetPos[1] = b_R_w * Eigen::Vector3d(0.44, -0.34, -0.55); // RF Foot Position
        margin.states.feetPos[2] = b_R_w * Eigen::Vector3d(-0.44, 0.34, -0.55); // LH Foot Position
        margin.states.feetPos[3] = b_R_w * Eigen::Vector3d(-0.44, -0.34, -0.55); // RH Foot Position.
        margin.states.feetPos[0](1) -= deltaPosRangeVec[i];
        margin.states.feetPos[1](1) -= deltaPosRangeVec[i];
        margin.states.feetPos[2](1) -= deltaPosRangeVec[i];
        margin.states.feetPos[3](1) -= deltaPosRangeVec[i];

        // std::cout << "margin.states.feetPos[0]: " << margin.states.feetPos[0] << std::endl;
        // std::cout << "margin.states.feetPos[1]: " << margin.states.feetPos[1] << std::endl;
        // std::cout << "margin.states.feetPos[2]: " << margin.states.feetPos[2] << std::endl;
        // std::cout << "margin.states.feetPos[3]: " << margin.states.feetPos[3] << std::endl;
 
        posMarginY[i] = margin.getFeasibilityMargin(margin.states);
        // std::cout << "grad " << -1 * (margin.getFeasibilityGradients(margin.states)(16) +
        //                                     margin.getFeasibilityGradients(margin.states)(19) +
        //                                     margin.getFeasibilityGradients(margin.states)(22) +
        //                                     margin.getFeasibilityGradients(margin.states)(25)) << std::endl;
        std::cout << "marg " << posMarginY[i] << std::endl;
    }

    // Test the inference and gradient computation latency
    std::cout << "\nComputing Inference Latency" << std::endl;

    std::chrono::high_resolution_clock::time_point startTimePoint{}, endTimePoint{};

    // jointPositionErrors.setRandom(12, 1000);
    // jointVelocities.setRandom(12, 1000);
    FeasibilityMargin::StatesStruct latency_states;
    std::array<long, 1000> latency_margin_times{};
    std::array<long, 1000> latency_gradient_times{};
    double latencyMargin;
    std::random_device rd;
    std::mt19937 gen(rd());

    // std::cout << "Latency margins: " << std::endl;
    for (auto c = 0; c < 1000; ++c) {

        latency_states.gravityRot = Eigen::Vector3d::Random();
        latency_states.linAcc = Eigen::Vector3d::Random();
        latency_states.angAcc = Eigen::Vector3d::Random();
        latency_states.extForce = Eigen::Vector3d::Random();
        latency_states.extTorque = Eigen::Vector3d::Random();
        latency_states.feetPos.resize(4);
        latency_states.feetPos[0] = Eigen::Vector3d::Random();
        latency_states.feetPos[1] = Eigen::Vector3d::Random();
        latency_states.feetPos[2] = Eigen::Vector3d::Random();
        latency_states.feetPos[3] = Eigen::Vector3d::Random();
        std::uniform_real_distribution<float> disFric(0.5, 0.8);
        latency_states.friction = disFric(gen);
        latency_states.normals.resize(4);
        for (auto &i : latency_states.normals)
            i = Eigen::Vector3d(0., 0., 1);
        startTimePoint = std::chrono::high_resolution_clock::now();
        latencyMargin = margin.getFeasibilityMargin(latency_states);
        endTimePoint = std::chrono::high_resolution_clock::now();
        latency_margin_times[c] = std::chrono::duration_cast<std::chrono::microseconds>(endTimePoint - startTimePoint).count();
        startTimePoint = std::chrono::high_resolution_clock::now();
        auto latencyGradients = margin.getFeasibilityGradients(latency_states);
        endTimePoint = std::chrono::high_resolution_clock::now();
        latency_gradient_times[c] = std::chrono::duration_cast<std::chrono::microseconds>(endTimePoint - startTimePoint).count();
        // if (c % 10 == 0) // Print less
        //     std::cout << latencyMargin << std::endl;
    }

    double mean_margin = 0;
    double mean_gradient = 0;

    for (auto c = 0; c < 1000; ++c) {
        mean_margin += static_cast<double>(latency_margin_times[c]);
        mean_gradient += static_cast<double>(latency_gradient_times[c]);
    }

    mean_margin = mean_margin / 1000.;
    mean_gradient = mean_gradient / 1000.;

    std::cout << "Latency Margin Mean: " << mean_margin << " us" << std::endl;
    std::cout << "Latency Gradient Mean: " << mean_gradient << " us" << std::endl;

    double std_margin = 0;
    double std_gradient = 0;

    for (auto c = 0; c < 1000; ++c) {
        std_margin += std::pow(static_cast<double>(latency_margin_times[c]) - mean_margin, 2);
        std_gradient += std::pow(static_cast<double>(latency_gradient_times[c]) - mean_gradient, 2);
    }

    std_margin = std::pow(std_margin / 1000., 0.5);
    std_gradient = std::pow(std_gradient / 1000., 0.5);

    std::cout << "Latency Std: " << std_margin << " us" << std::endl;
    std::cout << "Latency Gradient Std: " << std_gradient << " us" << std::endl;

    return 0;
}