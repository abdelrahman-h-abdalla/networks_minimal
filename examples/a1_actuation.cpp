// Created by Siddhant Gangapurwala


#include <iostream>
#include <experimental/filesystem>
#include <random>

#include "Actuation.hpp"
#include "Transform.hpp"


int main() {
    std::string currentPath(std::experimental::filesystem::current_path());
    std::string parametersDirectory(currentPath + "/../examples/parameters/");

    Eigen::MatrixXd networkInputOffset(47, 1);
    networkInputOffset.block(0, 0, 3, 1) = Eigen::Vector3d(0., 0., 1.);
    networkInputOffset.block(3, 0, 15, 1) = Eigen::MatrixXd::Zero(15, 1);
    networkInputOffset.block(18, 0, 3, 1) = Eigen::Vector3d(0.29, 0.19, -0.46);
    networkInputOffset.block(21, 0, 3, 1) = Eigen::Vector3d(0.29, -0.19, -0.46);
    networkInputOffset.block(24, 0, 3, 1) = Eigen::Vector3d(-0.29, 0.19, -0.46);
    networkInputOffset.block(27, 0, 3, 1) = Eigen::Vector3d(-0.29, -0.19, -0.46);
    networkInputOffset.block(30, 0, 1, 1) << 0.51;
    networkInputOffset.block(31, 0, 4, 1) = Eigen::Vector4d::Ones() * 0.767;
    networkInputOffset.block(35, 0, 3, 1) = Eigen::Vector3d(0., 0., 1.);
    networkInputOffset.block(38, 0, 3, 1) = Eigen::Vector3d(0., 0., 1.);
    networkInputOffset.block(41, 0, 3, 1) = Eigen::Vector3d(0., 0., 1.);
    networkInputOffset.block(44, 0, 3, 1) = Eigen::Vector3d(0., 0., 1.);

    Eigen::MatrixXd networkInputScaling(47, 1);
    networkInputScaling.block(0, 0, 3, 1) = Eigen::Vector3d(0.0734, 0.0750, 0.003);
    networkInputScaling.block(3, 0, 3, 1) = Eigen::Vector3d(0.198, 0.198, 0.053);
    networkInputScaling.block(6, 0, 3, 1) = Eigen::Vector3d(2.219, 2.203, 0.233);
    networkInputScaling.block(9, 0, 3, 1) = Eigen::Vector3d(0.199, 0.199, 0.304);
    networkInputScaling.block(12, 0, 3, 1) = Eigen::Vector3d(1.00, 1.00, 1.0);
    networkInputScaling.block(15, 0, 3, 1) = Eigen::Vector3d(1.00, 1.00, 1.0);
    networkInputScaling.block(18, 0, 3, 1) = Eigen::Vector3d(0.173, 0.087, 0.087);
    networkInputScaling.block(21, 0, 3, 1) = Eigen::Vector3d(0.173, 0.087, 0.087);
    networkInputScaling.block(24, 0, 3, 1) = Eigen::Vector3d(0.173, 0.087, 0.087);
    networkInputScaling.block(27, 0, 3, 1) = Eigen::Vector3d(0.173, 0.087, 0.087);
    networkInputScaling.block(30, 0, 1, 1) << 0.112;
    networkInputScaling.block(31, 0, 4, 1) = Eigen::Vector4d::Ones() * 0.422;
    networkInputScaling.block(35, 0, 3, 1) = Eigen::Vector3d(0.166, 0.160, 0.0348);
    networkInputScaling.block(38, 0, 3, 1) = Eigen::Vector3d(0.166, 0.160, 0.0348);
    networkInputScaling.block(41, 0, 3, 1) = Eigen::Vector3d(0.166, 0.160, 0.0348);
    networkInputScaling.block(44, 0, 3, 1) = Eigen::Vector3d(0.166, 0.160, 0.0348);

    double networkOutputScaling = 0.1111;

    Actuation actuation(parametersDirectory + "a1", networkInputOffset, networkInputScaling,
                        networkOutputScaling, 12);
    // actuation.loadDataScalersFromFile(parametersDirectory + "a1/dataScalers.txt");
    Transform inputTrasnform;
    // actuation.reset();

    // Eigen::MatrixXd jointPositionErrors, jointVelocities;
    // jointPositionErrors.setOnes(12, 1);
    // jointVelocities.setOnes(12, 1);s
    
    // Compute rotation matrix (from base to world) from roll/pitch/yaw - Intrinsic rotations.
    Eigen::Matrix3d w_R_b = inputTrasnform.eulerToRotationMatrix(0.0, 0.0, 0.0);
    Eigen::Matrix3d b_R_w = w_R_b.transpose();

    actuation.states.gravityRot = w_R_b.row(2);
    actuation.states.linVel = b_R_w * Eigen::Vector3d(0., 0., 0.);
    actuation.states.linAcc = b_R_w * Eigen::Vector3d(0., 0., 0.);
    actuation.states.angAcc = b_R_w * Eigen::Vector3d(0., 0., 0.);
    actuation.states.extForce = b_R_w * Eigen::Vector3d(0., 0., 0.);
    actuation.states.extTorque = b_R_w * Eigen::Vector3d(0., 0., 0.);
    // Convert to the base frame
    actuation.states.feetPos.resize(4);
    actuation.states.feetPos[0] = b_R_w * Eigen::Vector3d(0.3, 0.2, -0.4); // LF Foot Position
    actuation.states.feetPos[1] = b_R_w * Eigen::Vector3d(0.3, -0.2, -0.4); // RF Foot Position
    actuation.states.feetPos[2] = b_R_w * Eigen::Vector3d(-0.3, 0.2, -0.4); // LH Foot Position
    actuation.states.feetPos[3] = b_R_w * Eigen::Vector3d(-0.3, -0.2, -0.4); // RH Foot Position.
    // Set friction
    actuation.states.friction = 0.5;
    // Set contact status
    actuation.states.contacts.resize(4);
    actuation.states.contacts[0] = 1;
    actuation.states.contacts[1] = 1;
    actuation.states.contacts[2] = 1;
    actuation.states.contacts[3] = 1;
    // Set feet normals
    actuation.states.normals.resize(4);
    actuation.states.normals[0] = Eigen::Vector3d(0., 0., 1.);
    actuation.states.normals[0] = Eigen::Vector3d(0., 0., 1.);
    actuation.states.normals[0] = Eigen::Vector3d(0., 0., 1.);
    actuation.states.normals[0] = Eigen::Vector3d(0., 0., 1.);

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
        actuation.states.feetPos[0] = b_R_w * Eigen::Vector3d(0.3, 0.2, -0.4); // LF Foot Position
        actuation.states.feetPos[1] = b_R_w * Eigen::Vector3d(0.3, -0.2, -0.4); // RF Foot Position
        actuation.states.feetPos[2] = b_R_w * Eigen::Vector3d(-0.3, 0.2, -0.4); // LH Foot Position
        actuation.states.feetPos[3] = b_R_w * Eigen::Vector3d(-0.3, -0.2, -0.4); // RH Foot Position.
        actuation.states.feetPos[0](0) -= deltaPosRangeVec[i];
        actuation.states.feetPos[1](0) -= deltaPosRangeVec[i];
        actuation.states.feetPos[2](0) -= deltaPosRangeVec[i];
        actuation.states.feetPos[3](0) -= deltaPosRangeVec[i];
        // std::cout << "feetPos[0]" << actuation.states.feetPos[0] << std::endl;
        // std::cout << "feetPos[0]" << actuation.states.feetPos[1] << std::endl;
        // std::cout << "feetPos[0]" << actuation.states.feetPos[2] << std::endl;
        // std::cout << "feetPos[0]" << actuation.states.feetPos[3] << std::endl;
        // Reset the foothold if foot is not in stance. Change its position to nominal
        if (actuation.states.contacts[0] == 0)
        {
            actuation.states.feetPos[0] = Eigen::Vector3d(0.36, 0.21, -0.47);
            actuation.states.normals[0] = Eigen::Vector3d(0., 0., 1.);
        }
        if (actuation.states.contacts[1] == 0)
            actuation.states.feetPos[1] = Eigen::Vector3d(0.36, -0.21, -0.47);
            actuation.states.normals[1] = Eigen::Vector3d(0., 0., 1.);
        if (actuation.states.contacts[2] == 0)
            actuation.states.feetPos[2] = Eigen::Vector3d(-0.36, 0.21, -0.47);
            actuation.states.normals[2] = Eigen::Vector3d(0., 0., 1.);
        if (actuation.states.contacts[3] == 0)
            actuation.states.feetPos[3] = Eigen::Vector3d(-0.36, -0.21, -0.47);
            actuation.states.normals[3] = Eigen::Vector3d(0., 0., 1.);

        posMarginX[i] = actuation.getActuationTorques(actuation.states);
        std::cout << -1 * (actuation.getActuationGradients(actuation.states)(18) +
                                            actuation.getActuationGradients(actuation.states)(21) +
                                            actuation.getActuationGradients(actuation.states)(24) +
                                            actuation.getActuationGradients(actuation.states)(27)) << std::endl;
        // std::cout << posMarginX[i] << std::endl;
    }
    actuation.states.feetPos[0] = b_R_w * Eigen::Vector3d(0.3, 0.2, -0.4); // LF Foot Position
    actuation.states.feetPos[1] = b_R_w * Eigen::Vector3d(0.3, -0.2, -0.4); // RF Foot Position
    actuation.states.feetPos[2] = b_R_w * Eigen::Vector3d(-0.3, 0.2, -0.4); // LH Foot Position
    actuation.states.feetPos[3] = b_R_w * Eigen::Vector3d(-0.3, -0.2, -0.4); // RH Foot Position.
    // Loop over CoM y
    std::cout << "posMarginY: " << std::endl;
    for (auto i = 0; i < numOfTests; ++i)
    {
        actuation.states.feetPos[0] = b_R_w * Eigen::Vector3d(0.3, 0.2, -0.4); // LF Foot Position
        actuation.states.feetPos[1] = b_R_w * Eigen::Vector3d(0.3, -0.2, -0.4); // RF Foot Position
        actuation.states.feetPos[2] = b_R_w * Eigen::Vector3d(-0.3, 0.2, -0.4); // LH Foot Position
        actuation.states.feetPos[3] = b_R_w * Eigen::Vector3d(-0.3, -0.2, -0.4); // RH Foot Position.
        actuation.states.feetPos[0](1) -= deltaPosRangeVec[i];
        actuation.states.feetPos[1](1) -= deltaPosRangeVec[i];
        actuation.states.feetPos[2](1) -= deltaPosRangeVec[i];
        actuation.states.feetPos[3](1) -= deltaPosRangeVec[i];
        // std::cout << "feetPos[0]" << actuation.states.feetPos[0] << std::endl;
        // std::cout << "feetPos[0]" << actuation.states.feetPos[1] << std::endl;
        // std::cout << "feetPos[0]" << actuation.states.feetPos[2] << std::endl;
        // std::cout << "feetPos[0]" << actuation.states.feetPos[3] << std::endl;
        // Reset the foothold if foot is not in stance. Change its position to nominal
        if (actuation.states.contacts[0] == 0)
        {
            actuation.states.feetPos[0] = Eigen::Vector3d(0.36, 0.21, -0.47);
            actuation.states.normals[0] = Eigen::Vector3d(0., 0., 1.);
        }
        if (actuation.states.contacts[1] == 0)
            actuation.states.feetPos[1] = Eigen::Vector3d(0.36, -0.21, -0.47);
            actuation.states.normals[1] = Eigen::Vector3d(0., 0., 1.);
        if (actuation.states.contacts[2] == 0)
            actuation.states.feetPos[2] = Eigen::Vector3d(-0.36, 0.21, -0.47);
            actuation.states.normals[2] = Eigen::Vector3d(0., 0., 1.);
        if (actuation.states.contacts[3] == 0)
            actuation.states.feetPos[3] = Eigen::Vector3d(-0.36, -0.21, -0.47);
            actuation.states.normals[3] = Eigen::Vector3d(0., 0., 1.);

        posMarginY[i] = actuation.getActuationTorques(actuation.states);
        std::cout << -1 * (actuation.getActuationGradients(actuation.states)(19) +
                                            actuation.getActuationGradients(actuation.states)(22) +
                                            actuation.getActuationGradients(actuation.states)(25) +
                                            actuation.getActuationGradients(actuation.states)(28)) << std::endl;
        // std::cout << posMarginY[i] << std::endl;
    }
    // Loop over CoM acc x
    actuation.states.feetPos[0] = b_R_w * Eigen::Vector3d(0.3, 0.2, -0.4); // LF Foot Position
    actuation.states.feetPos[1] = b_R_w * Eigen::Vector3d(0.3, -0.2, -0.4); // RF Foot Position
    actuation.states.feetPos[2] = b_R_w * Eigen::Vector3d(-0.3, 0.2, -0.4); // LH Foot Position
    actuation.states.feetPos[3] = b_R_w * Eigen::Vector3d(-0.3, -0.2, -0.4); // RH Foot Position.
    if (actuation.states.contacts[0] == 0)
    {
        actuation.states.feetPos[0] = Eigen::Vector3d(0.36, 0.21, -0.47);
        actuation.states.normals[0] = Eigen::Vector3d(0., 0., 1.);
    }
    if (actuation.states.contacts[1] == 0)
        actuation.states.feetPos[1] = Eigen::Vector3d(0.36, -0.21, -0.47);
        actuation.states.normals[1] = Eigen::Vector3d(0., 0., 1.);
    if (actuation.states.contacts[2] == 0)
        actuation.states.feetPos[2] = Eigen::Vector3d(-0.36, 0.21, -0.47);
        actuation.states.normals[2] = Eigen::Vector3d(0., 0., 1.);
    if (actuation.states.contacts[3] == 0)
        actuation.states.feetPos[3] = Eigen::Vector3d(-0.36, -0.21, -0.47);
        actuation.states.normals[3] = Eigen::Vector3d(0., 0., 1.);
    std::cout << "posMarginAccX: " << std::endl;
    actuation.states.contacts[0] = 0;
    actuation.states.contacts[1] = 1;
    actuation.states.contacts[2] = 1;
    actuation.states.contacts[3] = 0;
    for (auto i = 0; i < numOfTests; ++i)
    {
        actuation.states.linAcc = b_R_w * Eigen::Vector3d(0., 0., 0.);
        actuation.states.linAcc(0) += deltaAccRangeVec[i];
        posMarginAccX[i] = actuation.getActuationTorques(actuation.states);
        std::cout << actuation.getActuationGradients(actuation.states)(6) << std::endl;
        // std::cout << posMarginAccX[i] << std::endl;
    }

    // std::cout << "posMarginX: " << std::endl;
    // for (auto &p : posMarginX)
    // {
    //     std::cout << p << std::endl;
    // }
    // std::cout << "posMarginY: " << std::endl;
    // for (auto &p : posMarginY)
    // {
    //     std::cout << p << std::endl;
    // }


    // std::cout << "w_R_b: " << w_R_b << std::endl;
    // std::cout << "gravityRot: " << actuation.states.gravityRot << std::endl;
    // std::cout << "linVel: " << actuation.states.linVel << std::endl;
    // std::cout << "linAcc: " << actuation.states.linAcc << std::endl;
    // std::cout << "angAcc: " << actuation.states.angAcc << std::endl;
    // std::cout << "extForce: " << actuation.states.extForce << std::endl;
    // std::cout << "extTorque: " << actuation.states.extTorque << std::endl;
    // for (auto &i : actuation.states.feetPos)
    //     std::cout << "feetPos: " << i << std::endl;
    // std::cout << "friction: " << actuation.states.friction << std::endl;
    // for (auto &i : actuation.states.contacts)
    //     std::cout << "contacts: " << i << std::endl;
    // for (auto &i : actuation.states.normals)
    //     std::cout << "normals: " << i << std::endl;


    // std::cout << "\nMargin:" << std::endl;
    // for (auto s = 0; s < 1; ++s) {
    //     std::cout << actuation.getActuationTorques(actuation.states) << std::endl;
    //     std::cout << "---------" <<std::endl;
    // }

    // actuation.reset();

    // std::cout << "\nComputing Inference Latency" << std::endl;

    // std::chrono::high_resolution_clock::time_point startTimePoint{}, endTimePoint{};

    // // jointPositionErrors.setRandom(12, 1000);
    // // jointVelocities.setRandom(12, 1000);
    // Actuation::StatesStruct latency_states;
    // std::array<long, 1000> latency{};
    // double latencyMargin;
    // std::random_device rd;
    // std::mt19937 gen(rd());

    // std::cout << "Latency margins: " << std::endl;
    // for (auto c = 0; c < 1000; ++c) {

    //     latency_states.gravityRot = Eigen::Vector3d::Random();
    //     latency_states.linVel = Eigen::Vector3d::Random();
    //     latency_states.linAcc = Eigen::Vector3d::Random();
    //     latency_states.angAcc = Eigen::Vector3d::Random();
    //     latency_states.extForce = Eigen::Vector3d::Random();
    //     latency_states.extTorque = Eigen::Vector3d::Random();
    //     latency_states.feetPos.resize(4);
    //     latency_states.feetPos[0] = Eigen::Vector3d::Random();
    //     latency_states.feetPos[1] = Eigen::Vector3d::Random();
    //     latency_states.feetPos[2] = Eigen::Vector3d::Random();
    //     latency_states.feetPos[3] = Eigen::Vector3d::Random();
    //     std::uniform_real_distribution<float> disFric(0.5, 0.8);
    //     latency_states.friction = disFric(gen);
    //     std::bernoulli_distribution disCont(0.5);
    //     latency_states.contacts.resize(4);
    //     for (auto &i : latency_states.contacts)
    //         i = (int)disCont(gen);
    //     latency_states.normals.resize(4);
    //     for (auto &i : latency_states.normals)
    //         i = Eigen::Vector3d(0., 0., 1);
    //     startTimePoint = std::chrono::high_resolution_clock::now();
    //     latencyMargin = actuation.getActuationTorques(latency_states);
    //     endTimePoint = std::chrono::high_resolution_clock::now();

    //     latency[c] = std::chrono::duration_cast<std::chrono::microseconds>(endTimePoint - startTimePoint).count();
    //     if (c % 10 == 0) // Print less
    //         std::cout << latencyMargin << std::endl;
    // }

    // double mean = 0;

    // for (auto c = 0; c < 1000; ++c) {
    //     mean += static_cast<double>(latency[c]);
    // }

    // mean = mean / 1000.;

    // std::cout << "Latency Mean: " << mean << " us" << std::endl;

    // double std = 0;

    // for (auto c = 0; c < 1000; ++c) {
    //     std += std::pow(static_cast<double>(latency[c]) - mean, 2);
    // }

    // std = std::pow(std / 1000., 0.5);

    // std::cout << "Latency Std: " << std << " us" << std::endl;

    return 0;
}