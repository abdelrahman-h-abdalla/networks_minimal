// Created by Abdelrahman Abdalla.

#ifndef FEASIBILITYMARGIN_HPP
#define FEASIBILITYMARGIN_HPP

#include "networks_minimal/MultiLayerPerceptron.hpp"
#include <Eigen/Dense>


class FeasibilityMargin {
public:

    struct StatesStruct{
        Eigen::Vector3d gravityRot;
        Eigen::Vector3d linAcc;
        Eigen::Vector3d angAcc;
        Eigen::Vector3d extForce;
        Eigen::Vector3d extTorque;
        std::vector <Eigen::Vector3d> feetPos;
        double friction;
        std::vector <Eigen::Vector3d> normals;
    } states;
    
    FeasibilityMargin() = delete;

    explicit FeasibilityMargin(const std::string &parametersDirectory) :
            mlpBlock_({40, 256, 128, 128, 1}, activation.softsign, parametersDirectory + "/mlp.txt"),
            networkOutputOffsets_{0.},
            networkOutputMultipliers_{1.},
            networkOutput_{0} {

        networkInput_.setZero(40, 1);
        networkInputOffsets_.setOnes(40, 1);
        networkInputMultipliers_.setOnes(40, 1);
    }

    FeasibilityMargin(const std::string &parametersDirectory,
                    const std::vector<unsigned int> &networkLayers,
                    const std::reference_wrapper <Activation> &networkActivation) :
            mlpBlock_(networkLayers, networkActivation, parametersDirectory + "/mlp.txt"),
            networkOutputOffsets_{0.},
            networkOutputMultipliers_{1.},
            networkOutput_{0} {

        networkInput_.setZero(40, 1);
        networkInputOffsets_.setOnes(40, 1);
        networkInputMultipliers_.setOnes(40, 1);
    }

    FeasibilityMargin(const std::string &parametersDirectory, const Eigen::MatrixXd &networkInputOffsets,
              const Eigen::MatrixXd &networkInputMultipliers, const double networkOutputOffsets,
              const double networkOutputMultipliers) :
            FeasibilityMargin(parametersDirectory) {
        networkInputOffsets_ = networkInputOffsets;
        networkInputMultipliers_ = networkInputMultipliers;
        networkOutputOffsets_ = networkOutputOffsets;
        networkOutputMultipliers_ = networkOutputMultipliers;
    }

    // TODO: Only works for full stance
    void loadDataScalersFromFile(const std::string &dataScalersPath) {
        /// https://stackoverflow.com/a/22988866

        if (dataScalersPath_ == dataScalersPath) return;

        std::ifstream dataFile;
        dataFile.open(dataScalersPath);
        std::string line;
        std::vector<double> values1, values2; // Two vectors for offsets and multipliers
        unsigned int rows = 0;

        if (!dataFile.is_open()) {
            std::cerr << "Failed to open file: " << dataScalersPath << std::endl;
            return;
        }

        // Load first array
        if (std::getline(dataFile, line)) {
            std::stringstream lineStream(line);
            std::string cell;

            while (std::getline(lineStream, cell, ' ')) {
                values1.push_back(std::stod(cell));
            }

            ++rows;
        }
        // Load second array
        if (std::getline(dataFile, line)) {
            std::stringstream lineStream(line);
            std::string cell;

            while (std::getline(lineStream, cell, ' ')) {
                values2.push_back(std::stod(cell));
            }

            ++rows;
        }

        if (rows != 2) {
            std::cerr << "Normalization file does not contain two arrays of correct shape" << std::endl;
            values1 = std::vector<double>(41, 0.0);
            values2 = std::vector<double>(41, 0.0);
        }

        // Import offsets and multipliers (include inputs then outputs for each array)
        Eigen::VectorXd dataOffsets, dataMultipliers;
        dataOffsets = Eigen::Map<Eigen::VectorXd>(values1.data(), values1.size());
        dataMultipliers = Eigen::Map<Eigen::VectorXd>(values2.data(), values2.size());
        // Extract input scalers
        networkInputOffsets_ = dataOffsets.head(40);
        networkInputMultipliers_ = dataMultipliers.head(40);
        // Extract output scalers
        networkOutputOffsets_ = dataOffsets.tail(1)(0);
        networkOutputMultipliers_ = dataMultipliers.tail(1)(0);
        // std::cout << "File networkInputOffset_: " << std::endl;
        // std::cout <<networkInputOffsets_ << std::endl;
        // std::cout << "File networkInputScaling_: " << std::endl;
        // std::cout << networkInputMultipliers_ << std::endl;
        // std::cout << "File networkOutputOffset_: " << std::endl;
        // std::cout << networkOutputOffsets_ << std::endl;
        // std::cout << "File networkOutputScaling_: " << std::endl;
        // std::cout << networkOutputMultipliers_ << std::endl;

        dataScalersPath_ = dataScalersPath;
    }

    // TODO: Only works for full stance
    const double &getFeasibilityMargin(const StatesStruct &states) {

        networkInput_.col(0) << states.gravityRot,
                                states.linAcc,
                                states.angAcc,
                                states.extForce,
                                states.extTorque,
                                states.feetPos[0],
                                states.feetPos[1],
                                states.feetPos[2],
                                states.feetPos[3],
                                states.friction,
                                states.normals[0],
                                states.normals[1],
                                states.normals[2],
                                states.normals[3];
        networkInput_ = (networkInput_ - networkInputOffsets_).cwiseQuotient(networkInputMultipliers_);
        networkOutput_ = mlpBlock_.forward(networkInput_)(0, 0) * networkOutputMultipliers_ + networkOutputOffsets_;

        return networkOutput_;
    }

    // TODO: Only works for full stance
    Eigen::MatrixXd getFeasibilityGradients(const StatesStruct &states) {

        networkInput_.col(0) << states.gravityRot,
                                states.linAcc,
                                states.angAcc,
                                states.extForce,
                                states.extTorque,
                                states.feetPos[0],
                                states.feetPos[1],
                                states.feetPos[2],
                                states.feetPos[3],
                                states.friction,
                                states.normals[0],
                                states.normals[1],
                                states.normals[2],
                                states.normals[3];
        networkInput_ = (networkInput_ - networkInputOffsets_).cwiseQuotient(networkInputMultipliers_);
        Eigen::MatrixXd networkGradient;
        networkGradient = (mlpBlock_.gradient(networkInput_) *
                            networkOutputMultipliers_).cwiseQuotient(networkInputMultipliers_);

        return networkGradient;
    }

private:

    MultiLayerPerceptron mlpBlock_;

    // Scaling offsets for (sorted inputs then outputs)
    std::string dataScalersPath_;
    Eigen::MatrixXd networkInputOffsets_, networkInputMultipliers_;
    double networkOutputOffsets_, networkOutputMultipliers_;

    Eigen::MatrixXd networkInput_;
    double networkOutput_;


};

#endif // FEASIBILITYMARGIN_HPP
