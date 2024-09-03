// Created by Abdelrahman Abdalla

#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include <Eigen/Dense>

class Transform {
public:

    // Transform() : roll_{0.}, pitch_{0.}, yaw_{0.} {}
    // Transform(){}

    // Function to compute rotation matrix from Euler angles using Eigen
    const Eigen::Matrix3d eulerToRotationMatrix(double roll, double pitch, double yaw)
    {
        // Convert Euler angles to radians
        double phi = roll;
        double theta = pitch;
        double psi = yaw;

        // Compute trigonometric values
        double cosPhi = std::cos(phi);
        double sinPhi = std::sin(phi);
        double cosTheta = std::cos(theta);
        double sinTheta = std::sin(theta);
        double cosPsi = std::cos(psi);
        double sinPsi = std::sin(psi);

        // Compute rotation matrix elements
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix(0, 0) = cosTheta * cosPsi;
        rotationMatrix(0, 1) = sinPhi * sinTheta * cosPsi - cosPhi * sinPsi;
        rotationMatrix(0, 2) = cosPhi * sinTheta * cosPsi + sinPhi * sinPsi;
        rotationMatrix(1, 0) = cosTheta * sinPsi;
        rotationMatrix(1, 1) = sinPhi * sinTheta * sinPsi + cosPhi * cosPsi;
        rotationMatrix(1, 2) = cosPhi * sinTheta * sinPsi - sinPhi * cosPsi;
        rotationMatrix(2, 0) = -sinTheta;
        rotationMatrix(2, 1) = sinPhi * cosTheta;
        rotationMatrix(2, 2) = cosPhi * cosTheta;

        return rotationMatrix;
    }

};

#endif // TRANSFORM_HPP