
#ifndef ROBOT_UTILS_H
#define ROBOT_UTILS_H

#include <eigen/Eigen/Eigen>
#include <eigen/Eigen/LU>


#include <math.h>

#define DEG2RAD 0.01745329252 //(PI() / 180.0)
#define RAD2DEG 57.2957795131 //(180.0 / PI())

namespace roblib
{
    namespace utils {

        // Create a vector from 3 input values
        // inputs: v1, v2 and v3
        // return: [v1, v2, v3]
        Eigen::Vector3d vector3(double v1, double v2, double v3);

        // Create a mass matrix from 9 input values
        // inputs: m11, m12, m1, m21, m22, m23, m31, m32, m33
        // return: [m11, m12, m13
        //          m21, m22, m23
        //          m31, m32, m33]
        Eigen::Matrix3d matrix3(double m11, double m12, double m13, double m21, double m22, double m23, double m31, double m32, double m33);

        // Create a symmetry inertia matrix from 6 input values
        // inputs: ixx, ixy, ixz, iyy, iyz, izz
        // return: [ixx, ixy, ixz
        //          ixy, iyy, iyz
        //          ixz, iyz, izz]
        Eigen::Matrix3d inertiaMatrix(double ixx, double ixy, double ixz, double iyy, double iyz, double izz);

        // Convert position values to a translation vector
        // inputs: x, y, z
        // return: [x, y, z]'
        Eigen::Vector3d convertXYZToVector(double x, double y, double z);

        // Convert roll angle to a rotation matrix
        Eigen::Matrix3d convertRollAngleToRotationMatrix(double angle);

        // Convert pitch angle to a rotation matrix
        Eigen::Matrix3d convertPitchAngleToRotationMatrix(double angle);

        // Convert yaw angle to a rotation matrix
        Eigen::Matrix3d convertYawAngleToRotationMatrix(double angle);

        // Obtain roll, pitch and yaw angles from a rotation matrix
        Eigen::Vector3d convertRotationMatrixToRPYVector(const Eigen::Matrix3d& rotation_matrix);
       
        // Obtain rotation matrix from roll, pitch and yaw angles 
        Eigen::Matrix3d convertRPYToRotationMatrix(double roll, double pitch, double yaw);
        
        // Obtain unit quaternion from roll, pitch and yaw angles 
        Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw);
       
        // Convert rotation matrix to unit quaternion 
        Eigen::Quaterniond convertRotationMatrixToQuaternion(const Eigen::Matrix3d& rotation_matrix);
       
        // Convert quaternion to roll, pitch and yaw angles 
        Eigen::Vector3d convertQuaternionToRPYVector(const Eigen::Quaterniond& quaternion);
        
        // Convert unit quaternio to rotation matrix
        Eigen::Matrix3d convertQuaternionToRotationMatrix(const Eigen::Quaterniond& quaternion);
        
        // Convert rotation matrix to angular speed vector 
        Eigen::Vector3d convertRotationMatrixToOmega(const Eigen::Matrix3d& rotation_matrix);

        // Obtain homogeneous transformation matrix
        Eigen::Matrix4d convertXYZRPYToTransformationMatrix(double x, double y, double z, double roll, double pitch, double yaw);

        // Obtain homogeneous transformation matrix with no rotation but only translation
        Eigen::Matrix4d convertXYZToTransformationMatrix(double x, double y, double z);
        
        // Obtain homogeneous transformation matrix from roll, pit and yaw
        Eigen::Matrix4d convertRPYToTransformationMatrix(double roll, double pitch, double yaw);

        // Obtain angular acceleration
        Eigen::Vector3d convertOmegaToRPYVelocity(Eigen::Vector3d rpy_vector, Eigen::Vector3d omega);

        // Obtain angular speed
        Eigen::Vector3d convertRPYVelocityToOmega(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity);
        
        Eigen::Vector3d convertOmegaDotToRPYAcceleration(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d omega_dot);
        
        Eigen::Vector3d convertRPYAccelerationToOmegaDot(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d rpy_acceleration);

        //sign function
        double sign(double value);

        // Inverse transformation
        Eigen::Matrix4d inverseTransformationMatrix(const Eigen::MatrixXd& transformation_matrix);

        // Matrix logarithm
        Eigen::Vector3d matrixLogarithm(Eigen::Matrix3d rotation_matrix);

        // Crete  skew symmetry matrix from a vector
        Eigen::Matrix3d skewSymmetricMatrix(Eigen::Vector3d v);

        //Rodrigues formula to obtain rotation matrix
        Eigen::Matrix3d rodriguesRotationMatrix(Eigen::Vector3d axis, double angle);

        // Poisiton error
        Eigen::Vector3d positionDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position);

        // Orientation error
        Eigen::Vector3d orientationDifference(Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation);

        // Pose error
        Eigen::VectorXd poseDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position,
            Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation);

        bool NearZero(const double);

        Eigen::Vector4d AxisAng3(const Eigen::Vector3d&);

        Eigen::MatrixXd Normalize(Eigen::MatrixXd);

        Eigen::MatrixXd TransInv(const Eigen::MatrixXd&);

        std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd&);

        Eigen::Matrix3d VecToso3(const Eigen::Vector3d&);

        Eigen::Vector3d so3ToVec(const Eigen::MatrixXd&);

        Eigen::MatrixXd ad(Eigen::VectorXd);

        Eigen::MatrixXd Adjoint(const Eigen::MatrixXd&);

        Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d&);

        Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd&);

        Eigen::MatrixXd VecTose3(const Eigen::VectorXd&);

        Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd&);

        Eigen::MatrixXd MassMatrix(const Eigen::VectorXd&,
            const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

        Eigen::VectorXd VelQuadraticForces(const Eigen::VectorXd&, const Eigen::VectorXd&,
            const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

        Eigen::VectorXd EndEffectorForces(const Eigen::VectorXd&, const Eigen::VectorXd&,
            const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

        Eigen::VectorXd GravityForces(const Eigen::VectorXd&, const Eigen::VectorXd&,
            const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

        Eigen::VectorXd InverseDynamics(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
            const Eigen::VectorXd&, const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&,
            const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

        Eigen::VectorXd ForwardDynamics(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
            const Eigen::VectorXd&, const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&,
            const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

        void EulerStep(Eigen::VectorXd&, Eigen::VectorXd&, const Eigen::VectorXd&, double);

        Eigen::VectorXd ComputedTorque(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
            const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&,
            const Eigen::MatrixXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, double, double, double);

        double CubicTimeScaling(double, double);

        double QuinticTimeScaling(double, double);

        Eigen::MatrixXd JointTrajectory(const Eigen::VectorXd&, const Eigen::VectorXd&, double, int, int);

        std::vector<Eigen::MatrixXd> SimulateControl(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
            const Eigen::MatrixXd&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&,
            const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&,
            const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&,
            double, double, double, double, int);


    } // utils
} // namespace roblib

#endif // ROBOT_UTILS_H
