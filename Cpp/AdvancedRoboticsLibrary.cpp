
#include "AdvancedRoboticsLibrary.h"

Eigen::Vector3d roblib::utils::vector3(double v1, double v2, double v3)
{
    Eigen::Vector3d temp;
    temp << v1, v2, v3;
    return temp;
}

Eigen::Matrix3d roblib::utils::matrix3(double m11, double m12, double m13,
    double m21, double m22, double m23,
    double m31, double m32, double m33)
{
    Eigen::Matrix3d temp;
    temp << m11, m12, m13, m21, m22, m23, m31, m32, m33;
    return temp;
}

Eigen::Matrix3d roblib::utils::inertiaMatrix(double ixx, double ixy, double ixz, double iyy, double iyz, double izz)
{
    Eigen::Matrix3d inertia;
    inertia <<
        ixx, ixy, ixz,
        ixy, iyy, iyz,
        ixz, iyz, izz;

    return inertia;
}

// Translation Vector from x, y and z co-ordinates
Eigen::Vector3d roblib::utils::convertXYZToVector(double x, double y, double z)
{
    Eigen::Vector3d position;
    position << x, y, z;

    return position;
}

//Rotation about x-axis by an angle roll
Eigen::Matrix3d roblib::utils::convertRollAngleToRotationMatrix(double angle)
{
    Eigen::Matrix3d rotation(3, 3);
    rotation <<
        1.0, 0.0, 0.0,
        0.0, cos(angle), -sin(angle),
        0.0, sin(angle), cos(angle);

    return rotation;
}

//Rotation about y-axis by an angle pitch
Eigen::Matrix3d roblib::utils::convertPitchAngleToRotationMatrix(double angle)
{
    Eigen::Matrix3d rotation(3, 3);
    rotation <<
        cos(angle), 0.0, sin(angle),
        0.0, 1.0, 0.0,
        -sin(angle), 0.0, cos(angle);

    return rotation;
}

//Rotation about z-axis by an angle yaw
Eigen::Matrix3d roblib::utils::convertYawAngleToRotationMatrix(double angle)
{
    Eigen::Matrix3d rotation(3, 3);
    rotation <<
        cos(angle), -sin(angle), 0.0,
        sin(angle), cos(angle), 0.0,
        0.0, 0.0, 1.0;

    return rotation;
}

// Convert rotation matrix to roll, pitch and yaw angles and return angles
Eigen::Vector3d roblib::utils::convertRotationMatrixToRPYVector(const Eigen::Matrix3d& rotation)
{
    Eigen::Vector3d rpy;// = Eigen::MatrixXd::Zero(3,1);
    rpy.coeffRef(0, 0) = atan2(rotation.coeff(2, 1), rotation.coeff(2, 2));
    rpy.coeffRef(1, 0) = atan2(-rotation.coeff(2, 0), sqrt(pow(rotation.coeff(2, 1), 2) + pow(rotation.coeff(2, 2), 2)));
    rpy.coeffRef(2, 0) = atan2(rotation.coeff(1, 0), rotation.coeff(0, 0));

    return rpy;
}

// Calculate rotation matrix from individual rotation about roll, pitch and yaw angles
Eigen::Matrix3d roblib::utils::convertRPYToRotationMatrix(double roll, double pitch, double yaw)
{
    Eigen::Matrix3d rotation = roblib::utils::convertYawAngleToRotationMatrix(yaw) * roblib::utils::convertPitchAngleToRotationMatrix(pitch) * roblib::utils::convertRollAngleToRotationMatrix(roll);

    return rotation;
}

Eigen::Quaterniond roblib::utils::convertRPYToQuaternion(double roll, double pitch, double yaw)
{
    Eigen::Quaterniond quaternion;
    quaternion = roblib::utils::convertRPYToRotationMatrix(roll, pitch, yaw);

    return quaternion;
}

Eigen::Quaterniond roblib::utils::convertRotationMatrixToQuaternion(const Eigen::Matrix3d& rotation)
{
    Eigen::Quaterniond quaternion;
    quaternion = rotation;

    return quaternion;
}

Eigen::Vector3d roblib::utils::convertQuaternionToRPYVector(const Eigen::Quaterniond& quaternion)
{
    Eigen::Vector3d rpy = roblib::utils::convertRotationMatrixToRPYVector(quaternion.toRotationMatrix());

    return rpy;
}

Eigen::Matrix3d roblib::utils::convertQuaternionToRotationMatrix(const Eigen::Quaterniond& quaternion)
{
    Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

    return rotation;
}

Eigen::Vector3d roblib::utils::convertRotationMatrixToOmega(const Eigen::Matrix3d& rotation_matrix)
{
    return roblib::utils::matrixLogarithm(rotation_matrix);
}

// Calculate Homogeneous Transformation Matrix
Eigen::Matrix4d roblib::utils::convertXYZRPYToTransformationMatrix(double position_x, double position_y, double position_z, double roll, double pitch, double yaw)
{
    Eigen::Matrix4d transformation = roblib::utils::convertRPYToTransformationMatrix(roll, pitch, yaw);
    transformation.coeffRef(0, 3) = position_x;
    transformation.coeffRef(1, 3) = position_y;
    transformation.coeffRef(2, 3) = position_z;

    return transformation;
}

Eigen::Matrix4d roblib::utils::convertXYZToTransformationMatrix(double position_x, double position_y, double position_z)
{
    Eigen::Matrix4d mat_translation;

    mat_translation <<
        1, 0, 0, position_x,
        0, 1, 0, position_y,
        0, 0, 1, position_z,
        0, 0, 0, 1;

    return mat_translation;
}

Eigen::Matrix4d roblib::utils::convertRPYToTransformationMatrix(double roll, double pitch, double yaw)
{
    double sr = sin(roll), cr = cos(roll);
    double sp = sin(pitch), cp = cos(pitch);
    double sy = sin(yaw), cy = cos(yaw);

    Eigen::Matrix4d mat_roll;
    Eigen::Matrix4d mat_pitch;
    Eigen::Matrix4d mat_yaw;

    mat_roll <<
        1, 0, 0, 0,
        0, cr, -sr, 0,
        0, sr, cr, 0,
        0, 0, 0, 1;

    mat_pitch <<
        cp, 0, sp, 0,
        0, 1, 0, 0,
        -sp, 0, cp, 0,
        0, 0, 0, 1;

    mat_yaw <<
        cy, -sy, 0, 0,
        sy, cy, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Matrix4d mat_rpy = (mat_yaw * mat_pitch) * mat_roll;

    return mat_rpy;
}

Eigen::Matrix4d roblib::utils::inverseTransformationMatrix(const Eigen::MatrixXd& transform)
{
    // If T is Transform Matrix A from B, the BOA is translation component coordi. B to coordi. A

    Eigen::Vector3d vec_boa;
    Eigen::Vector3d vec_x, vec_y, vec_z;
    Eigen::Matrix4d inv_t;

    vec_boa(0) = -transform(0, 3);
    vec_boa(1) = -transform(1, 3);
    vec_boa(2) = -transform(2, 3);

    vec_x(0) = transform(0, 0); vec_x(1) = transform(1, 0); vec_x(2) = transform(2, 0);
    vec_y(0) = transform(0, 1); vec_y(1) = transform(1, 1); vec_y(2) = transform(2, 1);
    vec_z(0) = transform(0, 2); vec_z(1) = transform(1, 2); vec_z(2) = transform(2, 2);

    inv_t <<
        vec_x(0), vec_x(1), vec_x(2), vec_boa.dot(vec_x),
        vec_y(0), vec_y(1), vec_y(2), vec_boa.dot(vec_y),
        vec_z(0), vec_z(1), vec_z(2), vec_boa.dot(vec_z),
        0, 0, 0, 1;

    return inv_t;
}

//Dynamic value
Eigen::Vector3d roblib::utils::convertOmegaToRPYVelocity(Eigen::Vector3d rpy_vector, Eigen::Vector3d omega)
{
    Eigen::Matrix3d c_inverse;
    Eigen::Vector3d rpy_velocity;

    c_inverse << 1, sin(rpy_vector(0))* tan(rpy_vector(1)), cos(rpy_vector(0))* tan(rpy_vector(1)),
        0, cos(rpy_vector(0)), -sin(rpy_vector(0)),
        0, sin(rpy_vector(0)) / cos(rpy_vector(1)), cos(rpy_vector(0)) / cos(rpy_vector(1));

    rpy_velocity = c_inverse * omega;
    return rpy_velocity;
}

Eigen::Vector3d roblib::utils::convertRPYVelocityToOmega(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity)
{
    Eigen::Matrix3d c;
    Eigen::Vector3d omega;

    c << 1, 0, -sin(rpy_vector(1)),
        0, cos(rpy_vector(0)), sin(rpy_vector(0))* cos(rpy_vector(1)),
        0, -sin(rpy_vector(0)), cos(rpy_vector(0))* cos(rpy_vector(1));

    omega = c * rpy_velocity;
    return omega;
}

Eigen::Vector3d roblib::utils::convertOmegaDotToRPYAcceleration(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d omega_dot)
{
    Eigen::Vector3d c_dot;
    Eigen::Matrix3d c_inverse;
    Eigen::Vector3d rpy_acceleration;

    c_dot << -cos(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2],
        -sin(rpy_vector[0]) * rpy_velocity[0] * rpy_velocity[1] - sin(rpy_vector[0]) * sin(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2] + cos(rpy_vector[0]) * cos(rpy_vector[1]) * rpy_velocity[0] * rpy_velocity[2],
        -cos(rpy_vector[0]) * rpy_velocity[0] * rpy_velocity[1] - sin(rpy_vector[0]) * cos(rpy_vector[1]) * rpy_velocity[0] * rpy_velocity[2] - cos(rpy_vector[0]) * sin(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2];

    c_inverse << 1, sin(rpy_vector(0))* tan(rpy_vector(1)), cos(rpy_vector(0))* tan(rpy_vector(1)),
        0, cos(rpy_vector(0)), -sin(rpy_vector(0)),
        0, sin(rpy_vector(0)) / cos(rpy_vector(1)), cos(rpy_vector(0)) / cos(rpy_vector(1));

    rpy_acceleration = c_inverse * (omega_dot - c_dot);
    return rpy_acceleration;
}

Eigen::Vector3d roblib::utils::convertRPYAccelerationToOmegaDot(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d rpy_acceleration)
{
    Eigen::Vector3d c_dot;
    Eigen::Matrix3d c;
    Eigen::Vector3d omega_dot;

    c_dot << -cos(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2],
        -sin(rpy_vector[0]) * rpy_velocity[0] * rpy_velocity[1] - sin(rpy_vector[0]) * sin(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2] + cos(rpy_vector[0]) * cos(rpy_vector[1]) * rpy_velocity[0] * rpy_velocity[2],
        -cos(rpy_vector[0]) * rpy_velocity[0] * rpy_velocity[1] - sin(rpy_vector[0]) * cos(rpy_vector[1]) * rpy_velocity[0] * rpy_velocity[2] - cos(rpy_vector[0]) * sin(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2];

    c << 1, 0, -sin(rpy_vector(1)),
        0, cos(rpy_vector(0)), sin(rpy_vector(0))* cos(rpy_vector(1)),
        0, -sin(rpy_vector(0)), cos(rpy_vector(0))* cos(rpy_vector(1));

    omega_dot = c_dot + c * rpy_acceleration;
    return omega_dot;
}


double roblib::utils::sign(double value)
{
    if (value >= 0.0)
    {
        return 1.0;
    }
    else
    {
        return -1.0;
    }
}

Eigen::Vector3d roblib::utils::matrixLogarithm(Eigen::Matrix3d rotation_matrix)
{
    Eigen::Matrix3d R = rotation_matrix;
    Eigen::Vector3d l = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation_vector = Eigen::Vector3d::Zero();

    double theta = 0.0;
    // double diag = 0.0;
    bool diagonal_matrix = R.isDiagonal();

    l << R(2, 1) - R(1, 2),
        R(0, 2) - R(2, 0),
        R(1, 0) - R(0, 1);
    theta = atan2(l.norm(), R(0, 0) + R(1, 1) + R(2, 2) - 1);
    // diag = R.determinant();

    if (R.isIdentity())
    {
        rotation_vector.setZero(3);
        return rotation_vector;
    }

    if (diagonal_matrix == true)
    {
        rotation_vector << R(0, 0) + 1, R(1, 1) + 1, R(2, 2) + 1;
        //rotation_vector = rotation_vector * M_PI_2; (jadav did it for work)
    }
    else
    {
        rotation_vector = theta * (l / l.norm());
    }
    return rotation_vector;
}

Eigen::Matrix3d roblib::utils::skewSymmetricMatrix(Eigen::Vector3d v)
{
    Eigen::Matrix3d skew_symmetric_matrix = Eigen::Matrix3d::Zero();
    skew_symmetric_matrix << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return skew_symmetric_matrix;
}

Eigen::Matrix3d roblib::utils::rodriguesRotationMatrix(Eigen::Vector3d axis, double angle)
{
    Eigen::Matrix3d skew_symmetric_matrix = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Identity_matrix = Eigen::Matrix3d::Identity();

    skew_symmetric_matrix = roblib::utils::skewSymmetricMatrix(axis);
    rotation_matrix = Identity_matrix +
        skew_symmetric_matrix * sin(angle) +
        skew_symmetric_matrix * skew_symmetric_matrix * (1 - cos(angle));
    return rotation_matrix;
}

Eigen::Vector3d roblib::utils::positionDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position)
{
    Eigen::Vector3d position_difference;
    position_difference = desired_position - present_position;

    return position_difference;
}

Eigen::Vector3d roblib::utils::orientationDifference(Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation)
{
    Eigen::Vector3d orientation_difference;
    orientation_difference = present_orientation * roblib::utils::matrixLogarithm(present_orientation.transpose() * desired_orientation);

    return orientation_difference;
}

Eigen::VectorXd roblib::utils::poseDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position,
    Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation)
{
    Eigen::Vector3d position_difference;
    Eigen::Vector3d orientation_difference;
    Eigen::VectorXd pose_difference(6);

    position_difference = roblib::utils::positionDifference(desired_position, present_position);
    orientation_difference = roblib::utils::orientationDifference(desired_orientation, present_orientation);
    pose_difference << position_difference(0), position_difference(1), position_difference(2),
        orientation_difference(0), orientation_difference(1), orientation_difference(2);

    return pose_difference;
}



bool roblib::utils::NearZero(const double val)
{
    return (std::abs(val) < .000001);
}

Eigen::MatrixXd roblib::utils::TransInv(const Eigen::MatrixXd& transform)
{
    auto rp = roblib::utils::TransToRp(transform);
    auto Rt = rp.at(0).transpose();
    auto t = -(Rt * rp.at(1));
    Eigen::MatrixXd inv(4, 4);
    inv = Eigen::MatrixXd::Zero(4, 4);
    inv.block(0, 0, 3, 3) = Rt;
    inv.block(0, 3, 3, 1) = t;
    inv(3, 3) = 1;
    return inv;
}

std::vector<Eigen::MatrixXd> roblib::utils::TransToRp(const Eigen::MatrixXd& T) {
    std::vector<Eigen::MatrixXd> Rp_ret;
    Eigen::Matrix3d R_ret;
    // Get top left 3x3 corner
    R_ret = T.block<3, 3>(0, 0);

    Eigen::Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));

    Rp_ret.push_back(R_ret);
    Rp_ret.push_back(p_ret);

    return Rp_ret;
}

Eigen::Matrix3d roblib::utils::VecToso3(const Eigen::Vector3d& omg)
{
    Eigen::Matrix3d m_ret;
    m_ret << 0, -omg(2), omg(1),
        omg(2), 0, -omg(0),
        -omg(1), omg(0), 0;
    return m_ret;
}

Eigen::Vector3d roblib::utils::so3ToVec(const Eigen::MatrixXd& so3mat)
{
    Eigen::Vector3d v_ret;
    v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
    return v_ret;
}

Eigen::MatrixXd roblib::utils::VecTose3(const Eigen::VectorXd& V)
{
    // Separate angular (exponential representation) and linear velocities
    Eigen::Vector3d exp(V(0), V(1), V(2));
    Eigen::Vector3d linear(V(3), V(4), V(5));

    // Fill in values to the appropriate parts of the transformation matrix
    Eigen::MatrixXd m_ret(4, 4);
    m_ret << roblib::utils::VecToso3(exp), linear,
        0, 0, 0, 0;

    return m_ret;
}

Eigen::MatrixXd roblib::utils::Normalize(Eigen::MatrixXd V)
{
    V.normalize();
    return V;
}

Eigen::Vector4d roblib::utils::AxisAng3(const Eigen::Vector3d& expc3)
{
    Eigen::Vector4d v_ret;
    v_ret << roblib::utils::Normalize(expc3), expc3.norm();
    return v_ret;
}

Eigen::MatrixXd roblib::utils::Adjoint(const Eigen::MatrixXd& T) {
    std::vector<Eigen::MatrixXd> R = roblib::utils::TransToRp(T);
    Eigen::MatrixXd ad_ret(6, 6);
    ad_ret = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3, 3);
    ad_ret << R[0], zeroes,
        roblib::utils::VecToso3(R[1])* R[0], R[0];
    return ad_ret;
}

Eigen::MatrixXd roblib::utils::ad(Eigen::VectorXd V)
{
    Eigen::Matrix3d omgmat = roblib::utils::VecToso3(Eigen::Vector3d(V(0), V(1), V(2)));

    Eigen::MatrixXd result(6, 6);
    result.topLeftCorner<3, 3>() = omgmat;
    result.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero(3, 3);
    result.bottomLeftCorner<3, 3>() = roblib::utils::VecToso3(Eigen::Vector3d(V(3), V(4), V(5)));
    result.bottomRightCorner<3, 3>() = omgmat;
    return result;
}

Eigen::Matrix3d roblib::utils::MatrixExp3(const Eigen::Matrix3d& so3mat) {
    Eigen::Vector3d omgtheta = roblib::utils::so3ToVec(so3mat);

    Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
    if (roblib::utils::NearZero(so3mat.norm()))
    {
        return m_ret;
    }
    else 
    {
        double theta = (roblib::utils::AxisAng3(omgtheta))(3);
        Eigen::Matrix3d omgmat = so3mat * (1 / theta);
        return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
    }
}

Eigen::MatrixXd roblib::utils::MatrixExp6(const Eigen::MatrixXd& se3mat)
{
    // Extract the angular velocity vector from the transformation matrix
    Eigen::Matrix3d se3mat_cut = se3mat.block<3, 3>(0, 0);
    Eigen::Vector3d omgtheta = roblib::utils::so3ToVec(se3mat_cut);

    Eigen::MatrixXd m_ret(4, 4);

    // If negligible rotation, m_Ret = [[Identity, angular velocty ]]
    //									[	0	 ,		1		   ]]
    if (roblib::utils::NearZero(omgtheta.norm())) {
        // Reuse previous variables that have our required size
        se3mat_cut = Eigen::MatrixXd::Identity(3, 3);
        omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
        m_ret << se3mat_cut, omgtheta,
            0, 0, 0, 1;
        return m_ret;
    }
    
    else {
        double theta = (roblib::utils::AxisAng3(omgtheta))(3);
        Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
        Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
        Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
        Eigen::Vector3d GThetaV = (expExpand * linear) / theta;
        m_ret << roblib::utils::MatrixExp3(se3mat_cut), GThetaV,
            0, 0, 0, 1;
        return m_ret;
    }

}

double roblib::utils::CubicTimeScaling(double Tf, double t) {
    double timeratio = 1.0 * t / Tf;
    double st = 3 * pow(timeratio, 2) - 2 * pow(timeratio, 3);
    return st;
}

double roblib::utils::QuinticTimeScaling(double Tf, double t) {
    double timeratio = 1.0 * t / Tf;
    double st = 10 * pow(timeratio, 3) - 15 * pow(timeratio, 4) + 6 * pow(timeratio, 5);
    return st;
}

Eigen::VectorXd roblib::utils::VelQuadraticForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist,
    const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) 
{
    int n = thetalist.size();
    Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd c = roblib::utils::InverseDynamics(thetalist, dthetalist, dummylist,
        dummyg, dummyforce, Mlist, Glist, Slist);
    return c;
}

Eigen::MatrixXd roblib::utils::JointTrajectory(const Eigen::VectorXd& thetastart, const Eigen::VectorXd& thetaend, double Tf, int N, int method) {
    double timegap = Tf / (N - 1);
    Eigen::MatrixXd trajT = Eigen::MatrixXd::Zero(thetastart.size(), N);
    double st;
    for (int i = 0; i < N; ++i) {
        if (method == 3)
            st = CubicTimeScaling(Tf, timegap * i);
        else
            st = QuinticTimeScaling(Tf, timegap * i);
        trajT.col(i) = st * thetaend + (1 - st) * thetastart;
    }
    Eigen::MatrixXd traj = trajT.transpose();
    return traj;
}

Eigen::VectorXd roblib::utils::InverseDynamics(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& ddthetalist,
    const Eigen::VectorXd& g, const Eigen::VectorXd& Ftip, const std::vector<Eigen::MatrixXd>& Mlist,
    const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
    // the size of the lists
    int n = thetalist.size();

    Eigen::MatrixXd Mi = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd Ai = Eigen::MatrixXd::Zero(6, n);
    std::vector<Eigen::MatrixXd> AdTi;
    for (int i = 0; i < n + 1; i++) {
        AdTi.push_back(Eigen::MatrixXd::Zero(6, 6));
    }
    Eigen::MatrixXd Vi = Eigen::MatrixXd::Zero(6, n + 1);    // velocity
    Eigen::MatrixXd Vdi = Eigen::MatrixXd::Zero(6, n + 1);   // acceleration

    Vdi.block(3, 0, 3, 1) = -g;
    AdTi[n] = roblib::utils::Adjoint(roblib::utils::TransInv(Mlist[n]));
    Eigen::VectorXd Fi = Ftip;

    Eigen::VectorXd taulist = Eigen::VectorXd::Zero(n);

    // forward pass
    for (int i = 0; i < n; i++) {
        Mi = Mi * Mlist[i];
        Ai.col(i) = roblib::utils::Adjoint(roblib::utils::TransInv(Mi)) * Slist.col(i);

        AdTi[i] = roblib::utils::Adjoint(roblib::utils::MatrixExp6(roblib::utils::VecTose3(Ai.col(i) * -thetalist(i)))
            * roblib::utils::TransInv(Mlist[i]));

        Vi.col(i + 1) = AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
        Vdi.col(i + 1) = AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i)
            + roblib::utils::ad(Vi.col(i + 1)) * Ai.col(i) * dthetalist(i); // this index is different from book!
    }

    // backward pass
    for (int i = n - 1; i >= 0; i--) {
        Fi = AdTi[i + 1].transpose() * Fi + Glist[i] * Vdi.col(i + 1)
            - roblib::utils::ad(Vi.col(i + 1)).transpose() * (Glist[i] * Vi.col(i + 1));
        taulist(i) = Fi.transpose() * Ai.col(i);
    }
    return taulist;
}

Eigen::MatrixXd roblib::utils::MassMatrix(const Eigen::VectorXd& thetalist,
    const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
    int n = thetalist.size();
    Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < n; i++) {
        Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(n);
        ddthetalist(i) = 1;
        M.col(i) = roblib::utils::InverseDynamics(thetalist, dummylist, ddthetalist,
            dummyg, dummyforce, Mlist, Glist, Slist);
    }
    return M;
}

Eigen::VectorXd roblib::utils::ForwardDynamics(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& taulist,
    const Eigen::VectorXd& g, const Eigen::VectorXd& Ftip, const std::vector<Eigen::MatrixXd>& Mlist,
    const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {

    Eigen::VectorXd totalForce = taulist - roblib::utils::VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)
        - roblib::utils::GravityForces(thetalist, g, Mlist, Glist, Slist)
        - roblib::utils::EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);

    Eigen::MatrixXd M = roblib::utils::MassMatrix(thetalist, Mlist, Glist, Slist);

    // Use LDLT since M is positive definite
    Eigen::VectorXd ddthetalist = M.ldlt().solve(totalForce);

    return ddthetalist;
}

Eigen::VectorXd roblib::utils::GravityForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& g,
    const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
    int n = thetalist.size();
    Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dummyForce = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd grav = roblib::utils::InverseDynamics(thetalist, dummylist, dummylist, g,
        dummyForce, Mlist, Glist, Slist);
    return grav;
}

Eigen::VectorXd roblib::utils::EndEffectorForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& Ftip,
    const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
    int n = thetalist.size();
    Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd JTFtip = roblib::utils::InverseDynamics(thetalist, dummylist, dummylist,
        dummyg, Ftip, Mlist, Glist, Slist);
    return JTFtip;
}

void roblib::utils::EulerStep(Eigen::VectorXd& thetalist, Eigen::VectorXd& dthetalist, const Eigen::VectorXd& ddthetalist, double dt) {
    thetalist += dthetalist * dt;
    dthetalist += ddthetalist * dt;
    return;
}

Eigen::VectorXd roblib::utils::ComputedTorque(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& eint,
    const Eigen::VectorXd& g, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist,
    const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetalistd, const Eigen::VectorXd& dthetalistd, const Eigen::VectorXd& ddthetalistd,
    double Kp, double Ki, double Kd) {

    Eigen::VectorXd e = thetalistd - thetalist;  // position err
    Eigen::VectorXd tau_feedforward = roblib::utils::MassMatrix(thetalist, Mlist, Glist, Slist) * (Kp * e + Ki * (eint + e) + Kd * (dthetalistd - dthetalist));

    Eigen::VectorXd Ftip = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd tau_inversedyn = roblib::utils::InverseDynamics(thetalist, dthetalist, ddthetalistd, g, Ftip, Mlist, Glist, Slist);

    Eigen::VectorXd tau_computed = tau_feedforward + tau_inversedyn;
    return tau_computed;
}

std::vector<Eigen::MatrixXd> roblib::utils::SimulateControl(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& g,
    const Eigen::MatrixXd& Ftipmat, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist,
    const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& thetamatd, const Eigen::MatrixXd& dthetamatd, const Eigen::MatrixXd& ddthetamatd,
    const Eigen::VectorXd& gtilde, const std::vector<Eigen::MatrixXd>& Mtildelist, const std::vector<Eigen::MatrixXd>& Gtildelist,
    double Kp, double Ki, double Kd, double dt, int intRes) 
{
    Eigen::MatrixXd FtipmatT = Ftipmat.transpose();
    Eigen::MatrixXd thetamatdT = thetamatd.transpose();
    Eigen::MatrixXd dthetamatdT = dthetamatd.transpose();
    Eigen::MatrixXd ddthetamatdT = ddthetamatd.transpose();
    int m = thetamatdT.rows(); int n = thetamatdT.cols();
    Eigen::VectorXd thetacurrent = thetalist;
    Eigen::VectorXd dthetacurrent = dthetalist;
    Eigen::VectorXd eint = Eigen::VectorXd::Zero(m);
    Eigen::MatrixXd taumatT = Eigen::MatrixXd::Zero(m, n);
    Eigen::MatrixXd thetamatT = Eigen::MatrixXd::Zero(m, n);
    Eigen::VectorXd taulist;
    Eigen::VectorXd ddthetalist;
    for (int i = 0; i < n; ++i) {
        taulist = roblib::utils::ComputedTorque(thetacurrent, dthetacurrent, eint, gtilde, Mtildelist, Gtildelist, Slist, thetamatdT.col(i),
            dthetamatdT.col(i), ddthetamatdT.col(i), Kp, Ki, Kd);
        for (int j = 0; j < intRes; ++j) {
            ddthetalist = roblib::utils::ForwardDynamics(thetacurrent, dthetacurrent, taulist, g, FtipmatT.col(i), Mlist, Glist, Slist);
            roblib::utils::EulerStep(thetacurrent, dthetacurrent, ddthetalist, dt / intRes);
        }
        taumatT.col(i) = taulist;
        thetamatT.col(i) = thetacurrent;
        eint += dt * (thetamatdT.col(i) - thetacurrent);
    }
    std::vector<Eigen::MatrixXd> ControlTauTraj_ret;
    ControlTauTraj_ret.push_back(taumatT.transpose());
    ControlTauTraj_ret.push_back(thetamatT.transpose());
    return ControlTauTraj_ret;
}
