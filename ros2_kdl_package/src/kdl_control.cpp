#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd * de + _Kp * e) + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::velocity_ctrl_null(KDL::Frame &_desPos, double _Kp, double _lambda)
{
    unsigned int n = robot_->getNrJnts();
    Eigen::VectorXd q = robot_->getJntValues();
    KDL::Frame current_pose = robot_->getEEFrame();
    Eigen::MatrixXd J = robot_->getEEJacobian().data;

    KDL::Twist e_twist = KDL::diff(current_pose, _desPos);
    Eigen::Matrix<double, 6, 1> ep;
    ep(0) = e_twist.vel.x();
    ep(1) = e_twist.vel.y();
    ep(2) = e_twist.vel.z();
    ep(3) = e_twist.rot.x();
    ep(4) = e_twist.rot.y();
    ep(5) = e_twist.rot.z();

    Eigen::Matrix<double, 6, 6> Kp_matrix = Eigen::Matrix<double, 6, 6>::Identity() * _Kp;
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd q0_dot = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd joint_limits = robot_->getJntLimits();
    for (unsigned int i = 0; i < n; ++i)
    {
        double q_i = q(i);

        double q_min_i = joint_limits(i, 0);
        double q_max_i = joint_limits(i, 1);
        double den_H = (q_max_i - q_i) * (q_i - q_min_i);
        if (std::abs(den_H) < 1e-6)
        {
            q0_dot(i) = 0.0;
        }
        else
        {
            double C_i = std::pow(q_max_i - q_min_i, 2) / _lambda;
            double numerator = C_i * (2 * q_i - q_max_i - q_min_i);
            double common_denominator_sq = std::pow(den_H, 2);
            q0_dot(i) = -numerator / common_denominator_sq;
        }
    }
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd null_space_projector = I - (J_pinv * J);
    Eigen::VectorXd q_dot = (J_pinv * Kp_matrix * ep) + (null_space_projector * q0_dot);
    return q_dot;
}

// ============ VISION BASED CONTROL IMPLEMENTATION ============
Eigen::VectorXd KDLController::vision_ctrl(
    KDL::Frame pose_in_camera_frame,
    KDL::Frame camera_frame,
    KDL::Jacobian camera_jacobian,
    Eigen::VectorXd q0_dot)
{
    Matrix6d R = spatialRotation(camera_frame.M);

    Eigen::Vector3d c_P_o = toEigen(pose_in_camera_frame.p);

    Eigen::Vector3d s = c_P_o / c_P_o.norm();

    Eigen::Matrix<double, 3, 6> L = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix3d L_11 = (-1 / c_P_o.norm()) * (Eigen::Matrix3d::Identity() - s * s.transpose());

    L.block<3, 3>(0, 0) = L_11;
    L.block<3, 3>(0, 3) = skew(s);
    L = L * R;

    Eigen::MatrixXd J_c = camera_jacobian.data;
    Eigen::MatrixXd LJ = L * J_c;

    Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(J_c.cols(), J_c.cols());
    Eigen::MatrixXd N = I - (LJ_pinv * LJ);

    Eigen::Vector3d s_d(0, 0, 1);
    double k = 1;

    Eigen::VectorXd joint_velocities = k * LJ_pinv * s_d + N * q0_dot;

    Eigen::Vector3d s_error = s - s_d;
    std::cout << "Error norm (s_error): " << s_error.norm() << std::endl;

    return joint_velocities;
}
