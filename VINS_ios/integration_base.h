//
//  integration_base.h
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/11/25.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef integration_base_h
#define integration_base_h

#include "utility.hpp"
#include <ceres/ceres.h>
#include "global_param.hpp"

using namespace Eigen;
class IntegrationBase
{
public:
    IntegrationBase() = delete;
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
    : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
    linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
    m_jacobian{Eigen::Matrix<double, 16, 16>::Identity()}, m_covariance{Eigen::Matrix<double, 16, 16>::Zero()},
    jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, covariance{Eigen::Matrix<double, 15, 15>::Zero()},
    sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}
    
    {
        noise = Eigen::Matrix<double, 18, 18>::Zero();
        noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
    }
    
    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
    {
        dt_buf.push_back(dt);
        acc_buf.push_back(acc);
        gyr_buf.push_back(gyr);
        propagate(dt, acc, gyr);
    }
    
    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
    {
		//相邻两帧之间的时间间隔
		sum_dt = 0.0;
		//预积分的线性点，即第一帧对应的测量值
        acc_0 = linearized_acc;
        gyr_0 = linearized_gyr;
		//p,q,v预积分值
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
		//加速度，陀螺仪bias，相邻两帧之间bias恒定
        linearized_ba = _linearized_ba;
        linearized_bg = _linearized_bg;
		//预积分的雅克比以及方差矩阵,相邻两帧间初始雅克比为单位矩阵，初始协方差为0
        jacobian.setIdentity();
        covariance.setZero();
		//相邻两帧之间的所有Imu测量数值
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
            propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
    }
    
    void midPointIntegration(double _dt,
                             const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                             const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                             const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                             const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                             Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                             Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    {
        //ROS_INFO("midpoint integration");
		//计算相对相邻两帧的第一帧的前一时刻加速度，使用前一时刻的预积分
        Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
		//角速度为当前和前一时刻的均值
        Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
		//更新姿态的预积分
        result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
		//计算相对相邻两帧的第一帧的当前时刻加速度，使用当前时刻的预积分
        Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
		//相对相邻两帧的第一帧的加速度为前一时刻和当前时刻的均值
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
		//更新P的预积分，V的预积分，均与重力，两帧的位姿，速度无关
        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
        result_delta_v = delta_v + un_acc * _dt;
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;
		
		//更新相邻帧i，j间的雅克比矩阵，参考VINS技术报告
        if(update_jacobian)
        {
			//角速度为当前和前一时刻的均值
			Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Vector3d a_0_x = _acc_0 - linearized_ba;
            Vector3d a_1_x = _acc_1 - linearized_ba;
			//[w]x,[a0]x,[a1]x；各自的skewmatrix
            Matrix3d R_w_x, R_a_0_x, R_a_1_x;
            
            R_w_x<< 0, -w_x(2), w_x(1),
                    w_x(2), 0, -w_x(0),
                    -w_x(1), w_x(0), 0;
            
            R_a_0_x<< 0, -a_0_x(2), a_0_x(1),
                      a_0_x(2), 0, -a_0_x(0),
                      -a_0_x(1), a_0_x(0), 0;
            
            R_a_1_x<< 0, -a_1_x(2), a_1_x(1),
                      a_1_x(2), 0, -a_1_x(0),
                      -a_1_x(1), a_1_x(0), 0;
            //论文中公式8中的（1+F*dt)
            MatrixXd F = MatrixXd::Zero(15, 15);
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
            -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
            -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9) = Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Matrix3d::Identity();
            //cout<<"A"<<endl<<A<<endl;
            
            MatrixXd V = MatrixXd::Zero(15,18);
            V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;
            
            //step_jacobian = F;
            //step_V = V;
            jacobian = F * jacobian;
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
        }
        
    }
    
    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
    {
        dt = _dt;
        acc_1 = _acc_1;
        gyr_1 = _gyr_1;
        Vector3d result_delta_p;
        Quaterniond result_delta_q;
        Vector3d result_delta_v;
        Vector3d result_linearized_ba;
        Vector3d result_linearized_bg;
        //两帧图像之间进行中值预积分，每来一次imu数据更新一下
        midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v,
                            result_linearized_ba, result_linearized_bg, 1);
        
        //checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
        //                    linearized_ba, linearized_bg);
		//迭代各变量，进行下一次更新
        delta_p = result_delta_p;
        delta_q = result_delta_q;
        delta_v = result_delta_v;
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_q.normalize();
        sum_dt += dt;
        acc_0 = acc_1;
        gyr_0 = gyr_1;
        
    }
    
    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
        Eigen::Matrix<double, 15, 1> residuals;
        
        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);
        
        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);
        
        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);
        
        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;
        
        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;
        
        Vector3d G{0,0,GRAVITY};
        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
        return residuals;
    }
    
    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;
    
    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;
    
    Eigen::Matrix<double, 16, 16> m_jacobian, m_covariance;
    
    Eigen::Matrix<double, 15, 15> jacobian, covariance;
    Eigen::Matrix<double, 15, 15> step_jacobian;
    Eigen::Matrix<double, 15, 18> step_V;
    Eigen::Matrix<double, 18, 18> noise;
    
    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;
    
    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;
};
#endif /* integration_base_h */
