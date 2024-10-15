#include "constrains/dynamic_constrain.hh"

namespace Robot_State_Transition
{
    namespace
    {
        Vector3 init_max_torque()
        {
            Vector3 max_T;
            max_T(0) = 0.286 * 2240; // 根关节扭矩：电机额定转矩 * 减速比 = 640.64Nm
            max_T(1) = 0.747 * 2240; // 根关节扭矩：电机额定转矩 * 减速比 = 1673.28Nm  //  0.1 * 2240
            max_T(2) = 0.286 * 4640; // 根关节扭矩：电机额定转矩 * 减速比 = 1327.04Nm  // 0.02 * 4640

            return max_T;
        }

        /**
         * @ : 获取雅克比矩阵
         * @description:
         * @param {double} &t：关节角度构成的向量
         * @return {*}
         */
        Eigen::Matrix<double, 3, 3> get_Jacobian(const Vector3 &t)
        {
            Eigen::Matrix<double, 3, 3> J;
            const double t1 = t(0);
            const double t2 = t(1);
            const double t3 = t(2);

            J << -sin(t1) * (sin(t2 + t3) / (double)2.0 + cos(t2) / (double)2.0 + 0.18),
                cos(t1) * (cos(t2 + t3) / (double)2.0 - sin(t2) / (double)2.0), (cos(t2 + t3) * cos(t1)) / (double)2.0,
                cos(t1) * (sin(t2 + t3) / (double)2.0 + cos(t2) / (double)2.0 + 0.18), sin(t1) * (cos(t2 + t3) / (double)2.0 - sin(t2) / (double)2.0), (cos(t2 + t3) * sin(t1)) / (double)2.0,
                0, sin(t2 + t3) / (double)2.0 + cos(t2) / (double)2.0, sin(t2 + t3) / (double)2.0;

            return J;
        }

        /**
         * @ : 单腿动力学中的重力加速度项
         * @description:
         * @param {Vector3} &t：关节角度
         * @return {*}
         */
        Vector3 get_legMss_consume_Torque(const Vector3 &t)
        {
            Vector3 Leg_T;
            // const double theta1 = t(0);
            const double theta2 = t(1);
            const double theta3 = t(2);

            Leg_T(0) = 0;
            Leg_T(1) = (629153.0 * 9.81 * cos(theta2 + theta3)) / (double)62500.0 + (37009.0 * 9.81 * sin(theta2 + theta3)) / (double)20000.0 - (869173.0 * 9.81 * cos(theta2)) / (double)1000000.0;
            Leg_T(2) = (37009.0 * 9.81 * sin(theta2 + theta3)) / (double)20000.0;

            return Leg_T;
        }

    }

    const double Mass = 121.8957;
    // const double Mass = 110;
    const Vector3 MaxTorque = init_max_torque();



    std::pair<MatrixXX, VectorX> get_dynamic_eq_constrain(const Vector3 &c, const int &num_support_leg, const MatrixX3 &contactPoints)
    {
        assert(num_support_leg == contactPoints.rows());
        MatrixXX D(6, 3 * num_support_leg);
        VectorX d(6);

        // 对D赋值
        for (int i = 0; i < num_support_leg; ++i)
        {
            D.block(0, 3 * i, 3, 3).setIdentity();                                   // 单位矩阵
            D.block(3, 3 * i, 3, 3) = crossMatrix(contactPoints.row(i).transpose()); // 足端位置构成的反对称矩阵
        }

        // 对d赋值
        d.setZero();
        d(2) = 9.81 * Mass;        // 重力平衡
        d(3) = c(1) * 9.81 * Mass; // 重力构成的倾覆力矩平衡
        d(4) = -1 * c(0) * 9.81 * Mass;

        return std::make_pair(D, d);
    }

    std::pair<MatrixXX, VectorX> get_jointToque_neq_constrain(const int &num_support_leg, const MatrixX3 &Joints_angles)
    {
        assert(num_support_leg == Joints_angles.rows());

        MatrixXX A;
        VectorX b;

        A.resize(6 * num_support_leg, 3 * num_support_leg);
        b.resize(6 * num_support_leg);
        A.setZero();
        b.setZero();

        for (int i = 0; i < num_support_leg; ++i)
        {
            Eigen::Matrix<double, 3, 3> J = get_Jacobian(Joints_angles.row(i).transpose());
            Vector3 Leg_T = get_legMss_consume_Torque(Joints_angles.row(i).transpose());

            A.block(6 * i, 3 * i, 3, 3) = J.transpose();      // J^T <= maxT - Leg_T
            A.block(6 * i + 3, 3 * i, 3, 3) = -J.transpose(); //-J^T <= maxT + Leg_T

            b.block(6 * i, 0, 3, 1) = MaxTorque - Leg_T;     // J^T <= maxT - Leg_T
            b.block(6 * i + 3, 0, 3, 1) = MaxTorque + Leg_T; //-J^T <= maxT + Leg_T
        }

        return std::make_pair(A, b);
    }

    /**
     * @ : 判断是否满足力矩约束
     * @description:
     * @param c: 质心位置
     * @param num_support_leg: 支撑腿数
     * @param contactPoints: 接触点位置
     * @param Joints_angles: 关节角度
     * 
    */
    bool is_meet_dynamic_con(const Vector3 &c, const int &num_support_leg, const MatrixX3 &contactPoints, const MatrixX3 &Joints_angles)
    {
        auto Dd = get_dynamic_eq_constrain(c, num_support_leg, contactPoints);
        auto Ab = get_jointToque_neq_constrain(num_support_leg, Joints_angles);

        VectorX g, minBound, maxBound;

        bool flag = solve_LP_GLPK(Ab.first, Ab.second, Dd.first, Dd.second, g, minBound, maxBound);

        return flag;
    }

}
