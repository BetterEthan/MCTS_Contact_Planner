#include "constrains/util.hh"
#include "constrains/solve_LP_GLPK.hh"

namespace Robot_State_Transition
{
    extern const double Mass;       // 机体质量
    extern const Vector3 MaxTorque; // 电机最大扭矩

    // 获取质心动力学方程约束Df=d
    std::pair<MatrixXX, VectorX> get_dynamic_eq_constrain(const Vector3 &c, const int &num_support_leg, const MatrixX3 &contactPoints);

    // 获取关节扭矩不等式约束Ax<=b
    std::pair<MatrixXX, VectorX> get_jointToque_neq_constrain(const int &num_support_leg, const MatrixX3 &Joints_angles);

    // 当前状态是否满足动力学约束
    bool is_meet_dynamic_con(const Vector3 &c, const int &num_support_leg, const MatrixX3 &contactPoints, const MatrixX3 &Joints_angles);
}
