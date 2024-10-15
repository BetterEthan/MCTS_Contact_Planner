#include "constrains/util.hh"
#include <cddlib/setoper.h>
#include <cddlib/cdd.h>

namespace Robot_State_Transition
{
    // 顶点转换为不等式
    bool vertices_to_H(const MatrixXX &vertices_set, MatrixXX &H_output, VectorX &h_output);
}