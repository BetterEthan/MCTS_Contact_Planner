#ifndef PLANNING_H
#define PLANNING_H

#include <myDataType.h>
#include <vector>
#include <geometryFun.h>
// #include <grid_map_ros/grid_map_ros.hpp> //grid_map地图
// #include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <HexapodParameter.h>
#include <user.h>
#include <collisionCheck.h>

namespace PLANNING{

    struct ContactsInfos{
        MatrixX3 position;
        MatrixX3 normalVector;
        VectorX  frcitionMu;
        VectorX  maxNormalF;
    }; 


    /**
     * \brief 根据当前状态 随机计算下一步可行接触状态集合
     * \param hexapodState 机器人当前状态
     * \return 返回下一步可行接触状态集合
    */
    std::vector<MDT::RobotState> getNextMCTSstateList(const MDT::RobotState rState, const grid_map::GridMap &mapData);


    /**
     * \brief 根据当前状态 计算下一步可行接触状态 (挑选最大步长步态)
     * \param hexapodState 机器人当前状态
     * \param mapData  地图数据
     * \return 返回下一步可行接触状态
    */
    MDT::RobotState getNextMCTSstateByExpert_forSim(const MDT::RobotState rState, const grid_map::GridMap &mapData);


    MDT::AvailableContactsInfo getAvailableFootholds(const MDT::RobotState &hexapodState, const grid_map::GridMap &mapData);

    std::vector<MDT::RobotState> getNextMCTSstateList_underConstrains_moreStates(const MDT::RobotState rState, const grid_map::GridMap &mapData);

    MDT::RobotState getNextMCTSstateByExpert_underConstrain(const MDT::RobotState rState, const grid_map::GridMap &mapData);
}






#endif

