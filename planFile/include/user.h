#ifndef _USER_H
#define _USER_H
#include <iostream>
#include <string>
#include <iostream>
#include <fstream>   //文件操作
#include <sstream>   //字符串操作,搭配fstream读取文件数据
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <map>
// #include "grid_map_sdf/SignedDistanceField.hpp"

#define MAX_DEPTH 50
namespace USER
{
    
    grid_map::GridMap init_grid_map(std::map<std::string, std::string> configMap);

    // #define terrain_file_name "/home/xp/aConstrainedContactPlan/MPI_ROS_Workspace/src/hit_spider/config/terrain/中间空.txt"

    #define configFilePath "/home/xp/aContactPlanning/MPI_CPP_Version/config.txt"
    
    extern std::map<std::string, std::string> configMap;
    extern const std::string key_element; // HashKey 子元素
    extern const int max_depth; // 最大深度
    extern const int simStepNum; // 模拟步数
    extern grid_map::GridMap mapData; // 地图数据
    extern const std::string availableFootholdLayerName;
    extern const float LegWorkspaceR;

    extern const bool COLLISION_CHECK;
    extern const bool IsMaxForceConstraint;
    extern const bool IsToruqeLimitConstraint;
    extern const bool IsVirtualLoss;
    extern const bool IsBestBP;

    extern const float SEARCH_TIME_LIMIT;
    extern const int JOB_FACTOR;
}


#endif
