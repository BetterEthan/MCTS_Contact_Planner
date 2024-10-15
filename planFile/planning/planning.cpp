#include <planning.h>
#include "constrains/dynamic_constrain.hh"
#include "constrains/util.hh"                         //数据类型
#include "constrains/Bretl.hh" //CWC
#include "constrains/my_cdd.hh"
#include "constrains/kinematics_constrain.hh" //运动学约束
#include "constrains/dynamic_constrain.hh"    //动力学约束

namespace PLANNING
{

    /**
     * @brief 计算reduced kinematic margin  ---- (非通用, 已修改)
     * @par   pnts: 多边形的顶点
     * @par  footW: 足端坐标
     * @par  forwardDirection_W: 足端坐标系下的前进方向, 参考坐标系为World!!!
     * @par  LegNum: 腿号
    */
    float getReduceKinematicMargin(const MDT::Pose &robotPoseW, const MDT::POINT &footW, 
        const float &forwardDirection_W, const int &legNum_)
    // (const int &legNum_, const hit_spider::hexapod_Base_Pose &robotPose, const geometry_msgs::Point &footW, const geometry_msgs::Point &direction_)
    {
        float inverseDirection = forwardDirection_W + _PI_;
        MDT::POINT n_= {cos(inverseDirection), sin(inverseDirection), 0}; // 机体Trunk坐标系下的方向
        // Eigen::Vector3d n_ = Eigen::Vector3d(-direction_.x, -direction_.y, -direction_.z); 

        // 计算足端目标机器人坐标系到世界坐标系的旋转平移矩阵T
        Eigen::Isometry3d T_W_B = robotPoseW.getT_W_B();

        // 计算足端到固定基节（扇形坐标系）坐标系下的坐标
        Eigen::Vector3d p1_ = footW;
        Eigen::Vector3d foot_FixGu;
        Eigen::Vector3d foot_Body;
        foot_Body = T_W_B.inverse() * p1_;
        foot_FixGu = HexapodParameter::TransMatrix_FixGu_Body[legNum_] * foot_Body;

        // 计算机体坐标系下的前进向量
        Eigen::Vector3d tmp = n_ + foot_Body;

        // 计算固定股节坐标系下的方向向量
        tmp = HexapodParameter::TransMatrix_FixGu_Body[legNum_] * tmp;
        Eigen::Vector3d nn_ = tmp - foot_FixGu;

        // 计算与扇形的交点
        MDT::POINT intersectionP = GEOMETRY::getIntersectionSector(USER::LegWorkspaceR, foot_FixGu, nn_);
        intersectionP.z() = foot_FixGu.z(); // z坐标就是足端的z坐标
        float reducedMargin = sqrt((intersectionP.x() - foot_FixGu.x()) * (intersectionP.x() - foot_FixGu.x()) +
                                   (intersectionP.y() - foot_FixGu.y()) * (intersectionP.y() - foot_FixGu.y()));

        return reducedMargin; // 单腿足端能移动的距离
    }




    /**
     * \brief 获得COG到前进方向与支撑多边形的交点的距离 ---- 足式通用
     * \par RobotCenter: 足端坐标
     * \par forwardDirection_W: 足端坐标系下的前进方向, 参考坐标系为World!!!
     * \par pnts: 支撑多边形的顶点
     * 
     * 
    */
    float getMaxLengthCOGtoPolygon(const MDT::POINT &trunkPosition_W, const float &forwardDirection_W ,const std::vector<MDT::POINT> &supportPnts_W)
    {
        MDT::POINT dir_W = {cos(forwardDirection_W), sin(forwardDirection_W), 0};
        MDT::POINT intersection = GEOMETRY::getIntersection(supportPnts_W, trunkPosition_W, dir_W);
        float maxL = sqrt((intersection.x() - trunkPosition_W.x())*(intersection.x() - trunkPosition_W.x()) + (intersection.y() - trunkPosition_W.y()) * (intersection.y() - trunkPosition_W.y())); //  + (intersection.z - footG.z) * (intersection.z - footG.z)

        return maxL;
    }



    //计算静态稳定裕度  ---- 足式通用
    bool getStabilityMargin(MDT::POINT RobotCenter,std::vector<MDT::POINT> Pnt,int n , float *margin_)
    {
        std::vector<float> dis(n);

        if (!GEOMETRY::InPolygon(RobotCenter,n,Pnt)) {return false;}
        for(int i=0;i<n-1;i++)
        {
            dis[i] = GEOMETRY::distancePtSeg(RobotCenter,Pnt[i],Pnt[i+1]);
        }
        dis[n-1] = GEOMETRY::distancePtSeg(RobotCenter,Pnt[n-1],Pnt[0]);

        std::vector<float>::iterator myMin = min_element(dis.begin(), dis.end());

        *margin_ = *myMin;
        return true;
    }


    /**
     * @brief 获取可行落足点    ---- (非通用, 已修改)
     * @par  state_: 机器人状态
     * @par  mapData: 地图数据
     * @par  footholds: 落足点集合, 需要注意传参前提前定义 std::vector<MDT::POINT> footholds[6]
    */
    void getAvailableFoothold(const MDT::RobotState &robotPose, const grid_map::GridMap &mapData, 
        std::vector<MDT::POINT> *foothold_)
    {
        Eigen::Isometry3d T_W_B = robotPose.pose.getT_W_B(); // 机器人位姿

        for (int j = 0; j < 6; ++j)
        {
            for (int i = 0; i < HexapodParameter::LegWorkspaceCloud[j].size(); ++i)
            {
                Eigen::Vector3d footW_tmp = T_W_B * HexapodParameter::LegWorkspaceCloud[j][i]; // 可落足点在世界系下的描述
                grid_map::Position gm_position(footW_tmp.x(), footW_tmp.y());
                grid_map::Index gm_index;
                mapData.getIndex(gm_position, gm_index); // 获取落足位置在grid_map中的index

                if (mapData.isValid(gm_index, USER::availableFootholdLayerName)) // 它是否在高程图内
                {
                    foothold_[j].push_back(footW_tmp); // 把可落足点加入到对应的腿的集合中
                }
            }
        }
    }




    /**
     * \brief 根据margin和废腿信息进行选取下一帧可行的支撑状态  ---- (六足通用)
     * \param hexapodState 机器人当前状态
     * \return 返回可行的支撑状态集
    */
    std::vector<MDT::Vector6b> findAvailableSupportStates(const MDT::RobotState &hexapodState)
    {
        // 计算fault leg 数目
        int faultLegNum = 0;
        for(int i=0;i<6;i++)
        {
            if (hexapodState.faultStateToNow[i] == hexapodState.faultFlag())
                faultLegNum++;
        }

        //计算允许的support leg状态。
        std::vector<MDT::Vector6b> tmpSupportList;

        // 如果当前状态下有fault leg存在, 将fault leg作为support leg的状态删掉
        if(faultLegNum > 0)
        {
            for(int i=0; i<HexapodParameter::initialSupportList.size();i++)
            {
                int tmpNum_ = 0;
                for(int j=0;j<6;j++)
                {
                    if(hexapodState.faultStateToNow[j] == hexapodState.faultFlag() 
                        && HexapodParameter::initialSupportList[i][j] == hexapodState.supportFlag())
                    {
                        tmpNum_++;
                    }
                }
                // 无使用fault leg作为support leg的情况
                if(tmpNum_ == 0)
                {
                    tmpSupportList.push_back(HexapodParameter::initialSupportList[i]);
                }
            }
        }
        else
        {
            tmpSupportList = HexapodParameter::initialSupportList;
        }

        //去除与上一周期相同的摆动状态,限制机器人不能重复摆动相同的足
        std::vector<MDT::Vector6b> tmpSupportList2;
        for(int i=0;i<tmpSupportList.size();i++)
        {
            int tmpNum_ = 0;
            for(int j=0;j<6;j++)
            {
                if(tmpSupportList[i](j) == SWING)
                {
                    if(hexapodState.gaitToNow[j] == SWING && hexapodState.faultStateToNow[j] != hexapodState.faultFlag())
                    {
                        tmpNum_++;
                    }
                }
            }
            if(tmpNum_ == 0)
            {
                tmpSupportList2.push_back(tmpSupportList[i]);
            }
        }



        std::vector<MDT::Vector6b> availableSupportList;


        //根据静态稳定裕度排除不稳定的状态
        for (int i = 0; i < tmpSupportList2.size();i++)
        {

            //支持多边形顶点
            std::vector<MDT::POINT> Pnts_;
            // std::vector<float> supportLegsReducedMargins;
            for (int j = 0; j < 6; j++)
            {
                if(tmpSupportList2[i](j) == hexapodState.supportFlag())
                {
                    //记录支持腿的坐标，以计算静态稳定裕度
                    Pnts_.push_back(hexapodState.feetPosition[j]);
                }
                /* code */
            }

            // 计算静态稳定裕度
            float stabilityMargin_ = 0;
            
            // getStabilityMargin(MDT::POINT RobotCenter,std::vector<MDT::POINT> Pnt,int n , float *margin_)
            bool isInPolygon =  PLANNING::getStabilityMargin(MDT::POINT(hexapodState.pose.x, hexapodState.pose.y, hexapodState.pose.z), 
                Pnts_, Pnts_.size(), &stabilityMargin_);

            if(isInPolygon == true && stabilityMargin_ > 0.1f)
            {
                // cout << stabilityMargin_ << endl;
                availableSupportList.push_back(tmpSupportList2[i]);
            }
        }

        return availableSupportList;
    }

    /** 
     *    **************(通用性取决于调用的getReduceKinematicMargin函数)
     * \brief 根据选取的supportState和reduced kinamatic margin计算可行的步长
     * \param hexapodState 机器人当前状态
     * \param availableSupportList 可行的支撑状态集
     * \return 返回可行的步长集
    */
    std::vector<float> findAvailableStepLength(const MDT::RobotState &hexapodState,
        const std::vector<MDT::Vector6b> &availableSupportList)
    {
        // 计算每条腿的reduced margin *************************************************************************************88
        float reduxceKinematicMargin_[6];
        for(int i=0;i<6;i++)
        {
            reduxceKinematicMargin_[i] = PLANNING::getReduceKinematicMargin(hexapodState.pose, 
                hexapodState.feetPosition[i], hexapodState.moveDirection, i);
        }

        std::vector<float> maxStepLengthForSupportState;
        for (int i = 0; i < availableSupportList.size(); i++)
        {
            //支撑多边形顶点
            std::vector<MDT::POINT> Pnts_;
            std::vector<float> supportLegsReducedMargins;

            for (int j = 0; j < 6; j++)
            {
                if(availableSupportList[i](j) == hexapodState.supportFlag())
                {
                    //记录支持腿的坐标，以计算COG到前进方向与支撑多边形的交点的距离
                    Pnts_.push_back(hexapodState.feetPosition[j]);
                    //记录支撑腿相应的运动学裕度
                    supportLegsReducedMargins.push_back(reduxceKinematicMargin_[j]);
                }
                /* code */
            }


            // 获得COG到前进方向与支撑多边形的交点的距离
            MDT::POINT trunkP_ = {hexapodState.pose.x,hexapodState.pose.y, hexapodState.pose.z};
            float COGtoPolygon_ = PLANNING::getMaxLengthCOGtoPolygon(trunkP_, hexapodState.moveDirection, Pnts_);

            // float COGtoPolygon_ = ParameterHexapod->getMaxLengthCOGtoPolygon
            //     (hexapodState.poseNow.position,hexapodState.moveDirectionNext, Pnts_, Pnts_.size());

            supportLegsReducedMargins.push_back(COGtoPolygon_);
            
            int minIndex = min_element(supportLegsReducedMargins.begin(),supportLegsReducedMargins.end()) - supportLegsReducedMargins.begin();

            maxStepLengthForSupportState.push_back(supportLegsReducedMargins[minIndex]);
        }

        return maxStepLengthForSupportState;
        
    }


    
    /** 
     * **************(通用性取决于调用的getAvailableFoothold函数)
     * \brief 根据机器人trunk位姿 和 支撑状态 和 地图 规划落足点
     * \param hexapodState 机器人当前状态
     * \param mapData  地图数据
     * \return hexapodState
    */
    bool findRandomFootholdsCombination(MDT::RobotState &hexapodState, const grid_map::GridMap &mapData)
    {   
        // 计算下一帧中的可落足点点集
        std::vector<MDT::POINT> footholds[6];
        PLANNING::getAvailableFoothold(hexapodState, mapData, footholds);

        for(int i=0;i<6;i++)
        {
            if(hexapodState.gaitToNow[i] == hexapodState.swingFLag())
            {
                MDT::POINT nominalFoothold_W= MDT::pointRotationAndTrans(HexapodParameter::norminalFoothold_B[i], 
                    hexapodState.pose.getT_W_B()); 
                // 获得footholds[i]中离nominalFoothold_W最近的点
                std::vector<float> dis;
                if(footholds[i].size() == 0) return false;

                for(int j=0;j<footholds[i].size();j++)
                {
                    float tempDis = sqrt(pow(footholds[i][j].x() - nominalFoothold_W.x(), 2) + 
                        pow(footholds[i][j].y() - nominalFoothold_W.y(), 2));
                    dis.push_back(tempDis);
                }
                int minIndex = std::min_element(dis.begin(), dis.end()) - dis.begin();
                hexapodState.feetPosition[i] = footholds[i][minIndex];
            }
        }
        
        return true;
    }




    /**
     * \brief 根据当前状态 随机计算下一步可行接触状态集合
     * \param hexapodState 机器人当前状态
     * \return 返回下一步可行接触状态集合
    */
    std::vector<MDT::RobotState> getNextMCTSstateList(const MDT::RobotState rState, const grid_map::GridMap &mapData)
    {
        std::vector<MDT::RobotState> resultLists;
        std::vector<MDT::Vector6b> availableSupportState_ = PLANNING::findAvailableSupportStates(rState);
        std::vector<float> maxStepLengthForSupportState = findAvailableStepLength(rState,availableSupportState_);

        for(int i=0;i<availableSupportState_.size();i++)
        {
            MDT::RobotState hexapodState = rState;
            hexapodState.gaitToNow = availableSupportState_[i];
            float stepLength_ = maxStepLengthForSupportState[i]*2.0/3.0;

            if(stepLength_ > 0.05f)
            {
                // 防止落在支撑多边形边上
                stepLength_ -= 0.05f;
            }
            else if(stepLength_ < 0.01) //防止在仿真过程中机器人不断挪动，由于数值问题移动出扇形工作空间
            {
                stepLength_ = 0;
            }
            

            hexapodState.pose.x += stepLength_ * cos(hexapodState.moveDirection);
            hexapodState.pose.y += stepLength_ * sin(hexapodState.moveDirection);

            //findAvailableFootholdsCombination
            //findRandomFootholdsCombination
            if(PLANNING::findRandomFootholdsCombination(hexapodState, mapData)) 
            {
                resultLists.push_back(hexapodState);
            }

        }
        return resultLists;
    }




    // 稳定性约束, 运动学约束, 力矩约束, (膨胀约束, 足地力约束)
    std::pair<std::vector<MDT::Vector6b>, std::vector<MDT::POINT>> getSupportListAndStepL_underConstrains(const MDT::RobotState hexapodState, const grid_map::GridMap &mapData)
    {


        std::vector<MDT::RobotState> resultLists;
        std::vector<MDT::POINT> maxMoving;

        std::vector<MDT::Vector6b> availableSupportState_ = PLANNING::findAvailableSupportStates(hexapodState);
        if(availableSupportState_.size() == 0)
        {
            return std::make_pair(availableSupportState_, maxMoving);
        }

        // std::cout << "availableSupportState_ size: " << availableSupportState_.size() << std::endl;
        assert(availableSupportState_.size() != 0 && "availableSupportState_ should not be empty22.");
        
        // 通过最小二乘拟合所有接触点的平面, 并计算该平面与xy平面的夹角
        std::vector<Vector3> pnts;
        float sumZ = 0.0f;
        int ccount = 0;
        for(int j = 0; j < 6; ++j)
        {
            if(hexapodState.faultStateToNow[j] != FAULT_LEG)
            {
                pnts.push_back(Vector3(hexapodState.feetPosition[j]));
                sumZ += hexapodState.feetPosition[j].z();
                ccount++;
            }
        }
        pnts.push_back(Vector3(hexapodState.pose.x+0.4, hexapodState.pose.y, hexapodState.pose.z +  1*(sumZ/(float)ccount) + 0.5 - hexapodState.pose.z) );

        // 示例：参考坐标系下的向量n
        Vector3 n = Vector3(cos(hexapodState.moveDirection),sin(hexapodState.moveDirection),0); // 假设向量n在参考坐标系下的值为(0, 0, 1)
        // 计算平面法向量
        Vector3 planeNormal = Robot_State_Transition::calculatePlaneNormal(pnts);
        // std::cout << "平面法向量：" << planeNormal.transpose() << std::endl;
        // 计算指定向量n在平面上的投影
        Vector3 projection = n - n.dot(planeNormal) / planeNormal.squaredNorm() * planeNormal;  // 这就是我要的前进方向的三维向量


        std::vector<MDT::Vector6b> availableSupportState_Final;



        for (int i = 0; i < availableSupportState_.size(); ++i)
        {
            // 数据准备
            int support_leg_num = 0; // 支撑腿数量
            for (int j = 0; j < 6; ++j)
            {
                if(availableSupportState_[i][j] == SUPPORT)
                    support_leg_num += 1;
            }

            std::vector<int> support_leg;                 // 支撑腿标号
            MatrixX3 contact_Points(support_leg_num, 3);  // 支撑腿接触点
            MatrixXX CWC_inputs(support_leg_num, 2);      // 支撑点的x,y坐标
            MatrixX3 contact_normals(support_leg_num, 3); // 接触点法向量
            contact_normals.setZero();
            contact_normals.col(2).setOnes(); // 先考虑简单情况, 将第三列设置为1, 其余元素不变. 即接触点法向量为(0,0,1)

            VectorX contact_maxmumF(support_leg_num); // 接触点最大地面反作用力
            
            int row_index = 0;

            // 记录接触点数据
            for (int j = 0; j < 6; ++j)
            {
                if (availableSupportState_[i][j] == SUPPORT)  // 支撑
                {
                    // contact_maxmumF(row_index)  = 1000;
                    contact_maxmumF(row_index)  = hexapodState.maxNormalForce[j];


                    CWC_inputs.row(row_index) << hexapodState.feetPosition[j].x(),
                        hexapodState.feetPosition[j].y();
                    
                    contact_Points.row(row_index++) << hexapodState.feetPosition[j].x(),
                        hexapodState.feetPosition[j].y(),
                        hexapodState.feetPosition[j].z();


                    support_leg.push_back(j + 1);  // 记录几号腿为支撑腿
                }
            }





            MatrixXX A;
            VectorX b;
            MatrixXX D;
            VectorX d;
            VectorX g, minBound, maxBound;

            // 1.判断是否在支撑多边形内部*************************************************************************
            double dis_cwc = 0.0;
            MatrixXX CWC_Vertices;

            if(USER::IsMaxForceConstraint)
            {
                if(!Robot_State_Transition::comput_friction_region_considerMaxF(contact_Points, contact_normals, contact_maxmumF, 0.2, Robot_State_Transition::Mass, CWC_Vertices))
                {
                    continue;
                }
            }
            else
            {
                if (!Robot_State_Transition::comput_friction_region(contact_Points, contact_normals, 0.2, Robot_State_Transition::Mass, CWC_Vertices))
                {
                    continue;
                }
            }


            if(CWC_Vertices.rows() < 3)
            {
                continue;
            }


            // 将平面凸多边形顶点转换为不等式形式
            auto Ab_cwc = Robot_State_Transition::Bretl_Vettices_to_face(CWC_Vertices);

            // 判断质心位置是否在cwc内部, 其实用判断是否在凸多边形内就可以了.
            VectorX is_in_cwc = Ab_cwc.first * point_Planar(hexapodState.pose.x, hexapodState.pose.y);

            if(!((is_in_cwc.array() <= Ab_cwc.second.array()).all()))
            {
                continue; // 质心位置不在cwc内部
            }

            // 求前进方向到cwc的交点
            MatrixXX A_cwc(Ab_cwc.first.rows(), 3);
            A_cwc.block(0, 0, Ab_cwc.first.rows(), 2) = Ab_cwc.first;
            A_cwc.col(2).setZero(); ////////////////////////////////////////////// 第三列为0
            auto intersectionP_CWC =  Robot_State_Transition::findIntersection(A_cwc, Ab_cwc.second, 
                Vector3(hexapodState.pose.x, hexapodState.pose.y, 0), 
                Vector3(1,0,0)); ////////////////////////////////// ?

            dis_cwc = intersectionP_CWC.x() - hexapodState.pose.x;


            
            // 2.当前支撑腿对质心的运动约束，求解最大前进步长*****************************************************************************
            auto feet_con_cog_Ab = Robot_State_Transition::get_kinematics_con_foot_cog(hexapodState.pose, support_leg, contact_Points, contact_normals);

            A.resize(7 * support_leg_num, 3);  // 7 * support_leg_num是什么?
            b.resize(7 * support_leg_num);
            A.setZero();
            b.setZero();
            A.block(0, 0, 7 * support_leg_num, 3) = feet_con_cog_Ab.first;
            b.block(0, 0, 7 * support_leg_num, 1) = feet_con_cog_Ab.second;


            if(!Robot_State_Transition::isInConvex( A, b, Vector3(hexapodState.pose.x, hexapodState.pose.y, hexapodState.pose.z)))
            {
                continue; // 质心位置不在运动学凸包内部
            }

            auto intersection_ = Robot_State_Transition::findIntersection(A, b, Vector3(hexapodState.pose.x, hexapodState.pose.y, hexapodState.pose.z),
                projection);

            // 临时处理的方案, 后续需要更改!!!
            double kin_reslut_length = intersection_(0) - hexapodState.pose.x;

            double maxDeltaZ = intersection_(2) - hexapodState.pose.z;



            // 3.选择的支撑状态满足关节扭矩约束，接着确定最大步长 ***********************************************************************
            double length_ = std::min(dis_cwc, kin_reslut_length);

            if(USER::IsToruqeLimitConstraint)
            {
                Vector3 c;
                c << 0, 0, 0.5;
                MDT::Pose base_pose = hexapodState.pose;
                MatrixX3 joints(support_leg_num, 3);
                joints.setZero();

                // 离散为10个点
                int sbsb = 0;
                int ii = 0;
                while (1)
                {
                    for (ii = 0; ii < 11; ++ii)
                    {
                        c(0) = hexapodState.pose.x + (double)(ii) * length_ / (double)10;
                        base_pose.x = c(0);

                        MDT::FeetPositions feet_pos;
                        for (int i = 0; i < 6; i++) {
                            feet_pos.footP[i] = hexapodState.feetPosition[i];
                        }

                        if ((!HexapodParameter::support_leg_inverse_kin(base_pose, feet_pos, support_leg, joints)) || 
                            (!Robot_State_Transition::is_meet_dynamic_con(c, support_leg_num, contact_Points, joints)))
                        {
                            ++sbsb;
                            length_ = (c(0) - hexapodState.pose.x) * 0.6;
                            break;
                        }
                    }
                    if (sbsb >= 3)
                    {
                        length_ = 0.0;
                        break;
                    }

                    if (ii == 11)
                    {
                        break;
                    }
                }
            }

            // 4.根据支撑腿的碰撞检测来生成步长 ***********************************************************************
            if(USER::COLLISION_CHECK)
            {
                for (int ii = 0; ii < 11; ++ii)
                {
                    bool breakFlag = false;
                    float tmpX = hexapodState.pose.x + (double)(ii) * length_ / (double)10;

                    Eigen::Isometry3d T_W_B = hexapodState.pose.getT_W_B();
                    T_W_B.translation() = Eigen::Vector3d(tmpX, hexapodState.pose.y, hexapodState.pose.z);

                    for(int j=0;j<6;j++)
                    {
                        if(availableSupportState_[i][j] == SUPPORT)
                        {
                            auto aaP =  COLLISION_CHECK::getLegKeyPoints(T_W_B, hexapodState.feetPosition[j], j);

                            for(int i = 0; i< aaP.size(); i++)
                            {
                                // std::cout << aaP[i] << std::endl;
                                if(!COLLISION_CHECK::isCollision_onePoint(aaP[i], mapData))
                                {
                                    breakFlag = true;
                                }
                            }
                
                        }
                    }

                    if(breakFlag == true)
                    {
                        length_ = tmpX - hexapodState.pose.x;
                        break;
                    }
                    // auto aaP =  COLLISION_CHECK::getLegKeyPoints(T_W_B, keyPnts[i], 0);
                }
            }




            if(length_ < 0.01) continue;
            
            MDT::POINT maxMovingPoint = {length_, 0, maxDeltaZ};
            maxMoving.push_back(maxMovingPoint);
            availableSupportState_Final.push_back(availableSupportState_[i]);
        }


        return std::make_pair(availableSupportState_Final, maxMoving);
    }

    

    


    ContactsInfos get_now_Feasible_foot_position(const MDT::Pose &pose, const grid_map::GridMap &mapData)
    {
        // MatrixX3 AA;
        // MatrixX3 NormalVector;
        // VectorX  frcitionMu;
        // VectorX  maxNormalF;

        ContactsInfos contactDates;


        int num = 0;

        grid_map::Position position;
        grid_map::Position center(pose.x, pose.y);
        double radius = 1.5;

        for (grid_map::CircleIterator it(mapData, center, radius); !it.isPastEnd(); ++it)
        {
            if(mapData.isValid(*it,"elevation"))
            {
                ++num;
            }

            // mapData.getPosition(*it, position);
            // if (mapData.at("elevation", *it) == 0)
            // {
            //     ++num;
            // }
        }

        contactDates.position.resize(num, 3);
        contactDates.normalVector.resize(num, 3);
        contactDates.frcitionMu.resize(num);
        contactDates.maxNormalF.resize(num);

        // AA.resize(num, 3);
        // NormalVector.resize(num, 3);
        // frcitionMu.resize(num);
        // maxNormalF.resize(num);


        int row_index = 0;

        for (grid_map::CircleIterator it(mapData, center, radius); !it.isPastEnd(); ++it)
        {
            // grid_map::Position position;
            // mapData.getPosition(*it, position);
            if(mapData.isValid(*it,"elevation"))
            {
                grid_map::Position tmpP_;
                mapData.getPosition(*it, tmpP_);
                contactDates.normalVector.row(row_index) << mapData.at("normal_x", *it), mapData.at("normal_y", *it), mapData.at("normal_z", *it);
                contactDates.position.row(row_index) << tmpP_.x(), tmpP_.y(), mapData.at("elevation", *it);
                // contactDates.frcitionMu(row_index) = mapData.at("friction_mu", *it);
                // contactDates.maxNormalF(row_index) = mapData.at("max_normal_force", *it);
                contactDates.maxNormalF(row_index) =  1000 ; //- 100*tmpP_.x();
                row_index++;

            }
        }

        return contactDates;
    }

    MDT::AvailableContactsInfo getAvailableFootholds(const MDT::RobotState &hexapodState, const grid_map::GridMap &mapData)
    {
        MDT::AvailableContactsInfo availableContacts;

        // 环境可落足点集合 //这里是获得机体周围一个圆形区域的点
        auto result = get_now_Feasible_foot_position(hexapodState.pose, mapData);

        MatrixX3 map_feasible_position = result.position;

        // base约束foot
        auto Ab = Robot_State_Transition::get_kinematics_con_cog_foot(hexapodState.pose);

        for (int i = 0; i < 6; ++i)
        {
            if (hexapodState.gaitToNow[i] == SWING)
            {
                MatrixX3 A = Ab.first.block(7 * i, 0, 7, 3);
                VectorX b = Ab.second.block(7 * i, 0, 7, 1);

                // 判断环境中的点是否在kin质心约束foot的约束范围内
                MatrixXX temp = A * map_feasible_position.transpose();
                for (int j = 0; j < temp.cols(); ++j)
                {
                    if ((temp.col(j).array() <= b.array()).all())
                    {
                        availableContacts.position.leg[i].push_back(map_feasible_position.row(j).transpose());
                        availableContacts.normalVector.leg[i].push_back(result.normalVector.row(j).transpose());
                        availableContacts.frcitionMu[i].push_back(result.frcitionMu(j));
                        availableContacts.maxNormalF[i].push_back(result.maxNormalF(j));
                    }
                }
            }
        }
        return availableContacts;
    }




    bool swingLegFoot_position_Expert_forSim2(MDT::RobotState &hexapodState, const grid_map::GridMap &mapData)
    {
        // MDT::VectorList feasible_positions;
        for (int i = 0; i < 6; ++i)
        {
            if (hexapodState.gaitToNow[i] == SWING)
            {
                Eigen::Vector3d nF = HexapodParameter::norminalFoothold_B[i];
                Vector3 default_p0 = hexapodState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0, nF.y(), nF.z());
                Vector3 default_p1 = hexapodState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.25, nF.y(), nF.z());
                Vector3 default_p2 = hexapodState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.5, nF.y(), nF.z());

                // 检查default_p是否在地图中
                std::vector<Vector3> feasible_positions;

                grid_map::Position tmpP_;
                tmpP_ << default_p0.x(), default_p0.y();
                grid_map::Index it11;
                mapData.getIndex(tmpP_,it11);
                if(mapData.isValid(it11,"elevation"))
                {
                    default_p0.z() = mapData.at("elevation",it11);
                    feasible_positions.push_back(default_p0);
                }

                tmpP_ << default_p1.x(), default_p1.y();
                grid_map::Index it12;
                mapData.getIndex(tmpP_,it12);
                if(mapData.isValid(it12,"elevation"))
                {
                    default_p1.z() = mapData.at("elevation",it12);
                    feasible_positions.push_back(default_p1);
                }

                tmpP_ << default_p2.x(), default_p2.y();
                grid_map::Index it13;
                mapData.getIndex(tmpP_,it13);
                if(mapData.isValid(it13,"elevation"))
                {
                    default_p2.z() = mapData.at("elevation",it13);
                    feasible_positions.push_back(default_p2);
                }


                if (feasible_positions.size() != 0)
                {
                    // 获得一个0到10的随机数
                    int maxRow = rand() % feasible_positions.size();

                    hexapodState.feetPosition[i].x() = feasible_positions[maxRow](0);
                    hexapodState.feetPosition[i].y() = feasible_positions[maxRow](1);
                    hexapodState.feetPosition[i].z() = feasible_positions[maxRow](2);
                    hexapodState.feetNormalVector[i].x() = 0;
                    hexapodState.feetNormalVector[i].y() = 0;
                    hexapodState.feetNormalVector[i].z() = 1;

                    hexapodState.faultStateToNow[i] = NORMAL_LEG;
                }
                else
                {
                    // return false;
                    hexapodState.faultStateToNow[i] = FAULT_LEG;

                    Eigen::Vector3d dafult_position = HexapodParameter::norminalFoothold_B[i];
                    dafult_position(2) += 0.3;
                    dafult_position = hexapodState.pose.getT_W_B() * dafult_position;

                    hexapodState.feetPosition[i].x() = dafult_position.x();
                    hexapodState.feetPosition[i].y() = dafult_position.y();
                    hexapodState.feetPosition[i].z() = dafult_position.z();
                }
            }
        }
        return true;
    }




/**
     * \brief 根据当前状态 计算下一步可行接触状态 (挑选最大步长步态)
     * \param hexapodState 机器人当前状态
     * \param mapData  地图数据
     * \return 返回下一步可行接触状态
    */
    MDT::RobotState getNextMCTSstateByExpert_forSim(const MDT::RobotState rState, const grid_map::GridMap &mapData)
    {

        std::vector<MDT::RobotState> resultLists;
        std::vector<MDT::Vector6b> availableSupportState_ = PLANNING::findAvailableSupportStates(rState);

        if(availableSupportState_.size() == 0)
        {
            return rState;
        }
        assert(availableSupportState_.size() > 0 && "availableSupportState_ should not be empty.");

        std::vector<float> maxStepLengthForSupportState = findAvailableStepLength(rState,availableSupportState_);
        
        //获得maxStepLengthForSupportState中最大元素的索引
        int maxIndex = std::max_element(maxStepLengthForSupportState.begin(), maxStepLengthForSupportState.end()) - maxStepLengthForSupportState.begin();
        MDT::RobotState hexapodState = rState;
        hexapodState.gaitToNow = availableSupportState_[maxIndex];
        float stepLength_ = maxStepLengthForSupportState[maxIndex]*2.0/3.0;

        if(stepLength_ > 0.05f)
        {
            // 防止落在支撑多边形边上
            stepLength_ -= 0.05f;
        }
        else if(stepLength_ < 0.01) //防止在仿真过程中机器人不断挪动，由于数值问题移动出扇形工作空间
        {
            stepLength_ = 0;
        }
        

        hexapodState.pose.x += stepLength_ * cos(hexapodState.moveDirection);
        hexapodState.pose.y += stepLength_ * sin(hexapodState.moveDirection);
        // z轴方向的移动
        float averageFootholdsHeight = 0.0f;
        int count = 0;
        for(int j = 0; j < 6; ++j)
        {
            if(rState.faultStateToNow[j] == NORMAL_LEG)
            {
                count++;
                averageFootholdsHeight += rState.feetPosition[j].z();
            }
        }
        averageFootholdsHeight/=float(count);
        hexapodState.pose.z = averageFootholdsHeight + 0.5f;

        //findAvailableFootholdsCombination
        //findRandomFootholdsCombination
        if(PLANNING::swingLegFoot_position_Expert_forSim2(hexapodState, mapData)) 
        {
            return hexapodState;
        }

        return rState;


        // std::vector<MDT::RobotState> lists = PLANNING::getNextMCTSstateList(rState, mapData);

        // //随机选择一个lists里的元素
        // if(lists.size() == 0) return rState;
        // int randomIndex = rand() % lists.size();
        // return lists[randomIndex];
    }



    std::vector<MDT::RobotState> swingLegFoot_position_lists(const MDT::RobotState &rState, const grid_map::GridMap &mapData)
    {
        auto resultF = getAvailableFootholds(rState, mapData);
        MDT::VectorList feasible_positions = resultF.position;
        MDT::VectorList feasible_normalVector = resultF.normalVector;
        MDT::RobotState hexapodState1 = rState;
        MDT::RobotState hexapodState2 = rState;
        MDT::RobotState hexapodState3 = rState;

        std::vector<MDT::RobotState> resultLists;

        for (int i = 0; i < 6; ++i)
        {
            if (rState.gaitToNow[i] == SWING)
            {
                if (feasible_positions.leg[i].size() != 0)
                {
                    // 寻找x最大的落足点
                    // auto max_iter = std::max_element(feasible_positions.leg[i].begin(), feasible_positions.leg[i].end(),
                    //                                  [](const Vector3 &v1, const Vector3 &v2)
                    //                                  { return v1(0) < v2(0); }); // 找x坐标最大的点

                    // hexapodState1.feetPosition[i].x() = max_iter->x();
                    // hexapodState1.feetPosition[i].y() = max_iter->y();
                    // hexapodState1.feetPosition[i].z() = max_iter->z();
                    // hexapodState1.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(max_iter - feasible_positions.leg[i].begin()).x();
                    // hexapodState1.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(max_iter - feasible_positions.leg[i].begin()).y();
                    // hexapodState1.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(max_iter - feasible_positions.leg[i].begin()).z();

                    // // 获得一个随机落足点
                    // maxRow = rand() % feasible_positions.leg[i].size();
                    // hexapodState3.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    // hexapodState3.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    // hexapodState3.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    // hexapodState3.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    // hexapodState3.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    // hexapodState3.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();

                    // 寻找最靠近默认位置的落足点
                    Eigen::Vector3d nF = HexapodParameter::norminalFoothold_B[i];
                    Vector3 default_p = rState.pose.getT_W_B() * nF;
                    VectorX dis2(feasible_positions.leg[i].size());
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    VectorX::Index maxRow;
                    dis2.minCoeff(&maxRow);
                    hexapodState2.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState2.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState2.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState2.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState2.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState2.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState2.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState2.frcitionMu[i] = resultF.frcitionMu[i][maxRow];

                    Vector3 default_p1 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.5, nF.y(), nF.z());
                    Vector3 default_p2 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.25, nF.y(), nF.z());
                    // Vector3 default_p3 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.5, nF.y()-0.5, nF.z());
                    

                    default_p = default_p1;
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    dis2.minCoeff(&maxRow);
                    hexapodState1.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState1.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState1.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState1.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState1.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState1.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState1.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState1.frcitionMu[i] = resultF.frcitionMu[i][maxRow];

                    default_p = default_p2;
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    dis2.minCoeff(&maxRow);
                    hexapodState3.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState3.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState3.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState3.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState3.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState3.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState3.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState3.frcitionMu[i] = resultF.frcitionMu[i][maxRow];

                    hexapodState1.faultStateToNow[i] = NORMAL_LEG;
                    hexapodState2.faultStateToNow[i] = NORMAL_LEG;
                    hexapodState3.faultStateToNow[i] = NORMAL_LEG;
                }
                else
                {
                    Eigen::Vector3d dafult_position = HexapodParameter::norminalFoothold_B[i];
                    dafult_position(2) += 0.3;
                    dafult_position = hexapodState1.pose.getT_W_B() * dafult_position;
                    hexapodState1.feetPosition[i].x() = dafult_position.x();
                    hexapodState1.feetPosition[i].y() = dafult_position.y();
                    hexapodState1.feetPosition[i].z() = dafult_position.z();

                    hexapodState2.feetPosition[i] = hexapodState1.feetPosition[i];  
                    hexapodState3.feetPosition[i] = hexapodState1.feetPosition[i];

                    hexapodState1.faultStateToNow[i] = FAULT_LEG;
                    hexapodState2.faultStateToNow[i] = FAULT_LEG;
                    hexapodState3.faultStateToNow[i] = FAULT_LEG;
                }
            }
        }

        resultLists.push_back(hexapodState1);
        resultLists.push_back(hexapodState2);
        resultLists.push_back(hexapodState3);

        return resultLists;
    }


    std::vector<MDT::RobotState> swingLegFoot_position_lists_afterCollisionCheck(MDT::Vector3 &bodyPositionBegin, MDT::Vector3 &bodyPoseBegin, const MDT::RobotState &rState, 
        const grid_map::GridMap &mapData)
    {
        auto resultF = getAvailableFootholds(rState, mapData);
        MDT::VectorList feasible_positions = resultF.position;
        MDT::VectorList feasible_normalVector = resultF.normalVector;
        MDT::RobotState hexapodState1 = rState;
        MDT::RobotState hexapodState2 = rState;
        MDT::RobotState hexapodState3 = rState;

        std::vector<MDT::RobotState> resultLists;
        bool flag1 = true;
        bool flag2 = true;
        bool flag3 = true;

        MDT::Vector3 bodyPositionEnd;
        bodyPositionEnd << rState.pose.x, rState.pose.y, rState.pose.z;

        MDT::Vector3 bodyPoseEnd;
        bodyPoseEnd << rState.pose.roll, rState.pose.pitch, rState.pose.yaw;


        for (int i = 0; i < 6; ++i)
        {
            if (rState.gaitToNow[i] == SWING)
            {
                if (feasible_positions.leg[i].size() != 0)
                {
                    // 寻找x最大的落足点
                    // auto max_iter = std::max_element(feasible_positions.leg[i].begin(), feasible_positions.leg[i].end(),
                    //                                  [](const Vector3 &v1, const Vector3 &v2)
                    //                                  { return v1(0) < v2(0); }); // 找x坐标最大的点

                    // hexapodState1.feetPosition[i].x() = max_iter->x();
                    // hexapodState1.feetPosition[i].y() = max_iter->y();
                    // hexapodState1.feetPosition[i].z() = max_iter->z();
                    // hexapodState1.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(max_iter - feasible_positions.leg[i].begin()).x();
                    // hexapodState1.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(max_iter - feasible_positions.leg[i].begin()).y();
                    // hexapodState1.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(max_iter - feasible_positions.leg[i].begin()).z();

                    // // 获得一个随机落足点
                    // maxRow = rand() % feasible_positions.leg[i].size();
                    // hexapodState3.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    // hexapodState3.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    // hexapodState3.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    // hexapodState3.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    // hexapodState3.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    // hexapodState3.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();




                    // 寻找最靠近默认位置的落足点
                    Eigen::Vector3d nF = HexapodParameter::norminalFoothold_B[i];
                    Vector3 default_p = rState.pose.getT_W_B() * nF;
                    VectorX dis2(feasible_positions.leg[i].size());
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    VectorX::Index maxRow;
                    dis2.minCoeff(&maxRow);
                    hexapodState2.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState2.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState2.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState2.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState2.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState2.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState2.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState2.frcitionMu[i] = resultF.frcitionMu[i][maxRow];

                    Vector3 default_p1 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.5, nF.y(), nF.z());
                    Vector3 default_p2 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.25, nF.y(), nF.z());
                    // Vector3 default_p3 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.5, nF.y()-0.5, nF.z());
                    

                    default_p = default_p1;
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    dis2.minCoeff(&maxRow);
                    hexapodState1.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState1.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState1.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState1.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState1.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState1.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState1.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState1.frcitionMu[i] = resultF.frcitionMu[i][maxRow];

                    default_p = default_p2;
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    dis2.minCoeff(&maxRow);
                    hexapodState3.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState3.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState3.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState3.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState3.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState3.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState3.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState3.frcitionMu[i] = resultF.frcitionMu[i][maxRow];

                    hexapodState1.faultStateToNow[i] = NORMAL_LEG;
                    hexapodState2.faultStateToNow[i] = NORMAL_LEG;
                    hexapodState3.faultStateToNow[i] = NORMAL_LEG;

                    MDT::Vector3 startP;
                    startP << rState.feetPosition[i].x(), rState.feetPosition[i].y(), rState.feetPosition[i].z();

                    MDT::Vector3 endP;
                    endP << hexapodState1.feetPosition[i].x(), hexapodState1.feetPosition[i].y(), hexapodState1.feetPosition[i].z();
                    
                    if(!COLLISION_CHECK::isSwingLegCollision(startP, endP,  mapData, i, bodyPositionBegin, bodyPositionEnd, bodyPoseBegin, bodyPoseEnd))
                    {
                        flag1 = false;
                    }

                    endP << hexapodState2.feetPosition[i].x(), hexapodState2.feetPosition[i].y(), hexapodState2.feetPosition[i].z();
                    if(!COLLISION_CHECK::isSwingLegCollision(startP, endP,  mapData, i, bodyPositionBegin, bodyPositionEnd, bodyPoseBegin, bodyPoseEnd))
                    {
                        flag2 = false;
                    }

                    endP << hexapodState3.feetPosition[i].x(), hexapodState3.feetPosition[i].y(), hexapodState3.feetPosition[i].z();
                    if(!COLLISION_CHECK::isSwingLegCollision(startP, endP, mapData, i, bodyPositionBegin, bodyPositionEnd, bodyPoseBegin, bodyPoseEnd))
                    {
                        flag3 = false;
                    }
                }
                else
                {
                    Eigen::Vector3d dafult_position = HexapodParameter::norminalFoothold_B[i];
                    dafult_position(2) += 0.3;
                    dafult_position = hexapodState1.pose.getT_W_B() * dafult_position;
                    hexapodState1.feetPosition[i].x() = dafult_position.x();
                    hexapodState1.feetPosition[i].y() = dafult_position.y();
                    hexapodState1.feetPosition[i].z() = dafult_position.z();

                    hexapodState2.feetPosition[i] = hexapodState1.feetPosition[i];  
                    hexapodState3.feetPosition[i] = hexapodState1.feetPosition[i];

                    hexapodState1.faultStateToNow[i] = FAULT_LEG;
                    hexapodState2.faultStateToNow[i] = FAULT_LEG;
                    hexapodState3.faultStateToNow[i] = FAULT_LEG;
                }
            }
        }

        if(flag1)
        {
            resultLists.push_back(hexapodState1);
        }
        if(flag2)
        {
            resultLists.push_back(hexapodState2);
        }
        if(flag3)
        {
            resultLists.push_back(hexapodState3);
        }
        // resultLists.push_back(hexapodState1);
        // resultLists.push_back(hexapodState2);
        // resultLists.push_back(hexapodState3);

        return resultLists;
    }



    std::vector<MDT::RobotState> getNextMCTSstateList_underConstrains_moreStates(const MDT::RobotState rState, const grid_map::GridMap &mapData)
    {
        std::vector<MDT::RobotState> resultLists;
        // std::vector<MDT::Vector6b> availableSupportState_ = PLANNING::findAvailableSupportStates(rState);
        // std::vector<float> maxStepLengthForSupportState = findAvailableStepLength(rState,availableSupportState_);

        auto result = PLANNING::getSupportListAndStepL_underConstrains(rState,mapData);
        std::vector<MDT::Vector6b> availableSupportState_ = result.first;

        if(availableSupportState_.size() == 0)
        {
            return resultLists;
        }

        assert(availableSupportState_.size() > 0 && "availableSupportState_ should not be empty.");

        std::vector<Eigen::Vector3d> maxMotion = result.second;
        std::vector<double> maxStepLengthForSupportState(maxMotion.size());
        std::transform(maxMotion.begin(), maxMotion.end(), maxStepLengthForSupportState.begin(), [](const Eigen::Vector3d& motion) {
            return motion.x();
        });


        // z轴方向的移动
        float averageFootholdsHeight = 0.0f;
        int count = 0;
        for(int j = 0; j < 6; ++j)
        {
            if(rState.faultStateToNow[j] == NORMAL_LEG)
            {
                count++;
                averageFootholdsHeight += rState.feetPosition[j].z();
            }
        }
        averageFootholdsHeight/=float(count);

        for(int i=0;i<availableSupportState_.size();i++)
        {
            for(int j = 0; j<1; j++)
            {
                MDT::RobotState hexapodState = rState;
                hexapodState.gaitToNow = availableSupportState_[i];
                float stepLength_ = maxStepLengthForSupportState[i];


                hexapodState.pose.x += stepLength_ * cos(hexapodState.moveDirection);
                hexapodState.pose.y += stepLength_ * sin(hexapodState.moveDirection);
                // z轴方向的移动
                float maxZ = hexapodState.pose.z + result.second[i].z();
                if(averageFootholdsHeight+0.5f > maxZ)
                {
                    hexapodState.pose.z = maxZ;
                }
                else
                {
                    hexapodState.pose.z = averageFootholdsHeight + 0.5f;
                }

                


                if(USER::COLLISION_CHECK)
                {
                    Vector3 bodyPositionBegin;
                    bodyPositionBegin << rState.pose.x, rState.pose.y, rState.pose.z;
                    Vector3 bodyPoseBegin;
                    bodyPoseBegin << rState.pose.roll, rState.pose.pitch, rState.pose.yaw;
                    auto results = swingLegFoot_position_lists_afterCollisionCheck(bodyPositionBegin, bodyPoseBegin, hexapodState, mapData);
                    for(auto state_:results)
                    {
                        resultLists.push_back(state_);
                    }
                }
                else
                {
                    auto results = swingLegFoot_position_lists(hexapodState, mapData);
                    for(auto state_:results)
                    {
                        resultLists.push_back(state_);
                    }
                }
   

            }
        }
        return resultLists;
    }

    /**
     * \brief 根据当前状态 计算下一步可行接触状态 (挑选最大步长步态)
     * \param hexapodState 机器人当前状态
     * \param mapData  地图数据
     * \return 返回下一步可行接触状态
    */
    MDT::RobotState getNextMCTSstateByExpert_underConstrain(const MDT::RobotState rState, const grid_map::GridMap &mapData)
    {

        auto result = PLANNING::getSupportListAndStepL_underConstrains(rState,USER::mapData);
        std::vector<MDT::Vector6b> availableSupportState_ = result.first;

        if(availableSupportState_.size() == 0)
        {
            std::cout<<"availableSupportState_.size() == 0"<<std::endl;
            return rState;
        }

        assert(availableSupportState_.size() > 0 && "availableSupportState_ should not be empty.");

        std::vector<Eigen::Vector3d> maxMotion = result.second;
        std::vector<double> maxStepLengthForSupportState(maxMotion.size());
        std::transform(maxMotion.begin(), maxMotion.end(), maxStepLengthForSupportState.begin(), [](const Eigen::Vector3d& motion) {
            return motion.x();
        });


        
        //获得maxStepLengthForSupportState中最大元素的索引
        int maxIndex = std::max_element(maxStepLengthForSupportState.begin(), maxStepLengthForSupportState.end()) - maxStepLengthForSupportState.begin();
        MDT::RobotState hexapodState = rState;
        hexapodState.gaitToNow = availableSupportState_[maxIndex];
        float stepLength_ = maxStepLengthForSupportState[maxIndex]*3.0/3.0;

        std::cout << "************ index : " << maxIndex << std::endl;

        if(stepLength_ > 0.05f)
        {
            // 防止落在支撑多边形边上
            stepLength_ -= 0.05f;
        }
        else if(stepLength_ < 0.01) //防止在仿真过程中机器人不断挪动，由于数值问题移动出扇形工作空间
        {
            stepLength_ = 0;
        }
        

        hexapodState.pose.x += stepLength_ * cos(hexapodState.moveDirection);
        hexapodState.pose.y += stepLength_ * sin(hexapodState.moveDirection);
        // z轴方向的移动
        float averageFootholdsHeight = 0.0f;
        int count = 0;
        for(int j = 0; j < 6; ++j)
        {
            if(rState.faultStateToNow[j] == NORMAL_LEG)
            {
                count++;
                averageFootholdsHeight += rState.feetPosition[j].z();
            }
        }
        averageFootholdsHeight/=float(count);
        hexapodState.pose.z = averageFootholdsHeight + 0.5f;

        //findAvailableFootholdsCombination
        //findRandomFootholdsCombination

        // if(PLANNING::swingLegFoot_position_Expert_forSim2(hexapodState, mapData)) 
        // {
        //     return hexapodState;
        // }
        // grid_map::SignedDistanceField sdf(mapData, "elevation", -1, 3);

        Vector3 bodyPositionBegin;
        bodyPositionBegin << rState.pose.x, rState.pose.y, rState.pose.z;

        Vector3 bodyPoseBegin;
        bodyPoseBegin << rState.pose.roll, rState.pose.pitch, rState.pose.yaw;


        std::vector<MDT::RobotState> resultLists = swingLegFoot_position_lists_afterCollisionCheck(bodyPositionBegin, bodyPoseBegin, hexapodState, mapData);
        // std::vector<MDT::RobotState> resultLists = swingLegFoot_position_lists(hexapodState, mapData);

        if(resultLists.size() == 0)
        {
            std::cout << "resultLists.size() == 0" << std::endl;
            exit(0);
        }

        int randIndex = rand() % resultLists.size();
        return resultLists[randIndex];

    }


}