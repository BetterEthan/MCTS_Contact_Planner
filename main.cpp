#include <mpi.h>
#include <iostream>
#include <cstring>
#include <myDataType.h>
#include <geometryFun.h>
#include <grid_map_core/GridMap.hpp>
#include <planning.h>
#include "ros/ros.h"
#include <std_msgs/String.h>
#include "zobrist_hash.h"
#include <deque>
#include <hit_spider/hexapod_State.h>
#include <saveHashTable.h>
#include "constrains/Bretl.hh" //CWC
#include "constrains/dynamic_constrain.hh" //CWC


enum JobType {
    SEARCH = 0,
    BACKPROPAGATION = 1,
    PRIORITY_BORDER = 128,
    TIMEUP = 254,
    FINISH = 255,
    STORE_DATA = 2,
    PUB_DATA = 3,
    BEST_CHILD = 200,
    UPDATA_PRAENTS_CHILD =4,
    FORCE_BP = 5
};


//------机器人相关常量,全局共享变量声明------
#define FAULT_LEG 1
#define NORMAL_LEG 0
hit_spider::hexapod_State transRobotState(const MDT::RobotState &state_)
{
        hit_spider::hexapod_State hexapodState;
        hexapodState.base_Pose_Now.position.x = state_.pose.x;
        hexapodState.base_Pose_Now.position.y = state_.pose.y;
        hexapodState.base_Pose_Now.position.z = state_.pose.z;
        hexapodState.base_Pose_Now.orientation.roll = state_.pose.roll;
        hexapodState.base_Pose_Now.orientation.pitch = state_.pose.pitch;
        hexapodState.base_Pose_Now.orientation.yaw = state_.pose.yaw;
        hexapodState.base_Pose_Next = hexapodState.base_Pose_Now;

        for(int i=0;i<6;i++)
        {
            hexapodState.feetPositionNow.foot[i].x = state_.feetPosition[i].x();
            hexapodState.feetPositionNow.foot[i].y = state_.feetPosition[i].y();
            hexapodState.feetPositionNow.foot[i].z = state_.feetPosition[i].z();
            hexapodState.feetNormalVector.foot[i].x = state_.feetNormalVector[i].x();
            hexapodState.feetNormalVector.foot[i].y = state_.feetNormalVector[i].y();
            hexapodState.feetNormalVector.foot[i].z = state_.feetNormalVector[i].z();
            hexapodState.support_State_Now[i] = !state_.gaitToNow[i];
            hexapodState.faultLeg_State_Now[i] = NORMAL_LEG;
            hexapodState.maxNormalF[i] = state_.maxNormalForce[i];
            hexapodState.mu[i] = state_.frcitionMu[i];
        }

        hexapodState.move_Direction.x = cos(state_.moveDirection);
        hexapodState.move_Direction.y = sin(state_.moveDirection);
        hexapodState.move_Direction.z = 0;

        // 下一步落足点
        hexapodState.feetPositionNext = hexapodState.feetPositionNow;
        // 下一步支撑状态和容错状态
        hexapodState.support_State_Next = hexapodState.support_State_Now;
        hexapodState.faultLeg_State_Next = hexapodState.faultLeg_State_Now;

        

        return hexapodState;
}



struct MessageMPI
{
    // std::string hashKey;
    char hashKey[MAX_DEPTH];
    MDT::RobotState rState;
    double score;
    int visits;
    int num_thread_visited;
    // MDT::RobotState parentNode;

    MessageMPI(const std::string& key, const MDT::RobotState& state, double s, int v, int nt_visited)
        : rState(state), score(s), visits(v), num_thread_visited(nt_visited)
    {
        // 复制 hashKey 的内容到结构体的成员变量中
        std::strncpy(hashKey, key.c_str(), MAX_DEPTH);
        hashKey[MAX_DEPTH - 1] = '\0'; // 确保字符串以'\0'结尾
    }

    MessageMPI(const TreeNode_ptr& node)
        : rState(node->rState), score(node->score), visits(node->visits), num_thread_visited(node->num_thread_visited)
    {
        // 复制 hashKey 的内容到结构体的成员变量中
        std::strncpy(hashKey, node->hashKey.c_str(), MAX_DEPTH);
        hashKey[MAX_DEPTH - 1] = '\0'; // 确保字符串以'\0'结尾
    }


    MessageMPI()
    {
        // 构造函数体
    }

};

// 将 MessageMPI 对象序列化为字符数组
char* serializeMessage(const MessageMPI& message) {
    const int bufferSize = sizeof(MessageMPI);
    char* buffer = new char[bufferSize];
    std::memcpy(buffer, &message, bufferSize);
    return buffer;
}

// 从接收到的字符数组中反序列化为 MessageMPI 对象
MessageMPI deserializeMessage(const char* buffer) {
    MessageMPI message;
    std::memcpy(&message, buffer, sizeof(MessageMPI));
    return message;
}

void sendMessage(const MessageMPI &message, int receiver, int tag, MPI_Comm comm) {
    char* serializedMessage = serializeMessage(message);
    MPI_Request sreq;
    // 发送数据
    MPI_Send(serializedMessage, sizeof(MessageMPI), MPI_CHAR, receiver, tag, comm);

    // 释放内存
    delete[] serializedMessage; 
}

std::pair<int, MessageMPI> receiveMessage(MPI_Comm comm) {
    MPI_Status status;
    // MPI_Probe(MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &status);
    // std::cout << "receiveMessage: " << status.MPI_TAG << std::endl;
    int bufferSize = sizeof(MessageMPI);
    // 创建接收缓冲区
    char* buffer = new char[bufferSize];
    // 接收数据
    MPI_Recv(buffer, bufferSize, MPI_CHAR, MPI_ANY_SOURCE, MPI_ANY_TAG, comm, &status);
    MessageMPI message = deserializeMessage(buffer);
    delete[] buffer;
    return std::make_pair(status.MPI_TAG, message);

}

void sendBackpropagationMessage(HashTable &hsm ,const TreeNode_ptr &node, MPI_Comm comm) {
    std::string parentKey_ = node->hashKey.substr(0, node->hashKey.length() - 1);
    auto result = hsm.hashing(parentKey_);  // 查找进程号
    int dest = result.second;
    // 将新节点发送给对应的进程
    MessageMPI newMsg(node); 
    sendMessage(newMsg, dest, JobType::BACKPROPAGATION, comm);
    }

void sendForceBPnMessage(HashTable &hsm ,const TreeNode_ptr &node, MPI_Comm comm) {
        std::string parentKey_ = node->hashKey.substr(0, node->hashKey.length() - 1);
        auto result = hsm.hashing(parentKey_);  // 查找进程号
        int dest = result.second;
        // 将新节点发送给对应的进程
        MessageMPI newMsg(node); 
        sendMessage(newMsg, dest, JobType::FORCE_BP, comm);
    }


void sendUpdateChildMessage(HashTable &hsm ,const TreeNode_ptr &node, MPI_Comm comm) {
    std::string parentKey_ = node->hashKey.substr(0, node->hashKey.length() - 1);
    auto result = hsm.hashing(parentKey_);  // 查找进程号
    int dest = result.second;
    // 将新节点发送给对应的进程
    MessageMPI newMsg(node); 
    sendMessage(newMsg, dest, JobType::UPDATA_PRAENTS_CHILD, comm);
    }


void sendSearchMessage(HashTable &hsm ,const TreeNode_ptr &node, MPI_Comm comm) {
        auto result = hsm.hashing(node->hashKey);  // 查找进程号
        int dest = result.second;
        // 将新节点发送给对应的进程
        MessageMPI newMsg(node); // 这里parent有问题!!!!
        sendMessage(newMsg, dest, JobType::SEARCH, comm);
    }

struct Person {
  int id;
  char name[20];
};




void ParalleMCTS_FUN(HashTable &hsm, const grid_map::GridMap &mapData_,  MPI_Comm comm, int &argc, char** &argv) {
    MPI_Status status;
    int rank, nprocs;
    MPI_Comm_rank(comm, &rank);
    MPI_Comm_size(comm, &nprocs);


    std::pair<int, int> result = hsm.hashing("&");
    int rootdest = result.second;


    std::cout << "rank:" << rank << ":  " ;
    std::cout << "rootdest: " << rootdest << std::endl;

    std::deque<std::pair<JobType, MessageMPI>> jobq;
    bool timeup = false;


    // 初始化机器人状态,并赋初值
    MDT::Pose robotPoseW = {0.0, 0, 0.5,  0,  0,  0*_PI_/6};
    MDT::Vector6b gaitToNow;
    gaitToNow << SUPPORT,SUPPORT,SUPPORT,SUPPORT,SUPPORT,SUPPORT;
    float moveDir = 0*_PI_/2;
    MDT::RobotState state_ = HexapodParameter::initRobotState(robotPoseW, gaitToNow, moveDir);
    
    // 初始化单个任务, 后续运行后会增加新任务
    if (rank == rootdest) {
        
        MessageMPI initMsg("&", state_, 0, 0, 0);
        jobq.push_front(std::make_pair(JobType::SEARCH, initMsg));
        // std::cout << "jobq size: " << jobq.size() << std::endl;
    }



    double timeWork = 0;
    float  maxExtendedNode_Length = 0.0f;
    std::string maxExtendedNode_hashKey = "";

    double start_time = MPI_Wtime();

    if(rank == 0) std::cout << "运行时间为:" << USER::SEARCH_TIME_LIMIT << "s" << std::endl;
    bool STOP_FLAG = false;
    static float notworkTime = 0;
    bool assignTaskFlag = false;
    
    while (!timeup) {
        auto startT = MPI_Wtime();
        bool noneJob1 = false;
        bool noneJob2 = false;
        
        if(jobq.empty()) noneJob1 = true;

        // 时间停止判断, 如果超过阈值,则发送TIMEUP消息, 要求其他进程停止
        if (rank == 0) {
            if (MPI_Wtime() - start_time > USER::SEARCH_TIME_LIMIT || STOP_FLAG) {
                timeup = true;
                for (int dest = 1; dest < nprocs; dest++) {
                    int dummy_data = static_cast<int>(JobType::TIMEUP);
                    MessageMPI newMsg;
                    sendMessage(newMsg, dest, JobType::TIMEUP, comm);
                }
            }
        }

        if(assignTaskFlag == false)
        {
            if (rank == rootdest) {
                if (MPI_Wtime() - start_time > 0.05)
                {
                    MessageMPI initMsg("&", state_, 0, 0, 0);
                    for (int i = 0; i < USER::JOB_FACTOR * nprocs; i++) {
                        jobq.push_front(std::make_pair(JobType::SEARCH, initMsg));
                    }
                    assignTaskFlag = true;
                    std::cout << "jobq size: " << jobq.size() << std::endl;
                }
            }
        }

        

        // receive all incoming messages and push to job_queue
        while (true) {
            MPI_Status cur_status;
            int flag;
            //查询指定源和标签的消息是否在接收队列中。
            MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, comm, &flag, &cur_status);
            
            if (flag == 0) {
                break;  // 没有消息, 退出接收消息循环
            } else {
                std::pair<int, MessageMPI> data_recieve = receiveMessage(comm);

                std::pair<JobType, MessageMPI> job(static_cast<JobType>(data_recieve.first), data_recieve.second);
                if (static_cast<JobType>(data_recieve.first) == JobType::TIMEUP ||
                    static_cast<JobType>(data_recieve.first) == JobType::FINISH) {
                    jobq.push_front(job);  // 在队列的前面插入元素  (插队)
                } else {
                    jobq.push_back(job);  // 在队列的后面插入元素
                }
            }
        }

        if(jobq.empty()) noneJob2 = true;
        // 再处理所有的消息
        bool jobq_non_empty = !jobq.empty();

        if(noneJob1 && noneJob2) {
            notworkTime += MPI_Wtime() - startT;
        }

        if (jobq_non_empty) {  // 如果任务不为空
            std::pair<JobType, MessageMPI> message = jobq.front(); // 访问队列的最后一个元素
            jobq.pop_front(); // 弹出列队前面元素


            if (message.first == JobType::SEARCH) {  // SELECT 阶段
                std::string key_(message.second.hashKey);
                // 如果该节点不在哈希表中, 那么对它进行扩展仿真
                if (hsm.search_table(key_) == nullptr) {
                    std::shared_ptr<TreeNode> node = std::make_shared<TreeNode>(message.second.rState, key_);
                    // 进行仿真
                    float simDistance = node->simulation(node->rState, mapData_); // 唯一仿真位置..............

                    // 计算当前节点的分值
                    float score_ = node->calculateLocalScore(simDistance);
                    // backpropagation on local memory
                    node->updateLocalSimDis(simDistance);
                    node->updateLocalNode(score_);  
                    
                    if (node->hashKey.length() > MAX_DEPTH-1)
                    {
                        std::cout << "节点深度超过阈值" << std::endl;
                        exit(0);
                    }
                    
                    hsm.insert(Item(node->hashKey, node));
                    sendBackpropagationMessage(hsm, node, comm);
                }
                // 如果该节点在哈希表中
                else {

                    TreeNode_ptr node = hsm.search_table(message.second.hashKey);

                    // 异常处理
                    if(node->score < 0)
                    {

                        if(node->candidateNodes.empty())
                        {
                            std::cout << "节点分值为0,可能冲突, 应该强制反向传播" << std::endl;
                            std::string parentKey_ = node->hashKey.substr(0, node->hashKey.length() - 1);
                            std::cout << "hashKey: " << node->hashKey << std::endl;
                            std::cout << "num_thread_visited:" << message.second.num_thread_visited << std::endl;
                            std::cout << "score: " << node->score << std::endl;
                            std::cout << "messageScore: " << message.second.score << std::endl;
                            std::cout << "childNode size: " << node->childNodes.size() << std::endl << std::endl;
                            if(!node->childNodes.empty())
                            {
                                TreeNode_ptr childnode = node->findBestChild();
                                std::cout << "Best child score: " << childnode->score << std::endl;
                            }

                            std::cout << "candidateNodes为空" << std::endl;
                            std::cout << std::endl;
                            // sleep(0.2);
                            sendForceBPnMessage(hsm, node, comm);
                            continue;
                            // continue;
                        }
                        for(int i =0;i<node->candidateNodes.size();i++)
                        {
                            std::cout << i << " _ candidateNodes scores: " << node->childNodes[i]->score << std::endl;
                        }
                        std::cout << "异常情况,退出运行." << std::endl;
                        exit(0);
                    }

                    if(node->rState.pose.x > maxExtendedNode_Length) {
                        maxExtendedNode_Length = node->rState.pose.x;
                        maxExtendedNode_hashKey = node->hashKey;
                    }

                    // 如果仍有备选节点，将其选出放进儿子集合中，然后可以进行仿真
                    if (!node->candidateNodes.empty()) {
                        // 从候选节点中随机选择一个节点
                        int random_index = rand() % node->candidateNodes.size();
                        // 将其添加到儿子集合中
                        TreeNode_ptr newChild = node->addNode(random_index);
                        // 扩展后才把node加入hash表中
                        hsm.insert(Item(node->hashKey, node));
                        sendSearchMessage(hsm ,newChild, comm); 
                    }
                    else {  // 无备选儿子
                        // 无备选儿子，且最开始就没有任何备选节点，说明没有展开过
                        if (node->check_candidateNodes == false) {
                            node->expansion(mapData_);
                            if (node->candidateNodes.empty()) {  // 如果扩展后仍无备选节点，直接设定该点的分值为负无穷
                                int score_ = -10000;
                                node->updateLocalNode(score_);   // todo: 进行仿真的时候节点传过来没有parent, 而且childNode等都没有(这个不一定需要), 应该在add儿子时把节点添加到hash表中,然后通过查找hash表,避免太多的通信传递
                                hsm.insert(Item(node->hashKey, node));
                                sendUpdateChildMessage(hsm, node, comm);
                            }
                            else {
                                // 从候选节点中随机选择一个节点
                                int random_index = rand() % node->candidateNodes.size();
                                // 将其添加到儿子集合中
                                TreeNode_ptr newChild = node->addNode(random_index);
                                // 扩展后才把node加入hash表中
                                hsm.insert(Item(node->hashKey, node));
                                sendSearchMessage(hsm ,newChild, comm); 
                            }
                        }
                        else {  // 节点无备选儿子且已经扩展过，选择最佳儿子并进入SELECTION阶段继续扩展
                            TreeNode_ptr childnode = node->selection(USER::IsVirtualLoss);

                            if (childnode->score < 0) {  
                                sendBackpropagationMessage(hsm, childnode, comm);  // 这里是可以反传播的,因为最佳儿子是负值,那么父节点就没有更好的儿子了.
                            }
                            else {
                                hsm.insert(Item(node->hashKey, node));
                                sendSearchMessage(hsm, childnode, comm); 
                            }
                        }
                    }
                }
            }
            else if (message.first == JobType::BACKPROPAGATION) { // 反向传播部分
                std::string key_(message.second.hashKey);
                TreeNode_ptr node = std::make_shared<TreeNode>(message.second.rState, key_);
                node->score = message.second.score;

                TreeNode_ptr local_parent = hsm.search_table(node->hashKey.substr(0, node->hashKey.length() - 1)); // local_parent是node的parent ,[0:-1]即表示切片第一个到倒数第一个(不包含倒数第一个)的所有元素。
                
                // 这是处理第一次根节点仿真的情况
                if(node->hashKey == "&") {
                    if (node->score < 0) {
                        std::cout << "卡死，结束运行**" << std::endl;
                        exit(0);
                    }
                    sendSearchMessage(hsm ,node, comm); 
                }
                else if (local_parent->hashKey == "&") { // 如果传播到根节点

                    local_parent->backpropagation(node);

                    hsm.insert(Item(local_parent->hashKey, local_parent));
                    sendSearchMessage(hsm ,local_parent, comm); 
                }
                else { // 未到根节点则继续反向传播

                    int flag = local_parent->backpropagation(node);
                    if(flag == BP_TYPE::BETTER_CHILD_BP)
                    {
                        hsm.insert(Item(local_parent->hashKey, local_parent));
                        sendBackpropagationMessage(hsm, local_parent, comm);
                    }
                    else if(flag == BP_TYPE::NEGATIVE_BP)
                    {
                        hsm.insert(Item(local_parent->hashKey, local_parent));
                        sendBackpropagationMessage(hsm ,local_parent, comm); 
                    }
                    else if(flag == BP_TYPE::NO_BETTER_CHILD_BP)
                    {
                        hsm.insert(Item(local_parent->hashKey, local_parent));
                        sendSearchMessage(hsm ,local_parent, comm); 
                    }

                }
            }
            else if (message.first == JobType::FORCE_BP) { // 反向传播部分
                std::string key_(message.second.hashKey);
                TreeNode_ptr node = std::make_shared<TreeNode>(message.second.rState, key_);
                node->score = message.second.score;

                

                TreeNode_ptr local_parent = hsm.search_table(node->hashKey.substr(0, node->hashKey.length() - 1)); // local_parent是node的parent ,[0:-1]即表示切片第一个到倒数第一个(不包含倒数第一个)的所有元素。
                 if (node->hashKey == "&")
                 {
                        std::cout << "\033[1;33m"; // 1表示粗体，33表示黄色
                        std::cout << "local_parent为空!";
                        std::cout << "\033[0m"; // 恢复默认颜色
                        sendSearchMessage(hsm ,node, comm); 
                        continue;
                 }
                // 更新visit次数和num_thread_visited次数, 不更新节点分值.
                local_parent->backpropagation_Force(node);
                hsm.insert(Item(local_parent->hashKey, local_parent));

                if (local_parent->hashKey == "&") { // 如果传播到根节点
                    sendSearchMessage(hsm ,local_parent, comm); 
                }
                else {
                    sendForceBPnMessage(hsm, local_parent, comm);
                }
            }
            else if (message.first == JobType::UPDATA_PRAENTS_CHILD)
            {
               std::string key_(message.second.hashKey);
                TreeNode_ptr node = std::make_shared<TreeNode>(message.second.rState, key_);
                node->score = message.second.score;

                TreeNode_ptr local_parent = hsm.search_table(node->hashKey.substr(0, node->hashKey.length() - 1)); // local_parent是node的parent ,[0:-1]即表示切片第一个到倒数第一个(不包含倒数第一个)的所有元素。
                
                if(node->hashKey == "&") {
                    if (node->score < 0) {
                        std::cout << "负值反向传播到根节点, 结束运行." << std::endl;
                        STOP_FLAG = true;
                        exit(0);
                    }
                    sendSearchMessage(hsm ,node, comm); 
                }
                else  //更新parent的child中分值信息,然后继续搜索parent节点
                {
                    local_parent->updateChildBP(node);
                    hsm.insert(Item(local_parent->hashKey, local_parent));

                    sendForceBPnMessage(hsm, local_parent, comm);
                }
            }
            else if (message.first == JobType::TIMEUP) {
                timeup = true;
            }



        }
    

    }



    std::cout << "Processor " << rank << "notWorkTime:" << notworkTime << std::endl;
    /***************task2 find the furtherest node****************************************************/
    // 使用MPI_Barrier可以确保在继续执行后续代码之前，所有进程都已经完成了之前的计算任务。
    MPI_Barrier(MPI_COMM_WORLD); 
    jobq.clear();
    bool firstFlag = true;
    MDT::RobotState max_element;
    std::string maxExtendedNode_hashKey_allProcess;

    while (true) {
        if (rank == 0) {
            if (firstFlag) {
                TreeNode_ptr node = hsm.search_table(maxExtendedNode_hashKey);
                MessageMPI newMsg(node); 
                sendMessage(newMsg, 0, JobType::BEST_CHILD, comm);
                std::cout << "Processor " << rank << ", expandedN: " << hsm.hashTable.size() << std::endl;
                firstFlag = false;
            }

            while (true) {
                MPI_Status cur_status;
                int flag;
                //查询指定源和标签的消息是否在接收队列中。
                MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, comm, &flag, &cur_status);
                
                if (flag == 0) {
                    break;  // 没有消息, 退出接收消息循环
                } else {
                    std::pair<int, MessageMPI> data_recieve = receiveMessage(comm);

                    std::pair<JobType, MessageMPI> job(static_cast<JobType>(data_recieve.first), data_recieve.second);
                    if (static_cast<JobType>(data_recieve.first) == JobType::BEST_CHILD) {
                        jobq.push_back(job);  // 在队列的后面插入元素
                    }
                }
            }

            // 如果收到的消息数等于进程数，说明所有进程都已经返回了结果
            if (jobq.size() == nprocs) {
                std::vector<MDT::RobotState> nodeList;
                std::vector<std::string> keyList;
                for (const auto& job : jobq) {
                    nodeList.push_back(job.second.rState);
                    keyList.push_back(job.second.hashKey);
                    std::cout << "furthest for each processor: " << job.second.rState.pose.x << ", hashKey:" << job.second.hashKey << std::endl;
                }
                
                int max_index = -1;
                double max_value = -std::numeric_limits<double>::infinity();
                for (int i = 0; i < nodeList.size(); i++) {
                    if (nodeList[i].pose.x > max_value) {
                        max_value = nodeList[i].pose.x;
                        max_index = i;
                        maxExtendedNode_hashKey_allProcess = keyList[i];
                    }
                }
                
                max_element = nodeList[max_index];
                std::cout << "furtherst: " << max_element.pose.x << std::endl;

                break;
            }
        }
        // 其余所有线程发送最佳儿子给0号线程
        else {
            TreeNode_ptr node = hsm.search_table(maxExtendedNode_hashKey);
            MessageMPI newMsg(node); 
            sendMessage(newMsg, 0, JobType::BEST_CHILD, comm);
            std::cout << "Processor " << rank << ", expandedN: " << hsm.hashTable.size() << std::endl;
            break;
        }
    }

    /***************task3 get the contact sequence****************************************************/
    // 使用MPI_Barrier可以确保在继续执行后续代码之前，所有进程都已经完成了之前的计算任务。
    sleep(1);
    
    MPI_Barrier(MPI_COMM_WORLD); 
    std::cout << "Processor " << rank << " is ready! " << std::endl;

    jobq.clear();
    bool firstFlag2 = true;
    std::vector<MDT::RobotState> sequence_result;

    while (true) {
        // 发送所要最远节点父节点信息的消息
        if (firstFlag2 && rank == 0) { 
            // sequence_result.push_back(max_element);
            int dest = hsm.hashing(maxExtendedNode_hashKey_allProcess).second; // 查找进程号
            // 将新节点发送给对应的进程
            MessageMPI newMsg(maxExtendedNode_hashKey_allProcess, max_element, 0,0,0);
            sendMessage(newMsg, dest, JobType::PUB_DATA, comm);
            std::cout << "Processor " << rank << ", pub node: " << maxExtendedNode_hashKey_allProcess << std::endl;
            firstFlag2 = false;
        }

        // 先接收所有的消息 
        while (true) {
            MPI_Status cur_status;
            int flag;
            //查询指定源和标签的消息是否在接收队列中。
            MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, comm, &flag, &cur_status);
            
            if (flag == 0) {
                break;  // 没有消息, 退出接收消息循环
            } else {
                std::pair<int, MessageMPI> data_recieve = receiveMessage(comm);
                std::pair<JobType, MessageMPI> job(static_cast<JobType>(data_recieve.first), data_recieve.second);
                if(static_cast<JobType>(data_recieve.first) == JobType::TIMEUP)
                    jobq.push_front(job);  // 在队列的前面插入元素
                else if(static_cast<JobType>(data_recieve.first) == JobType::STORE_DATA)
                    jobq.push_front(job);  // 在队列的前面插入元素
                else if(static_cast<JobType>(data_recieve.first) == JobType::PUB_DATA)
                    jobq.push_back(job);  // 在队列的后面插入元素
            }
        }

        if (rank == 0) {

            // 再处理所有的消息sour
            bool jobq_non_empty = !jobq.empty();
            if (jobq_non_empty) {  // 如果任务不为空
                std::pair<JobType, MessageMPI> message = jobq.front(); // 访问队列的最后一个元素
                jobq.pop_front(); // 弹出列队前面元素
                std::string key_(message.second.hashKey);

                // 获得所有数据之后的输出操作
                if ((key_ == "&") && (message.first == JobType::STORE_DATA)) {
                    std::cout << "Find root node!" << std::endl;
                    std::cout << key_ << std::endl;
                    std::cout << message.second.rState.pose.x << std::endl;
                    sequence_result.push_back(message.second.rState);

                    // timeup = true;
                    for (int dest = 1; dest < nprocs; dest++) {
                        int dummy_data = static_cast<int>(JobType::TIMEUP);
                        MessageMPI newMsg;
                        sendMessage(newMsg, dest, JobType::TIMEUP, comm);
                    }

                    std::reverse(sequence_result.begin(), sequence_result.end());
                    std::vector<hit_spider::hexapod_State> stateList;
                    for (const auto& data : sequence_result) {
                        stateList.push_back(transRobotState(data));
                    }

                    // 输出stateList最后一个元素
                    stateList.back().remarks.data = "end_flag";

                    std::cout << "press any key to publish..." << std::endl;
                    ros::init(argc, argv, "singleThreadMCTS");
                    ros::NodeHandle nh_;
                    ros::Publisher supportStatePub = nh_.advertise<hit_spider::hexapod_State>("supportStateTopic", 500, true);
                    getchar();
                    // 发布结果
                    for (const auto& state : stateList)
                    {
                        supportStatePub.publish(state);
                        std::cout << state.base_Pose_Now.position.x << " " << state.base_Pose_Now.position.y << " " << state.base_Pose_Now.position.z << " " << state.base_Pose_Now.orientation.roll << " " << state.base_Pose_Now.orientation.pitch << " " << state.base_Pose_Now.orientation.yaw << std::endl;
                        
                        
                        ros::Duration(0.1).sleep();
                    }
                    std::cout << "publish over" << std::endl;
                    break;
                }                

                if (message.first == JobType::STORE_DATA) {
                    sequence_result.push_back(message.second.rState);

                    std::string parentKey_ = key_.substr(0, key_.length() - 1);
                    int dest = hsm.hashing(parentKey_).second; // 查找进程号
                    // 将新节点发送给对应的进程
                    MessageMPI newMsg(parentKey_, message.second.rState, 0,0,0);
                    sendMessage(newMsg, dest, JobType::PUB_DATA, comm);
                }
                else if (message.first == JobType::PUB_DATA) {
                    auto node_ = hsm.search_table(key_);
                    MessageMPI newMsg(node_);
                    sendMessage(newMsg, 0, JobType::STORE_DATA, comm);
                    // std::cout << "rank0 pub job end!" << std::endl;
                }
            }
        }
        else {
            bool jobq_non_empty = !jobq.empty();
            if (jobq_non_empty) {
                std::pair<JobType, MessageMPI> message = jobq.front(); // 访问队列的最后一个元素
                jobq.pop_front(); // 弹出列队前面元素
                std::string key_(message.second.hashKey);

                if (message.first == JobType::TIMEUP) {
                    break;
                }
                auto node_ = hsm.search_table(key_);

                MessageMPI newMsg(node_);
                sendMessage(newMsg, 0, JobType::STORE_DATA, comm);
            }
        }
    }


    /***************task 4: collet all the tree data********************************************************************/
    MPI_Barrier(MPI_COMM_WORLD); 

    while(true)
    {
        // 先接收所有的消息 
        while (true) {
            MPI_Status cur_status;
            int flag;
            //查询指定源和标签的消息是否在接收队列中。
            MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, comm, &flag, &cur_status);
            if (flag == 0) {
                break;  // 没有消息, 退出接收消息循环
            } else {
                std::pair<int, MessageMPI> data_recieve = receiveMessage(comm);
                std::pair<JobType, MessageMPI> job(static_cast<JobType>(data_recieve.first), data_recieve.second);
                if(static_cast<JobType>(data_recieve.first) == JobType::TIMEUP)
                    jobq.push_front(job);  // 在队列的前面插入元素
                else if(static_cast<JobType>(data_recieve.first) == JobType::STORE_DATA)
                    jobq.push_front(job);  // 在队列的前面插入元素
                else if(static_cast<JobType>(data_recieve.first) == JobType::PUB_DATA)
                    jobq.push_back(job);  // 在队列的后面插入元素
            }
        }
        if (rank == 0) {
            static int count = 0; 
            // 再处理所有的消息
            bool jobq_non_empty = !jobq.empty();
            if (jobq_non_empty || nprocs == 1) {  // 如果任务不为空, 或只有一个线程
                if(nprocs > 1)
                {
                    std::pair<JobType, MessageMPI> message = jobq.front(); // 访问队列的最后一个元素
                    jobq.pop_front(); // 弹出列队前面元素
                    std::string key_(message.second.hashKey);
                    if (message.first == JobType::STORE_DATA) {
                        TreeNode_ptr node = std::make_shared<TreeNode>(message.second.rState, key_);
                        node->score = message.second.score;
                        node->visits = message.second.visits;
                        node->num_thread_visited = message.second.num_thread_visited;

                        hsm.insert(Item(key_, node));
                    }
                    else if (message.first == JobType::TIMEUP) {
                        count++;
                    }
                }
                if(count == nprocs-1)
                {
                    std::cout << "all data is collected!" << std::endl;
                    std::cout << "there are " << hsm.hashTable.size() << " nodes in the tree!" << std::endl;
                    std::cout << "press any key to store data..." << std::endl;
                    // getchar();
                    // 保存数据
                    std::vector<SAVE_HASH_TABLE::saveData> saveDataList;

                    for (const auto& data : hsm.hashTable) {
                        saveDataList.push_back(SAVE_HASH_TABLE::saveData(*data.second.value));
                    }
                    SAVE_HASH_TABLE::saveBinaryFile(saveDataList, "saveDataList.bin");
                    std::cout << "data are stored in saveDataList.bin!" << std::endl;
                    break;
                }
            }
        }
        else {
            TreeNode_ptr temp;
            for (const auto& data : hsm.hashTable) {
                MessageMPI newMsg(data.second.value);
                sendMessage(newMsg, 0, JobType::STORE_DATA, comm);
                temp = data.second.value;
                sleep(0.01);
                // saveDataList.push_back(SAVE_HASH_TABLE::saveData(*data.second.value));
            }
            MessageMPI newMsg(temp);
            sendMessage(newMsg, 0, JobType::TIMEUP, comm);
            break;
        }
    }

 
}

// #include "gperftools/profiler.h"
int main(int argc, char** argv) {
    USER::mapData = USER::init_grid_map(USER::configMap);
    
    MPI_Init(&argc, &argv); // 初始化MPI环境
    int size, rank;
    MPI_Comm_size(MPI_COMM_WORLD, &size); // 获取进程数
    MPI_Comm_rank(MPI_COMM_WORLD, &rank); // 获取当前进程的rank
    
    int processorN = size;

    HashTable hsm(processorN, USER::key_element, USER::max_depth, USER::key_element.size());  // 每个线程都有一个hash表

    MPI_Barrier(MPI_COMM_WORLD); 

    ParalleMCTS_FUN(hsm, USER::mapData, MPI_COMM_WORLD, argc, argv);

    MPI_Finalize(); // 结束MPI环境
    return 0;
}