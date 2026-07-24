#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream> 
#include <math.h>
#include <cmath>
#include <random>
#include <zmqpp/zmqpp.hpp>
#include <nlohmann/json.hpp>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h" 
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "rclcpp/rclcpp.hpp"
#include "bfc_msgs/msg/button.hpp"
#include "bfc_msgs/msg/head_movement.hpp"
#include "bfc_msgs/msg/coordination.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "rrt_planner.hpp"
#include "nav_msgs/msg/path.hpp"

//#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
//#include "darknet_ros_msgs/msg/object_count.hpp"
//#include "yolo_msgs/msg/midpoints.hpp"
#include <functional>
#include <algorithm>
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <fstream>
#include <sstream>
#include <vector>

#define PI 3.1415926535897932384626433832795
using namespace std::chrono_literals;
using namespace BT;
using namespace std;

class main_strategy : public rclcpp::Node
{
public:
    main_strategy()
        : Node("main_strategy", rclcpp::NodeOptions().use_intra_process_comms(true)), socket_(context_, zmqpp::socket_type::reply)
    {
        socket_.bind("tcp://0.0.0.0:5555");
        declareParameters();
        getParameters();

        robotNumber = this->get_parameter("robotNumber").as_int();

        button_ = this->create_subscription<bfc_msgs::msg::Button>(
            "button", 1,
            std::bind(&main_strategy::readButton, this, std::placeholders::_1));
        imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 1,
            std::bind(&main_strategy::readImu, this, std::placeholders::_1));
        gameControllerSubscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "game_controller", 1,
            std::bind(&main_strategy::readGameControllerData, this, std::placeholders::_1));

        trackbarSubscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "trackbar", 1,
            std::bind(&main_strategy::readTrackbar, this, std::placeholders::_1));

        voltage_n_odometry = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "voltodom", 1,
            std::bind(&main_strategy::readVoltageAndOdom, this, std::placeholders::_1));

        GridSub_ = this->create_subscription<std_msgs::msg::Int32>(
            "grid", 10,
            std::bind(&main_strategy::readGrid, this, std::placeholders::_1));
            
        //subscriber_yolo = this->create_subscription<yolo_msgs::msg::Midpoints>(
	  //  "/yolo/midpoints", 1,
	    //std::bind(&main_strategy::callbackMidpoints, this, std::placeholders::_1));

        /*subscriber_darknet = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
            "darknet_ros/bounding_boxes", 1,
            std::bind(&main_strategy::callbackBoundingBox, this, std::placeholders::_1));
        
        subscriber_object_count = this->create_subscription<darknet_ros_msgs::msg::ObjectCount>(
            "darknet_ros/found_object", 1,
            std::bind(&main_strategy::callbackFoundObject, this, std::placeholders::_1));*/
	peluit_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    	    "/robot_2/peluit_hz", 1,
    	    std::bind(&main_strategy::readPeluit, this, std::placeholders::_1));
    
        TargetPoseSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&main_strategy::callbackTargetPose, this, std::placeholders::_1));

        object_distance = this->create_subscription<std_msgs::msg::Float32>(
            "object_distance", 10,
            std::bind(&main_strategy::callbackObjectDistance, this, std::placeholders::_1));
            
        subscriber_detections = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/robot_2/detections", 1,
            std::bind(&main_strategy::callbackdetections, this, std::placeholders::_1));

        ball_distance = this->create_subscription<std_msgs::msg::Float32>(
            "/robot_2/ball_distance", 1,
            std::bind(&main_strategy::callbackBallDistance, this, std::placeholders::_1));

        path_finding_subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "path_finding", 10,
            std::bind(&main_strategy::callbackPathFinding, this, std::placeholders::_1));

        keyboard_teleop = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&main_strategy::callbackTeleop, this, std::placeholders::_1));

        ball_pose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "ball_pose", 1,
            std::bind(&main_strategy::callbackBallPose, this, std::placeholders::_1));

        camera_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "camera_odom", 1,
            std::bind(&main_strategy::callbackCameraOdom, this, std::placeholders::_1));
            
        if (robotNumber != 1)
        {
            robot1Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_1/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData1, this, std::placeholders::_1));
        }

        if (robotNumber != 2)
        {
            robot2Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_2/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData2, this, std::placeholders::_1));
        }

        if (robotNumber != 3)
        {
            robot3Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_3/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData3, this, std::placeholders::_1));
        }

        if (robotNumber != 4)
        {
            robot4Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_4/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData4, this, std::placeholders::_1));
        }

        if (robotNumber != 5)
        {
            robot5Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_5/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData5, this, std::placeholders::_1));
        }
        
        if (robotNumber != 6)
        {
            robot6Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_6/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData6, this, std::placeholders::_1));
        }
        
        if (robotNumber != 7)
        {
            robot7Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_7/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData7, this, std::placeholders::_1));
        }

        robotCoordination_ = this->create_publisher<bfc_msgs::msg::Coordination>(
            "coordination", 1);
        cmd_head_ = this->create_publisher<bfc_msgs::msg::HeadMovement>(
            "head", 1);
        cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "walk", 1);
        cmd_mot_ = this->create_publisher<std_msgs::msg::String>(
            "motion", 1);
        Odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "pose", 1);
        Update_coor_ = this->create_publisher<std_msgs::msg::Bool>(
            "update", 1);
        request_pub = this->create_publisher<std_msgs::msg::String>(
            "request_path", 1);
        ball_status_pub = this->create_publisher<std_msgs::msg::Bool>(
            "ball_status", 1);
        rrtPathPub_ = this->create_publisher<nav_msgs::msg::Path>(
            "rrt_path", 1);
        rrtObstaclePub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "rrt_obstacles", 1);

        timer_ = this->create_wall_timer(40ms, std::bind(&main_strategy::timer_callback, this));

        factory.registerSimpleCondition("KillRun", std::bind(&main_strategy::KillRun, this));
        factory.registerSimpleCondition("isReady", std::bind(&main_strategy::isReady, this));
        factory.registerSimpleCondition("WalkTowardsBall", std::bind(&main_strategy::WalkTowardsBall, this));
        factory.registerSimpleCondition("isAttacker", std::bind(&main_strategy::isAttacker, this));
        factory.registerSimpleCondition("isRobotOut", std::bind(&main_strategy::isRobotOut, this));
        factory.registerSimpleCondition("KickTowardsGoal", std::bind(&main_strategy::KickTowardsGoal, this));
        factory.registerSimpleCondition("GameControllerInit", std::bind(&main_strategy::GameControllerInit, this));
        factory.registerSimpleCondition("GameControllerReady", std::bind(&main_strategy::GameControllerReady, this));
        factory.registerSimpleCondition("GameControllerSet", std::bind(&main_strategy::GameControllerSet, this));
        factory.registerSimpleCondition("GameControllerPlay", std::bind(&main_strategy::GameControllerPlay, this));
        factory.registerSimpleCondition("GameControllerFinish", std::bind(&main_strategy::GameControllerFinish, this));
        factory.registerSimpleCondition("isDefenderleft", std::bind(&main_strategy::isDefenderleft, this));
        factory.registerSimpleCondition("isDefenderright", std::bind(&main_strategy::isDefenderright, this));
        factory.registerSimpleCondition("Executor", std::bind(&main_strategy::Executor, this));
        factory.registerSimpleCondition("RelaxLocked", std::bind(&main_strategy::RelaxLocked, this));
        factory.registerSimpleCondition("StatePickup", std::bind(&main_strategy::StatePickup, this));
        factory.registerSimpleCondition("StateRelease", std::bind(&main_strategy::StateRelease, this));
        factory.registerSimpleCondition("BallFoundTeam", std::bind(&main_strategy::BallFoundTeam, this));
        factory.registerSimpleCondition("DoneKick", std::bind(&main_strategy::DoneKick, this));
        factory.registerSimpleCondition("StateKickOff", std::bind(&main_strategy::StateKickOff, this));
        factory.registerSimpleAction("Followpen", std::bind(&main_strategy::Followpen, this));
        factory.registerSimpleCondition("Interrupt", std::bind(&main_strategy::Interrupt, this));
        factory.registerSimpleAction("Forward", std::bind(&main_strategy::Forward, this));
        factory.registerSimpleAction("Backward", std::bind(&main_strategy::Backward, this));
        factory.registerSimpleAction("Sideway", std::bind(&main_strategy::Sideway, this));
        factory.registerSimpleAction("testGrid", std::bind(&main_strategy::testGrid, this));
        factory.registerSimpleAction("GetData", std::bind(&main_strategy::GetData, this));
        factory.registerSimpleAction("SetHeadPos", std::bind(&main_strategy::SetHeadPos, this));
        factory.registerSimpleAction("BallFound", std::bind(&main_strategy::BallFound, this));
        factory.registerSimpleAction("BodyTrack", std::bind(&main_strategy::BodyTrack, this));
        factory.registerSimpleAction("BallApproach", std::bind(&main_strategy::BallApproach, this));
        factory.registerSimpleAction("SearchingBall", std::bind(&main_strategy::SearchingBall, this));
        factory.registerSimpleAction("RotateToGoal", std::bind(&main_strategy::RotateToGoal, this));
        factory.registerSimpleAction("Kick", std::bind(&main_strategy::Kick, this));
        factory.registerSimpleAction("InitialPosition", std::bind(&main_strategy::InitialPosition, this));
        factory.registerSimpleAction("RobotPositioning", std::bind(&main_strategy::RobotPositioning, this));
        factory.registerSimpleAction("ResetVar", std::bind(&main_strategy::ResetVar, this));
        factory.registerSimpleAction("BallTracking", std::bind(&main_strategy::BallTracking, this));
        factory.registerSimpleAction("Relax", std::bind(&main_strategy::Relax, this));
        factory.registerSimpleAction("Defend", std::bind(&main_strategy::Defend, this));
        factory.registerSimpleAction("Communication", std::bind(&main_strategy::Communication, this));
        factory.registerSimpleAction("FirstKick", std::bind(&main_strategy::FirstKick, this));
        factory.registerSimpleAction("SearchAfterKick", std::bind(&main_strategy::SearchAfterKick, this));
        factory.registerSimpleAction("WalkSearchBall", std::bind(&main_strategy::WalkSearchBall, this));
        factory.registerSimpleAction("RobotPassing", std::bind(&main_strategy::RobotPassing, this));
        factory.registerSimpleAction("RobotAction", std::bind(&main_strategy::RobotAction, this));
        factory.registerSimpleAction("RobotAction2", std::bind(&main_strategy::RobotAction2, this));
        factory.registerSimpleAction("HighKick", std::bind(&main_strategy::HighKick, this));
        factory.registerSimpleAction("Corner", std::bind(&main_strategy::Corner, this));
        factory.registerSimpleAction("Shoot", std::bind(&main_strategy::Shoot, this));
        factory.registerSimpleAction("ShootImu", std::bind(&main_strategy::ShootImu, this));
        factory.registerSimpleAction("ShootImu2", std::bind(&main_strategy::ShootImu2, this));
        //factory.registerSimpleAction("Obstacle", std::bind(&main_strategy::Obstacle, this));
        factory.registerSimpleAction("OdomUpdate", std::bind(&main_strategy::OdomUpdate, this));
        factory.registerSimpleAction("Sprint", std::bind(&main_strategy::Sprint, this));
        factory.registerSimpleAction("Marathon", std::bind(&main_strategy::Marathon, this));
        factory.registerSimpleAction("Obstacle_run", std::bind(&main_strategy::Obstacle_run, this));
        factory.registerSimpleAction("MoveGrid2", std::bind(&main_strategy::MoveGrid2, this));
        factory.registerSimpleCondition("isSet", std::bind(&main_strategy::isSet, this));
        factory.registerSimpleCondition("isPlay", std::bind(&main_strategy::isPlay, this));
        factory.registerSimpleCondition("isPickup", std::bind(&main_strategy::isPickup, this));
        factory.registerSimpleCondition("unPenalty", std::bind(&main_strategy::unPenalty, this));
        factory.registerSimpleCondition("resetTilt", std::bind(&main_strategy::resetTilt, this));
        factory.registerSimpleCondition("Mode1", std::bind(&main_strategy::Mode1, this));
        factory.registerSimpleCondition("Mode2", std::bind(&main_strategy::Mode2, this));
        factory.registerSimpleCondition("Mode3", std::bind(&main_strategy::Mode3, this));
        factory.registerSimpleCondition("isGoal", std::bind(&main_strategy::isGoal, this));
        factory.registerSimpleCondition("noSet", std::bind(&main_strategy::noSet, this));
        factory.registerSimpleCondition("noReady", std::bind(&main_strategy::noReady, this));
        factory.registerSimpleCondition("isGoalkeeper", std::bind(&main_strategy::isGoalkeeper, this));
        factory.registerSimpleAction("Goalkeeper", std::bind(&main_strategy::Goalkeeper, this));
        factory.registerSimpleAction("GoToPosition", std::bind(&main_strategy::GoToPosition, this));
        factory.registerSimpleAction("GoToPosition1", std::bind(&main_strategy::GoToPosition1, this));
        factory.registerSimpleAction("GoToPosition2", std::bind(&main_strategy::GoToPosition2, this));
        factory.registerSimpleAction("gridPosition", std::bind(&main_strategy::gridPosition, this));
        factory.registerSimpleAction("Goal", std::bind(&main_strategy::Goal, this));
        factory.registerSimpleAction("Search", std::bind(&main_strategy::Search, this));
        factory.registerSimpleAction("release", std::bind(&main_strategy::release, this));
        
        factory.registerSimpleCondition("StateDirectFreeKick",   std::bind(&main_strategy::StateDirectFreeKick, this));
        factory.registerSimpleCondition("StateIndirectFreeKick", std::bind(&main_strategy::StateIndirectFreeKick, this));
        factory.registerSimpleCondition("StatePenaltyKick",      std::bind(&main_strategy::StatePenaltyKick, this));
        factory.registerSimpleCondition("StateThrowIn",          std::bind(&main_strategy::StateThrowIn, this));
        factory.registerSimpleCondition("StateGoalKick",         std::bind(&main_strategy::StateGoalKick, this));
        factory.registerSimpleCondition("StateCornerKick",       std::bind(&main_strategy::StateCornerKick, this));
       
        factory.registerSimpleCondition("IsKickingTeam",         std::bind(&main_strategy::IsKickingTeam, this));
        factory.registerSimpleCondition("StateStopped",          std::bind(&main_strategy::StateStopped, this));
        
        tree = factory.createTreeFromFile(tree_path);
        pubZ = new BT::PublisherZMQ(tree);
        setupBtStatusSubscribers();
    }

    void run()
    {
        executor_.add_node(this->get_node_base_interface());
        executor_.spin();
    }

private:
    std::string current_bt_status = "";
    std::vector<TreeNode::StatusChangeSubscriber> bt_status_subscribers_;
    std::vector<std::string> bt_tick_condition_events_;
    std::vector<std::string> bt_tick_action_events_;

    static void pushUnique(std::vector<std::string>& dest, const std::string& value)
    {
        for (const auto& item : dest)
        {
            if (item == value)
            {
                return;
            }
        }
        dest.push_back(value);
    }

    void setupBtStatusSubscribers()
    {
        bt_status_subscribers_.clear();
        for (const auto& node_ptr : tree.nodes)
        {
            if (!node_ptr)
            {
                continue;
            }

            auto sub = node_ptr->subscribeToStatusChange(
                [this](TimePoint /*timestamp*/, const TreeNode& node,
                       NodeStatus /*prev_status*/, NodeStatus status)
                {
                    if (status == NodeStatus::FAILURE || status == NodeStatus::IDLE)
                    {
                        return;
                    }

                    const auto node_type = node.type();
                    const std::string name = node.name();
                    if (node_type == NodeType::CONDITION)
                    {
                        if (name == "KillRun")
                        {
                            return;
                        }
                        pushUnique(bt_tick_condition_events_, name);
                    }
                    else if (node_type == NodeType::ACTION)
                    {
                        if (name == "Communication")
                        {
                            return;
                        }
                        pushUnique(bt_tick_action_events_, name);
                    }
                });

            bt_status_subscribers_.push_back(sub);
        }
    }

    std::string summarizeFromTickEvents(bool is_playing) const
    {
        std::string key_condition = "";
        std::string key_action = "";

        for (auto it = bt_tick_condition_events_.rbegin(); it != bt_tick_condition_events_.rend(); ++it)
        {
            if (is_playing && *it == "GameControllerPlay")
            {
                continue;
            }
            key_condition = "Condition:" + *it;
            break;
        }

        if (!bt_tick_action_events_.empty())
        {
            key_action = "Action:" + bt_tick_action_events_.back();
        }

        if (key_condition.empty() && key_action.empty())
        {
            return "";
        }
        if (key_condition.empty())
        {
            return key_action;
        }
        if (key_action.empty())
        {
            return key_condition;
        }
        return key_condition + "\n" + key_action;
    }

    std::string summarizeBehaviorTreeStatus(bool is_playing) const
    {
        std::string key_condition = "";
        std::string key_action = "";

        for (const auto& node_ptr : tree.nodes)
        {
            if (!node_ptr)
            {
                continue;
            }

            const auto node_type = node_ptr->type();
            if (node_type != NodeType::ACTION && node_type != NodeType::CONDITION)
            {
                continue;
            }

            const auto node_status = node_ptr->status();
            if (node_status == NodeStatus::FAILURE || node_status == NodeStatus::IDLE)
            {
                continue;
            }

            const std::string name = node_ptr->name();
            if (node_type == NodeType::CONDITION)
            {
                if (is_playing && name == "GameControllerPlay")
                {
                    continue;
                }
                if (name == "KillRun")
                {
                    continue;
                }
                key_condition = "Condition:" + name;
            }
            else
            {
                if (name == "Communication")
                {
                    continue;
                }
                key_action = "Action:" + name;
            }
        }

        if (key_condition.empty() && key_action.empty())
        {
            return "";
        }
        if (key_condition.empty())
        {
            return key_action;
        }
        if (key_action.empty())
        {
            return key_condition;
        }
        return key_condition + "\n" + key_action;
    }
    bool doneScan = false;
    void scan_landmark()
    {
        if (cnt_sbr > 2)
        {
            doneScan = true;
        } else 
        {
            searchBallRectang(-1.6, -1.6, -0.8, 1.6);
            if (object_count > 1)
            {    
                auto msg_update = std_msgs::msg::Bool();
                msg_update.data = true;
                Update_coor_->publish(msg_update);
                // saveToCSV(Grid, robotPos_X, robotPos_Y, msg_yaw, headPan, headTilt, Left_X_Cross_X, Left_X_Cross_Y, Right_X_Cross_X, Right_X_Cross_Y, Left_T_Cross_X, Left_T_Cross_Y, Right_T_Cross_X, Right_T_Cross_Y, Left_Corner_X, Left_Corner_Y, Right_Corner_X, Right_Corner_Y, Left_L_Cross_X, Left_L_Cross_Y, Right_L_Cross_X, Right_L_Cross_Y, Pinalty_X, Pinalty_Y,Left_T_Corner_X,Left_T_Corner_Y, Right_T_Corner_X,Right_T_Corner_Y);
            }    
        }
    }
void callbackdetections(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    line_detected 	= false;
    wall_detected 	= false;
    forward_detected	= false;
    left_detected 	= false;
    right_detected 	= false;
    robot_detected 	= false;
    hole_detected 	= false;
    gate_detected 	= false;
    
    for (const auto& detection : msg->detections)
    { 
    
        for (const auto& result : detection.results)
        {
            // Konversi class_id dari string ke integer
            int id = std::stoi(result.hypothesis.class_id);

            // Akses posisi center dari bbox
            float center_x = detection.bbox.center.position.x;
            float center_y = detection.bbox.center.position.y;
            float size_x = detection.bbox.size_x;
            float size_y = detection.bbox.size_y;

            switch (id)
            {
                case 0:
                    //forward_x = center_x;
                    //forward_y = center_y;
                    //forward_detected = true;
                    
                    Ball_X = center_x;
                    Ball_Y = center_y;
                    Ball_detected = true;
                    
                    //robot_w = size_x;
                    //robot_h = size_y;
                    
                    //robot_x = center_x;
                    //robot_y = center_y;
                    //robot_detected = true;
                    
                    break;
                case 1:
                    left_x = center_x;
                    left_y = center_y;
                    left_detected = true;
                    
                    //robot_w = size_x;
                    //robot_h = size_y;
                    
                    //robot_x = center_x;
                    //robot_y = center_y;
                    //robot_detected = true;
                    break;
                case 2:
                    right_x = center_x; 
                    right_y = center_y;
                    right_detected = true;
                    break;
                case 3:
                    line_center_x = center_x;
                    line_center_y = center_y;
                    line_detected = true;
                    break;
                case 4:
                    wall_x = center_x;
                    wall_y = center_y;
                    wall_w = size_x;
                    wall_h = size_y;
                    wall_detected = true;
                    break;
                case 5:
                    hole_x = center_x;
                    hole_y = center_y;
                    hole_detected = true;
                    break;
                case 6:
                    gate_x = center_x;
                    gate_y = center_y;
                    gate_detected = true;
                    break;
                case 10:
                    robot_w = size_x;
                    robot_h = size_y;
                    
                    robot_x = center_x;
                    robot_y = center_y;
                    robot_detected = true;
                    break;    
                // Tambahkan case lainnya sesuai kebutuhan
                default:
                    // Tindakan untuk class_id lainnya
                    break;
            }
        }
    }
    
    updateLinePosition(line_center_x, line_center_y, line_detected);
    updateWallPosition(wall_x, wall_y, wall_w, wall_h, wall_detected);
    updateSignForward(forward_x, forward_y, forward_detected);
    updateSignLeft(left_x, left_y, left_detected);
    updateSignRight(right_x, right_y, right_detected);
    updateBall(Ball_X, Ball_Y, Ball_detected);
    updateRobot(robot_x, robot_y, robot_w, robot_h, robot_detected);
    updateHole(hole_x, hole_y, hole_detected);
    updateGate(gate_x, gate_y, gate_detected);
}

double line_center_x = 0.0;
double line_center_y = 0.0;
bool line_detected = false;

double wall_x = 0.0;
double wall_y = 0.0;
double wall_w = 0.0;
double wall_h = 0.0;
bool wall_detected = false;

double forward_x = 0.0;
double forward_y = 0.0;
bool forward_detected = false;

double left_x = 0.0;
double left_y = 0.0;
bool left_detected = false;

double right_x = 0.0;
double right_y = 0.0;
bool right_detected = false;

//double Ball_X = 0.0;
//double Ball_Y = 0.0;
bool Ball_detected = false;

// Coordination / defend-executor lock state
bool executor = false;
// executor status of teammates (updated from coordination messages)
bool robot1_executor = false;
bool robot2_executor = false;
bool robot3_executor = false;
bool robot4_executor = false;
bool robot5_executor = false;
bool robot6_executor = false;
bool robot7_executor = false;
double robot_x = 0.0;
double robot_y = 0.0;
double robot_w = 0.0;
double robot_h = 0.0;
bool robot_detected = false;

double hole_x = 0.0;
double hole_y = 0.0;
bool hole_detected = false;

double gate_x = 0.0;
double gate_y = 0.0;
bool gate_detected = false;

double last_error = 0.0;
double integral = 0.0;
rclcpp::Time last_time;

    void updateLinePosition(double x_center, double y_center, bool detected_line){
    line_center_x = x_center;
    line_center_y = y_center;
    line_detected = detected_line;
    }
    
    void updateWallPosition(double x_wall, double y_wall, float w, float h, bool detected_wall){
    wall_x = x_wall;
    wall_y = y_wall;
    wall_w = w;
    wall_h = h;
    wall_detected = detected_wall;
    }
    
    void updateSignForward(double x_forward, double y_forward, bool detected_forward){
    forward_x = x_forward;
    forward_y = y_forward;
    forward_detected = detected_forward;
    }
    
    void updateSignLeft(double x_left, double y_left, bool detected_left){
    left_x = x_left;
    left_y = y_left;
    left_detected = detected_left;
    }
    
    void updateSignRight(double x_right, double y_right, bool detected_right){
    right_x = x_right;
    right_y = y_right;
    right_detected = detected_right;
    }
    
    void updateBall(double x_ball, double y_ball, bool detected_ball){
    Ball_X = x_ball;
    Ball_Y = y_ball;
    Ball_detected = detected_ball;
    }
    
    void updateRobot(double x_robot, double y_robot, float w, float h, bool detected_robot){
    robot_x = x_robot;
    robot_y = y_robot;
    robot_h = h;
    robot_w = w;
    robot_detected = detected_robot;
    }
    
    void updateHole(double x_hole, double y_hole, bool detected_hole){
    hole_x = x_hole;
    hole_y = y_hole;
    hole_detected = detected_hole;
    }
    
    void updateGate(double x_gate, double y_gate, bool detected_gate){
    gate_x = x_gate;
    gate_y = y_gate;
    gate_detected = detected_gate;
    }

    NodeStatus Marathon()
    {
    const double frame_center = 320.0;  // posisi tengah frame
    const double base_speed = 0.025;    // kecepatan maju dasar
    const double turn_speed = 0.1;    // besar koreksi arah saat belok
    const double deadzone = 30.0;       // toleransi agar tidak goyang
    const double speed_lateral = 0.03;
    static std::string last_line_side = "CENTER";
    
    static int line_lost_counter = 0;
    const int max_lost_frames = 100;
    
    double error_x = line_center_x - frame_center;
    double vel_x = base_speed;
    double vel_y = 0.0;
    double vel_a = 0.0; // rotasi
    double target_belok = 90.0;
    
    enum RobotState {
    STATE_LINEFOLLOW,
    STATE_SIGN_LEFT,
    STATE_SIGN_RIGHT,
    STATE_SEARCH,
    STATE_GO_STRAIGHT
    };
    
    static RobotState current_state = STATE_LINEFOLLOW;
    static bool sign_locked = false;
    double target_yaw = 0.0;
    static int forward_counter = 0;
    
    // Posisi kepala konstan
    headMove(0.0, -0.3);  
      
    if (!sign_locked) {
    
    	if (left_detected) {
        target_yaw = msg_yaw + 90.0;   // belok kiri 90
        sign_locked = true;
        current_state = STATE_SIGN_LEFT;
        
    	}
    	
    	else if (right_detected) {
        target_yaw = msg_yaw - 90.0;   // belok kanan 90
        sign_locked = true;
        current_state = STATE_SIGN_RIGHT;
    	}
    	
    	else if (forward_detected) {
    	Walk(base_speed, 0, 0);
        current_state = STATE_GO_STRAIGHT;
        sign_locked = true;
    	}
    }
    
    if (!sign_locked && current_state == STATE_LINEFOLLOW) {
        if (!line_detected) {
            line_lost_counter++;
	    if (line_lost_counter > max_lost_frames){
	        current_state = STATE_SEARCH;
	    }
	}
	else {
            line_lost_counter = 0;
	}
    }
    
    switch(current_state)
    {

    // SIGN BEL0K KIRI
    case STATE_SIGN_LEFT:
        printf("SIGN LEFT\n");

        if (!koreksiBelok(-90)) {
            Walk(0, 0, turn_speed);
            
        } else {
            current_state = STATE_GO_STRAIGHT;
        }
    break;

    // SIGN BEL0K KANAN
    case STATE_SIGN_RIGHT:
        printf("SIGN RIGHT\n");

        if (!koreksiBelok(90)) {
            Walk(0, 0, -turn_speed);
           
        } else {
            current_state = STATE_GO_STRAIGHT;
        }
    break;

    // LINE FOLLOWING
    case STATE_LINEFOLLOW: {
        double frame_center = 320.0;
        double turn_speed   = 0.09;
        double deadzone     = 30.0;
        double error_x      = line_center_x - frame_center;

        double vel_x = 0.025;
        double vel_y = 0;
        double vel_a = 0;
        
        if (line_detected){
        
            if (fabs(error_x) < deadzone) {
               vel_a = 0;
            }
               
            else if (error_x > deadzone) {
               vel_a = -turn_speed;
               vel_x = 0.01;
               vel_y = -0.015;
               last_line_side = "RIGHT";
            }
                    
            else if (error_x < -deadzone) {
               vel_a = turn_speed;
               vel_x = 0.01;
               vel_y = 0.015;
               last_line_side = "LEFT";
            }
            
        Walk(vel_x, vel_y, vel_a);
        printf("LINE FOLLOW | error=%.2f\n", error_x);
        
        }
        else{
               motion("0");
               last_line_side = "CENTER";
        }
        }
   
    break;

    //garis hilang
    case STATE_SEARCH:
        printf("SEARCHING LINE...\n");
        
        if (line_detected) {
            line_lost_counter = 0;
            current_state = STATE_LINEFOLLOW;
            break;
        }

        if (last_line_side == "LEFT"){
            Walk(0, 0.03, 0);
            }
        else if (last_line_side == "RIGHT"){
            Walk(0, -0.03, 0);
            }
        else{
            motion("0");  // belum tahu arah
            SearchBall(3);
            }
    break;

    case STATE_GO_STRAIGHT:
        printf("GO STRAIGHT UNTIL LINE FOUND\n");
        Walk(base_speed, 0, 0);
        forward_counter++;
        
        if (line_detected || forward_counter > 300) {
            sign_locked = false;
            forward_counter = 0;
            printf("LINE FOUND! Back to LINEFOLLOW\n");
            current_state = STATE_LINEFOLLOW;
        } 
    break;
    
    }
    printf("Marathon | line_x=%.2f err=%.2f velA=%.3f\n", line_center_x, error_x, vel_a);
    fflush(stdout);
    return NodeStatus::FAILURE;
    }
    
bool koreksiBelok(double target_yaw)
{
    double tolerance = 5.0;  // bebas atur
    double selisih   = angleDiff(msg_yaw, target_yaw);

    if (selisih <= tolerance) {
        return true;   // target sudut tercapai
    } else {
        return false;  // masih dalam proses belok
    }
}

double angleDiff(double a, double b)
{
    double diff = fmod(fabs(a - b), 360.0);
    return (diff > 180.0) ? 360.0 - diff : diff;
}
    /*
    int geser = 0;
    bool right = false;
    bool left = false;
    NodeStatus Obstacle_run()
    {
    switch (stateCondition)
        {
            case 0:
                Walk(0.025, 0.0, 0.0);
                if (robot_detected)
                {
                    if (robot_w >= 170 && robot_h >= 270){
                        printf("Robot detected\n");
                        Walk(0.0, 0.0, 0.0);
                        stateCondition = 1;
                    }
                }
                
                else{
                }
            
            break;

            case 1:
                Walk(0.015, 0.08, 0.0);
                if (!robot_detected){
                    Walk (0.02, 0.0, 0.0);
                    }
             break;
             
         default:
             break;
            }

    return NodeStatus::FAILURE;
    }
    */
    
    NodeStatus Obstacle_run()
    {
        headMove(0.0, -0.78);
        
        static bool kanan = false;
        float deadzone_left  = 640 * 0.25f;
	float deadzone_right = 640 * 0.75f;
	
	float deadzone_hole_left  = 320 - 130;
	float deadzone_hole_right = 320 + 130;
	float deadzone_hole_up = 250;

	float frame_w = 640.0f;
	
	float left_zone  = frame_w * 0.33f;
	float right_zone = frame_w * 0.66f;
	
	enum AvoidDir {
	    AVOID_NONE,
	    AVOID_LEFT,
	    AVOID_RIGHT
	};
	static AvoidDir avoid_dir = AVOID_NONE;
	
	enum GateState {
	    GATE_INIT,
	    GATE_BACKWARD,
	    GATE_DONE
	};
	static GateState gate_state = GATE_INIT;
	
	static bool avoid_locked = false;
		
    	switch (stateCondition)
    	{
    	    case 0:
    	        Walk (0.013, 0.0, 0.0);
    	        
    	        if (wall_w > 200 && wall_h > 100 && !gate_detected && wall_y > 190)
    	        {
	    	            printf("Wall or hole detected\n"); 
	    	            if (wall_x < frame_w / 0.5f && wall_x > 140){
				avoid_dir = AVOID_RIGHT;
				motion("0");
				//Walk(-0.013, 0.0, 0.0);   // mundur dikit
                		stateCondition = 1;       // masuk avoid
			    }
			    else if (wall_x > frame_w / 0.5f && wall_x < 500){
				avoid_dir = AVOID_LEFT;
				motion("0");
				//Walk(-0.013, 0.0, 0.0);   // mundur dikit
                		stateCondition = 1;       // masuk avoid
			     
	    	            }
	    	        
	    	        //Walk(-0.013, 0.0, 0.0);   // mundur dikit
                	//stateCondition = 1;       // masuk avoid
    	            
    	        }
            
    	        else if (gate_detected){
    	            stateCondition = 2;
    	        }
    	        
    	        else if (hole_detected){
    	            stateCondition = 3;
    	        }
    
    	            	        
    	    break;
    	        
    	    case 1:
    	        if (gate_detected)
    	        {
    	            avoid_dir = AVOID_NONE;
    	            avoid_locked = false;
    	            stateCondition = 2;
    	            break;
    	        }
    	        else{       	       
	        if (avoid_dir == AVOID_RIGHT)
	           {
	                if (wall_x < frame_w / 0.5f){
	           		Walk(-0.013, -0.016, 0.0);   // mundur + geser kanan
	           		
			   	if (wall_w < 195 && wall_h < 95 || wall_x < 140)
			   	{
			   	printf("Wall cleared\n");
				avoid_dir = AVOID_NONE;
				avoid_locked = false;  //
				stateCondition = 0;
				}
				
				if 
				(wall_x > frame_w / 0.5f && wall_y > 190){
				
				avoid_dir = AVOID_LEFT;
				}
		        }
		        
		   }
	        else if (avoid_dir == AVOID_LEFT)
		   {
		        if (wall_x > frame_w / 0.5f){
				Walk(-0.013, 0.016, 0.0); // mundur + geser kiri
				
				if (wall_w < 195 && wall_h < 95 || wall_x > 500)
				{
			   	printf("Wall cleared\n");
				avoid_dir = AVOID_NONE;
				avoid_locked = false;  //
				stateCondition = 0;
				}
				
				if 
				(wall_x < frame_w / 0.5f && wall_y > 190){
		        	
		        	avoid_dir = AVOID_RIGHT; //l
		        	}
		        }
		        
		   } 
		   
	         // Wall sudah aman
	        if (wall_w < 195 && wall_h < 95 || wall_x > 500 || wall_x < 140){
		    printf("Wall cleared\n");
		    avoid_dir = AVOID_NONE;
		    avoid_locked = false;  //
		    stateCondition = 0;
		}
             }
		
	    
    	    break;
    	    
            case 2:
            {
                 float frame_center = 320.0f;
    	         float gate_deadzone = 30.0f;   // ±30 pixll
    	         static bool gate_centered = false;
    	         static bool motion_done = false;
    	         
                 if (gate_detected){
                    motion("0");
                    if (gate_x < frame_center - gate_deadzone){     //gate masih di kanan 
                       Walk (-0.013, 0.013, 0.0);
                    }
                    else if (gate_x > frame_center + gate_deadzone){ //gate masih di kiri
                       Walk (-0.013, -0.013, 0.0);
                    }
                    else{
            	        printf("gate_detected\n");
                        motion("2");
                        motion("0"); 
                        return NodeStatus::SUCCESS;
                      
                   }   
                 }
                 else{
                     stateCondition = 0;
                 }  
                 
            }	
            break;
            
            case 3:
		if (hole_detected){
		    // Hole masih jauh di atas → maju
		    if (hole_y < deadzone_hole_up)
		    {
			Walk(0.013, 0.0, 0.0);
		    }
		    
		    
		    else
		    {
			// Hole di kiri frame
			if (hole_x < deadzone_hole_left)
			{
			    printf("hole_on_left -> AVOID RIGHT\n");
			    motion("0");
			    Walk(-0.013, -0.016, 0.0);
			    if (!hole_detected || hole_x > deadzone_hole_left){
	           	      stateCondition = 0;}
			}
			// Hole di kanan frame
			else if (hole_x > deadzone_hole_right)
			{
			    printf("hole_on_right -> AVOID LEFT\n");
			    motion("0");
			    Walk(-0.013, 0.016, 0.0);
			    if (!hole_detected || hole_x > deadzone_hole_right){
	           	      stateCondition = 0;}
			}
			// Hole di tengah
			else
			{
			    Walk(-0.007, 0.016, 0.0);  // default geser kiri
			}
		    }
		 }
		 else{
		 stateCondition = 0;
		 }
		
		break;
            
    	default:
             break;
        }
             
    return NodeStatus::FAILURE;
    }
    
    bool backwardForSeconds(float duration)
	{
	    static bool started = false;
	    static auto t0 = std::chrono::steady_clock::now();

	    if (!started)
	    {
		t0 = std::chrono::steady_clock::now();
		started = true;
	    }

	    float elapsed = std::chrono::duration<float>(
		std::chrono::steady_clock::now() - t0).count();

	    if (elapsed < duration)
	    {
		Walk(-0.01, 0.0, 0.0);
		return true;   // masih mundur
	    }
	    else
	    {
		motion("0");
		started = false;
		return false;  // selesai
	    }
	}
    
     NodeStatus KickTowardsGoal()
    {
        
        return NodeStatus::FAILURE;
    }
    
    bool avoidingRobot = false;
    int  avoidRobotCounter = 0;

    bool checkAndAvoidRobot()
    {
    const float ROBOT_W_THRESHOLD = 110.0f;  
    const float ROBOT_H_THRESHOLD = 170.0f;  
    const float FRAME_CENTER      = 320.0f;  
    const float CENTER_ZONE       = 100.0f;  
    const double AVOID_SPEED      = 0.02;   
    const int    AVOID_HOLD       = 20;      

    if (!robot_detected) {
        if (avoidRobotCounter > 0) avoidRobotCounter--;
        if (avoidRobotCounter == 0) avoidingRobot = false;
        return false;
    }

    // Robot terdeteksi dan cukup besar (dekat/menghalangi)
    bool robotBlocking = (robot_w >= ROBOT_W_THRESHOLD && robot_h >= ROBOT_H_THRESHOLD && enableAvoid);
    if (!robotBlocking && !avoidingRobot) return false;

    avoidingRobot = false;
    avoidRobotCounter = AVOID_HOLD;

    // Tentukan arah menghindar berdasarkan posisi robot di frame
    if (robot_x < FRAME_CENTER - CENTER_ZONE) {
        // Robot di kiri frame - geser ke kanan
        printf("[AVOID] Robot di kiri (x=%.0f) → geser KANAN\n", robot_x);
        Walk(0.008, -AVOID_SPEED, 0.0);
    } else if (robot_x > FRAME_CENTER + CENTER_ZONE) {
        // Robot di kanan frame - geser ke kiri
        printf("[AVOID] Robot di kanan (x=%.0f) → geser KIRI\n", robot_x);
        Walk(0.008, AVOID_SPEED, 0.0);
    } else {
        // Robot tepat di tengah - mundur sedikit + geser ke sisi yang lebih bebas
        // Pilih geser berdasarkan mana sisi lebih lebar frame yang kosong
        double avoidDir = (robot_x <= FRAME_CENTER) ? -AVOID_SPEED : AVOID_SPEED;
        printf("[AVOID] Robot di tengah (x=%.0f) → mundur + geser\n", robot_x);
        Walk(-0.010, avoidDir, 0.0);
    }
    return true;
    }
	
    bool action_afterKick = false;
    int KickEntry = 0;
    NodeStatus Kick()
    {
        if (KickEntry > 5)
        {
            if (ballLost(35))
            {
            	headMove(0.0, -1.4);
            }
            else 
            {
            	trackBall();
            	if (delayWaitBall > 20)
            	{
		        	if (robotDirection && headPan >= -0.2 && headPan <= 0.6) //+0.1
		        	{
		        		if (tendang)
		        		{
		        			sleep(2);
		        			motion("0");
		        		} else 
		        		{
		        			kick(tendangJauh);
		        		}
		        	}
		        	else 
		        	{
		        		if (msg_strategy == 1)
		        		{
		        			Imu(30, cSekarang);
		        		} else if (msg_strategy == 2)
		        		{
		        			Imu(-30, cSekarang);
		        		} else 
		        		{
		        			Imu(0, cSekarang);
		        		}
		        	}
		        } else 
		        {
		        	delayWaitBall++;
		        }
            }
        } else 
        {
            motion("0");
            delayWaitBall = 0;
            robotDirection = ballPos = tendang = false;
            KickEntry++;
        }
        return NodeStatus::FAILURE;
    }
    
    NodeStatus Shoot()
    {
        //Strattegy TC HighKick 2024
        switch (stateCondition)
        {
            case 0: // search ball
                tendang = false;
                ballPos = false;
                robotDirection = false;
                if (ballLost(35))
                {
                    motion("0");
                    tiltSearchBall(0.0);
                    //headMove(0.04, -1.4);
                    //panSearchBall(-1.6);
                } else 
                {
                    trackBall();
                    if (delayWaitBall > 20)
                    {  
                        stateCondition = 1;
                    } else
                    {
                        delayWaitBall++;
                    }
                }
                
            break;

            case 1: // follow Ball
            printf("case 1 cuy");
                motion("9");
                tendang = false;
                ballPos = false;
                robotDirection = false;
                delayWaitBall = 0;
                if (ballLost(35))
                {
                    resetCase0();
                    stateCondition = 0;
                } else
                {
                    trackBall();
                    if (headTilt >= cAktif)// && headPan >= -0.3 && headPan <= 0.6)
                    {   
		                if (msg_yaw < 3 && msg_yaw > -3 && headPan >= -0.3 && headPan <= 0.6) {
		                    stateCondition = 2;
		                } else {
		                    Imu(0, cSekarang);
		                }
                    } else {
                        followBall(0);
                    }
                }
                
            break;

            case 2: // BallPos -> Tendang
                
                if (ballLost(35))
                {
                    resetCase0();
                    stateCondition = 0;
                } else 
                {
                    trackBall();
                    if (tendang)
                    {
                        resetCase0();
                        stateCondition = 0;
                        passed = true;
                    } else 
                    {

                        //kickNoSudut(2);
                        if (passed) {
                            modeKick = 99;
                        } else {
                            modeKick = 7;
                        }
                        kick(modeKick);
                    }
                }

            break;

        default:
            break;
        }
        return NodeStatus::FAILURE;
    }
    
    NodeStatus Sprint()
    {
        //Strategy TC HighKick 2024
        switch (stateCondition)
        {
            case 0: // search ball
                tendang = false;
                ballPos = false;
                robotDirection = false;
                
              // if (msg_yaw < 10 && msg_yaw > -10)
              //  {
                    printf("lari kedepan");
                    Walk(lari, 0.0, 0.0);
                    
                    //motion("0");
                    //tiltSearchBall(0.0);
                    //headMove(0.04, -1.4);
                    //panSearchBall(-1.6);
                 
                if (msg_yaw > 10)
                    {
                    printf("belok kiri");
                    Walk(lari, 0.0, -0.02);
                    
                    }
                    
                else if (msg_yaw > -10)
                    {
                    printf("belok kanan");
                    Walk(lari, 0.0, 0.02);
                    
                    }
                //}  
                
                else 
                {
                    //trackBall();
                    if ( robotPos_Y < -200)
                    {  
                        stateCondition = 1;
                    } else
                    {
                        delayWaitBall++;
                    }
                }
                
            break;

            case 1: // follow Ball
            printf("case 1 cuy");
                motion("9");
                tendang = false;
                ballPos = false;
                robotDirection = false;
                delayWaitBall = 0;
                if (ballLost(35))
                {
                    resetCase0();
                    stateCondition = 0;
                } else
                {
                    trackBall();
                    if (headTilt >= cAktif)// && headPan >= -0.3 && headPan <= 0.6)
                    {   
		                if (msg_yaw < 3 && msg_yaw > -3 && headPan >= -0.3 && headPan <= 0.6) {
		                    stateCondition = 2;
		                } else {
		                    Imu(0, cSekarang);
		                }
                    } else {
                        followBall(0);
                    }
                }
                
            break;

            case 2: // BallPos -> Tendang
                
                if (ballLost(35))
                {
                    resetCase0();
                    stateCondition = 0;
                } else 
                {
                    trackBall();
                    if (tendang)
                    {
                        resetCase0();
                        stateCondition = 0;
                        passed = true;
                    } else 
                    {

                        //kickNoSudut(2);
                        if (passed) {
                            modeKick = 99;
                        } else {
                            modeKick = 7;
                        }
                        kick(modeKick);
                    }
                }

            break;

        default:
            break;
        }
        return NodeStatus::FAILURE;
    }
    
    NodeStatus Corner()
    {
    	switch (stateCondition)
    	{
    	   case 0:	//search ball
    	   	tendang = kanan = kiri = false;
    	   	if (ballLost(35))
                {
                    motion("0");
                    panSearchBall(-1.6);
                } else 
                {
                    trackBall();
                    if (delayWaitBall > 20)
                    {  
                        stateCondition = 1;
                    } else
                    {
                        delayWaitBall++;
                    }
                }
           break;
    	   
    	   case 1:	//config direction
    	   	if (ballLost(20))
                {
                    resetCase0();
                    stateCondition = 0;
                } else
                {
                    trackBall();
                    motion("0");
                    if (kanan || kiri) 
                    {
                    	stateCondition = 2;
                    } else 
                    {
		        if (headPan <= panTengah) 
		        {
		            kanan = true;
		            kiri = false;
		        } else 
		        {
		            kanan = false;
		            kiri = true;
		        }
                    }
                 }
                    
           break;
           
    	   case 2:	//shoot
    	   	if (ballLost(35))
                {
                    resetCase0();
                    stateCondition = 0;
                } else
                {
                    trackBall();
                    if (kanan)
                    {
                    	if (headPan < (pPanTendangKanan - 0.60))
                    	{
                    	    motion("0");
                    	} else
                    	{
                    	    motion("7");
                    	    kanan = false;
                    	    resetCase0();
                    	    stateCondition = 0;
                    	}
                    } else if (kiri)
                    {
                    	if (headPan > (pPanTendangKiri + 0.60))
                    	{
                    	    motion("0");
                    	} else
                    	{
                    	    motion("1");
                    	    kanan = false;
                    	    resetCase0();
                    	    stateCondition = 0;
                    	}
                    } else
                    {
                    	//sleep(1);
                    	motion("0");
                    }
                }
                
    	   break;
    	   
    	default:
           break;
        }
        return NodeStatus::FAILURE;
    }
    
    int getClosestGridIDBall(int targetX, int targetY) 
{
    int bestGrid = 0;
    double minDistance = 999999.0;

    for (int i = 0; i <= 53; i++) 
    {
        int gridX = convertGridX(i, 0); 
        int gridY = convertGridY(i, 0); 

        // Hitung jarak Euclidean (Pythagoras)
        double dX = targetX - gridX;
        double dY = targetY - gridY;
        double distance = sqrt((dX * dX) + (dY * dY));

        // Cari yang jaraknya paling dekat (minimum distance)
        if (distance < minDistance) 
        {
            minDistance = distance;
            bestGrid = i;
        }
    }
    return bestGrid;
}

    NodeStatus MoveGrid2()
    {
    if (WalkSearchBallEntry <= 5)
    {
        refreshMoveGrid();
        saveSudutImu();
        
        stateSearchBall = 0;
        RobotPositioningEntry = 0;
        BallApproachEntry = 0;
        sabar = 0;
        relaxEntry = 0;
        
        WalkSearchBallEntry++;
        return NodeStatus::FAILURE; // Return Failure saat preparing
    }

    // DEFEND POSITION: bantu kipper jaga gawang 
    int defendX = -250;
    int defendY;

    // Tentukan sisi defend berdasarkan posisi Y robot saat ini
    if (robotPos_Y < 0) {
        defendY = -100;  // jaga sisi kiri gawang
    } else {
        defendY = 100;   // jaga sisi kanan gawang
    }

    printf("jadi defend target=(%d, %d) pos=(%.0f, %.0f)\n", 
           defendX, defendY, robotPos_X, robotPos_Y);

        new_out_pos(defendX, defendY, true);

        // Sambil jalan, tetap cari/track bola
        if (ballLost(50)) {
            searchBallBreak();
        } else {
            trackBall();
            
        }
    

    // Selalu return FAILURE agar BT re-evaluate setiap tick
    // Kalau tiba-tiba robot lihat bola, BallFound() di atas akan SUCCESS
    // dan robot otomatis keluar dari branch ini
    return NodeStatus::FAILURE;
  }
  
  //Lama Kak kayla  
/*
   NodeStatus MoveGrid2()
   {
    if (WalkSearchBallEntry <= 5)
    {
        refreshMoveGrid();
        saveSudutImu();
        
        stateSearchBall = 0;
        RobotPositioningEntry = 0;
        BallApproachEntry = 0;
        sabar = 0;
        relaxEntry = 0;
        
        WalkSearchBallEntry++;
        return NodeStatus::FAILURE; // Return Failure saat preparing
    }

    // Kita cari siapa yang mengirim DBall == 232 (Kode Executor)
    int targetX = 0;
    int targetY = 0;
    bool foundExecutor = false;

    // Cek Robot 1 (Prioritas Utama)
    if (robot1DBall == 232) 
    {
        targetX = robot1XBall;
        targetY = robot1YBall;
        foundExecutor = true;
    }
    // Cek Robot 2 (Jika R1 bukan 232)
    else if (robot2DBall == 232) 
    {
        targetX = robot2XBall;
        targetY = robot2YBall;
        foundExecutor = true;
    }
    // Cek Robot 3 (Jika R1 & R2 bukan 232)
    else if (robot3DBall == 232) 
    {
        targetX = robot3XBall;
        targetY = robot3YBall;
        foundExecutor = true;
    }


    if (foundExecutor)
    {
        
        int targetGridID = getClosestGridIDBall(targetX, targetY);

        moveGrid(targetGridID, 0, 0); 

        if (doneMoved) 
        {
             return NodeStatus::SUCCESS; 
        }
        return NodeStatus::RUNNING; 
    }
    else
    {

        refreshMoveGrid(); // PENTING: Reset status moveGrid biar counternya nol lagi

        if (role == 0) // Jika Striker
        {
            walkTarget(200, -25); // Jalan cari bola
        } 
        else if (role == 1) // Jika Defender
        {
            if (robot4Status == 1) // Ada defender lain
            {
                walkTarget(0, -25); 
            } 
            else 
            {
                walkTarget(-200, -25);
            }
        }
        
        return NodeStatus::FAILURE; 
    }
  }

*/ 
//sampe sini
    
    int  goalKickPhase  = 0;
    bool goalKickKicked = false;
    int  goalKickWait   = 0;

    NodeStatus StateGoalKick()
    {
        // Jika setPlay sudah bukan 5, reset
        if (secondaryInfo[0] != 5) {
            goalKickPhase  = 0;
            goalKickKicked = false;
            goalKickWait   = 0;
            robotDirection = false;
            BallApproachEntry = 0;
            return NodeStatus::FAILURE;
        }

        // Phase 1: stopped=1 → wasit sedang taruh bola di sudut goal area, SEMUA DIAM
        if (Stopped == 1) {
            goalKickPhase  = 1;
            goalKickKicked = false;
            goalKickWait   = 0;
            motion("0");
            printf("[GoalKick] Phase1-STOPPED: wasit posisikan bola, diam\n");
            return NodeStatus::SUCCESS;
        }

        // stopped=0 → GC resume, mulai eksekusi
        if (goalKickPhase != 2 && goalKickPhase != 3) {
            goalKickPhase  = 2;
            goalKickWait   = 0;
            goalKickKicked = false;
            robotDirection = false;
            BallApproachEntry = 0;
            printf("[GoalKick] Phase2-EXECUTING: kicking=%d barelang=%d\n",
                   KickOff, barelang_color);
        }

        // Phase 3: sudah tendang, tunggu setPlay=0
        if (goalKickPhase == 3) {
            motion("0");
            printf("[GoalKick] Phase3-DONE: tunggu GC reset setPlay\n");
            return NodeStatus::SUCCESS;
        }

        // Delay kecil saat masuk phase 2
        if (goalKickWait < 5) {
            motion("0");
            goalKickWait++;
            return NodeStatus::SUCCESS;
        }

        // Tim kita yang tendang (attacking)
        if (KickOff == barelang_color) {
            printf("[GoalKick] Attacking: SecTime=%d\n", SecondaryTime);

            if (goalKickKicked) {
                goalKickPhase = 3;
                motion("0");
                return NodeStatus::SUCCESS;
            }

            if (ballLost(35)) {
                searchBallBreak();
            } else {
                trackBall();
                if (ballDistance > 40) {
                    followBall(0);
                } else {
                    if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) {
                        if (!robotDirection) {
                            // § 16.5: Ball must be kicked DIRECTLY out of penalty area
                            // Use ±10° for direct forward kick (not lateral ±30°)
                            if (msg_yaw > 0) sudutTendang = -10;
                            else             sudutTendang =  10;
                            Imu(sudutTendang, cSekarang);
                        } else {
                            kick(modeKick);
                            goalKickKicked = true;
                            goalKickPhase  = 3;
                            printf("[GoalKick] KICK executed!\n");
                        }
                    } else {
                        followBall(0);
                    }
                }
            }
        }
        // Tim lawan (defending) — harus jaga jarak di luar penalty area
        else {
            printf("[GoalKick] Defending: SecTime=%d\n", SecondaryTime);

            // § 16.3: Defending robots must remain outside penalty area (X < goalAreaMinX)
            if (robotPos_X > goalAreaMinX) {
                printf("[GoalKick] Defending: ILLEGAL POSITION - inside penalty area! X=%d > %d\n",
                       robotPos_X, goalAreaMinX);
                motion("0");  // Stop immediately to enforce compliance
            } else if (!ballLost(50)) {
                trackBall();
            } else {
                motion("0");
            }
        }

        return NodeStatus::SUCCESS;
    }


    NodeStatus HighKick()
    {
        //Strattegy TC HighKick 2024
        switch (stateCondition)
        {
            case 0: // search ball
                tendang = false;
                ballPos = false;
                robotDirection = false;
                if (ballLost(35))
                {
                    motion("0");
                    tiltSearchBall(0.0);
                } else 
                {
                    trackBall();
                    if (delayWaitBall > 20)
                    {  
                        stateCondition = 1;
                    } else
                    {
                        delayWaitBall++;
                    }
                }
                
            break;

            case 1: // follow Ball
            printf("case 1 cuy");
                motion("9");
                tendang = false;
                ballPos = false;
                robotDirection = false;
                delayWaitBall = 0;
                if (ballLost(35))
                {
                    resetCase0();
                    stateCondition = 0;
                } else
                {
                    trackBall();
                    if (headTilt >= cAktif)
                    {   
		    	if (msg_yaw < 5 && msg_yaw > -5 && headPan < (pPanTendangKiri + 0.1) && headPan > (pPanTendangKanan - 0.1)) {
		        	stateCondition = 2;
		        } else {
		        	Imu(0, cSekarang);
		        }
                    } else {
                        followBall(0);
                    }
                }
                
            break;

            case 2: // BallPos -> Tendang
                
                if (ballLost(35))
                {
                    resetCase0();
                    stateCondition = 0;
                } else 
                {
                    trackBall();
                    if (tendang)
                    {
                        resetCase0();
                        stateCondition = 0;
                        passed = true;
                    } else 
                    {
                        if (passed) {
                            modeKick = 99;	//high kick
                        } else {
                            modeKick = 7;	//meter
                        }
                        kick(modeKick);
                    }
                }

            break;

        default:
            break;
        }
        return NodeStatus::FAILURE;
    }
    
    bool passed = false;
    NodeStatus RobotPassing()
    {
        if (passed)
        {
            sleep(0.5);
            motion("0");
            return NodeStatus::SUCCESS;
        }
        else
        {
            motion("7");	//short right
            passed = true;
        }
        return NodeStatus::FAILURE;
    }
    
    NodeStatus isReady()
    {
        if (State == 1)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;   
    }

    NodeStatus isSet()
    {
        if (State == 2)
        {
            posRotateNew = false;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isPickup()
    {
        if ((Penalty1 == 5 && timNumber1 == barelang_color) || (Penalty2 == 5 && timNumber2 == barelang_color))
        {
            refreshMoveGrid();
            motion("0");
            Pickup = true;
        }
         /*else
        {
            Pickup = false;
        }*/
        if (Pickup)
        {
            return NodeStatus::SUCCESS;
        }
        
        if (doneMoved)
        {
            Pickup = false;
            return NodeStatus::FAILURE;
        }      
        return NodeStatus::FAILURE;
    }

    NodeStatus unPenalty()
    {
        if ((Penalty1 != 5 && timNumber1 == barelang_color) || (Penalty2 != 5 && timNumber2 == barelang_color))
        {
            motion("0");
            Release = true;
            //Pickup = false;
            return NodeStatus::SUCCESS;
        }
        
        Release = false;
        return NodeStatus::FAILURE;
    }

    bool settilt = false; 
    NodeStatus resetTilt()
    {
        if(settilt)
        {
            if(delay > 3)
                {   
                    cekWaktu(3);
                    if(timer){
                        settilt = false;
                        tunggu = 0;
                        stateCondition = 0;
                        return NodeStatus::SUCCESS;
                    }else
                    {
                        headMove(0.0,-1.80);
                    }
                }else
                {
                    setWaktu();
                    delay++;
                } 
        }else{
        return NodeStatus::SUCCESS;
        }
    return NodeStatus::FAILURE;
    }

    NodeStatus isPlay()
    {
        if (State == 3)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

     NodeStatus Mode1()
    {
        if(msg_strategy == 1)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus Mode2()
    {
        if(msg_strategy == 2)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus Mode3()
    {
        if(msg_strategy == 3)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus Goal()
    {
        if (posRotateNew)
        {
            motion("0");
        }
        else
        {
            doneMoved = false;
            headMove(0.0, -1.60);
            rotateBodyImuNew(0);
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus GoToPosition()
    {
        /*if ((State == 1 && Remaining != 600)) // kondisi ready setelah goal (part1)
        {
           return NodeStatus::SUCCESS;
        }*/
        //doneMoved = depanGawang; 
        
                    if (msg_yaw > 0) // dari sisi kiri
            {
                initialPos_X = -350;
                initialPos_Y = -305;
            } else if (msg_yaw < 0) //dari sisi kanan
            {
                initialPos_X = -350;
                initialPos_Y = 305;
            }
            
        if ((State == 1 && Remaining == 600 || ((State == 3 && Remaining != 600) && doneMoved != true))) // ketika awal masuk || release
        {
            if (msg_yaw > 0) // dari sisi kiri
            {
                initialPos_X = -350;
                initialPos_Y = -305;
            } else if (msg_yaw < 0) //dari sisi kanan
            {
                initialPos_X = -350;
                initialPos_Y = 305;
            }

            if (doneMoved)
            {
                motion("0");
                reset_velocity();
                stateCondition = 8;
                Pickup = false;
                return NodeStatus::SUCCESS;
            } else 
            {
                threeSearchBall();
                //new_out_grid(3, 0, 50, true);
                new_out_pos(-400, 0, true);
                Grid = 3;
            }
            stateCondition = 0;
            return NodeStatus::FAILURE;
        }
        
        if ((State == 1 && Remaining != 600) || (State == 1 && Remaining != 300)) // ketika terjadi goal
        {
            if (doneMoved)
            {
                printf("===================udah di gawang===========================");
                motion("0");
                stateCondition = 0;
                return NodeStatus::SUCCESS;
            }
            else
            {
                loadKoordinatRobot();
                threeSearchBall();
                //new_out_grid(3, 0, 50, true);
                new_out_pos(-400, 0, true);
                printf("===================kembali ke gawang===========================");
                rotateBodyImuNew(0);
            }
        }
        
        /*if ((State == 1 && Remaining != 600) || (State == 1 && Remaining != 300)) // ketika terjadi goal
        {
            if (posRotateNew)
            {
                printf("===================udah di gawang===========================");
                motion("0");
                stateCondition = 0;
                return NodeStatus::SUCCESS;
            }
            else
            {
                threeSearchBall();
                //new_out_grid(3, 0, 50, true);
                printf("===================kembali ke gawang===========================");
                rotateBodyImuNew(0);
            }
        }*/
        return NodeStatus::FAILURE;
    }

    NodeStatus GoToPosition1()
    {
            switch (stateCondition)
            {
                case 0:
                tunggu = 0;
                posRotateNew = false;
                    headMove(0.0, -1.6);
                    motion("0");
                    if(msg_yaw < 0){
                    stateCondition = 1;
                    }else if(msg_yaw > 0){
                        stateCondition = 2;
                    }
                break;
                case 1:
                        tunggu =0;
                        if(penLost(10)){
                            motion("0");
                            headMove(-0.25, -1.6);
                            //searchBallRectang(-1.6, -1.6, -0.8, 0.0);
                        }else{
                            trackPen();
                            if(posPan <= -1.38){
                                stateCondition = 3;
                            }else{
                                motion("9");
                                jalanDirection(0.06,0.0,-90);
                            }
                        }
                break;
                case 2:
                        tunggu =0;
                        if(penLost(20)){
                            motion("0");
                            headMove(0.35, -1.6);
                            //searchBallRectang(-1.6, 0.0, -0.8, 1.6);
                        }else{
                            trackPen();
                            if(posPan > 1.48){
                                
                                stateCondition = 3;
                            
                            }else{
                                motion("9");
                                jalanDirection(0.06,0.0,90);
                            }
                        }
                break;
                
                case 3:
                        if(posRotateNew){
                            motion("0");
                            stateCondition = 4;
                        }else{
                            motion("9");
                            rotateBodyImuNew(0);
                        }
                break;
                case 4:
                        tunggu =0;
                        posRotateNew = false;
                        if(penLost(10)){
                            //tiltSearchBall(0);
                            headMove(0.0, -1.45);
                            Walk(0.0, 0.0, 0.0);
                        }else{
                            trackPen();
                            if(posTilt <= -1.57)
                            {
                                stateCondition = 5;
        
                            }else{
                                motion("9");
                                Walk(-0.03,0.0,0.0);
                            }
                        }
                break;
                case 5:
                        headMove(0.0, -1.5);
                        if(tunggu > 5){
                            if(posRotateNew){
                                motion("0");
                                Pickup = false;
                                // robotPos_X = -400;
                                // robotPos_Y = 0;
                                stateCondition = 10;
                            
                            }else{
                                motion("9");
                                rotateBodyImuNew(0);
                            }
                        }else{
                            tunggu++;
                            posRotateNew = false;
                        }
                break;
            }
        return NodeStatus::FAILURE;
    }
    //bool depanGawang = false;
    NodeStatus GoToPosition2()
    {
        Pickup = false;
        initialPos_X  = -450;
	    initialPos_Y  = 0;
	Grid = 3;
	//doneMoved = true;
	//depanGawang = true;
        if ((State == 1 && Remaining == 600) || (State == 1 && Remaining == 300)) // ketika awal masuk
        {      
            switch (stateCondition)
            {
                case 0:
                    printf("...case 0...\n");
                    posRotateNew = false;
                    delay = 0;
                    if(penLost(20)){
                        motion("0");
                        tiltSearchBall(0);
                    }else{
                        trackPen();
                        motion("0");
                        if(tunggu > 5){
                            stateCondition = 1;
                        }else{
                            tunggu++;
                        }
                    }

                break;
                case 1:             
                    printf("...case 1...\n");
                    tunggu = 0;
                    delayWaitBall = 0;
                    timer = false;
                    if(penLost(20)){
                        stateCondition = 0;
                    }else{
                        trackPen();
                            motion("9");
                            if (headTilt >= -0.90 && headPan >= -0.4 && headPan <= 0.4)
                            {
                                stateCondition = 2;
                            }
                            else
                            {
                                
                                followPen(0);
                            }
                        
                    }
                break;
                case 2:
                    printf("...case 2...\n");
                    if(delay > 30){                         
                        if(posRotateNew){
                            motion("0");
                            stateCondition = 3;
                        }else{
                            rotateBodyImuNew(180);
                        }
                    }else{
                        delay++;
                        Walk(0.06,0.0,0.0);
                    }


                break;
                case 3:
                    printf("...case 3...\n");
                    headMove(0.0, -1.5);
                    posRotateNew = false;
                    if (delayWaitBall > 5)
                    {     
                            cekWaktu(8);
                            if (timer)
                            {
                                motion("0");
                                tunggu = 0;
                                stateCondition =4;
                            }
                            else if (second > 9 || second < 1)
                            {
                                motion("9");
                                jalanDirection(0.0, 0.0, 175);
                            }
                            else
                            {
                                motion("9");
                                jalanDirection(0.06, 0.0, 175);
                            }
                    }
                    else
                    {
                        setWaktu();
                        delayWaitBall++;
                    }
                break;
                case 4:
                        printf("...case 4...\n");
                        delayWaitBall = 0;
                        tunggu = 0;
                        if(posRotateNew){
                            //motion("0");
                            if(penLost(10)){
                                tiltSearchBall(0);

                            }else{
                                trackPen();
                                if(posTilt <= -1.59)
                                {
                                    stateCondition = 5;
        
                                }else{
                                    motion("9");
                                    Walk(-0.04,0.0,0.0);
                                }
                            }
                        }else{
                            motion("9");
                            rotateBodyImuNew(1);
                        }

                break;

                case 5:
                        
                        printf("...case 5...\n");
                        headMove(0.0, -1.5);
                        if(tunggu > 5){
                            if(posRotateNew){
                                motion("0");
                                stateCondition = 8;
                            }else{
                                motion("9");
                                rotateBodyImuNew(0);
                            }
                        }else{
                            tunggu++;
                            posRotateNew = false;
                        }
                break;
            }
        }

        if ((State == 1 && Remaining != 600) || (State == 1 && Remaining != 300)) // ketika terjadi goal
        {
            if (posRotateNew)
            {
                printf("...case lain yang ke2...\n");
                motion("0");
                panSearchBall(-1.35);
                stateCondition = 8;
                //doneMoved = true;
                Pickup = false;
            }
            else
            {
                printf("...case lain...\n");
                threeSearchBall();
                rotateBodyImuNew(0);
            }
        }
        doneMoved = true;
        Pickup = false;
        return NodeStatus::FAILURE;
    }
    NodeStatus Followpen()
    {
        switch (stateCondition)
        {
        case 0:
            if(penLost(20))
            {
                motion("0");
                threeSearchBall();
            }else
            {
                trackPen();
            }
            break;
        
        default:
            break;
        }
        return NodeStatus::FAILURE;
    }
    
    NodeStatus Goalkeeper()
    {
    
        releaseInitDone = false;
        doneMoved = false;
        clearVizTarget();

	 if (State == 3 && Stopped == 1 && secondaryInfo[0] == 0) {
            motion("8");  // robot diam 
            printf("[STOP EMERGENCY] Robot stopped during play. State=%d Stopped=%d\n", State, Stopped);
            return NodeStatus::FAILURE;
        }
        //PENALTY KICK 
        if (StatePenaltyKick() == NodeStatus::SUCCESS) {
            motion("0");
            trackBall();
            return NodeStatus::FAILURE;
        }
        
        // === PROTEKSI GC 2026: SET-PLAY (ThrowIn, GoalKick, CornerKick, FreeKick, dll) ===
        // Untuk GOALKEEPER: saat ada set-play aktif (secondaryInfo[0] != 0),
        // goalkeeper cukup DIAM dan TRACK BOLA saja.
        // Kecuali: GoalKick (setPlay=5) milik KITA → goalkeeper harus tendang bola.
        if (secondaryInfo[0] != 0) {
            
            if (secondaryInfo[0] == 5 && KickOff == barelang_color) {
                
            } else {
                if (Stopped == 1) {
                    motion("0"); // Wasit masih posisikan bola → diam total
                } else {
                    motion("0"); // Set-play sedang dieksekusi → diam, pantau bola
                    trackBall();
                }
                // Reset state agar saat set-play selesai, mulai dari case 0
                stateCondition = 0;
                doneBanting = false;
                tendang = false;
                robotDirection = false;
                ballPos = false;
                tunggu = 0;
                resetCase0();
                printf("[GK] SET-PLAY: type=%d KickOff=%d Stopped=%d, diam & track bola\n", 
                       secondaryInfo[0], KickOff, Stopped);
                return NodeStatus::FAILURE;
            }
        }
        
        if (secondaryInfo[0] == 5) {
            // Phase 1: Stopped=1 → wasit taruh bola, DIAM
            if (Stopped == 1) {
                motion("0");
                printf("wasit taruh bola, diam\n");
                goalKickPhase = 1;
                goalKickKicked = false;
                goalKickWait = 0;
                return NodeStatus::FAILURE;
            }

            // Phase 3: Sudah tendang → homing kembali ke gawang
            if (goalKickKicked) {
                printf("homing ke gawang\n");
                if (doneMoved) {
                    motion("0");
                    doneMoved = false;
                    goalKickKicked = false;
                    goalKickPhase = 0;
                } else {
                    threeSearchBall();
                    new_out_pos(-400, 0, true);
                }
                return NodeStatus::FAILURE;
            }

            // Phase 2: GC resume → goalkeeper jalan ke bola dan tendang
            printf("cari bola dan tendang\n");
            if (ballLost(35)) {
                // Bola belum terlihat → cari sambil jalan maju pelan
                threeSearchBall();
                motion("9");
                Walk(0.03, 0.0, 0.0);
            } else {
                trackBall();
                if (headTilt >= cAktif) {
                    // Bola sudah dekat & lurus → positioning dan tendang ke yaw 0
                    if (!robotDirection) {
                        Imu(0, cSekarang); // Arahkan badan lurus ke depan
                    } else {
                        kick(1); // Tendang jauh ke depan
                        if (tendang) {
                            goalKickKicked = true;
                            printf("[GK-GoalKick] KICK executed!\n");
                        }
                    }
                } else {
                    followBall(0); // Dekati bola
                }
            }
            return NodeStatus::FAILURE;
        }

        switch (stateCondition)
        {
        case 0:
        printf("case 0\n");
            delay = 0;
            //initialPos_Y = initialPos_X = 0;
            //deltaPos_X = -400;
            //deltaPos_Y = 0;
            //doneBanting = false;

            if(ballLost(20))
            {
            	
                if(searchKe >= 1){
                    if(posRotateNew)
                    {
                        motion("0");
                        searchKe = 0;
                        printf("aaaaaaaaa\n");
                    }else
                    {
                        threeSearchBall();
                        //quickSearchBall();
                        printf("QSB 1934\n");
                        //rotateBodyImuNew(0);
                        ballAround();
                    }
                }else
                {
                    posRotateNew = false;
                    motion("0");
                    threeSearchBall();
                    //quickSearchBall();
                    printf("QSB 1943\n");
                    //panSearchBall(-1.35);
                    rotateBodyImuNew(0);
                    
                    /*if (ballLost(200)){
                    	new_out_grid(3, 0, 50, true);
                    	stateCondition = 0;
                    }*/
                }
            }else
            {
                //motion("0");
                trackBall();
                if(tunggu > 10){
                    if(posTilt <= tiltBolaJauh) // bola jauh -1.70
                    {
                    /*
                        printf("trackball");
                    }else if (!doneBanting) // 
                    {
                        posRotateNew = false;
                        bodyTrue = 0;
                        stateCondition = 7;
                        */
		        delayBolaJauh++;
		        printf("trackball - bola jauh (delay=%d, pan=%.2f)\n", delayBolaJauh, posPan);
		        
		        // Jika bola jauh > 3 detik (~90 siklus) DAN kepala menoleh cukup jauh,
		        // putar badan pelan ke arah bola agar kiper selalu menghadap arah ancaman.
		        if (delayBolaJauh > 10 && (posPan > 0.3 || posPan < -0.3))
		        {
		            // Rotasi badan ke arah pan (tanpa maju/mundur)
		            double rotSpeed = 0.0;
		            if (posPan > 0.3) {
		                rotSpeed = 0.15;  // putar kiri (bola di kiri)
		            } else if (posPan < -0.3) {
		                rotSpeed = -0.15; // putar kanan (bola di kanan)
		            }
		            Walk(0.0, 0.0, rotSpeed);
		            printf("[GK] Putar badan ke arah bola (pan=%.2f, rot=%.2f)\n", posPan, rotSpeed);
		        }
		        else
		        {
		            motion("0"); // bola jauh tapi kepala masih tengah, diam saja
		        }
                    }else if (!doneBanting)
                    {
                        delayBolaJauh = 0;
                        if(posTilt >= tiltBolaDekat) // bola DEKAT (-1.30) 
                        {
                            delayWaitBall = 0;
                            printf("[GK] Bola DEKAT (tilt=%.2f pan=%.2f) → BANTING!\n", posTilt, posPan);
                            stateCondition = 1; // langsung ke banting, tanpa body tracking
                        }
                        else
                        {
                        //motion("0")
                            // Bola di zona menengah (antara -1.70 dan -1.30)
                            // Strafing lateral: bergeser ke arah bola untuk menutup sudut
                            if (posPan > 0.2) {
                                Walk(0.0, 0.03, 0.0);  // strafe kiri (bola di kiri)
                                printf("[GK] Bola menengah → strafe KIRI (pan=%.2f)\n", posPan);
                            } else if (posPan < -0.2) {
                                Walk(0.0, -0.03, 0.0); // strafe kanan (bola di kanan)
                                printf("[GK] Bola menengah → strafe KANAN (pan=%.2f)\n", posPan);
                            } else {
                                motion("0"); // bola lurus di tengah, diam siap banting
                                printf("[GK] Bola menengah TENGAH (tilt=%.2f) → siap\n", posTilt);
                            }
                        }
                    }else // sudah banting 
                    {
                    	delayBolaJauh = 0;
                        delayWaitBall = 0;
                        stateCondition = 2;
                    }
                }else{
                    motion("0");
                    tunggu++;
                }
                 
            }
        break;

        case 1: //banting
            printf("case 1\n");
            if (doneBanting)
            {
                printf("anjY siap\n");
               // headMove(0.0,-1.70);
                timer = false;
                //doneBanting = false;
                delayWaitBall = delay = second = 0;
                stateCondition = 2; //2
            }
            else
            {
                banting();
            }
            break;



        case 2: // search ball setelah banting
            printf("case 2\n");
            if (ballLost(50))
            {
                if(searchKe >= 2){
                    if(posRotateNew)
                    {
                        motion("0");
                        searchKe = 0;
                    }else
                    {
                        rotateBodyImuNew(0);
                    }
                    
                }else
                {
                    posRotateNew = false;
                    motion("0");
                    headMove(lastBallPan, -1.40);
                    printf("Cari bola di arah terakhir\n");
                    rotateBodyImuNew(0);
                }

            }
            else
            {
                motion("0");
                searchKe = 0;
                trackBall();
                if (delayWaitBall > 5)
                {
                    tendang = ballPos = robotDirection = false;
                    stateCondition = 3;
                }
                else
                {
                    reset_velocity();
                    delayWaitBall++;
                }
            }

            break;

        case 3: // follow Ball
            printf("case 3\n");
            delayWaitBall = 0;
            
            // === CEK BATAS PENALTY AREA ===
            // Kiper tidak boleh keluar kotak penalti (rules RoboCup)
            // Batas: X > -300 (terlalu maju) atau |Y| > 250 (keluar samping)
            if (robotPos_X > -300 || robotPos_Y > 250 || robotPos_Y < -250)
            {
                printf("[GK] KELUAR PENALTY AREA! (pos=%.0f,%.0f) → HOMING\n", robotPos_X, robotPos_Y);
                motion("0");
                delay = 0;
                posRotateNew = doneMoved = false;
                tunggu = 0;
                refreshMoveGrid();
                stateCondition = 6;
                break;
            }
            
                if (ballLost(20))
                {
                    motion("0");
                    delayWaitBall = 0;
                    stateCondition = 0; // kembali track, bukan homing
                }
                else
                {
                    //motion("9");
                    
                    trackBall();
                        if (robotDirection && headPan >= -0.4 && headPan <= 0.4)
                        {
                            tendang = ballPos = false;
                            tunggu = 0;
                            printf("mbokkk\n");
                            stateCondition = 4;
                        }
                        else
                        {
                            // Goalkeeper: bila bola sudah dalam jarak Imu (cAktif), arahkan lurus
                            if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4)
                            {
                                Imu(0, cSekarang); // Rotasi + maju merapatkan gap
                            }
                            else
                            {
                                robotDirection = false;
                                followBall(0);
                            }
                        }
                }
                  
	     break;
        case 4: //Tendang
            printf("case 4\n");
            printf("aaaaaa = %d %d/n", tendang, ballPos);
            delay++; // timeout counter
            
            // === CEK BATAS PENALTY AREA ===
            if (robotPos_X > -300 || robotPos_Y > 250 || robotPos_Y < -250)
            {
                printf("[GK] KELUAR PENALTY AREA saat tendang! (pos=%.0f,%.0f) → HOMING\n", robotPos_X, robotPos_Y);
                motion("0");
                posRotateNew = doneMoved = false;
                tunggu = 0;
                refreshMoveGrid();
                stateCondition = 6;
                break;
            }
            
                if (ballLost(20))
                {
                    resetCase0();
                    tunggu = 0;
                    stateCondition = 0; // kembali track, bukan homing
                }
                else
                {
                    trackBall();
                    ballDistance = 1;
                        if (tendang)
                        {
                            if(tunggu > 10)
                            {
                                delay = 0;
                                stateCondition = 5; //5
                                motion("0");
                            }else{
                                tunggu++;
                            }
                        }
                        else
                        {
                            kick(1);
                        }
                }
            //}
        break;

        case 5: //setelah tendang → cek posisi, homing hanya jika keluar penalty area
            printf("case 5\n");
            if(robotPos_X < -300 && robotPos_Y >= -250 && robotPos_Y <= 250)
            {
                // Masih di dalam penalty area → skip homing, langsung ke case 8 (reset)
                printf("[GK] Masih di penalty area (pos=%.0f,%.0f) → ke case 8\n", robotPos_X, robotPos_Y);
                second = delay = 0;
                timer = false;
                motion("0");
                stateCondition = 8;
            }
            else
            {
                // Keluar dari penalty area → homing dulu ke -400,0
                printf("[GK] Keluar penalty area (pos=%.0f,%.0f) → HOMING\n", robotPos_X, robotPos_Y);
                posRotateNew = doneMoved = false;
                tunggu = 0;
                refreshMoveGrid();
                stateCondition = 6;
            
            /*if(robotPos_X > -250 && robotPos_Y >= -200 || robotPos_Y <= 200){            
                stateCondition = 6;
                posRotateNew = doneMoved = false;
            }

            if(robotPos_X > -250 && robotPos_Y <= -200 || robotPos_Y >= 200){            
                stateCondition = 6;
                posRotateNew = doneMoved = false;
            }

            if(robotPos_X < -250 && robotPos_Y <= -200 || robotPos_Y >= 200){            
                stateCondition = 6;
                posRotateNew = doneMoved = false;
            }*/

            // if(robotPos_X < -350)
            // {
            //     stateCondition = 8;
            //     motion("0");
            // }else{            
            //     stateCondition = 6;
            //     posRotateNew = doneMoved = false;
            // }
            }
        break;


        case 6: //Homing
            printf("case 6\n");
            tendang = ballPos = robotDirection = false;
            if (doneMoved) //doneMoved
            {
                motion("0");
                 timer = false;
                 delay = second = 0;
                // stateCondition = 8;
                //delay = 0;
                tunggu = 0;
                stateCondition = 8; // ke case 8 (reset, doneBanting, search)

            } else {
                tunggu++;
                threeSearchBall();
                //quickSearchBall();
                printf("QSB 2188 (tunggu=%d)\n", tunggu);
                //gkBackPos();
                //new_out_grid(3, 0, 50, true);
                new_out_pos(-400, 0, true);
                
                // Timeout: jika setelah ~300 siklus (~10 detik) belum sampai,
                // kemungkinan odometry ngawur. Paksa kembali ke case 0.
                if (tunggu > 300)
                 {
                    printf("[GK] Homing TIMEOUT! Ke case 8.\n");
                    tunggu = 0;
                    refreshMoveGrid();
                    stateCondition = 8;
                }
                //new_out_pos(-350, 310, true);
            }
        break;

        // case 7: //Trackingtoball
        //     if(ballLost(20))
        //     {
        //         motion("0");
        //         second = delay = 0;
        //         timer = false;
        //         stateCondition = 8;
        //     }else 
        //     {   
        //         tunggu = 0; 
        //         trackBall();
        //         if(headPan >= -0.1 && headPan <= 0.1)
        //         {
        //             motion("0");
        //             delayWaitBall = 0;
        //             stateCondition = 1;
        //         }else{
        //             bodyTrackingBall(10);
        //         }
        //     }
        // break;

        case 7: //Trackingtoball
            printf("case 7\n");
            if(ballLost(20))
            {
                motion("0");
                second = delay = 0;
                timer = false;
                stateCondition = 0;
            }else 
            {   
            	//statecondition = 232;
                tunggu = 0; 
                trackBall();
                if(posPan > -0.25 && posPan < 0.25) //if(posPan > -0.05 && posPan < 0.05)
                {
                    motion("0");
                    if(!useBanting){
                        trackBall();
                        if(posTilt > -1.65) //dimas: -1.55 dan yang terbaru -1,60
                        {
                            delayWaitBall = 0;
                            stateCondition = 2;
                        }else{
                            if(posPan > -0.20 && posPan < 0.20)
                            {
                                motion("0");
                            }else{
                                newBodyTracking();
                                //bodyTrackingBall(5);
                            }
                        }
                    }else{
                        delayWaitBall = 0;
                        headMove(0.0, -1.60); 
                        stateCondition = 1;
                      
                    }
                }else{
                    newBodyTracking();
                    //bodyTrackingBall(5);
                }
                // if (posPan > -0.05 && posPan < 0.05){
                //     motion("0");
                //     stateCondition = 1;
                // }else{
                //     newBodyTracking();
                // }
            }
        break;
/*
        case 8: //reset nilai pantilt kepala
            printf("case 8\n");
            tunggu = 0;
            doneBanting = false;
            if(ballLost(20)){
                if(msg_yaw <= -45 && msg_yaw >= 45)
                {
                    if(cnt_sbr >= 4){
                        if(posRotateNew)
                        {
                            motion("0");
                            cnt_sbr = 0;
                        }else
                        {
                            rotateBodyImuNew(0);
                        }
                        
                    }else
                    {
                        posRotateNew = false;
                        motion("0");
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        printf("SBR 2270\n");
                        //quickSearchBall();
                        /*if (ballLost(200)){
		            new_out_grid(3, 0, 50, true);
		            stateCondition = 0;
		        }
                    }
                }else{
                    if(searchKe >= 2){
                        if(posRotateNew)
                        {
                            motion("0");
                            searchKe = 0;
                        }else
                        {
                            rotateBodyImuNew(0);
                        }
                        
                    }else
                    {
                        posRotateNew = false;
                        motion("0");
                        threeSearchBall();
                        //quickSearchBall();
                        printf("QSB 2309\n");
                        rotateBodyImuNew(0);
                        /*if (ballLost(200)){
		            new_out_grid(3, 0, 50, true);
		            stateCondition = 0;
		        }
                    }
                }
                    // if(cnt_sbr >= 2){
                    //     if(posRotateNew)
                    //     {
                    //         motion("0");
                    //         cnt_sbr = 0;
                    //     }else
                    //     {
                    //         rotateBodyImuNew(0);
                    //     }
                        
                    // }else
                    // {
                    //     posRotateNew = false;
                    //     motion("0");
                    //     searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                    // }
                
            }else
            {
                trackBall();
                if(delay > 3)
                {   
                    cekWaktu(3);
                    if(timer){
                        stateCondition = 0;
                    }else
                    {
                     headMove(0.0,-1.80);
                    }
                }else
                {
                    setWaktu();
                    delay++;
                    printf("mbekkk\n");
                } 
            }
        break;
*/


        case 8: //reset nilai pantilt kepala + doneBanting
            printf("case 8\n");
            doneBanting = false; // Siap banting lagi
            
            if(ballLost(20)){
                // Bola hilang → search ball sebentar, lalu ke case 0
                if(searchKe >= 2){
                    // Sudah search 2 putaran → langsung ke case 0
                    printf("[GK] Case 8: search selesai, ke case 0\n");
                    motion("0");
                    searchKe = 0;
                    tunggu = 0;
                    stateCondition = 0;
                }else{
                    posRotateNew = false;
                    motion("0");
                    threeSearchBall();
                    printf("QSB case8\n");
                    rotateBodyImuNew(0);
                }
            }else{
                // Bola ditemukan → langsung ke case 0
                trackBall();
                if(tunggu > 30){
                    printf("[GK] Case 8: bola ketemu, ke case 0\n");
                    motion("0");
                    tunggu = 0;
                    stateCondition = 0;
                }else{
                    motion("0");
                    tunggu++;
                }
            }
        break;
/*
        case 8: //reset nilai pantilt kepala
            printf("case 8\n");
            tunggu = 0;
            doneBanting = false;
            if(ballLost(20)){
                if(msg_yaw <= -45 && msg_yaw >= 45)
                {
                    if(cnt_sbr >= 4){
                        if(posRotateNew)
                        {
                            motion("0");
                            cnt_sbr = 0;
                        }else
                        {
                            rotateBodyImuNew(0);
                        }
                        
                    }else
                    {
                        posRotateNew = false;
                        motion("0");
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        printf("SBR 2270\n");
                        //quickSearchBall();
                        /*if (ballLost(200)){
		            new_out_grid(3, 0, 50, true);
		            stateCondition = 0;
		        }
                    }
                }else{
                    if(searchKe >= 2){
                        if(posRotateNew)
                        {
                            motion("0");
                            searchKe = 0;
                        }else
                        {
                            rotateBodyImuNew(0);
                        }
                        
                    }else
                    {
                        posRotateNew = false;
                        motion("0");
                        threeSearchBall();
                        //quickSearchBall();
                        printf("QSB 2309\n");
                        rotateBodyImuNew(0);
                        /*if (ballLost(200)){
		            new_out_grid(3, 0, 50, true);
		            stateCondition = 0;
		        }
                    }
                }
                    // if(cnt_sbr >= 2){
                    //     if(posRotateNew)
                    //     {
                    //         motion("0");
                    //         cnt_sbr = 0;
                    //     }else
                    //     {
                    //         rotateBodyImuNew(0);
                    //     }
                        
                    // }else
                    // {
                    //     posRotateNew = false;
                    //     motion("0");
                    //     searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                    // }
                
            }else
            {
                // Bola ditemukan → kembali ke posisi gawang dulu sebelum main normal
                trackBall();
                if (doneMoved) {
                    // Sudah di posisi gawang → mulai main normal
                    motion("0");
                    stateCondition = 0;
                    printf("[GK] case 8: sudah di gawang, pindah ke case 0\n");
                } else {
                    // Belum di posisi gawang → jalan ke gawang sambil track bola
                    new_out_pos(-400, 0, true);
                    printf("[GK] case 8: jalan balik ke gawang\n");
                }
            }
        break;
*/
        case 9:
        printf("case 9\n");
            delay = 0;
            initialPos_Y = initialPos_X = 0;
            deltaPos_X = -400;
            deltaPos_Y = 0;
            doneBanting = false;
            if(ballLost(20))
            {
                if(searchKe >= 2){
                    if(posRotateNew)
                    {
                        motion("0");
                        searchKe = 0;
                    }else
                    {
                        threeSearchBall();
                        rotateBodyImuNew(0);
                    }
                }else
                {
                    posRotateNew = false;
                    motion("0");
                    threeSearchBall();
                    rotateBodyImuNew(0);
                }
            }else
            {
                motion("0");
                trackBall();
                if(tunggu > 30){
                    if(posTilt <= -1.75) // bola jauh -1.75
                    {
                        printf("trackball");
                    }else if( posTilt > -1.75 && posTilt < -1.50) //bola sedang
                    {
                        doneBanting = posRotateNew = false;
                        bodyTrue = 0;
                        stateCondition = 7;
                    }else if(posTilt >= -1.50) //bola dekat -1.50
                    {
                        delayWaitBall = 0;
                        stateCondition = 2;
                        
                    }
                }else{
                    tunggu++;
                }
                
            }
        break;

        case 10: //reset nilai pantilt kepala
            tunggu = 0;
            if(ballLost(20)){
                    if(searchKe >= 3){
                        if(posRotateNew)
                        {
                            motion("0");
                            searchKe = 0;
                        }else
                        {
                            rotateBodyImuNew(0);
                        }
                        
                    }else
                    {
                        posRotateNew = false;
                        motion("0");
                        threeSearchBall();
                        rotateBodyImuNew(0);
                    }
                
            }else
            {
                trackBall();
                if(delay > 3)
                {   
                    cekWaktu(3);
                    if(timer){
                        stateCondition = 9;
                    }else
                    {
                        headMove(0.0,-1.80);
                    }
                }else
                {
                    setWaktu();
                    delay++;
                } 
            }
        break;
            
        case 11:
            if(penLost(20))
            {
                motion("0");
                tiltSearchBall(0);
            }else
            {
                trackPen();
                if(delayWaitBall > 120){
                    if(posTilt >= -1.70)
                    {
                        posRotateNew = false;
                        stateCondition = 12;
                    }else{
                        Walk(0.03, 0.0, 0.0);
                    }
                }else{
                    delayWaitBall++;
                }
            }
            
            break;

        case 12:
            if(penLost(20))
            {
                delayWaitBall = 0;
                stateCondition = 11;
            }else{
                trackPen();
                if(posRotateNew)
                {
                    motion("0");
                    stateCondition = 13;
                }else
                {
                    rotateBodyImuNew(0);
                }
            }
            break;

        case 13:
            if(penLost(20))
            {
                delayWaitBall = 0;
                stateCondition = 11;
            }else{
                trackPen();
                if(posPan >= -0.05 && posPan <= 0.1){
                    stateCondition = 14;
                }else{
                    gkBodyTrackingBall();
                }
            }
            break;

        case 14:
            if(penLost(20)){
                delayWaitBall = 0;
                stateCondition = 11;
            }else{
                trackPen();
                timer = false;
                delay = second = 0;
                stateCondition = 0;
                motion("0");
            }
            break;

        case 15:
            if(penLost(20))
            {
                motion("0");
                threeSearchBall();
            }else{
                trackPen();
                if(ballPos){
                    stateCondition = 16;
                    posRotateNew = false;
                }else{
                    ballPositioning(0.06, -1.70, ballPositioningSpeed);
                }
            }
            break;

        case 16:
            if(penLost(20)){
                ballPos = false;
                stateCondition = 15;
            }else{
                if(posRotateNew)
                {
                    motion("0");
                }else
                {
                    rotateBodyImuNew(0);
                }
            }
            break;

        case 17:
                headMove(0.0, -1.60);
                if (delay > 40)
                {
                    timer = false;
                    delay = second = 0;
                    stateCondition = 0;
                }else{
                    motion("9");
                    Walk(0.055, 0.0, 0.0);
                    delay++;
                }
            break;

        default:
            break;
        }
        return NodeStatus::FAILURE;
    }
      NodeStatus noReady()
    { 
        cekWaktu(30);
            if(timer){
                return NodeStatus::SUCCESS;
            }
        setWaktu();
        return NodeStatus::FAILURE;
    }
    
    NodeStatus noSet()
    { 
        cekWaktu(40);
            if(timer){
                return NodeStatus::SUCCESS;
            }
        setWaktu();
        return NodeStatus::FAILURE;
    }
    
    NodeStatus isGoalkeeper()
    {
        if(msg_strategy == 0)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

     NodeStatus isGoal()
    {
        if ((Remaining == 600 && SecondaryState == 0) || (Remaining == 300 && SecondaryState == 2))//rotate ke 0 setelah ada goal/dropball
        {
            doneMoved = false;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }
    
    NodeStatus RobotAction()
    {
        if (passed)
        {
            sleep(0.5);
            motion("0");
            return NodeStatus::SUCCESS;
        }
        else
        {
            motion("$");	//push up
            passed = true;
        }
        return NodeStatus::FAILURE;
    }
    
    NodeStatus RobotAction2()
    {
        if (passed)
        {
            sleep(0.5);
            motion("0");
            return NodeStatus::SUCCESS;
        }
        else
        {
            motion("!");	//parkour
            passed = true;
        }
        return NodeStatus::FAILURE;
    }
    
    NodeStatus Forward()
    {	
    	Walk(jalan, 0.00, 0.0);
        return NodeStatus::FAILURE;
    }
    
    NodeStatus Backward()
    {	
    	Walk(-lari, 0.00, 0.0);
        return NodeStatus::FAILURE;
    }
    
    NodeStatus Sideway()
    {	
    	if (robotPos_Y < -200)
    	{
    		motion("0");
    	} else {
    		Walk(0.0, kejar, 0.0);
    	}
        return NodeStatus::FAILURE;
    }
    bool Jalankanan = true;
    bool Jalankiri = false;
    /*NodeStatus Obstacle()
    {
    	headMove(0.0, -0.70);
    	if(msg_yaw <= 5 && msg_yaw >=-5) {
    	
    	if (ballLost(30))
    	{ 
     	    printf("...Ga ada Obstacle...\n");
     	    Walk (0.02,0.0,0.0);
     	} else 
     	{
     	    if(robotPos_Y <= -50)
     	    { 
     		Jalankanan = true;
     		Jalankiri =false;
     	    }
      	    else if (robotPos_Y >= 50)
      	    {
      	    	Jalankanan = false;
        	Jalankiri = true;
            }
        
            if (Jalankanan)
            {
     	    	printf("...kanan...\n");
     	    	Walk (0.0,-0.04,0.0);
     	    } else if (Jalankiri) {
     	    	printf("...kiri...\n");
     	    	Walk (0.0,0.04,0.0);
     	    }
     	}
     	}else {
     	rotateBodyImuNew(0);
     	}
     	return NodeStatus::FAILURE;
     }
      */
 
    NodeStatus ShootImu()
    {
        switch (stateCondition)
        {
            case 0: // search ball
                tendang = false;
                ballPos = false;
                //robotDirection = false;
                if(msg_yaw <= 5 && msg_yaw >=-5) {
                if (ballLost(35))
                {
                    Walk (0.03,0.0,0.0);
                    //tiltSearchBall(0.0);
                    headMove(0.0, -0.7); // -0.4,-1.2
                } else 
                {
                    trackBall();
                    if (delayWaitBall > 20)
                    {  
                        robotDirection = false;
                        stateCondition = 1;
                    } else
                    {
                        delayWaitBall++;
                    }
                }
               }else{
               rotateBodyImuNew(0);
               }
               
            break;

            case 1: // follow Ball
            printf("case 1 cuy");
                motion("9");
                tendang = false;
                ballPos = false;
                //robotDirection = false;
                delayWaitBall = 0;
                if (ballLost(35))
                {
                    resetCase0();
                    stateCondition = 0;
                } else
                {   
                    trackBall();
                    if (headTilt >= -0.8)   // && headPan <= 0.4 ) //&& headPan <= 0.4 
                    {
                     
                    //Walk (0.0,0.04,0.0);
                    printf("track\n");
                    //trackBall();
                    if (headPan >= 0.1  && headPan <= 0.4 ){
                        Walk (0.0,-0.04,0.0);
                        //headMove (0.0,-0.6);
                        //Walk (0.02,0.0,0.0);
                        printf("kanan dia lek\n");
                        }
                    else if (headPan >= -0.1 && headPan <= -0.4 ) {
                         Walk (0.0,0.04,0.0);
                         printf("kiri dia lek\n");
                         //resetCase0();
                         //stateCondition = 0;
                         }
                    }
                    else//( headPan <= 0.4 && headPan <= 0.4) //(headPan >= -0.4  )
                    {       
                    if (headPan >=0) {
                            
                            headMove (-0.3,-0.7);
                            sleep(1.5);
                            printf("nunduk setelah kanan\n");
                            }
                    else if (headPan <=0){
                    	     headMove (0.3,-0.7);
                    	     sleep(1.5);
                    	     printf("reset\n");
                         resetCase0();
                         stateCondition = 0;
                    	     printf("nunduk setelah kiri\n");
                    	     }
                    	     
                    /*else{
                         headMove (0.3,-0.7);
                         sleep(1);
                         }
                         printf("reset\n");
                         resetCase0();
                         stateCondition = 0;
                         
                            //Walk (0.0,-0.04,0.0);
                            ///robotDirection = false;
                            
                            //Walk(0.02, 0.0, 0.0);
                     /*trackBall();
                     if (headPan <= 0.6)
                     {
                            headMove (0.0,-0.6);
                            Walk(0.02,0.0,0.0);}
                            
                     else {
                            resetCase0();
                            stateCondition = 0;
                          }*/
                     }
                           /*if(ballLost(35))
               	     {
               	        
               	        printf("ga nt\n");
                    		} */
                            
		            /*if (robotDirection && headPan > -0.4 && headPan < 0.4)
		            {
		                reset_velocity();
		                tendang = ballPos = false;
		                stateCondition = 2;
		                printf("udah ni");
		            }*/
		            /*else
		            {
		               Imu(0, cSekarang);
		               if (robotPos_Y <= 0)
		               {
		               printf("....tendang samping\n");
		               Imu(5,cSekarang);
		               }
		               
				else{
				printf("....tendang depan\n");
				Imu(0,cSekarang);
				}	
		            }*/
                    /*} else 
                    {
                        followBall(0);
                    }*/
                }
                
            break;

            case 2: // BallPos -> Tendang
                
                if (ballLost(35))
                {
                    resetCase0();
                    stateCondition = 0;
                } else 
                {
                    trackBall();
                    if (tendang)
                    {
                        resetCase0();
                        stateCondition = 0;
                    } else 
                    {
                        kick(2);
                    }
                }

            break;

        default:
            break;
        }
        return NodeStatus::FAILURE;
    }
    
    
    NodeStatus ShootImu2()
    {
        switch (stateCondition)
        {
            case 0: // search ball
                tendang = false;
                ballPos = false;
                //robotDirection = false;
                if (ballLost(35))
                {
                    motion("0");
                    //tiltSearchBall(0.0);
                    headMove(0.0, -1.2); // -0.4,-1.2
                } else 
                {
                    trackBall();
                    if (delayWaitBall > 20)
                    {  
                        robotDirection = false;
                        stateCondition = 1;
                    } else
                    {
                        delayWaitBall++;
                    }
                }
                
            break;

            case 1: // follow Ball
            printf("case 1 cuy");
                motion("9");
                tendang = false;
                ballPos = false;
                //robotDirection = false;
                delayWaitBall = 0;
                if (ballLost(35))
                {
                    resetCase0();
                    stateCondition = 0;
                } else
                {
                    trackBall();
                    if (headTilt >= cAktif && headPan >= -0.3 && headPan <= 0.3)
                    {   
		            if (robotDirection && headPan > -0.4 && headPan < 0.4)
		            {
		                reset_velocity();
		                tendang = ballPos = false;
		                stateCondition = 2;
		            }
		            else
		            {
		               Imu(sudutKiri, cSekarang);
		               /*if (robotPos_Y <= 0)
		               {
		               printf("....tendang samping\n");
		               Imu(5,cSekarang);
		               }
		               
				else{
				printf("....tendang depan\n");
				Imu(0,cSekarang);
				}*/	
		            }
                    } else 
                    {
                        followBall(0);
                    }
                }
                
            break;

            case 2: // BallPos -> Tendang
                
                if (ballLost(35))
                {
                    resetCase0();
                    stateCondition = 0;
                } else 
                {
                    trackBall();
                    if (tendang)
                    {
                        resetCase0();
                        stateCondition = 0;
                    } else 
                    {
                        kick(2);
                    }
                }

            break;

        default:
            break;
        }
        return NodeStatus::FAILURE;
    }

    bool peluit_bunyi = false;
    NodeStatus KillRun()
    {
        // printf("...KillRun\n");
        if (lastState != msg_kill)
        {
            if (msg_kill == 0)
            {
                stateCondition = firstStateCondition;
                // State = 3;
                // Pickup = true;
                if (State == 1)
                {
                    dont_calibrate = true;
                }
                play = true;
            }
            else
            {
                //motion("gc,0");
                motion("8");
                resetVariable();
                play = false;
            }
            lastState = msg_kill;
        }

        if (play)
        {

            // printf("...KillRun::SUCCESS\n");
            robotStatus = 1;
            return NodeStatus::SUCCESS;
        }
        else
        {
            stateCondition = 272;
            ballDistance = 999;
            BallX = BallY = 999;
            Grid = 88;
            foundBall = robotStatus = 0;
            return NodeStatus::FAILURE;
        }
        return NodeStatus::SUCCESS;
    }
    
    NodeStatus isRobotOut()
    {
        if (msg_strategy == 0)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }
    
    NodeStatus isAttacker()
    /*{
        if (msg_strategy == 1)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }*/
    {
        if (msg_kill == 1){
            stateCondition = firstStateCondition;
                resetVariable();
                //peluit_bunyi = false;
                }  
        if (msg_peluit_hz > 2000){
          
          peluit_bunyi = true;
          }
        if (msg_strategy == 1)
        { 
          if (peluit_bunyi)
          {
            return NodeStatus::SUCCESS;}
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isDefenderleft()
    /*{
        if (msg_strategy == 2)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }*/
    {
        if (msg_kill == 1){
            stateCondition = firstStateCondition;
                //kondisi = firstStateCondition;
                resetVariable();
                //peluit_bunyi = false;
                }  
        if (msg_peluit_hz > 2000){
          
          peluit_bunyi = true;
          }
        if (msg_strategy == 2)
        { 
          if (peluit_bunyi)
          {
            return NodeStatus::SUCCESS;}
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isDefenderright()
    /*{
        if (msg_strategy == 3)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }*/
    {
        if (msg_kill == 1){
            stateCondition = firstStateCondition;
                //kondisi = firstStateCondition;
                resetVariable();
                //peluit_bunyi = false;
                }  
        if (msg_peluit_hz > 2000){
          
          peluit_bunyi = true;
          }
        if (msg_strategy == 3)
        { 
          if (peluit_bunyi)
          {
            return NodeStatus::SUCCESS;}
        }
        return NodeStatus::FAILURE;
    }

    void saveToCSV(int grid, double posx, double posy, double yaw, double pan, double tilt, int c0_x, int c0_y, int c1_x, int c1_y, int c2_x, int c2_y, int c3_x, int c3_y, int c4_x, int c4_y, int c5_x, int c5_y, int c6_x, int c6_y)
    {
        std::ofstream file;
        file.open("/home/tegra/bfc_ros2/src/main/data_posisi.csv", std::ios::out | std::ios::app);
        file << grid << ", " << posx << ", " << posy << ", " << yaw << ", " << pan << ", " << tilt << ", " << c0_x << ", " << c0_y << ", " << c1_x << ", " << c1_y << ", " << c2_x << ", " << c2_y << ", " << c3_x << ", " << c3_y << ", " << c4_x << ", " << c4_y << ", " << c5_x << ", " << c5_y << ", " << c6_x << ", " << c6_y << std::endl;
        file.close();
        std::cout << grid << ", " << posx << ", " << posy << ", " << yaw << ", " << pan << ", " << tilt << ", " << c0_x << ", " << c0_y << ", " << c1_x << ", " << c1_y << ", " << c2_x << ", " << c2_y << ", " << c3_x << ", " << c3_y << ", " << c4_x << ", " << c4_y << ", " << c5_x << ", " << c5_y << ", " << c6_x << ", " << c6_y << std::endl;
    }

    bool doneGetData = false;
    bool doneSaveData = false;
    int cnt_get_data = 0;
    NodeStatus GetData()
    {
        // if (!doneSaveData && triggerSave)
        // {
        //     saveToCSV(Grid, robotPos_X, robotPos_Y, msg_yaw, headPan, headTilt, Left_X_Cross_X, Left_X_Cross_Y, Right_X_Cross_X, Right_X_Cross_Y, Left_T_Cross_X, Left_T_Cross_Y, Right_T_Cross_X, Right_T_Cross_Y, Left_Corner_X, Left_Corner_Y, Right_Corner_X, Right_Corner_Y, Pinalty_X, Pinalty_Y);
        //     doneSaveData = true;
        //     triggerSave = false;
        // }
        initialPos_X = -350;
        initialPos_Y = 0;
        if (doneMoved)
        {
            motion("0");
            //Walk(0.0, 0.0, 0.0);
            return NodeStatus::FAILURE;
        } else 
        {
            //new_out_grid(15, -50, 50, true);
            new_out_pos(250, 0, true);
            //Walk(jalan, 0.0, 0.0); // grid tujuan
        }
        // new_out_grid(50, 50, -50, false);
        // printf("msg_yaw : %d\n", msg_yaw);
        // printf("ArahTendangan : %d\n", outTheta);
        // if (sumWalkX >= max_current)
        // {
        //     motion("0");
        // } else 
        // {
        //     Walk(0.0, jalan, 0.0);
        // }
        // calculate_target(450, 0, 0, 200);
        return NodeStatus::FAILURE;
    }

    NodeStatus SetHeadPos()
    {
        robotPos_X = convertGridX(6, 0);
        robotPos_Y = convertGridY(6, 0);
        headMove(dataPanKey, dataTiltKey);
        if (object_count > 1)
        {
            return NodeStatus::SUCCESS;
        }
        doneSaveData = false;
        return NodeStatus::FAILURE;
    }
    /*
    
    bool useBanting;
    bool tracked = false;
    int BallFoundEntry = 0;
    NodeStatus BallFound()
    {
        if (ballLost(50))
        {
            delayWaitBall = 0;
            // searchBallRectang(-1.6, -1.6, -0.8, 1.6);
		    foundBall = 0;
		    arahGoal = ballDistance = 999;
		    stateCondition = 272;
            	    BallX = BallY = 999;
            BallApproachEntry = 0;
            if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
		    {
		    	initialPos_X = initialPos_Y = 0;
		    	deltaPos_X = cam_x;
		    	deltaPos_Y = cam_y;
		    }	
            return NodeStatus::FAILURE;
        }

        foundBall = 1;
        //lockRelax = false;
        refreshMoveGrid();
        return NodeStatus::SUCCESS;       
    }
    
    */
    
       bool useBanting;
    bool tracked = false;
    bool useDribbleMode;
    int BallFoundEntry = 0;
    int lastSeenBallDistance = 999;  // jarak terakhir saat masih lihat bola
    int ballLostGraceCounter = 0;    // berapa lama bola hilang (dalam frame)
    static const int BALL_GRACE_PERIOD = 100; // ~2 detik grace period @50Hz
    static const int BALL_NEAR_THRESHOLD = 100; // threshold "dekat bola" dalam cm
    
        NodeStatus BallFound()
    {
        xTarget = 300, yTarget = 0;
        if (ballLost(35))
        {
            delayWaitBall = 0;
            // searchBallRectang(-1.6, -1.6, -0.8, 1.6);
		    foundBall = 0;
		    arahGoal = ballDistance = 999;
		    stateCondition = 272;
            	    BallX = BallY = 999;
            BallApproachEntry = 0;
            if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
		    {
		    	initialPos_X = initialPos_Y = 0;
		    	deltaPos_X = cam_x;
		    	deltaPos_Y = cam_y;
		    }	
            return NodeStatus::FAILURE;
        }

        foundBall = 1;
        refreshMoveGrid();
        return NodeStatus::SUCCESS;       
    }
    
    bool bodyTracked = false;
    bool useBodyTracking = false;
    int BodyTrackEntry = 0;
    NodeStatus BodyTrack()
    {
        
        return NodeStatus::FAILURE;
    }
    
    int ballOnLeftGoal = 0;
    int ballOnGoalSide()
    {
        if (robotPos_X > 325)
        {
            if (robotPos_Y < -130)
            {
            	ballOnLeftGoal = 1;
            	return 1;
            } else if (robotPos_Y > 130)
            {
            	ballOnLeftGoal = 0;
            	return 1;
            } else 
            {
            	ballOnLeftGoal = 2; //Bola depan gawang
            	return 1; /////
            }
        } else 
        {
            return 0;
        }
    }
   
    // Sudut tendang khusus ThrowIn berdasarkan posisi di lapangan.
    // Zona X dibagi: area penalty lawan (X>goalAreaMinX), setengah lawan (X>0),
    // setengah sendiri (X<=0). Tendang ke tengah lapangan.
    //
    // Konvensi sudut IMU:
    //   + = ke kiri,  - = ke kanan
    //   Y > 0 = pinggir kanan → harus serong kiri (+)
    //   Y < 0 = pinggir kiri  → harus serong kanan (-)
    // (kalau terbalik di robot, cukup ganti tanda di YAML: angleSideRightOpp, dll)
    int getThrowInKickAngle()
    {
        int X = (int)robotPos_X;
        int Y = (int)robotPos_Y;

        // === AREA GAWANG LAWAN (X > goalAreaMinX) ===
        // Bola di sudut pojok → sudut besar agar masuk ke tengah gawang
        if (X > goalAreaMinX)
        {
            if      (Y < goalCornerLeftMaxY)  return -75; // pojok kiri  → serong kanan kuat
            else if (Y > goalCornerRightMinY) return  75; // pojok kanan → serong kiri kuat
            else if (Y <= 0)                  return -15; // depan gawang kiri  → serong kanan sedikit
            else                              return  15; // depan gawang kanan → serong kiri sedikit
        }

        // === SETENGAH LAPANGAN LAWAN (0 < X <= goalAreaMinX) ===
        // Jauh dari gawang tapi sudah di setengah lawan → sudut sedang
        if (X > 0)
        {
            if      (Y < leftSideMaxY)   return  40; // pinggir kiri  (Y negatif = kiri) → serong kanan
            else if (Y > rightSideMinY)  return -40; // pinggir kanan (Y positif = kanan) → serong kiri
            else                         return   0; // tengah → lurus
        }

        // === SETENGAH LAPANGAN SENDIRI (X <= 0) ===
        // Dekat tengah atau di belakang → sudut lebih kecil
        if      (Y < leftSideMaxY)   return  25; // pinggir kiri  → serong kanan sedikit
        else if (Y > rightSideMinY)  return -25; // pinggir kanan → serong kiri sedikit
        else                         return   0; // tengah → lurus
    }
    
    // Konversi posisi X,Y robot ke nomor grid (0-53)
    int getRobotGrid()
    {
        int tempGridX, tempGridY, GridX, GridY;
        GridX = (int)robotPos_X;
        GridY = (int)robotPos_Y;

        if (GridX >= 450)  GridX = 450;
        else if (GridX <= -450) GridX = -450;
        if (GridY >= 300)  GridY = 300;
        else if (GridY <= -300) GridY = -300;

        tempGridX = ((GridX + 450) / 100);
        tempGridY = ((GridY + 300) / 100);

        if (tempGridX >= 8) tempGridX = 8;
        else if (tempGridX <= 0) tempGridX = 0;
        if (tempGridY >= 5) tempGridY = 5;
        else if (tempGridY <= 0) tempGridY = 0;

        int grid = tempGridY + (6 * tempGridX);
        if (grid <= 0)  grid = 0;
        else if (grid >= 53) grid = 53;
        return grid;
    }
    
    // Sudut tendang berdasarkan posisi robot di lapangan
    int getKickAngleByGrid()
    {
        // 1. Area gawang lawan (X > goalAreaMinX)
        if ((int)robotPos_X > goalAreaMinX)
        {
            if ((int)robotPos_Y < goalCornerLeftMaxY)                   return angleCornerLeft;
            else if ((int)robotPos_Y > goalCornerRightMinY)             return angleCornerRight;
            else if ((int)robotPos_Y <= 0)                              return angleGoalLeft;
            else                                                        return angleGoalRight;
        }
        // 2. Pinggir kiri (Y < leftSideMaxY)
        if ((int)robotPos_Y < leftSideMaxY)
        {
            return ((int)robotPos_X > 0) ? angleSideLeftOpp : angleSideLeftOwn;
        }
        // 3. Pinggir kanan (Y > rightSideMinY)
        if ((int)robotPos_Y > rightSideMinY)
        {
            return ((int)robotPos_X > 0) ? angleSideRightOpp : angleSideRightOwn;
        }
        // 4. Tengah lapangan
        return 0;
    }
    
    NodeStatus Interrupt()
    {
    	if (SecondaryState != 0)
    	{
    		return NodeStatus::SUCCESS;
    	}
    	return NodeStatus::FAILURE;
    }
    
    bool action_walk = false, isKicked = false;
    int BallApproachEntry = 0;
    NodeStatus BallApproach()
    {
        //printf("ballapproach\n");
        if (BallApproachEntry > 5)
        {
            if (body_tracked)
            {
                if  ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    ((ballDistance) < robot2DBall) &&
                    ((ballDistance) < robot3DBall) &&
                    ((ballDistance) < robot4DBall) &&
                    ((ballDistance) < robot5DBall)
                    ) //150
                {
                    state_move_grid = 0;
                    stateCondition = 232;
                    //ballDistance = 1;
                } else {
                    trackBall();
                    if (headTilt > 1.2){
                    stateCondition = 272;
                    }else{
                    
                    followBall(0);
                    }
                }

                if (ballLost(35))
                {
                    Walk(0.0, 0.0, 0.0);
                    delayWaitBall = 0;
                    tracked = body_tracked = false;
                    ballLostCounter++;
		    if (ballLostCounter > BALL_MEMORY_TIMEOUT) {
			hasBallMemory  = false;   // habis waktu → search normal
			ballLostCounter = 0;
		    }
                    searchBallFromMemory();
                } else {
                    trackBall();
                    ballLostCounter = 0;
		    lastBallPanDir = headPan;
		    hasBallMemory  = true;

                    if (tracked)
                    {
                        if (headTilt >= cAktif)
                        {    
                            // bola sudah dekat
                            /*
                            if (robotDirection) 
                            {
                                //printf("...kick\n");
                                if ((msg_yaw < (arahGoal - 15)) || (msg_yaw > (arahGoal + 15))) {
                                    printf("...!!!RobotDirection\n");
                                    robotDirection = false;
                                }

                                else {
                                    if (tendang) 
                                    {
                                        isKicked = action_walk = true;
                                        searchKe = delayWaitBall = 0;
                                        robotDirection = ballPos = tendang = false;
                                    } else {
                                        if (useDribbleMode) {
                                            dribble(arahGoal, ballPositioningSpeed);
                                        } else {
                                            kick(modeKick);
                                        }
                                    }
                                }
                            }*/ 
                            if (robotDirection) 
{
    //printf("...kick\n");
    if ((msg_yaw < (arahGoal - 15)) || (msg_yaw > (arahGoal + 15))) {
        printf("...!!!RobotDirection\n");
        robotDirection = false;
        postImuCooldown = 0; // <<< reset juga kalau arah meleset
    }
    else {
        // <<< COOLDOWN: biarkan kepala stabil dulu setelah selesai rotate
        if (postImuCooldown < POST_IMU_COOLDOWN_MAX) {
            postImuCooldown++;
            Walk(0.0, 0.0, 0.0);
            printf("[PostImu] stabilizing... %d/%d\n", postImuCooldown, POST_IMU_COOLDOWN_MAX);
        } else {
            // kepala sudah stabil, baru eksekusi tendang
            if (tendang) 
            {
                isKicked = action_walk = true;
                searchKe = delayWaitBall = 0;
                robotDirection = ballPos = tendang = false;
                postImuCooldown = 0;
            } else {
                if (useDribbleMode) {
                    dribble(arahGoal, ballPositioningSpeed);
                } else {
                    kick(modeKick);
                }
            }
        }
    }
}
                            else {
                                // AUTO-ADJUST sudut tendang berdasarkan posisi robot di lapangan
                                int gridAngle = getKickAngleByGrid();
                                if (gridAngle != 0)
                                {
                                    // Posisi pinggir/pojok → tendang serong sesuai grid
                                    modeKick = tendangJauh;
                                    arahGoal = gridAngle;
                                    printf(">>> GRID KICK R5: grid=%d X=%.0f Y=%.0f -> arahGoal=%d <<<\n",
                                           getRobotGrid(), (double)robotPos_X, (double)robotPos_Y, gridAngle);
                                    Imu(arahGoal, cSekarang);
                                } else {
                                    // Posisi tengah (gridAngle=0) → gunakan logic posisi gawang
                                    if (ballOnGoalSide())
                                    {
                                        if (ballOnLeftGoal == 1)
                                        {
                                            if (useSideKick)
                                            {
                                                if (msg_yaw > 45) {
                                                    modeKick = tendangJauh;
                                                    arahGoal = angleCornerLeft;
                                                    Imu(angleCornerLeft, cSekarang);
                                                } else {
                                                    modeKick = 3;
                                                    arahGoal = 0;
                                                    Imu(0, cSekarang);
                                                }
                                            } else {
                                                modeKick = tendangJauh;
                                                arahGoal = angleCornerLeft;
                                                Imu(angleCornerLeft, cSekarang);
                                            }
                                        } else if (ballOnLeftGoal == 0) {
                                            if (useSideKick)
                                            {
                                                if (msg_yaw < -45) {
                                                    modeKick = 4;
                                                    arahGoal = angleCornerRight;
                                                    Imu(angleCornerRight, cSekarang);
                                                } else {
                                                    modeKick = 4;
                                                    arahGoal = 0;
                                                    Imu(0, cSekarang);
                                                }
                                            } else {
                                                modeKick = tendangJauh;
                                                arahGoal = angleCornerRight;
                                                Imu(angleCornerRight, cSekarang);
                                            }
                                        } else {    //bola pas depan gawang
                                            if (robotPos_Y <= 0) {
                                                modeKick = tendangJauh;
                                                arahGoal = angleGoalLeft;
                                                Imu(angleGoalLeft, cSekarang);
                                            } else {
                                                modeKick = tendangJauh;
                                                arahGoal = angleGoalRight;
                                                Imu(angleGoalRight, cSekarang);
                                            }
                                        }
                                    } else {
                                        // Tengah lapangan → tendang lurus
                                        if (useSideKick)
                                        {
                                            if (msg_yaw > 45) {
                                                modeKick = 4;
                                                arahGoal = 85;
                                                Imu(85, cSekarang);
                                            } else if (msg_yaw < -45) {
                                                modeKick = 3;
                                                arahGoal = -85;
                                                Imu(-85, cSekarang);
                                            } else {
                                                modeKick = tendangJauh;
                                                arahGoal = 0;
                                                Imu(0, cSekarang);
                                            }
                                        } else {
                                            modeKick = tendangJauh;
                                            arahGoal = 0;
                                            Imu(0, cSekarang);
                                        }
                                    }
                                }
                            }
                        } else {    // bola masih jauh
                            if (!checkAndAvoidRobot()) { //ballavoid
                                followBall(0);
                            }
                            robotDirection = ballPos = tendang = false;
                        }
                    } else {    // lockBall
                        if (delayWaitBall > 5)
                        {
                            tracked = true;
                        } else {
                            delayWaitBall++;
                            Walk(0.0, 0.0, 0.0);
                        }
                    }    
                }
            } else {
                if (ballLost(35))
                {
                    delayWaitBall = 0;
                    tracked = false;
                    Walk(0.0, 0.0, 0.0);
                    if (ballLostCounter > BALL_MEMORY_TIMEOUT) {
			hasBallMemory  = false;   // habis waktu → search normal
			ballLostCounter = 0;
		    }
                    searchBallFromMemory();
                } else {
                    trackBall();
                    ballLostCounter = 0;
		    lastBallPanDir = headPan;
		    hasBallMemory  = true;
                    if (delayWaitBall > 20)
                    {
                        newBodyTracking();
                    } else {
                        Walk(0.0, 0.0, 0.0);
                        delayWaitBall++;
                    }
                }
            }
        }
        else
        {
            action_kick = false;
            isKicked = body_tracked = tracked = tendang = ballPos = robotDirection = doneWalk = false;
            RotateToGoalEntry = SearchAfterKickEntry = RobotPositioningEntry = WalkSearchBallEntry = relaxEntry = KickEntry = 0;
            Walk(0.0, 0.0, 0.0);
            BallApproachEntry++;
        }
        return NodeStatus::FAILURE;
    }
    
    // Jarak aman defending dari bola throw-in (cm).
    // Aturan GC2026: menunggu di luar center-circle-radius (~150 cm lapangan middle).
    

    NodeStatus StateThrowIn()
    {
        // Jika setPlay sudah bukan 4 (Throw-In), reset dan keluar
        if (secondaryInfo[0] != 4) {
            throwInPhase       = 0;
            throwInKicked      = false;
            throwInWait        = 0;
            throwInBodyAligned = 0;
            robotDirection     = false;
            BallApproachEntry  = 0;
            return NodeStatus::FAILURE;
        }

        // == Phase 1: STOPPED — wasit sedang posisikan bola, semua robot DIAM ==
        if (Stopped == 1) {
            throwInPhase       = 1;
            throwInKicked      = false;
            throwInWait        = 0;
            throwInBodyAligned = 0;
            motion("0");
            printf("[ThrowIn] Phase1-STOPPED: bola diposisikan wasit, diam\n");
            return NodeStatus::SUCCESS;
        }

        // == setPlay=4, stopped=0 → GC sudah resume, mulai eksekusi ==
        if (throwInPhase != 2 && throwInPhase != 3) {
            throwInPhase       = 2;
            throwInWait        = 0;
            throwInKicked      = false;
            throwInBodyAligned = 0;
            robotDirection     = false;
            BallApproachEntry  = 0;
            printf("[ThrowIn] Phase2-EXECUTING: kicking=%d barelang=%d\n",
                   KickOff, barelang_color);
        }

        // == Phase 3: sudah tendang, tunggu setPlay kembali 0 ==
        if (throwInPhase == 3) {
            motion("0");
            printf("[ThrowIn] Phase3-DONE: tunggu GC reset setPlay\n");
            return NodeStatus::SUCCESS;
        }

        // == Phase 2: EXECUTING ==
        // Delay kecil saat pertama masuk
        if (throwInWait < 5) {
            motion("0");
            throwInWait++;
            return NodeStatus::SUCCESS;
        }

        // =========================================================
        // TIM KITA YANG TENDANG (attacking)
        // → approach bola → sesuaikan sudut ke arah gawang lawan → tendang
        // KickOff == barelang_color = GC menentukan tim kita yang tendang
        // =========================================================
        if (KickOff == barelang_color) {
            printf("[ThrowIn] Attacking: SecTime=%d dist=%d yaw=%d posY=%.0f\n",
                   SecondaryTime, ballDistance, msg_yaw, robotPos_Y);

            if (throwInKicked) {
                throwInPhase = 3;
                motion("0");
                return NodeStatus::SUCCESS;
            }

            if (ballLost(35)) {
                searchBallBreak();
            } else {
                trackBall();
                if (ballDistance > 40) {
                    // Masih jauh — dekati bola
                    followBall(0);
                } else {
                    // Sudah dekat — sesuaikan sudut tendang ke arah gawang lawan lalu tendang
                    if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) {
                        if (!robotDirection) {
                            // Hitung arah tendang berdasarkan posisi robot di lapangan:
                            // Throw-in dari touchline kiri  (Y > 100)  → tendang sedikit ke kanan (-10°)
                            // Throw-in dari touchline kanan (Y < -100) → tendang sedikit ke kiri  (+10°)
                            // Area tengah lapangan → tendang lurus ke gawang (0°)
                            if (robotPos_Y > 100) {
                                arahGoal = -10;
                            } else if (robotPos_Y < -100) {
                                arahGoal = 10;
                            } else {
                                arahGoal = 0;
                            }
                            modeKick = tendangJauh;
                            printf("[ThrowIn] Attacking: rotate to arahGoal=%d\n", arahGoal);
                            Imu(arahGoal, cSekarang);
                        } else {
                            kick(modeKick);
                            throwInKicked = true;
                            throwInPhase  = 3;
                            printf("[ThrowIn] KICK executed! arahGoal=%d mode=%d\n", arahGoal, modeKick);
                        }
                    } else {
                        followBall(0);
                    }
                }
            }
        }

        // =========================================================
        // TIM LAWAN YANG TENDANG (defending)
        // Alur:
        //   1. Bola belum kelihatan → WalkSearchBall: jalan pelan + scan kepala
        //   2. Bola kelihatan       → track kepala, putar badan sejajar bola
        //   3. Badan sejajar + jauh → maju mendekati bola letak throw-in
        //   4. Sudah di jarak aman (≈1-1.5 m) → berhenti, tetap hadap bola
        // =========================================================
        else {
            printf("[ThrowIn] Defending: SecTime=%d dist=%d bodyAligned=%d\n",
                   SecondaryTime, ballDistance, throwInBodyAligned);

            if (ballLost(50)) {
                // Bola belum terlihat → WalkSearchBall:
                // Jalan pelan ke depan sambil scan kepala kiri-kanan mencari bola
                throwInBodyAligned = 0;
                tiltSearchBall(0.0);          // scan kepala up-down (mencari bola)
                Walk(aruku * 0.5, 0.0, 0.0); // jalan pelan sambil cari bola
            } else {
                // Bola terlihat — 1) track kepala dulu
                trackBall();

                if (ballDistance > THROW_IN_SAFE_DISTANCE) {
                    // ---- Belum di jarak aman: sejajarkan badan ke arah bola dulu ----
                    if (headPan > 0.2) {
                        // Bola di kiri → putar kiri hingga headPan mendekati 0
                        Walk(0.0, 0.0, 0.1);
                        throwInBodyAligned = 0;
                    } else if (headPan < -0.2) {
                        // Bola di kanan → putar kanan hingga headPan mendekati 0
                        Walk(0.0, 0.0, -0.1);
                        throwInBodyAligned = 0;
                    } else {
                        // Badan sudah sejajar dengan bola → counter stabilisasi
                        throwInBodyAligned++;
                        if (throwInBodyAligned > 5) {
                            // Maju mendekati bola (berhenti begitu ballDistance <= THROW_IN_SAFE_DISTANCE)
                            Walk(aruku, 0.0, 0.0);
                        } else {
                            // Tunggu sebentar sebelum maju (stabilisasi orientasi)
                            Walk(0.0, 0.0, 0.0);
                        }
                    }
                } else {
                    // ---- Sudah di jarak aman (~1-1.5 m dari bola) → BERHENTI ----
                    // Tetap putar badan agar selalu menghadap bola, tapi tidak maju lagi
                    throwInBodyAligned = 0;
                    if (headPan > 0.2) {
                        Walk(0.0, 0.0, 0.08);  // putar pelan ke kiri
                    } else if (headPan < -0.2) {
                        Walk(0.0, 0.0, -0.08); // putar pelan ke kanan
                    } else {
                        motion("0"); // Diam — posisi aman, badan lurus ke bola
                    }
                    printf("[ThrowIn] Defending: di jarak aman (dist=%d <= %d), diam\n",
                           ballDistance, THROW_IN_SAFE_DISTANCE);
                }
            }
        }

        return NodeStatus::SUCCESS;
    }
    
    NodeStatus StateDirectFreeKick()
    {
        if (secondaryInfo[0] == 1) {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    // Indirect Free Kick (ball must touch another player before goal)
    NodeStatus StateIndirectFreeKick()
    {
        if (secondaryInfo[0] == 2) {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    // Penalty Kick
    NodeStatus StatePenaltyKick()
    {
        if (secondaryInfo[0] == 3) {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }
    
    int  cornerKickPhase  = 0;
    bool cornerKickKicked = false;
    int  cornerKickWait   = 0;

    NodeStatus StateCornerKick()
    {
        // Jika setPlay sudah bukan 6, reset
        if (secondaryInfo[0] != 6) {
            cornerKickPhase  = 0;
            cornerKickKicked = false;
            cornerKickWait   = 0;
            robotDirection   = false;
            BallApproachEntry = 0;
            return NodeStatus::FAILURE;
        }

        // Phase 1: stopped=1 → wasit taruh bola di sudut lapangan, SEMUA DIAM
        if (Stopped == 1) {
            cornerKickPhase  = 1;
            cornerKickKicked = false;
            cornerKickWait   = 0;
            motion("0");
            printf("[CornerKick] Phase1-STOPPED: wasit posisikan bola di sudut, diam\n");
            return NodeStatus::SUCCESS;
        }

        // stopped=0 → GC resume, mulai eksekusi
        if (cornerKickPhase != 2 && cornerKickPhase != 3) {
            cornerKickPhase  = 2;
            cornerKickWait   = 0;
            cornerKickKicked = false;
            robotDirection   = false;
            BallApproachEntry = 0;
            printf("[CornerKick] Phase2-EXECUTING: kicking=%d barelang=%d\n",
                   KickOff, barelang_color);
        }

        // Phase 3: sudah tendang, tunggu setPlay=0
        if (cornerKickPhase == 3) {
            motion("0");
            printf("[CornerKick] Phase3-DONE: tunggu GC reset setPlay\n");
            return NodeStatus::SUCCESS;
        }

        // Delay kecil saat masuk phase 2
        if (cornerKickWait < 5) {
            motion("0");
            cornerKickWait++;
            return NodeStatus::SUCCESS;
        }

        // Tim kita yang tendang (attacking)
        if (KickOff == barelang_color) {
            printf("[CornerKick] Attacking: SecTime=%d\n", SecondaryTime);

            if (cornerKickKicked) {
                cornerKickPhase = 3;
                motion("0");
                return NodeStatus::SUCCESS;
            }

            if (ballLost(35)) {
                searchBallBreak();
            } else {
                trackBall();
                if (ballDistance > 40) {
                    followBall(0);
                } else {
                    if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) {
                        if (!robotDirection) {
                            // Corner kick: tendang ke tengah lapangan (ke arah gawang)
                            if (msg_yaw > 0) sudutTendang = -45;
                            else             sudutTendang =  45;
                            Imu(sudutTendang, cSekarang);
                        } else {
                            kick(modeKick);
                            cornerKickKicked = true;
                            cornerKickPhase  = 3;
                            printf("[CornerKick] KICK executed!\n");
                        }
                    } else {
                        followBall(0);
                    }
                }
            }
        }
        // Tim lawan (defending) — jaga jarak center-circle-radius dari bola
        else {
            printf("[CornerKick] Defending: SecTime=%d\n", SecondaryTime);

            // § 13.4.2: Defend robots must maintain center-circle-radius distance (≈150cm)
            if (Ball_X != -1 && Ball_Y != -1) {  // Ball detected via vision
                int dx = robotPos_X - Ball_X;
                int dy = robotPos_Y - Ball_Y;
                int distanceToBall = sqrt(dx*dx + dy*dy);

                if (distanceToBall < 150) {  // Inside center circle radius
                    printf("[CornerKick] Defending: ILLEGAL POSITION - too close to ball! distance=%d < 150\n",
                           distanceToBall);
                    motion("0");  // Stop to maintain distance
                } else if (!ballLost(50)) {
                    trackBall();
                } else {
                    motion("0");
                }
            } else {
                // Ball not detected, maintain defensive stance
                motion("0");
            }
        }

        return NodeStatus::SUCCESS;
    }
    
    NodeStatus WalkTowardsBall()
    {
        if (checkAndAvoidRobot()) return NodeStatus::FAILURE;
    }

// ======================================================
// DEFEND POSITION
// ======================================================

double getDefendYPosition()
{
    int activeCount = 0;
    int myIndex = 0;
    bool foundSelf = false;

    int statuses[] = {
        robot1Status,
        robot2Status,
        robot3Status,
        robot4Status,
        robot5Status,
        robot6Status
    };

    int ids[] = {
        1,
        2,
        3,
        4,
        5,
        6
    };

    // hitung robot aktif
    for (int i = 0; i < 6; i++)
    {
        if (statuses[i] == 1)
        {
            // simpan index robot sendiri
            if (ids[i] == robotNumber)
            {
                foundSelf = true;
                myIndex = activeCount;
            }

            activeCount++;
        }
    }

    // fallback
    if (!foundSelf || activeCount == 0)
    {
        return 0.0;
    }

    // range defend Y
    double minY = -130.0;
    double maxY = 130.0;

    double range = maxY - minY;

    // bagi rata posisi robot
    double step = range / (activeCount + 1);

    // posisi defend robot
    double targetY = minY + step * (myIndex + 1);

    // clamp safety
    if (targetY > maxY)
        targetY = maxY;

    if (targetY < minY)
        targetY = minY;

    return targetY;
}

// ======================================================
// DEFEND STATE
// ======================================================

bool defend_lock = false;
bool defend_arrived = false;
bool defend_position_locked = false;

// counter hilang executor
int defend_lost_counter = 0;

// timeout executor hilang
const int DEFEND_LOST_TIMEOUT = 45;

// target defend
int defendTargetX = -200;
int defendTargetY = 0;

// simpan executor terakhir
int lastExecutor = -1;

// ======================================================
// GET EXECUTOR
// ======================================================

int getExecutorID()
{
    if (robot1DBall == 232) return 1;
    if (robot2DBall == 232) return 2;
    if (robot3DBall == 232) return 3;
    if (robot4DBall == 232) return 4;
    if (robot5DBall == 232) return 5;
    if (robot6DBall == 232) return 6;

    return -1;
}

// ======================================================
// DEFEND
// ======================================================

NodeStatus Defend()
{
    // ==================================================
    // CEK EXECUTOR
    // ==================================================

    int executorID = getExecutorID();

    bool executor_exist = (executorID != -1);

    // ==================================================
    // ADA EXECUTOR
    // ==================================================

    if (executor_exist)
    {
        // simpan executor terakhir
        lastExecutor = executorID;

        // lock defend
        defend_lock = true;

        // reset counter hilang
        defend_lost_counter = 0;
    }
    else
    {
        // executor hilang sementara
        defend_lost_counter++;

        // benar-benar hilang
        if (defend_lost_counter > DEFEND_LOST_TIMEOUT)
        {
            lastExecutor = -1;

            defend_lock = false;
            defend_arrived = false;
            defend_position_locked = false;

            motion("0");

            return NodeStatus::FAILURE;
        }
    }

    // ==================================================
    // BELUM LOCK
    // ==================================================

    if (!defend_lock)
    {
        return NodeStatus::FAILURE;
    }

    // ==================================================
    // TARGET DEFEND
    // ==================================================

    defendTargetX = -200;
    defendTargetY = getDefendYPosition();

    // ==================================================
    // WAJIB KE TITIK DEFEND DULU
    // ==================================================

    if (!defend_position_locked)
    {
        walkTarget(defendTargetX, defendTargetY);

        // sudah sampai
        if (abs(robotPos_X - defendTargetX) < 20 &&
            abs(robotPos_Y - defendTargetY) < 20)
        {
            defend_position_locked = true;
            defend_arrived = true;

            motion("0");
        }

        return NodeStatus::RUNNING;
    }

    // ==================================================
    // SUDAH LOCK TITIK DEFEND
    // ==================================================

    motion("0");

    // ==================================================
    // TRACK BALL
    // ==================================================

    if (!ballLost(20))
    {
        trackBall();
    }
    else
    {
        headMove(0.0, -1.0);
    }

    // ==================================================
    // TETAP DEFEND
    // ==================================================

    return NodeStatus::RUNNING;
}
    
    //MODE 2
    int  throwInPhase        = 0;
    bool throwInKicked       = false;
    int  throwInWait         = 0;
    int  throwInBodyAligned  = 0; // counter stabilisasi badan sejajar bola (defending)
    bool throwInSupportLatched = false;
    int  throwInSupportGrid = 0;
    int  throwInSupportX = 0;
    int  throwInSupportY = 0;
    int  throwInSupportMode = 0;
    bool throwInAnchorLocked = false;
    int  throwInAnchorOwner = 0;
    int  throwInAnchorBallX = 0;
    int  throwInAnchorBallY = 0;
    bool goalKickExitZone   = false; // flag: robot sedang keluar dari zona penalti saat defend
    int  goalKickBallSide   = 0;     // -1=kiri, +1=kanan (persist antar tick)
    int  goalKickDefendPhase = 0;    // 0=idle, 1=align_body (samping), 2=approach_target (mundur+adaptif)
    int  goalKickAlignWait  = 0;     // counter untuk delay/stability antar phase
    int  goalKickYawStableCount = 0; // counter yaw stabil di sekitar 0 sebelum masuk phase mundur
    
    bool action_kick = false;
    bool robotFinalDirection = false;
    int RotateToGoalEntry = 0, robotFinalDirectionEntry = 0;
    static constexpr int THROW_IN_SAFE_DISTANCE = 150;
    NodeStatus RotateToGoal()
    {
        // =========================================================
        // GC2026:
        //   secondaryInfo[0] = setPlay  (0=NONE, 4=THROW_IN, 5=GOAL_KICK, 6=CORNER_KICK)
        //   KickOff          = kickingTeam (our team number, atau 255=none)
        //   Stopped          = 1 → wasit stop play, semua robot WAJIB diam
        //                      0 → resume / eksekusi
        //
        //   Return SUCCESS → Sequence lanjut ke StateKickOff / normal play
        //   Return FAILURE → blok normal play (tree restart tiap tick, state machine tetap running
        //                    karena state tersimpan di member variables)
        // =========================================================

        // === STOP EMERGENCY: State 3 (Playing) + Stopped = 1 + No Set Play ===
        // Robot harus diam (motion 8) ketika wasit menekan stop emergency
        if (State == 3 && Stopped == 1 && secondaryInfo[0] == 0) {
            motion("0");  // robot diam
            printf("[STOP EMERGENCY] Robot stopped during play. State=%d Stopped=%d\n", State, Stopped);
            return NodeStatus::FAILURE;
        }

        // === TIDAK ADA SET PLAY: normal game → lanjutkan ===
        if (secondaryInfo[0] == 0) {
            // Reset state machine jika baru selesai dari set play
            if (throwInPhase != 0) {
                throwInPhase       = 0;
                throwInKicked      = false;
                throwInWait        = 0;
                throwInBodyAligned = 0;
                robotDirection     = false;
                BallApproachEntry  = 0;
            }
            return NodeStatus::SUCCESS;
        }

        // =========================================================
        // DIRECT (setPlay = 1)
        // =========================================================
        if (secondaryInfo[0] == 1) {
            // -- STOPPED (Stopped=1): wasit posisikan bola --
            if (Stopped == 1) {
                throwInPhase       = 1;
                throwInKicked      = false;
                throwInWait        = 0;
                throwInBodyAligned = 0;
                motion("8");  // badan diam
                if (!ballLost(50))
                    trackBall();     // bola sudah ketemu → track
                else
                    SearchBall(1);   // scan penuh: atas-bawah + kanan-kiri
                printf("[GoalKick] STOPPED: searchball, KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
                return NodeStatus::FAILURE;
            }

            // -- Transisi ke Phase 2 setelah Resume (stopped→0) --
            if (throwInPhase != 2 && throwInPhase != 3) {
                throwInPhase       = 2;
                throwInWait        = 0;
                throwInKicked      = false;
                throwInBodyAligned = 0;
                robotDirection     = false;
                BallApproachEntry  = 0;
                printf("[GoalKick] RESUME-EXECUTING: KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
            }

            // -- Phase 3: sudah tendang, tunggu setPlay reset ke 0 --
            if (throwInPhase == 3) {
                motion("8");
                printf("[GoalKick] DONE: tunggu setPlay=0\n");
                return NodeStatus::FAILURE;
            }

            // -- Delay kecil setelah pertama masuk phase 2 --
            if (throwInWait < 5) {
                motion("8");
                throwInWait++;
                return NodeStatus::FAILURE;
            }

            // TIM KITA YANG TENDANG (attacking)
            if (KickOff == barelang_color) {
                if (!isNearestExecutorForThrowIn()) {
                    int defendX = 0;
                    int defendY = 0;
                    if (robotNumber == 2) {
                        defendX = -100;
                        defendY = -100;
                    } else if (robotNumber == 4) {
                        defendX = -100;
                        defendY = 100;
                    }else if (robotNumber == 1) {
                        defendX = -100;
                        defendY = 0;
                    }
                     else if (robotNumber == 5) {
                        defendX = 50;
                        defendY = 0;
                    } else if (robotNumber == 3) {
                        defendX = -100;
                        defendY = 100;
                    } else if (robotNumber == 6) {
                        defendX = -100;
                        defendY = -100;
                    } else {
                        defendX = 0;
                        defendY = 0;
                    }

                    new_out_pos(defendX, defendY, true);

                    int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;

                    if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
                        trackHeadToFieldPoint(teamBallX, teamBallY);
                        const double bodyDeg = atan2((double)teamBallY - robotPos_Y, (double)teamBallX - robotPos_X) * 180.0 / PI;
                        rotateBodyImuNew((int)bodyDeg);
                    } else if (!ballLost(50)) {
                        trackBall();
                    } else {
                        SearchBall(1);
                    }

                    printf("[GoalKick] SupportDefend(our): target=(%d,%d)\n", defendX, defendY);
                    return NodeStatus::FAILURE;
                }

                printf("[GoalKick] Attacking: dist=%d yaw=%d posY=%.0f headTilt=%.2f\n",
                       ballDistance, msg_yaw, (double)robotPos_Y, (double)headTilt);

                if (ballLost(35)) {
                    Walk(0.0, 0.0, 0.0);
                    delayWaitBall = 0;
                    throwInBodyAligned = 0;
                    SearchBall(1);
                } else {
                    throwInBodyAligned = 0;  // reset counter
                    trackBall();

                    if (headTilt < cAktif) {
                        followBall(0);
                        throwInKicked = false;  // reset
                        robotDirection = false;
                        printf("[GoalKick] Attacking: followBall (headTilt=%.2f)\n", headTilt);
                    } else {
                        if (robotDirection) {
                            if ((msg_yaw < (arahGoal - 15)) || (msg_yaw > (arahGoal + 15))) {
                                robotDirection = false;
                                printf("[GoalKick] Adjusting yaw: current=%d target=%d\n", msg_yaw, arahGoal);
                            } else {
                                if (tendang) {
                                    throwInKicked = true;
                                    robotDirection = ballPos = tendang = false;
                                    printf("[GoalKick] KICK! arahGoal=%d mode=%d\n", arahGoal, modeKick);
                                } else {
                                    kick(modeKick);
                                }
                            }
                        } else {
                            int gridAngle = getKickAngleByGrid();
                            if (gridAngle != 0) {
                                modeKick = tendangJauh;
                                arahGoal = gridAngle;
                                printf("[GoalKick] GRID KICK: grid=%d X=%.0f Y=%.0f -> arahGoal=%d\n",
                                       getRobotGrid(), (double)robotPos_X, (double)robotPos_Y, gridAngle);
                                Imu(arahGoal, cSekarang);
                            } else {
                                if (ballOnGoalSide()) {
                                    if (ballOnLeftGoal == 1) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerLeft;
                                        Imu(angleCornerLeft, cSekarang);
                                    } else if (ballOnLeftGoal == 0) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerRight;
                                        Imu(angleCornerRight, cSekarang);
                                    } else {
                                        if (robotPos_Y <= 0) {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalLeft;
                                            Imu(angleGoalLeft, cSekarang);
                                        } else {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalRight;
                                            Imu(angleGoalRight, cSekarang);
                                        }
                                    }
                                } else {
                                    modeKick = tendangJauh;
                                    arahGoal = 0;
                                    Imu(0, cSekarang);
                                }
                            }
                            robotDirection = true;
                            printf("[GoalKick] Rotate to arahGoal=%d\n", arahGoal);
                        }
                    }
                }
            }
            // TIM LAWAN YANG TENDANG (defending)
            else {
                throwInKicked = false;
                throwInBodyAligned = 0;
                action_kick = false;
                robotDirection = false;
                robotFinalDirection = false;
                tendang = false;
                ballPos = false;
                BallApproachEntry = 0;
                int defendX = 0;
                int defendY = 0;

                if (robotNumber == 2) {
                    defendX = -250;
                    defendY = -100;
                } else if (robotNumber == 4) {
                    defendX = -250;
                    defendY = 100;
                }else if (robotNumber == 1) {
                    defendX = -250;
                    defendY = 0;
                } 
                else if (robotNumber == 5) {
                    defendX = -50;
                    defendY = 0;
                } else if (robotNumber == 3) {
                    defendX = -250;
                    defendY = 100;
                } else if (robotNumber == 6) {
                    defendX = -250;
                    defendY = -100;
                } else {
                    defendX = -250;
                    defendY = 0;
                }

                new_out_pos(defendX, defendY, true);
                if (!ballLost(50)) {
                    trackBall();
                } else {
                    SearchBall(1);
                }
                printf("[GoalKick] Defending(opponent): target=(%d,%d)\n", defendX, defendY);
            }

            return NodeStatus::FAILURE;
        }

        // =========================================================
        // INDIRECT (setPlay = 2)
        // =========================================================
        if (secondaryInfo[0] == 2) {
            // -- STOPPED (Stopped=1): wasit posisikan bola --
            if (Stopped == 1) {
                throwInPhase       = 1;
                throwInKicked      = false;
                throwInWait        = 0;
                throwInBodyAligned = 0;
                motion("8");  // badan diam
                if (!ballLost(50))
                    trackBall();     // bola sudah ketemu → track
                else
                    SearchBall(1);   // scan penuh: atas-bawah + kanan-kiri
                printf("[GoalKick] STOPPED: searchball, KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
                return NodeStatus::FAILURE;
            }

            // -- Transisi ke Phase 2 setelah Resume (stopped→0) --
            if (throwInPhase != 2 && throwInPhase != 3) {
                throwInPhase       = 2;
                throwInWait        = 0;
                throwInKicked      = false;
                throwInBodyAligned = 0;
                robotDirection     = false;
                BallApproachEntry  = 0;
                printf("[GoalKick] RESUME-EXECUTING: KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
            }

            // -- Phase 3: sudah tendang, tunggu setPlay reset ke 0 --
            if (throwInPhase == 3) {
                motion("8");
                printf("[GoalKick] DONE: tunggu setPlay=0\n");
                return NodeStatus::FAILURE;
            }

            // -- Delay kecil setelah pertama masuk phase 2 --
            if (throwInWait < 5) {
                motion("8");
                throwInWait++;
                return NodeStatus::FAILURE;
            }

            // TIM KITA YANG TENDANG (attacking)
            if (KickOff == barelang_color) {
                if (!isNearestExecutorForThrowIn()) {
                    int defendX = 0;
                    int defendY = 0;
                    if (robotNumber == 2) {
                        defendX = -100;
                        defendY = -100;
                    } else if (robotNumber == 4) {
                        defendX = -100;
                        defendY = 100;
                    }else if (robotNumber == 1) {
                        defendX = -100;
                        defendY = 0;
                    }
                     else if (robotNumber == 5) {
                        defendX = 50;
                        defendY = 0;
                    } else if (robotNumber == 3) {
                        defendX = -100;
                        defendY = 100;
                    } else if (robotNumber == 6) {
                        defendX = -100;
                        defendY = -100;
                    } else {
                        defendX = 0;
                        defendY = 0;
                    }

                    new_out_pos(defendX, defendY, true);

                    int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;

                    if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
                        trackHeadToFieldPoint(teamBallX, teamBallY);
                        const double bodyDeg = atan2((double)teamBallY - robotPos_Y, (double)teamBallX - robotPos_X) * 180.0 / PI;
                        rotateBodyImuNew((int)bodyDeg);
                    } else if (!ballLost(50)) {
                        trackBall();
                    } else {
                        SearchBall(1);
                    }

                    printf("[GoalKick] SupportDefend(our): target=(%d,%d)\n", defendX, defendY);
                    return NodeStatus::FAILURE;
                }

                printf("[GoalKick] Attacking: dist=%d yaw=%d posY=%.0f headTilt=%.2f\n",
                       ballDistance, msg_yaw, (double)robotPos_Y, (double)headTilt);

                if (ballLost(35)) {
                    Walk(0.0, 0.0, 0.0);
                    delayWaitBall = 0;
                    throwInBodyAligned = 0;
                    SearchBall(1);
                } else {
                    throwInBodyAligned = 0;  // reset counter
                    trackBall();

                    if (headTilt < cAktif) {
                        followBall(0);
                        throwInKicked = false;  // reset
                        robotDirection = false;
                        printf("[GoalKick] Attacking: followBall (headTilt=%.2f)\n", headTilt);
                    } else {
                        if (robotDirection) {
                            if ((msg_yaw < (arahGoal - 15)) || (msg_yaw > (arahGoal + 15))) {
                                robotDirection = false;
                                printf("[GoalKick] Adjusting yaw: current=%d target=%d\n", msg_yaw, arahGoal);
                            } else {
                                if (tendang) {
                                    throwInKicked = true;
                                    robotDirection = ballPos = tendang = false;
                                    printf("[GoalKick] KICK! arahGoal=%d mode=%d\n", arahGoal, modeKick);
                                } else {
                                    kick(modeKick);
                                }
                            }
                        } else {
                            int gridAngle = getKickAngleByGrid();
                            if (gridAngle != 0) {
                                modeKick = tendangJauh;
                                arahGoal = gridAngle;
                                printf("[GoalKick] GRID KICK: grid=%d X=%.0f Y=%.0f -> arahGoal=%d\n",
                                       getRobotGrid(), (double)robotPos_X, (double)robotPos_Y, gridAngle);
                                Imu(arahGoal, cSekarang);
                            } else {
                                if (ballOnGoalSide()) {
                                    if (ballOnLeftGoal == 1) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerLeft;
                                        Imu(angleCornerLeft, cSekarang);
                                    } else if (ballOnLeftGoal == 0) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerRight;
                                        Imu(angleCornerRight, cSekarang);
                                    } else {
                                        if (robotPos_Y <= 0) {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalLeft;
                                            Imu(angleGoalLeft, cSekarang);
                                        } else {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalRight;
                                            Imu(angleGoalRight, cSekarang);
                                        }
                                    }
                                } else {
                                    modeKick = tendangJauh;
                                    arahGoal = 0;
                                    Imu(0, cSekarang);
                                }
                            }
                            robotDirection = true;
                            printf("[GoalKick] Rotate to arahGoal=%d\n", arahGoal);
                        }
                    }
                }
            }
            // TIM LAWAN YANG TENDANG (defending)
            else {
                throwInKicked = false;
                throwInBodyAligned = 0;
                action_kick = false;
                robotDirection = false;
                robotFinalDirection = false;
                tendang = false;
                ballPos = false;
                BallApproachEntry = 0;
                int defendX = 0;
                int defendY = 0;

                if (robotNumber == 2) {
                    defendX = -250;
                    defendY = -100;
                } else if (robotNumber == 4) {
                    defendX = -250;
                    defendY = 100;
                }else if (robotNumber == 1) {
                    defendX = -250;
                    defendY = 0;
                }
                 else if (robotNumber == 5) {
                    defendX = -50;
                    defendY = 0;
                } else if (robotNumber == 3) {
                    defendX = -250;
                    defendY = 100;
                } else if (robotNumber == 6) {
                    defendX = -250;
                    defendY = -100;
                } else {
                    defendX = -250;
                    defendY = 0;
                }

                new_out_pos(defendX, defendY, true);
                if (!ballLost(50)) {
                    trackBall();
                } else {
                    SearchBall(1);
                }
                printf("[GoalKick] Defending(opponent): target=(%d,%d)\n", defendX, defendY);
            }

            return NodeStatus::FAILURE;
        }
        
        // =========================================================
        // PinaltyKick (setPlay = 3)
        // =========================================================
        if (secondaryInfo[0] == 3) {
            // -- STOPPED (Stopped=1): wasit posisikan bola --
            if (Stopped == 1) {
                throwInPhase       = 1;
                throwInKicked      = false;
                throwInWait        = 0;
                throwInBodyAligned = 0;
                motion("8");  // badan diam
                if (!ballLost(50))
                    trackBall();     // bola sudah ketemu → track
                else
                    SearchBall(1);   // scan penuh: atas-bawah + kanan-kiri
                printf("[GoalKick] STOPPED: searchball, KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
                return NodeStatus::FAILURE;
            }

            // -- Transisi ke Phase 2 setelah Resume (stopped→0) --
            if (throwInPhase != 2 && throwInPhase != 3) {
                throwInPhase       = 2;
                throwInWait        = 0;
                throwInKicked      = false;
                throwInBodyAligned = 0;
                robotDirection     = false;
                BallApproachEntry  = 0;
                printf("[GoalKick] RESUME-EXECUTING: KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
            }

            // -- Phase 3: sudah tendang, tunggu setPlay reset ke 0 --
            if (throwInPhase == 3) {
                motion("8");
                printf("[GoalKick] DONE: tunggu setPlay=0\n");
                return NodeStatus::FAILURE;
            }

            // -- Delay kecil setelah pertama masuk phase 2 --
            if (throwInWait < 5) {
                motion("8");
                throwInWait++;
                return NodeStatus::FAILURE;
            }

            // TIM KITA YANG TENDANG (attacking)
            if (KickOff == barelang_color) {
                if (!isNearestExecutorForThrowIn()) {
                    int defendX = 0;
                    int defendY = 0;
                    
                    if (robotNumber == 2) {
                        defendX = -100;
                        defendY = -100;
                    } else if (robotNumber == 4) {
                        defendX = -100;
                        defendY = 100;
                    }else if (robotNumber == 1) {
                        defendX = -100;
                        defendY = 0;
                    }
                     else if (robotNumber == 5) {
                        defendX = 50;
                        defendY = 0;
                    } else if (robotNumber == 3) {
                        defendX = -100;
                        defendY = 100;
                    } else if (robotNumber == 6) {
                        defendX = -100;
                        defendY = -100;
                    } else {
                        defendX = 0;
                        defendY = 0;
                    }

                    new_out_pos(defendX, defendY, true);

                    int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;

                    if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
                        trackHeadToFieldPoint(teamBallX, teamBallY);
                        const double bodyDeg = atan2((double)teamBallY - robotPos_Y, (double)teamBallX - robotPos_X) * 180.0 / PI;
                        rotateBodyImuNew((int)bodyDeg);
                    } else if (!ballLost(50)) {
                        trackBall();
                    } else {
                        SearchBall(1);
                    }

                    printf("[GoalKick] SupportDefend(our): target=(%d,%d)\n", defendX, defendY);
                    return NodeStatus::FAILURE;
                }

                printf("[GoalKick] Attacking: dist=%d yaw=%d posY=%.0f headTilt=%.2f\n",
                       ballDistance, msg_yaw, (double)robotPos_Y, (double)headTilt);

                if (ballLost(35)) {
                    Walk(0.0, 0.0, 0.0);
                    delayWaitBall = 0;
                    throwInBodyAligned = 0;
                    SearchBall(1);
                } else {
                    throwInBodyAligned = 0;  // reset counter
                    trackBall();

                    if (headTilt < cAktif) {
                        followBall(0);
                        throwInKicked = false;  // reset
                        robotDirection = false;
                        printf("[GoalKick] Attacking: followBall (headTilt=%.2f)\n", headTilt);
                    } else {
                        if (robotDirection) {
                            if ((msg_yaw < (arahGoal - 15)) || (msg_yaw > (arahGoal + 15))) {
                                robotDirection = false;
                                printf("[GoalKick] Adjusting yaw: current=%d target=%d\n", msg_yaw, arahGoal);
                            } else {
                                if (tendang) {
                                    throwInKicked = true;
                                    robotDirection = ballPos = tendang = false;
                                    printf("[GoalKick] KICK! arahGoal=%d mode=%d\n", arahGoal, modeKick);
                                } else {
                                    kick(modeKick);
                                }
                            }
                        } else {
                            int gridAngle = getKickAngleByGrid();
                            if (gridAngle != 0) {
                                modeKick = tendangJauh;
                                arahGoal = gridAngle;
                                printf("[GoalKick] GRID KICK: grid=%d X=%.0f Y=%.0f -> arahGoal=%d\n",
                                       getRobotGrid(), (double)robotPos_X, (double)robotPos_Y, gridAngle);
                                Imu(arahGoal, cSekarang);
                            } else {
                                if (ballOnGoalSide()) {
                                    if (ballOnLeftGoal == 1) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerLeft;
                                        Imu(angleCornerLeft, cSekarang);
                                    } else if (ballOnLeftGoal == 0) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerRight;
                                        Imu(angleCornerRight, cSekarang);
                                    } else {
                                        if (robotPos_Y <= 0) {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalLeft;
                                            Imu(angleGoalLeft, cSekarang);
                                        } else {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalRight;
                                            Imu(angleGoalRight, cSekarang);
                                        }
                                    }
                                } else {
                                    modeKick = tendangJauh;
                                    arahGoal = 0;
                                    Imu(0, cSekarang);
                                }
                            }
                            robotDirection = true;
                            printf("[GoalKick] Rotate to arahGoal=%d\n", arahGoal);
                        }
                    }
                }
            }
            // TIM LAWAN YANG TENDANG (defending)
            else {
                throwInKicked = false;
                throwInBodyAligned = 0;
                action_kick = false;
                robotDirection = false;
                robotFinalDirection = false;
                tendang = false;
                ballPos = false;
                BallApproachEntry = 0;
                int defendX = 0;
                int defendY = 0;

                if (robotNumber == 2) {
                    defendX = -250;
                    defendY = -100;
                } else if (robotNumber == 4) {
                    defendX = -100;
                    defendY = 100;
                }else if (robotNumber == 1) {
                    defendX = -250;
                    defendY = 0;
                }
                 else if (robotNumber == 5) {
                    defendX = -50;
                    defendY = 0;
                } else if (robotNumber == 3) {
                    defendX = -250;
                    defendY = 100;
                } else if (robotNumber == 6) {
                    defendX = -250;
                    defendY = -100;
                } else {
                    defendX = -250;
                    defendY = 0;
                }
/*
                new_out_pos(defendX, defendY, true);
                if (!ballLost(50)) {
                    trackBall();
                } else {
                    SearchBall(1);
                }
                printf("[GoalKick] Defending(opponent): target=(%d,%d)\n", defendX, defendY);
            }

            return NodeStatus::FAILURE;
        }*/	
        	int tolerance_x = abs(defendX - (int)robotPos_X);
          int tolerance_y = abs(defendY - (int)robotPos_Y);

          if (tolerance_x <= 15 && tolerance_y <= 15) {
            int targetYaw = 0;
            if (defendY > 0) {
              targetYaw = 90;
            } else if (defendY < 0) {
              targetYaw = -90;
            }
            rotateBodyImuNew(targetYaw);
            if (posRotateNew) {
              motion("0");
            }
          } else {
            new_out_pos(defendX, defendY, true);
          }

          if (!ballLost(50)) {
            trackBall();
          } else {
            SearchBall(1);
          }
          printf("[GoalKick] Defending(opponent): target=(%d,%d) targetYaw=%d\n", defendX,
                 defendY, (defendY > 0) ? 90 : ((defendY < 0) ? -90 : 0));
        }

        return NodeStatus::FAILURE;
      }
        
         // =========================================================
        // THROW-IN (setPlay = 4)
        // =========================================================
        if (secondaryInfo[0] == 4) {

            // -- STOPPED (Stopped=1): wasit posisikan bola --
            // Badan diam, kepala scan kanan-kiri + atas-bawah cari bola
            if (Stopped == 1) {
                throwInPhase       = 1;
                throwInKicked      = false;
                throwInWait        = 0;
                throwInBodyAligned = 0;
                motion("8");  // badan diam
                if (!ballLost(50))
                    trackBall();     // bola sudah ketemu → track
                else
                    SearchBall(1);   // scan penuh: atas-bawah + kanan-kiri
                printf("[ThrowIn] STOPPED: searchball, KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
                return NodeStatus::FAILURE;
            }

            // -- Transisi ke Phase 2 setelah Resume (stopped→0) --
            if (throwInPhase != 2 && throwInPhase != 3) {
                throwInPhase       = 2;
                throwInWait        = 0;
                throwInKicked      = false;
                throwInBodyAligned = 0;
                robotDirection     = false;
                BallApproachEntry  = 0;
                printf("[ThrowIn] RESUME-EXECUTING: KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
            }

            // -- Phase 3: sudah tendang, tunggu setPlay reset ke 0 --
            if (throwInPhase == 3) {
                motion("8");
                printf("[ThrowIn] DONE: tunggu setPlay=0\n");
                return NodeStatus::FAILURE;
            }

            // -- Delay kecil setelah pertama masuk phase 2 --
            if (throwInWait < 5) {
                motion("8");
                throwInWait++;
                return NodeStatus::FAILURE;
            }
            // =================================================
            // Cek giliran siapa:
            //   GC mengirim kickingTeam = nomor tim yang BERHAK TENDANG
            //   Sehingga: KickOff == barelang_color → giliran KITA (attacking)
            //             KickOff != barelang_color → giliran LAWAN (defending)
            // =================================================

            // =================================================
            // TIM KITA YANG TENDANG (attacking)
            //   Robot terdekat jadi eksekutor, robot lain cover area bertahan.
            // =================================================
            if (KickOff == barelang_color) {
                if (!isNearestExecutorForThrowIn()) {
                    int defendX = 0;
                    int defendY = 0;
                    if (robotNumber == 2) {
                        defendX = -100;
                        defendY = -100;
                    } else if (robotNumber == 4) {
                        defendX = -100;
                        defendY = 100;
                    }else if (robotNumber == 1) {
                        defendX = -100;
                        defendY = 0;
                    }
                     else if (robotNumber == 5) {
                        defendX = 50;
                        defendY = 0;
                    } else if (robotNumber == 3) {
                        defendX = -100;
                        defendY = 100;
                    } else if (robotNumber == 6) {
                        defendX = -100;
                        defendY = -100;
                    } else {
                        defendX = 0;
                        defendY = 0;
                    }

                    new_out_pos(defendX, defendY, true);

                    int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;

                    if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
                        trackHeadToFieldPoint(teamBallX, teamBallY);
                        const double bodyDeg = atan2((double)teamBallY - robotPos_Y, (double)teamBallX - robotPos_X) * 180.0 / PI;
                        rotateBodyImuNew((int)bodyDeg);
                    } else if (!ballLost(50)) {
                        trackBall();
                    } else {
                        SearchBall(1);
                    }

                    printf("[ThrowIn] SupportDefend(our): target=(%d,%d)\n", defendX, defendY);
                    return NodeStatus::FAILURE;
                }

                printf("[ThrowIn] Attacking: dist=%d yaw=%d posY=%.0f headTilt=%.2f\n",
                       ballDistance, msg_yaw, (double)robotPos_Y, (double)headTilt);

                // Reset throwInKicked setelah ball free atau waktu habis
                // Selama masih di fase throw-in, robot terus ngejar bola
                if (ballLost(35)) {
                    // Bola hilang → Walk + SearchBall
                    Walk(0.0, 0.0, 0.0);
                    delayWaitBall = 0;
                    throwInBodyAligned = 0;  // flag bahwa body lost
                    SearchBall(1);
                } else {
                    throwInBodyAligned = 0;  // reset counter
                    trackBall();

                    // Gunakan headTilt untuk deteksi jarak (sama seperti BallApproach)
                    if (headTilt < cAktif) {
                        // Bola masih jauh → gunakan followBall untuk smooth positioning
                        followBall(0);
                        throwInKicked = false;  // reset
                        robotDirection = false;
                        printf("[ThrowIn] Attacking: followBall (headTilt=%.2f)\n", headTilt);
                    } else {
                        // Sudah dekat → positioning dan tendang
                        if (robotDirection) {
                            // Cek arah robot sudah tepat (tolerance 15 derajat)
                            if ((msg_yaw < (arahGoal - 15)) || (msg_yaw > (arahGoal + 15))) {
                                robotDirection = false;
                                printf("[ThrowIn] Adjusting yaw: current=%d target=%d\n", msg_yaw, arahGoal);
                            } else {
                                // Arah sudah tepat → kick
                             /*   if (tendang) {
                                    throwInKicked = true;
                                    robotDirection = ballPos = tendang = false;
                                    printf("[ThrowIn] KICK! arahGoal=%d mode=%d\n", arahGoal, modeKick);
                                } else {
                                    kick(modeKick);*/
                                    
                                // Arah sudah tepat: khusus ThrowIn gunakan motion token "3"
                                // Syarat ekstra: bola harus benar-benar di tengah kepala.
                                const double throwInPanCenter = panTengah;
                                const double throwInPanTol = 0.15;
                                const double throwInTiltMin = cAktif;

                                if (tendang || throwInKicked)
                                {
                                    motion("8");
                                }
                                else if (headPan >= (throwInPanCenter - throwInPanTol) &&
                                         headPan <= (throwInPanCenter + throwInPanTol) &&
                                         headTilt >= throwInTiltMin)
                                {
                                    modeKick = 3;  // mode side left sesuai request throw-in
                                    motion("0");
                                    sleep(1);
                                     motion("2");
                                    throwInKicked = true;
                                    tendang = true;
                                     printf("[ThrowIn] EXECUTE motion '3' (SideLeft) | modeKick=%d pan=%.2f tilt=%.2f\n",
                                           modeKick, (double)headPan, (double)headTilt);
                                }
                                else
                                {
                                    // Belum center, lanjut rapikan posisi bola di tengah dulu.
                                    followBall(0);
                                    printf("[ThrowIn] Waiting center ball: pan=%.2f target=%.2f tol=%.2f tilt=%.2f min=%.2f\n",
                                           (double)headPan, (double)throwInPanCenter, (double)throwInPanTol,
                                           (double)headTilt, (double)throwInTiltMin);
                                }
                            }
                        } else {
                            // AUTO-ADJUST sudut tendang berdasarkan posisi robot di lapangan
                            int gridAngle = getKickAngleByGrid();
                            if (gridAngle != 0) {
                                // Posisi pinggir/pojok → tendang serong sesuai grid
                                modeKick = tendangJauh;
                                arahGoal = gridAngle;
                                printf("[ThrowIn] GRID KICK: grid=%d X=%.0f Y=%.0f -> arahGoal=%d\n",
                                       getRobotGrid(), (double)robotPos_X, (double)robotPos_Y, gridAngle);
                                Imu(arahGoal, cSekarang);
                            } else {
                                // Posisi tengah → gunakan logic posisi gawang seperti BallApproach
                                if (ballOnGoalSide()) {
                                    if (ballOnLeftGoal == 1) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerLeft;
                                        Imu(angleCornerLeft, cSekarang);
                                    } else if (ballOnLeftGoal == 0) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerRight;
                                        Imu(angleCornerRight, cSekarang);
                                    } else {
                                        if (robotPos_Y <= 0) {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalLeft;
                                            Imu(angleGoalLeft, cSekarang);
                                        } else {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalRight;
                                            Imu(angleGoalRight, cSekarang);
                                        }
                                    }
                                } else {
                                    // Tengah lapangan → tendang lurus
                                    modeKick = tendangJauh;
                                    arahGoal = 0;
                                    Imu(0, cSekarang);
                                }
                            }
                            robotDirection = true;
                            printf("[ThrowIn] Rotate to arahGoal=%d\n", arahGoal);
                        }
                    }
                }
            }
            // TIM LAWAN YANG TENDANG (defending)
            else {
                throwInKicked = false;
                throwInBodyAligned = 0;
                action_kick = false;
                robotDirection = false;
                robotFinalDirection = false;
                tendang = false;
                ballPos = false;
                BallApproachEntry = 0;
                int defendX = 0;
                int defendY = 0;

                if (robotNumber == 2) {
                    defendX = -250;
                    defendY = -100;
                } else if (robotNumber == 4) {
                    defendX = -250;
                    defendY = 100;
                }else if (robotNumber == 1) {
                    defendX = -250;
                    defendY = 0;
                }
                 else if (robotNumber == 5) {
                    defendX = -50;
                    defendY = 0;
                } else if (robotNumber == 3) {
                    defendX = -250;
                    defendY = 100;
                } else if (robotNumber == 6) {
                    defendX = -250;
                    defendY = -100;
                } else {
                    defendX = -250;
                    defendY = 0;
                }

                new_out_pos(defendX, defendY, true);
                if (!ballLost(50)) {
                    trackBall();
                } else {
                    SearchBall(1);
                }
                printf("[ThrowIn] Defending(opponent): target=(%d,%d)\n", defendX, defendY);
            }

            return NodeStatus::FAILURE;
        }

        /*TERAKHIR DI PAKAI 
        if (secondaryInfo[0] == 4) {

            // -- STOPPED (Stopped=1): wasit posisikan bola --
            // Badan diam, kepala scan kanan-kiri + atas-bawah cari bola
            if (Stopped == 1) {
                throwInPhase       = 1;
                throwInKicked      = false;
                throwInWait        = 0;
                throwInBodyAligned = 0;
                motion("8");  // badan diam
                if (!ballLost(50))
                    trackBall();     // bola sudah ketemu → track
                else
                    SearchBall(1);   // scan penuh: atas-bawah + kanan-kiri
                printf("[ThrowIn] STOPPED: searchball, KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
                return NodeStatus::FAILURE;
            }

            // -- Transisi ke Phase 2 setelah Resume (stopped→0) --
            if (throwInPhase != 2 && throwInPhase != 3) {
                throwInPhase       = 2;
                throwInWait        = 0;
                throwInKicked      = false;
                throwInBodyAligned = 0;
                robotDirection     = false;
                BallApproachEntry  = 0;
                printf("[ThrowIn] RESUME-EXECUTING: KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
            }

            // -- Phase 3: sudah tendang, tunggu setPlay reset ke 0 --
            if (throwInPhase == 3) {
                motion("8");
                printf("[ThrowIn] DONE: tunggu setPlay=0\n");
                return NodeStatus::FAILURE;
            }

            // -- Delay kecil setelah pertama masuk phase 2 --
            if (throwInWait < 5) {
                motion("8");
                throwInWait++;
                return NodeStatus::FAILURE;
            }
            // =================================================
            // Cek giliran siapa:
            //   GC mengirim kickingTeam = nomor tim yang BERHAK TENDANG
            //   Sehingga: KickOff == barelang_color → giliran KITA (attacking)
            //             KickOff != barelang_color → giliran LAWAN (defending)
            // =================================================

            // =================================================
            // TIM KITA YANG TENDANG (attacking)
            //   Robot terdekat jadi eksekutor, robot lain cover area bertahan.
            // =================================================
            if (KickOff == barelang_color) {
                if (!isNearestExecutorForThrowIn()) {
                    int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;
                    if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
                        int defendX = 0, defendY = 0;
                        getThrowInDefendTarget(teamBallX, teamBallY, defendX, defendY);
                        int defendGrid = getClosestGridID(defendX, defendY);
                        moveGrid(defendGrid, 0, 0);
                        if (!ballLost(50)) trackBall();
                        else SearchBall(1);
                        printf("[ThrowIn] SupportDefend: owner=%d ball=(%d,%d) defend=(%d,%d) grid=%d\n",
                               teamBallOwner, teamBallX, teamBallY, defendX, defendY, defendGrid);
                    } else {
                       // SearchBall(1);
                       // Walk(aruku * 0.5, 0.0, 0.0);
                       WalkSearchBall();
                    }
                    return NodeStatus::FAILURE;
                }

                printf("[ThrowIn] Attacking: dist=%d yaw=%d posY=%.0f headTilt=%.2f\n",
                       ballDistance, msg_yaw, (double)robotPos_Y, (double)headTilt);

                // Reset throwInKicked setelah ball free atau waktu habis
                // Selama masih di fase throw-in, robot terus ngejar bola

                if (ballLost(35)) {
                    // Bola hilang → increment counter dan search dengan rotate body bertahap
                    throwInBodyAligned++;
                    SearchBall(1);      // scan penuh: kanan-kiri + atas-bawah

                    // Bertahap: scan di tempat → rotate 90° → rotate 180° → rotate 270°
                    if (throwInBodyAligned < 50) {
                        // Fase 1: Scan di tempat (0-50 frame = ~2 detik)
                        motion("8");
                        printf("[ThrowIn] Attacking: Ball lost, scan di tempat (%d)\n", throwInBodyAligned);
                    } else if (throwInBodyAligned < 100) {
                        // Fase 2: Rotate body 90 derajat sambil search (50-100 frame)
                        int targetAngle = (msg_yaw + 90) % 360;
                        if (targetAngle > 180) targetAngle -= 360;
                        Imu(targetAngle, cSekarang);
                        printf("[ThrowIn] Attacking: Rotate 90° sambil search (target=%d, yaw=%d)\n", targetAngle, msg_yaw);
                    } else if (throwInBodyAligned < 150) {
                        // Fase 3: Rotate body 180 derajat sambil search (100-150 frame)
                        int targetAngle = (msg_yaw + 180) % 360;
                        if (targetAngle > 180) targetAngle -= 360;
                        Imu(targetAngle, cSekarang);
                        printf("[ThrowIn] Attacking: Rotate 180° sambil search (target=%d, yaw=%d)\n", targetAngle, msg_yaw);
                    } else {
                        // Fase 4: Rotate body 270 derajat / full rotation (150+ frame)
                        int targetAngle = (msg_yaw + 270) % 360;
                        if (targetAngle > 180) targetAngle -= 360;
                        Imu(targetAngle, cSekarang);
                        printf("[ThrowIn] Attacking: Rotate 270° sambil search (target=%d, yaw=%d)\n", targetAngle, msg_yaw);
                        // Reset counter untuk mulai cycle lagi
                        if (throwInBodyAligned > 200) throwInBodyAligned = 0;
                    }

                    throwInKicked = false;
                    robotDirection = false;
                } else {
                    // Bola ketemu → reset counter
                    throwInBodyAligned = 0;
                    trackBall();

                    // Gunakan headTilt untuk deteksi jarak (sama seperti followBall)
                    if (headTilt < cAktif) {
                        // Bola masih jauh → gunakan followBall untuk smooth positioning
                        followBall(0);
                        throwInKicked = false;  // reset
                        robotDirection = false;
                        printf("[ThrowIn] Attacking: followBall (headTilt=%.2f)\n", headTilt);
                    } else {
                        // Sudah dekat → positioning dan tendang
                        if (headTilt >= cAktif && headPan >= -0.3 && headPan <= 0.3) {
                            if (!robotDirection) {
                                arahGoal = getThrowInKickAngle();
                                modeKick = tendangJauh;
                                printf("[ThrowIn] Rotate to arahGoal=%d (X=%.0f Y=%.0f)\n",
                                        arahGoal, (double)robotPos_X, (double)robotPos_Y);
                                Imu(arahGoal, cSekarang);
                            } else {
                                // Tendang dan langsung reset untuk ngejar bola lagi
                                kick(modeKick);
                                throwInKicked = false;  // langsung reset agar bisa ngejar lagi
                                robotDirection = false;  // reset positioning
                                printf("[ThrowIn] KICK! arahGoal=%d mode=%d - Continue chasing\n", arahGoal, modeKick);
                            }
                        } else {
                            // Kepala belum lurus ke bola → luruskan dulu
                            if (headPan > 0.3)       Walk(0.0, 0.0,  0.1);
                            else if (headPan < -0.3) Walk(0.0, 0.0, -0.1);
                            else                     Walk(aruku * 0.3, 0.0, 0.0);
                            printf("[ThrowIn] Attacking: Adjusting position\n");
                        }
                    }
                }
            }

            // =================================================
            // TIM LAWAN YANG TENDANG (defending)
            //   Setelah resume: search ball → track → luruskan badan
            //                  → mendekati bola → berhenti di jarak aman
            //   + mundur kalau bola terlalu dekat
            // =================================================
            else {
                printf("[ThrowIn] Defending: dist=%d bodyAligned=%d\n",
                       ballDistance, throwInBodyAligned);

                if (ballLost(50)) {
                    // Bola belum terlihat → search sambil diam
                    throwInBodyAligned = 0;
                    SearchBall(1);
                    motion("8");
                } else {
                    trackBall();
                    if (ballDistance < THROW_IN_SAFE_DISTANCE) {
                        // Bola lebih dekat dari jarak aman → mundur sampai THROW_IN_SAFE_DISTANCE
                        throwInBodyAligned = 0;
                        Walk(-aruku * 0.9, 0.0, 0.0);
                        printf("[ThrowIn] Defending: mundur ke jarak aman (dist=%d < %d)\n",
                               ballDistance, THROW_IN_SAFE_DISTANCE);
                    } else {
                        // Sudah di jarak aman → BERHENTI, tetap hadap bola
                        throwInBodyAligned = 0;
                        if (headPan > 0.2)       Walk(0.0, 0.0,  0.12);
                        else if (headPan < -0.2) Walk(0.0, 0.0, -0.12);
                        else                     motion("8");
                        printf("[ThrowIn] Di jarak aman (dist=%d >= %d), diam\n",
                               ballDistance, THROW_IN_SAFE_DISTANCE);
                    }
                }
            }

            return NodeStatus::FAILURE;
        }
*/
        // =========================================================
        // GOAL KICK (setPlay = 5)
        // =========================================================
        if (secondaryInfo[0] == 5) {

            // -- STOPPED (Stopped=1): wasit posisikan bola --
            // Badan diam, kepala scan kanan-kiri + atas-bawah cari bola
            if (Stopped == 1) {
                throwInPhase       = 1;
                throwInKicked      = false;
                throwInWait        = 0;
                throwInBodyAligned = 0;
                motion("8");  // badan diam
                if (!ballLost(50))
                    trackBall();     // bola sudah ketemu → track
                else
                    SearchBall(1);   // scan penuh: atas-bawah + kanan-kiri
                printf("[GoalKick] STOPPED: searchball, KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
                return NodeStatus::FAILURE;
            }

            // -- Transisi ke Phase 2 setelah Resume (stopped→0) --
            if (throwInPhase != 2 && throwInPhase != 3) {
                throwInPhase       = 2;
                throwInWait        = 0;
                throwInKicked      = false;
                throwInBodyAligned = 0;
                robotDirection     = false;
                BallApproachEntry  = 0;
                printf("[GoalKick] RESUME-EXECUTING: KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
            }

            // -- Phase 3: sudah tendang, tunggu setPlay reset ke 0 --
            if (throwInPhase == 3) {
                motion("8");
                printf("[GoalKick] DONE: tunggu setPlay=0\n");
                return NodeStatus::FAILURE;
            }

            // -- Delay kecil setelah pertama masuk phase 2 --
            if (throwInWait < 5) {
                motion("8");
                throwInWait++;
                return NodeStatus::FAILURE;
            }
            // =================================================
            // Cek giliran siapa:
            //   GC mengirim kickingTeam = nomor tim yang BERHAK TENDANG
            //   Sehingga: KickOff == barelang_color → giliran KITA (attacking)
            //             KickOff != barelang_color → giliran LAWAN (defending)
            // =================================================

            // =================================================
            // TIM KITA YANG TENDANG (attacking)
            //   Robot terdekat jadi eksekutor, robot lain cover area bertahan.
            // =================================================
            if (KickOff == barelang_color) {
                if (!isNearestExecutorForThrowIn()) {
                int defendX = 0;
                int defendY = 0;

                if (robotNumber == 2) {
                    defendX = 100;
                    defendY = -50;
                } else if (robotNumber == 4) {
                    defendX = -200;
                    defendY = -100;
                }else if (robotNumber == 1) {
                    defendX = -150;
                    defendY = 0;
                }
                 else if (robotNumber == 5) {
                    defendX = 0;
                    defendY = 0;
                } else if (robotNumber == 3) {
                    defendX = -150;
                    defendY = 100;
                } else if (robotNumber == 6) {
                    defendX = -50;
                    defendY = -100;
                } else {
                    defendX = -150;
                    defendY = 0;
                }

                    new_out_pos(defendX, defendY, true);

                    int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;

                    if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
                        trackHeadToFieldPoint(teamBallX, teamBallY);
                        const double bodyDeg = atan2((double)teamBallY - robotPos_Y, (double)teamBallX - robotPos_X) * 180.0 / PI;
                        rotateBodyImuNew((int)bodyDeg);
                    } else if (!ballLost(50)) {
                        trackBall();
                    } else {
                        SearchBall(1);
                    }

                    printf("[GoalKick] SupportDefend(our): target=(%d,%d)\n", defendX, defendY);
                    return NodeStatus::FAILURE;
                }

                printf("[GoalKick] Attacking: dist=%d yaw=%d posY=%.0f headTilt=%.2f\n",
                       ballDistance, msg_yaw, (double)robotPos_Y, (double)headTilt);

                // Reset throwInKicked setelah ball free atau waktu habis
                // Selama masih di fase goal kick, robot terus ngejar bola

                if (ballLost(35)) {
                    // Bola hilang → Walk + SearchBall
                    Walk(0.0, 0.0, 0.0);
                    delayWaitBall = 0;
                    throwInBodyAligned = 0;
                    SearchBall(1);
                } else {
                    throwInBodyAligned = 0;  // reset counter
                    trackBall();

                    // Gunakan headTilt untuk deteksi jarak (sama seperti BallApproach)
                    if (headTilt < cAktif) {
                        // Bola masih jauh → gunakan followBall untuk smooth positioning
                        followBall(0);
                        throwInKicked = false;  // reset
                        robotDirection = false;
                        printf("[GoalKick] Attacking: followBall (headTilt=%.2f)\n", headTilt);
                    } else {
                        // Sudah dekat → positioning dan tendang
                        if (robotDirection) {
                            // Cek arah robot sudah tepat (tolerance 15 derajat)
                            if ((msg_yaw < (arahGoal - 15)) || (msg_yaw > (arahGoal + 15))) {
                                robotDirection = false;
                                printf("[GoalKick] Adjusting yaw: current=%d target=%d\n", msg_yaw, arahGoal);
                            } else {
                                // Arah sudah tepat → kick
                                if (tendang) {
                                    throwInKicked = true;
                                    robotDirection = ballPos = tendang = false;
                                    printf("[GoalKick] KICK! arahGoal=%d mode=%d\n", arahGoal, modeKick);
                                } else {
                                    kick(modeKick);
                                }
                            }
                        } else {
                            // AUTO-ADJUST sudut tendang berdasarkan posisi robot di lapangan
                            int gridAngle = getKickAngleByGrid();
                            if (gridAngle != 0) {
                                // Posisi pinggir/pojok → tendang serong sesuai grid
                                modeKick = tendangJauh;
                                arahGoal = gridAngle;
                                printf("[GoalKick] GRID KICK: grid=%d X=%.0f Y=%.0f -> arahGoal=%d\n",
                                       getRobotGrid(), (double)robotPos_X, (double)robotPos_Y, gridAngle);
                                Imu(arahGoal, cSekarang);
                            } else {
                                // Posisi tengah → gunakan logic posisi gawang seperti BallApproach
                                if (ballOnGoalSide()) {
                                    if (ballOnLeftGoal == 1) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerLeft;
                                        Imu(angleCornerLeft, cSekarang);
                                    } else if (ballOnLeftGoal == 0) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerRight;
                                        Imu(angleCornerRight, cSekarang);
                                    } else {
                                        if (robotPos_Y <= 0) {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalLeft;
                                            Imu(angleGoalLeft, cSekarang);
                                        } else {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalRight;
                                            Imu(angleGoalRight, cSekarang);
                                        }
                                    }
                                } else {
                                    // Tengah lapangan → tendang lurus
                                    modeKick = tendangJauh;
                                    arahGoal = 0;
                                    Imu(0, cSekarang);
                                }
                            }
                            robotDirection = true;
                            printf("[GoalKick] Rotate to arahGoal=%d\n", arahGoal);
                        }
                    }
                }
            }

            // =================================================
            // TIM LAWAN YANG TENDANG (defending)
            //   Setelah resume: search ball → track → luruskan badan
            //                  → mendekati bola → berhenti di jarak aman
            //   + mundur kalau bola terlalu dekat
            // =================================================
            else {
                throwInKicked = false;
                throwInBodyAligned = 0;
                action_kick = false;
                robotDirection = false;
                robotFinalDirection = false;
                tendang = false;
                ballPos = false;
                BallApproachEntry = 0;
                int defendX = 0;
                int defendY = 0;

                if (robotNumber == 2) {
                    defendX = -50;
                    defendY = -100;
                } else if (robotNumber == 4) {
                    defendX = -200;
                    defendY = -100;
                }else if (robotNumber == 1) {
                    defendX = -150;
                    defendY = 0;
                }
                 else if (robotNumber == 5) {
                    defendX = 0;
                    defendY = 0;
                } else if (robotNumber == 3) {
                    defendX = -150;
                    defendY = 100;
                } else if (robotNumber == 6) {
                    defendX = -50;
                    defendY = -100;
                } else {
                    defendX = -150;
                    defendY = 0;
                }

                new_out_pos(defendX, defendY, true);
                if (!ballLost(50)) {
                    // Kalau bola terlihat oleh vision sendiri: fokus track bola langsung.
                    trackBall();
                } else {
                    // Bola tidak terlihat: tetap jalan ke target defend sambil aktif mencari bola.
                    SearchBall(1);
                }
                /*

                int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;
                if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
                    trackHeadToFieldPoint(teamBallX, teamBallY);
                    const double bodyDeg = atan2((double)teamBallY - robotPos_Y, (double)teamBallX - robotPos_X) * 180.0 / PI;
                    rotateBodyImuNew((int)bodyDeg);
                } else if (!ballLost(50)) {
                    trackBall();
                } else {
                    SearchBall(1);
                }*/

                printf("[GoalKick] Defending(opponent): target=(%d,%d)\n", defendX, defendY);
            }

            return NodeStatus::FAILURE;
        }
        
        
        
        /*TERAKHIR DIPAKAI 
        if (secondaryInfo[0] == 5) {

            // -- STOPPED (Stopped=1): wasit posisikan bola --
            // Badan diam, kepala scan kanan-kiri + atas-bawah cari bola
            if (Stopped == 1) {
                throwInPhase       = 1;
                throwInKicked      = false;
                throwInWait        = 0;
                throwInBodyAligned = 0;
                motion("8");  // badan diam
                if (!ballLost(50))
                    trackBall();     // bola sudah ketemu → track
                else
                    SearchBall(1);   // scan penuh: atas-bawah + kanan-kiri
                printf("[GoalKick] STOPPED: searchball, KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
                return NodeStatus::FAILURE;
            }

            // -- Transisi ke Phase 2 setelah Resume (stopped→0) --
            if (throwInPhase != 2 && throwInPhase != 3) {
                throwInPhase       = 2;
                throwInWait        = 0;
                throwInKicked      = false;
                throwInBodyAligned = 0;
                robotDirection     = false;
                BallApproachEntry  = 0;
                printf("[GoalKick] RESUME-EXECUTING: KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
            }

            // -- Phase 3: sudah tendang, tunggu setPlay reset ke 0 --
            if (throwInPhase == 3) {
                motion("8");
                printf("[GoalKick] DONE: tunggu setPlay=0\n");
                return NodeStatus::FAILURE;
            }

            // -- Delay kecil setelah pertama masuk phase 2 --
            if (throwInWait < 5) {
                motion("8");
                throwInWait++;
                return NodeStatus::FAILURE;
            }
            // =================================================
            // Cek giliran siapa:
            // TIM KITA YANG TENDANG (attacking)

            if (KickOff == barelang_color) {
                if (!isNearestExecutorForThrowIn()) {
                    int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;
                    if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
                        int defendX = 0, defendY = 0;
                        getThrowInDefendTarget(teamBallX, teamBallY, defendX, defendY);
                        int defendGrid = getClosestGridID(defendX, defendY);
                        moveGrid(defendGrid, 0, 0);
                        if (!ballLost(50)) trackBall();
                        else SearchBall(1);
                        printf("[GoalKick] SupportDefend: owner=%d ball=(%d,%d) defend=(%d,%d) grid=%d\n",
                               teamBallOwner, teamBallX, teamBallY, defendX, defendY, defendGrid);
                    } else {
                     //   SearchBall(1);
                      //  Walk(aruku * 0.5, 0.0, 0.0);
                        WalkSearchBall();
                    }
                    return NodeStatus::FAILURE;
                }

                printf("[GoalKick] Attacking: dist=%d yaw=%d posY=%.0f headTilt=%.2f\n",
                       ballDistance, msg_yaw, (double)robotPos_Y, (double)headTilt);

                // Reset throwInKicked setelah ball free atau waktu habis
                // Selama masih di fase goal kick, robot terus ngejar bola

                if (ballLost(35)) {
                    // Bola hilang → Walk + SearchBall
                    Walk(0.0, 0.0, 0.0);
                    delayWaitBall = 0;
                    throwInBodyAligned = 0;
                    SearchBall(1);
                } else {
                    throwInBodyAligned = 0;  // reset counter
                    trackBall();

                    // Gunakan headTilt untuk deteksi jarak (sama seperti BallApproach)
                    if (headTilt < cAktif) {
                        // Bola masih jauh → gunakan followBall untuk smooth positioning
                        followBall(0);
                        throwInKicked = false;  // reset
                        robotDirection = false;
                        printf("[GoalKick] Attacking: followBall (headTilt=%.2f)\n", headTilt);
                    } else {
                        // Sudah dekat → positioning dan tendang
                        if (robotDirection) {
                            // Cek arah robot sudah tepat (tolerance 15 derajat)
                            if ((msg_yaw < (arahGoal - 15)) || (msg_yaw > (arahGoal + 15))) {
                                robotDirection = false;
                                printf("[GoalKick] Adjusting yaw: current=%d target=%d\n", msg_yaw, arahGoal);
                            } else {
                                // Arah sudah tepat → kick
                                if (tendang) {
                                    throwInKicked = true;
                                    robotDirection = ballPos = tendang = false;
                                    printf("[GoalKick] KICK! arahGoal=%d mode=%d\n", arahGoal, modeKick);
                                } else {
                                    kick(modeKick);
                                }
                            }
                        } else {
                            // AUTO-ADJUST sudut tendang berdasarkan posisi robot di lapangan
                            int gridAngle = getKickAngleByGrid();
                            if (gridAngle != 0) {
                                // Posisi pinggir/pojok → tendang serong sesuai grid
                                modeKick = tendangJauh;
                                arahGoal = gridAngle;
                                printf("[GoalKick] GRID KICK: grid=%d X=%.0f Y=%.0f -> arahGoal=%d\n",
                                       getRobotGrid(), (double)robotPos_X, (double)robotPos_Y, gridAngle);
                                Imu(arahGoal, cSekarang);
                            } else {
                                // Posisi tengah → gunakan logic posisi gawang seperti BallApproach
                                if (ballOnGoalSide()) {
                                    if (ballOnLeftGoal == 1) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerLeft;
                                        Imu(angleCornerLeft, cSekarang);
                                    } else if (ballOnLeftGoal == 0) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerRight;
                                        Imu(angleCornerRight, cSekarang);
                                    } else {
                                        if (robotPos_Y <= 0) {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalLeft;
                                            Imu(angleGoalLeft, cSekarang);
                                        } else {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalRight;
                                            Imu(angleGoalRight, cSekarang);
                                        }
                                    }
                                } else {
                                    // Tengah lapangan → tendang lurus
                                    modeKick = tendangJauh;
                                    arahGoal = 0;
                                    Imu(0, cSekarang);
                                }
                            }
                            robotDirection = true;
                            printf("[GoalKick] Rotate to arahGoal=%d\n", arahGoal);
                        }
                    }
                }
            }

            // =================================================
            // TIM LAWAN YANG TENDANG (defending)
            else {
                printf("[GoalKick] Defending: dist=%d bodyAligned=%d\n",
                       ballDistance, throwInBodyAligned);

                if (ballLost(50)) {
                    // Bola belum terlihat → search sambil diam
                    throwInBodyAligned = 0;
                    clearVizTarget();
                    SearchBall(1);
                    motion("8");
                } else {
                    trackBall();
                    if (ballDistance < THROW_IN_SAFE_DISTANCE) {
                        // Bola lebih dekat dari jarak aman → mundur sampai THROW_IN_SAFE_DISTANCE
                        throwInBodyAligned = 0;
                        double retreatCm = THROW_IN_SAFE_DISTANCE - ballDistance;
                        if (retreatCm < 0) retreatCm = 0;
                        setVizTargetFromPos(robotPos_X - retreatCm, robotPos_Y);
                        Walk(-aruku * 0.9, 0.0, 0.0);
                        printf("[GoalKick] Defending: mundur ke jarak aman (dist=%d < %d)\n",
                               ballDistance, THROW_IN_SAFE_DISTANCE);
                    } else {
                        // Sudah di jarak aman → BERHENTI, tetap hadap bola
                        throwInBodyAligned = 0;
                        setVizTargetFromPos(robotPos_X, robotPos_Y);
                        if (headPan > 0.2)       Walk(0.0, 0.0,  0.12);
                        else if (headPan < -0.2) Walk(0.0, 0.0, -0.12);
                        else                     motion("8");
                        printf("[GoalKick] Di jarak aman (dist=%d >= %d), diam\n",
                               ballDistance, THROW_IN_SAFE_DISTANCE);
                    }
                }
            }

            return NodeStatus::FAILURE;
        }
*/
       // =========================================================
        // CORNER KICK (setPlay = 6)
        // =========================================================
        if (secondaryInfo[0] == 6) {

            // -- STOPPED (Stopped=1): wasit posisikan bola --
            // Badan diam, kepala scan kanan-kiri + atas-bawah cari bola
            if (Stopped == 1) {
                throwInPhase        = 1;
                throwInKicked       = false;
                throwInWait         = 0;
                throwInBodyAligned  = 0;
                goalKickExitZone    = false; // reset flag keluar zona saat wasit stop
                goalKickDefendPhase = 0;     // reset state machine defend
                goalKickAlignWait   = 0;     // reset alignment counter
                goalKickYawStableCount = 0;  // reset yaw stable counter
                goalKickBallSide    = 0;     // reset ball side
                motion("8");  // badan diam
                if (!ballLost(50))
                    trackBall();     // bola sudah ketemu → track
                else
                    SearchBall(1);   // scan penuh: atas-bawah + kanan-kiri
                printf("[GoalKick] STOPPED: searchball, KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
                return NodeStatus::FAILURE;
            }

            // -- Transisi ke Phase 2 setelah Resume (stopped→0) --
            if (throwInPhase != 2 && throwInPhase != 3) {
                throwInPhase       = 2;
                throwInWait        = 0;
                throwInKicked      = false;
                throwInBodyAligned = 0;
                robotDirection     = false;
                BallApproachEntry  = 0;
                printf("[GoalKick] RESUME-EXECUTING: KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
            }

            // -- Phase 3: sudah tendang, tunggu setPlay reset ke 0 --
            if (throwInPhase == 3) {
                motion("8");
                printf("[GoalKick] DONE: tunggu setPlay=0\n");
                return NodeStatus::FAILURE;
            }

            // -- Delay kecil setelah pertama masuk phase 2 --
            if (throwInWait < 5) {
                motion("8");
                throwInWait++;
                return NodeStatus::FAILURE;
            }

            // =================================================
            // TIM KITA YANG TENDANG (attacking)
            //   Robot terdekat jadi eksekutor, robot lain cover area bertahan.
            // =================================================
            if (KickOff == barelang_color) {
                if (!isNearestExecutorForThrowIn()) {
                    int defendX = 0;
                    int defendY = 0;

                if (robotNumber == 2) {
                    defendX = 50;
                    defendY = -100;
                } else if (robotNumber == 4) {
                    defendX = 100;
                    defendY = 100;
                } else if (robotNumber == 5) {
                    defendX = 150;
                    defendY = 0;
                } else if (robotNumber == 3) {
                    defendX = 100;
                    defendY = 100;
                } else if (robotNumber == 6) {
                    defendX = 50;
                    defendY = -100;
                } else {
                    defendX = -150;
                    defendY = 0;
                }      

                    new_out_pos(defendX, defendY, true);

                    int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;
                    if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
                        trackHeadToFieldPoint(teamBallX, teamBallY);
                        const double bodyDeg = atan2((double)teamBallY - robotPos_Y, (double)teamBallX - robotPos_X) * 180.0 / PI;
                        rotateBodyImuNew((int)bodyDeg);
                    } else if (!ballLost(50)) {
                        trackBall();
                    } else {
                        SearchBall(1);
                    }

                    printf("[GoalKick] SupportDefend(our): target=(%d,%d)\n", defendX, defendY);
                    return NodeStatus::FAILURE;
                }

                printf("[GoalKick] Attacking: dist=%d yaw=%d posY=%.0f headTilt=%.2f\n",
                       ballDistance, msg_yaw, (double)robotPos_Y, (double)headTilt);

                // Reset throwInKicked setelah ball free atau waktu habis
                // Selama masih di fase goal kick, robot terus ngejar bola

                if (ballLost(35)) {
                    // Bola hilang → Walk + SearchBall
                    Walk(0.0, 0.0, 0.0);
                    delayWaitBall = 0;
                    throwInBodyAligned = 0;
                    SearchBall(1);
                } else {
                    throwInBodyAligned = 0;  // reset counter
                    trackBall();

                    // Gunakan headTilt untuk deteksi jarak (sama seperti BallApproach)
                    if (headTilt < cAktif) {
                        // Bola masih jauh → gunakan followBall untuk smooth positioning
                        followBall(0);
                        throwInKicked = false;  // reset
                        robotDirection = false;
                        printf("[GoalKick] Attacking: followBall (headTilt=%.2f)\n", headTilt);
                    } else {
                        // Sudah dekat → positioning dan tendang
                        if (robotDirection) {
                            // Cek arah robot sudah tepat (tolerance 15 derajat)
                            if ((msg_yaw < (arahGoal - 15)) || (msg_yaw > (arahGoal + 15))) {
                                robotDirection = false;
                                printf("[GoalKick] Adjusting yaw: current=%d target=%d\n", msg_yaw, arahGoal);
                            } else {
                                // Arah sudah tepat → kick
                                if (tendang) {
                                    throwInKicked = true;
                                    robotDirection = ballPos = tendang = false;
                                    printf("[GoalKick] KICK! arahGoal=%d mode=%d\n", arahGoal, modeKick);
                                } else {
                                    kick(modeKick);
                                }
                            }
                        } else {
                            // CORNER KICK: Tendang ke dalam lapangan!
                            // Robot ada di pojok lapangan lawan, jadi TIDAK BOLEH pakai getKickAngleByGrid()
                            // karena fungsi itu dirancang untuk tendangan dari area gawang (bukan corner).
                            // Gunakan sudut khusus corner kick yang bisa di-configure via YAML.
                            modeKick = tendangJauh;
                            if (robotPos_Y <= 0) {
                                // Bola di pojok KIRI lawan → tendang serong ke KANAN-DALAM lapangan
                                arahGoal = cornerKickAngleLeft;
                            } else {
                                // Bola di pojok KANAN lawan → tendang serong ke KIRI-DALAM lapangan
                                arahGoal = cornerKickAngleRight;
                            }
                            Imu(arahGoal, cSekarang);
                            robotDirection = true;
                            printf("[CornerKick] Executor rotate to arahGoal=%d (posY=%.0f)\n",
                                   arahGoal, (double)robotPos_Y);
                        }
                    }
                }
            }

            // =================================================
            // TIM LAWAN YANG TENDANG (defending) - SIMPLE 3 PHASE
            // 0) Jika yaw serong (>|15|), putar badan di tempat ke 0 derajat.
            // 1) Setelah lurus, mundur sampai jarak aman 150 cm dari bola.
            // 2) Setelah aman, luruskan badan ke bola (tanpa mendekat).
            // =================================================
            else {
                throwInKicked = false;
                throwInBodyAligned = 0;
                action_kick = false;
                robotDirection = false;
                robotFinalDirection = false;
                tendang = false;
                ballPos = false;
                BallApproachEntry = 0;
                int defendX = 0;
                int defendY = 0;

                if (robotNumber == 2) {
                    defendX = -200;
                    defendY = -100;
                } else if (robotNumber == 4) {
                    defendX = -200;
                    defendY = 100;
                } else if (robotNumber == 5) {
                    defendX = -150;
                    defendY = 0;
                } else if (robotNumber == 3) {
                    defendX = -200;
                    defendY = 100;
                } else if (robotNumber == 6) {
                    defendX = 200;
                    defendY = -100;
                } else {
                    defendX = -150;
                    defendY = 0;
                }

                new_out_pos(defendX, defendY, true);
                if (!ballLost(50)) {
                    // Kalau bola terlihat oleh vision sendiri: fokus track bola langsung.
                    trackBall();
                } else {
                    // Bola tidak terlihat: tetap jalan ke target defend sambil aktif mencari bola.
                    SearchBall(1);
                }
                /*

                int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;
                if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
                    trackHeadToFieldPoint(teamBallX, teamBallY);
                    const double bodyDeg = atan2((double)teamBallY - robotPos_Y, (double)teamBallX - robotPos_X) * 180.0 / PI;
                    rotateBodyImuNew((int)bodyDeg);
                } else if (!ballLost(50)) {
                    trackBall();
                } else {
                    SearchBall(1);
                }*/

                printf("[GoalKick] Defending(opponent): target=(%d,%d)\n", defendX, defendY);
                return NodeStatus::FAILURE;
            }

            return NodeStatus::FAILURE;
        }
        /*TERAKHIR DIPAKAI STATIS
        if (secondaryInfo[0] == 6) {

            // -- STOPPED (Stopped=1): wasit posisikan bola --
            // Badan diam, kepala scan kanan-kiri + atas-bawah cari bola
            if (Stopped == 1) {
                throwInPhase       = 1;
                throwInKicked      = false;
                throwInWait        = 0;
                throwInBodyAligned = 0;
                motion("8");  // badan diam
                if (!ballLost(50))
                    trackBall();     // bola sudah ketemu → track
                else
                    SearchBall(1);   // scan penuh: atas-bawah + kanan-kiri
                printf("[CornerKick] STOPPED: searchball, KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
                return NodeStatus::FAILURE;
            }

            // -- Transisi ke Phase 2 setelah Resume (stopped→0) --
            if (throwInPhase != 2 && throwInPhase != 3) {
                throwInPhase       = 2;
                throwInWait        = 0;
                throwInKicked      = false;
                throwInBodyAligned = 0;
                robotDirection     = false;
                BallApproachEntry  = 0;
                printf("[CornerKick] RESUME-EXECUTING: KickOff=%d barelang=%d\n",
                       KickOff, barelang_color);
            }

            // -- Phase 3: sudah tendang, tunggu setPlay reset ke 0 --
            if (throwInPhase == 3) {
                motion("8");
                printf("[CornerKick] DONE: tunggu setPlay=0\n");
                return NodeStatus::FAILURE;
            }

            // -- Delay kecil setelah pertama masuk phase 2 --
            if (throwInWait < 5) {
                motion("8");
                throwInWait++;
                return NodeStatus::FAILURE;
            }
            // Cek giliran siapa:
            // TIM KITA YANG TENDANG (attacking)

            if (KickOff == barelang_color) {
                if (!isNearestExecutorForThrowIn()) {
                    int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;
                    if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
                        int defendX = 0, defendY = 0;
                        getThrowInDefendTarget(teamBallX, teamBallY, defendX, defendY);
                        int defendGrid = getClosestGridID(defendX, defendY);
                        moveGrid(defendGrid, 0, 0);
                        if (!ballLost(50)) trackBall();
                        else SearchBall(1);
                        printf("[CornerKick] SupportDefend: owner=%d ball=(%d,%d) defend=(%d,%d) grid=%d\n",
                               teamBallOwner, teamBallX, teamBallY, defendX, defendY, defendGrid);
                    } else {
                      //  SearchBall(1);
                      //  Walk(aruku * 0.5, 0.0, 0.0);
                        WalkSearchBall();
                    }
                    return NodeStatus::FAILURE;
                }

                printf("[CornerKick] Attacking: dist=%d yaw=%d posY=%.0f headTilt=%.2f\n",
                       ballDistance, msg_yaw, (double)robotPos_Y, (double)headTilt);

                // Reset throwInKicked setelah ball free atau waktu habis
                // Selama masih di fase corner kick, robot terus ngejar bola

                if (ballLost(35)) {
                    // Bola hilang → Walk + SearchBall
                    Walk(0.0, 0.0, 0.0);
                    delayWaitBall = 0;
                    throwInBodyAligned = 0;
                    SearchBall(1);
                } else {
                    throwInBodyAligned = 0;  // reset counter
                    trackBall();

                    // Gunakan headTilt untuk deteksi jarak (sama seperti BallApproach)
                    if (headTilt < cAktif) {
                        // Bola masih jauh → gunakan followBall untuk smooth positioning
                        followBall(0);
                        throwInKicked = false;  // reset
                        robotDirection = false;
                        printf("[CornerKick] Attacking: followBall (headTilt=%.2f)\n", headTilt);
                    } else {
                        // Sudah dekat → positioning dan tendang
                        if (robotDirection) {
                            // Cek arah robot sudah tepat (tolerance 15 derajat)
                            if ((msg_yaw < (arahGoal - 15)) || (msg_yaw > (arahGoal + 15))) {
                                robotDirection = false;
                                printf("[CornerKick] Adjusting yaw: current=%d target=%d\n", msg_yaw, arahGoal);
                            } else {
                                // Arah sudah tepat → kick
                                if (tendang) {
                                    throwInKicked = true;
                                    robotDirection = ballPos = tendang = false;
                                    printf("[CornerKick] KICK! arahGoal=%d mode=%d\n", arahGoal, modeKick);
                                } else {
                                    kick(modeKick);
                                }
                            }
                        } else {
                            // AUTO-ADJUST sudut tendang berdasarkan posisi robot di lapangan
                            int gridAngle = getKickAngleByGrid();
                            if (gridAngle != 0) {
                                // Posisi pinggir/pojok → tendang serong sesuai grid
                                modeKick = tendangJauh;
                                arahGoal = gridAngle;
                                printf("[CornerKick] GRID KICK: grid=%d X=%.0f Y=%.0f -> arahGoal=%d\n",
                                       getRobotGrid(), (double)robotPos_X, (double)robotPos_Y, gridAngle);
                                Imu(arahGoal, cSekarang);
                            } else {
                                // Posisi tengah → gunakan logic posisi gawang seperti BallApproach
                                if (ballOnGoalSide()) {
                                    if (ballOnLeftGoal == 1) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerLeft;
                                        Imu(angleCornerLeft, cSekarang);
                                    } else if (ballOnLeftGoal == 0) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerRight;
                                        Imu(angleCornerRight, cSekarang);
                                    } else {
                                        if (robotPos_Y <= 0) {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalLeft;
                                            Imu(angleGoalLeft, cSekarang);
                                        } else {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalRight;
                                            Imu(angleGoalRight, cSekarang);
                                        }
                                    }
                                } else {
                                    // Tengah lapangan → tendang lurus
                                    modeKick = tendangJauh;
                                    arahGoal = 0;
                                    Imu(0, cSekarang);
                                }
                            }
                            robotDirection = true;
                            printf("[CornerKick] Rotate to arahGoal=%d\n", arahGoal);
                        }
                    }
                }
            }

            // =================================================
            // TIM LAWAN YANG TENDANG (defending)
            else {
                printf("[CornerKick] Defending: dist=%d bodyAligned=%d\n",
                       ballDistance, throwInBodyAligned);

                if (ballLost(50)) {
                    // Bola belum terlihat → search sambil diam
                    throwInBodyAligned = 0;
                    clearVizTarget();
                    SearchBall(1);
                    motion("8");
                } else {
                    trackBall();
                    if (ballDistance < THROW_IN_SAFE_DISTANCE) {
                        // Bola lebih dekat dari jarak aman → mundur sampai THROW_IN_SAFE_DISTANCE
                        throwInBodyAligned = 0;
                        double retreatCm = THROW_IN_SAFE_DISTANCE - ballDistance;
                        if (retreatCm < 0) retreatCm = 0;
                        setVizTargetFromPos(robotPos_X - retreatCm, robotPos_Y);
                        Walk(-aruku * 0.9, 0.0, 0.0);
                        printf("[CornerKick] Defending: mundur ke jarak aman (dist=%d < %d)\n",
                               ballDistance, THROW_IN_SAFE_DISTANCE);
                    } else {
                        // Sudah di jarak aman → BERHENTI, tetap hadap bola
                        throwInBodyAligned = 0;
                        setVizTargetFromPos(robotPos_X, robotPos_Y);
                        if (headPan > 0.2)       Walk(0.0, 0.0,  0.12);
                        else if (headPan < -0.2) Walk(0.0, 0.0, -0.12);
                        else                     motion("8");
                        printf("[CornerKick] Di jarak aman (dist=%d >= %d), diam\n",
                               ballDistance, THROW_IN_SAFE_DISTANCE);
                    }
                }
            }

            return NodeStatus::FAILURE;
        }*/

        // =========================================================
        // SET PLAY LAIN (lainnya)
        // Untuk sementara: diam, tetap track bola kalau kelihatan
        // =========================================================
        if (!ballLost(50)) trackBall();
        else               motion("8");
        return NodeStatus::FAILURE;
    }
    
    //MODE 1
/*
    // Tambahkan variable state baru di bagian atas
bool setPlayExecutorLocked = false;  // flag: sudah ada robot yang claim jadi executor
int setPlayExecutorID = 0;           // ID robot yang jadi executor (0 = belum ada)
int setPlayExecutorDecisionTimer = 0; // timer untuk delay decision (tunggu semua robot scan dulu)

// Fungsi helper untuk broadcast bahwa robot ini lihat bola dan jarak ke bola
void broadcastBallSighting() {
    if (!ballLost(50)) {
        // Kirim data: robot ini lihat bola dengan jarak tertentu
        // Implementasi tergantung sistem komunikasi tim Anda
        // Contoh: sendTeamMessage(robotNumber, ballDistance);
        printf("[SetPlay] Robot %d broadcast: lihat bola, dist=%d\n", robotNumber, ballDistance);
    }
}

// Fungsi helper untuk cek siapa executor berdasarkan jarak
// Return: true jika robot ini yang jadi executor
bool determineExecutorByDistance() {
    // CATATAN: Fungsi ini butuh data dari robot lain via komunikasi tim
    // Asumsi ada fungsi getTeamBallData() yang return siapa aja yang lihat bola
    
    // Untuk implementasi sederhana tanpa komunikasi kompleks:
    // Cek apakah robot ini lihat bola
    if (ballLost(50)) {
        printf("[SetPlay] Robot %d: tidak lihat bola, bukan executor\n", robotNumber);
        return false;
    }
    
    // Cek data tim: siapa lagi yang lihat bola?
    int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;
    bool teamHasBallData = getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true);
    
    if (teamHasBallData && teamBallOwner != 0 && teamBallOwner != robotNumber) {
        // Ada robot lain yang juga lihat bola
        // Bandingkan jarak
        if (ballDistance <= teamBallDist) {
            // Robot ini lebih dekat atau sama → jadi executor
            printf("[SetPlay] Robot %d EXECUTOR: dist=%d vs Robot %d dist=%d\n",
                   robotNumber, ballDistance, teamBallOwner, teamBallDist);
            return true;
        } else {
            // Robot lain lebih dekat → bukan executor
            printf("[SetPlay] Robot %d bukan executor: dist=%d > Robot %d dist=%d\n",
                   robotNumber, ballDistance, teamBallOwner, teamBallDist);
            return false;
        }
    } else {
        // Hanya robot ini yang lihat bola → otomatis executor
        printf("[SetPlay] Robot %d EXECUTOR: hanya robot ini yang lihat bola (dist=%d)\n",
               robotNumber, ballDistance);
        return true;
    }
}

//MODE 1 THROWIN GOALKICK DAN CORNERKICK
// Fungsi helper IMPROVED untuk determine executor
bool isSmartExecutor() {
    // Jika sudah ada executor yang locked, cek apakah itu robot ini
    if (setPlayExecutorLocked) {
        if (setPlayExecutorID == robotNumber) {
            printf("[SetPlay] Robot %d tetap executor (locked)\n", robotNumber);
            return true;
        } else {
            printf("[SetPlay] Robot %d bukan executor (locked ke Robot %d)\n", 
                   robotNumber, setPlayExecutorID);
            return false;
        }
    }
    
    // Belum ada executor yang locked
    // Tunggu beberapa frame agar semua robot punya kesempatan scan bola
    if (setPlayExecutorDecisionTimer < 10) {  // ~0.4 detik delay
        setPlayExecutorDecisionTimer++;
        printf("[SetPlay] Robot %d: waiting decision timer %d/10\n", 
               robotNumber, setPlayExecutorDecisionTimer);
        return false;  // Belum keputusan, semua diam dulu
    }
    
    // Setelah delay, tentukan executor berdasarkan jarak
    bool isExecutor = determineExecutorByDistance();
    
    if (isExecutor) {
        // Robot ini jadi executor → lock decision
        setPlayExecutorID = robotNumber;
        setPlayExecutorLocked = true;
        printf("[SetPlay] Robot %d LOCKED as EXECUTOR\n", robotNumber);
        return true;
    } else {
        // Robot ini bukan executor
        // Cek apakah ada robot lain yang claim executor
        int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;
        if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true) 
            && teamBallOwner != 0) {
            // Ada robot lain yang lebih dekat → lock ke dia
            setPlayExecutorID = teamBallOwner;
            setPlayExecutorLocked = true;
            printf("[SetPlay] Robot %d: Executor locked ke Robot %d\n", 
                   robotNumber, teamBallOwner);
        }
        return false;
    }
}

// Fungsi helper untuk reset state set play
void resetSetPlayState() {
    setPlayExecutorLocked = false;
    setPlayExecutorID = 0;
    setPlayExecutorDecisionTimer = 0;
    throwInPhase = 0;
    throwInKicked = false;
    throwInWait = 0;
    throwInBodyAligned = 0;
    robotDirection = false;
    BallApproachEntry = 0;
}
int  throwInPhase        = 0;
    bool throwInKicked       = false;
    int  throwInWait         = 0;
    int  throwInBodyAligned  = 0; // counter stabilisasi badan sejajar bola (defending)
    bool throwInSupportLatched = false;
    int  throwInSupportGrid = 0;
    int  throwInSupportX = 0;
    int  throwInSupportY = 0;
    int  throwInSupportMode = 0;
    bool throwInAnchorLocked = false;
    int  throwInAnchorOwner = 0;
    int  throwInAnchorBallX = 0;
    int  throwInAnchorBallY = 0;
    bool goalKickExitZone   = false; // flag: robot sedang keluar dari zona penalti saat defend
    int  goalKickBallSide   = 0;     // -1=kiri, +1=kanan (persist antar tick)
    int  goalKickDefendPhase = 0;    // 0=idle, 1=align_body (samping), 2=approach_target (mundur+adaptif)
    int  goalKickAlignWait  = 0;     // counter untuk delay/stability antar phase
    int  goalKickYawStableCount = 0; // counter yaw stabil di sekitar 0 sebelum masuk phase mundur
    
    bool action_kick = false;
    bool robotFinalDirection = false;
    int RotateToGoalEntry = 0, robotFinalDirectionEntry = 0;
    static constexpr int THROW_IN_SAFE_DISTANCE = 150;
NodeStatus RotateToGoal()
{
    // === STOP EMERGENCY ===
    if (State == 3 && Stopped == 1 && secondaryInfo[0] == 0) {
        motion("8");
        printf("[STOP EMERGENCY] Robot stopped during play. State=%d Stopped=%d\n", State, Stopped);
        return NodeStatus::FAILURE;
    }

    // === TIDAK ADA SET PLAY ===
    if (secondaryInfo[0] == 0) {
        if (throwInPhase != 0) {
            resetSetPlayState();
            printf("[SetPlay] Reset state - back to normal play\n");
        }
        return NodeStatus::SUCCESS;
    }

    // =========================================================
    // UNIFIED SET PLAY HANDLER (THROW_IN=4, GOAL_KICK=5, CORNER_KICK=6)
    // =========================================================
    if (secondaryInfo[0] == 4 || secondaryInfo[0] == 5 || secondaryInfo[0] == 6) {
        
        const char* setPlayName = (secondaryInfo[0] == 4) ? "THROW_IN" : 
                                  (secondaryInfo[0] == 5) ? "GOAL_KICK" : "CORNER_KICK";

        // -- STOPPED (Stopped=1): wasit posisikan bola --
        if (Stopped == 1) {
            resetSetPlayState();
            motion("8");  // badan diam
            
            // Kepala tetap search/track ball
            if (!ballLost(50))
                trackBall();
            else
                SearchBall(1);
                
            printf("[%s] STOPPED: searchball, KickOff=%d barelang=%d\n",
                   setPlayName, KickOff, barelang_color);
            return NodeStatus::FAILURE;
        }

        // -- Transisi ke Phase 2 setelah Resume --
        if (throwInPhase != 2 && throwInPhase != 3) {
            throwInPhase = 2;
            throwInWait = 0;
            throwInKicked = false;
            throwInBodyAligned = 0;
            robotDirection = false;
            BallApproachEntry = 0;
            setPlayExecutorDecisionTimer = 0;  // reset decision timer
            printf("[%s] RESUME-EXECUTING: KickOff=%d barelang=%d\n",
                   setPlayName, KickOff, barelang_color);
        }

        // -- Phase 3: sudah tendang, tunggu setPlay reset ke 0 --
        if (throwInPhase == 3) {
            motion("8");
            if (!ballLost(50))
                trackBall();
            else
                SearchBall(1);
            printf("[%s] DONE: tunggu ball free (setPlay=0)\n", setPlayName);
            return NodeStatus::FAILURE;
        }

        // -- Delay kecil setelah pertama masuk phase 2 --
        if (throwInWait < 5) {
            motion("8");
            if (!ballLost(50))
                trackBall();
            else
                SearchBall(1);
            throwInWait++;
            return NodeStatus::FAILURE;
        }

        // =================================================
        // TIM KITA YANG TENDANG (attacking)
        // Sistem SMART: 
        // - 1 robot lihat → dia executor
        // - >1 robot lihat → yang terdekat executor
        // - Robot lain DIAM sambil search/track
        // =================================================
        if (KickOff == barelang_color) {
            
            // Broadcast data bola (untuk komunikasi tim)
            broadcastBallSighting();
            
            // Tentukan executor dengan sistem smart
            if (!isSmartExecutor()) {
                // BUKAN EXECUTOR → DIAM DI TEMPAT, kepala search/track
                motion("8");
                
                if (!ballLost(50)) {
                    trackBall();
                    printf("[%s] Robot %d: WAITING (executor=%d, myDist=%d) - tracking\n",
                           setPlayName, robotNumber, setPlayExecutorID, ballDistance);
                } else {
                    SearchBall(1);
                    printf("[%s] Robot %d: WAITING (executor=%d) - searching\n",
                           setPlayName, robotNumber, setPlayExecutorID);
                }
                
                return NodeStatus::FAILURE;
            }

            // ========================================
            // ROBOT INI ADALAH EXECUTOR → EKSEKUSI SET PLAY
            // ========================================
            printf("[%s] Robot %d EXECUTING: dist=%d yaw=%d headTilt=%.2f\n",
                   setPlayName, robotNumber, ballDistance, msg_yaw, (double)headTilt);

            if (ballLost(35)) {
                // Bola hilang → SearchBall dengan rotate bertahap
                throwInBodyAligned++;
                SearchBall(1);

                if (throwInBodyAligned < 50) {
                    // Fase 1: Scan di tempat
                    motion("8");
                    printf("[%s] Executor: Ball lost, scan di tempat (%d)\n", 
                           setPlayName, throwInBodyAligned);
                } else if (throwInBodyAligned < 100) {
                    // Fase 2: Rotate 90°
                    int targetAngle = (msg_yaw + 90) % 360;
                    if (targetAngle > 180) targetAngle -= 360;
                    Imu(targetAngle, cSekarang);
                    printf("[%s] Executor: Rotate 90° (target=%d, yaw=%d)\n", 
                           setPlayName, targetAngle, msg_yaw);
                } else if (throwInBodyAligned < 150) {
                    // Fase 3: Rotate 180°
                    int targetAngle = (msg_yaw + 180) % 360;
                    if (targetAngle > 180) targetAngle -= 360;
                    Imu(targetAngle, cSekarang);
                    printf("[%s] Executor: Rotate 180° (target=%d, yaw=%d)\n", 
                           setPlayName, targetAngle, msg_yaw);
                } else {
                    // Fase 4: Rotate 270°
                    int targetAngle = (msg_yaw + 270) % 360;
                    if (targetAngle > 180) targetAngle -= 360;
                    Imu(targetAngle, cSekarang);
                    if (throwInBodyAligned > 200) throwInBodyAligned = 0;
                }
                
                throwInKicked = false;
                robotDirection = false;
            } else {
                // Bola ketemu
                throwInBodyAligned = 0;
                trackBall();

                if (headTilt < cAktif) {
                    // Bola masih jauh → followBall
                    followBall(0);
                    throwInKicked = false;
                    robotDirection = false;
                    printf("[%s] Executor: followBall (headTilt=%.2f)\n", setPlayName, headTilt);
                } else {
                    // Sudah dekat → positioning dan tendang
                    if (headTilt >= cAktif && headPan >= -0.3 && headPan <= 0.3) {
                        if (!robotDirection) {
                            // Tentukan arah tendang
                            int gridAngle = getKickAngleByGrid();
                            if (gridAngle != 0) {
                                modeKick = tendangJauh;
                                arahGoal = gridAngle;
                                printf("[%s] GRID KICK: grid=%d -> arahGoal=%d\n",
                                       setPlayName, getRobotGrid(), gridAngle);
                                Imu(arahGoal, cSekarang);
                            } else {
                                if (ballOnGoalSide()) {
                                    if (ballOnLeftGoal == 1) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerLeft;
                                        Imu(angleCornerLeft, cSekarang);
                                    } else if (ballOnLeftGoal == 0) {
                                        modeKick = tendangJauh;
                                        arahGoal = angleCornerRight;
                                        Imu(angleCornerRight, cSekarang);
                                    } else {
                                        if (robotPos_Y <= 0) {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalLeft;
                                            Imu(angleGoalLeft, cSekarang);
                                        } else {
                                            modeKick = tendangJauh;
                                            arahGoal = angleGoalRight;
                                            Imu(angleGoalRight, cSekarang);
                                        }
                                    }
                                } else {
                                    modeKick = tendangJauh;
                                    arahGoal = 0;
                                    Imu(0, cSekarang);
                                }
                            }
                            robotDirection = true;
                        } else {
                            // Tendang
                            kick(modeKick);
                            throwInKicked = false;  // reset untuk ngejar lagi
                            robotDirection = false;
                            printf("[%s] KICK! arahGoal=%d mode=%d\n", 
                                   setPlayName, arahGoal, modeKick);
                        }
                    } else {
                        // Adjust position
                        if (headPan > 0.3)       Walk(0.0, 0.0,  0.1);
                        else if (headPan < -0.3) Walk(0.0, 0.0, -0.1);
                        else                     Walk(aruku * 0.3, 0.0, 0.0);
                    }
                }
            }
        }

        // =================================================
        // TIM LAWAN YANG TENDANG (defending)
        // SEMUA ROBOT DIAM, kepala search/track ball
        // =================================================
        else {
            motion("8");  // DIAM DI TEMPAT
            
            if (!ballLost(50)) {
                trackBall();
                printf("[%s] Robot %d DEFENDING: tracking ball (dist=%d)\n",
                       setPlayName, robotNumber, ballDistance);
            } else {
                SearchBall(1);
                printf("[%s] Robot %d DEFENDING: searching ball\n",
                       setPlayName, robotNumber);
            }
        }

        return NodeStatus::FAILURE;
    }

    // Jika sampai sini, tidak ada set play yang cocok
    return NodeStatus::SUCCESS;
}
*/

    bool FirstKicked = false;
    int FirstKickEntry = 0;
    NodeStatus FirstKick()
    {
        
        return NodeStatus::FAILURE;
    }

    NodeStatus DoneKick()
    {
    	if (isKicked)
    	{
    		return NodeStatus::SUCCESS;
    	}
        return NodeStatus::FAILURE;
    }

    int SearchAfterKickEntry = 0;
    NodeStatus SearchAfterKick()
    {
    	if (SearchAfterKickEntry > 5)
    	{
    		motion("0");
    		//printf("...myTurn!!!\n");
            stateCondition = 232;
            //ballDistance = 1;
			if (ballLost(35))
			{
				if (modeKick == 3)
				{
				    headMove(-1.6, -1.6);
				} else if (modeKick == 4)
				{
				    headMove(1.6, -1.6);
				} else 
				{
					if (modeKick == 5)
				   	{
				   		headMove(0.6, -1.6);
				   		sleep(1);
				   	} else if (modeKick == 6)
				   	{
				   		headMove(-0.6, -1.6);
				   	} else 
				   	{
				   		headMove(0.0, -1.6);
				   	}
				}
			} else 
			{
				if (((modeKick == tendangJauh || modeKick == 3 || modeKick == 4) && second > 4) || ((modeKick == 5 || modeKick == 6) && second > 4))
				{
					trackBall();
				}
			}
			if (modeKick == 5 || modeKick == 6)
			{
				cekWaktu(1); // waktu tendang dekat
			} else 
			{
				cekWaktu(1); // waktu tendang jauh
			}
			if (timer)
			{
				isKicked = false;
			}
		} else 
		{
			setWaktu();
			BallApproachEntry = 0;
			SearchAfterKickEntry++;
		}
        return NodeStatus::FAILURE;
    }

    int SearchingBallEntry = 0;
    NodeStatus SearchingBall()
    {
        
        return NodeStatus::FAILURE;
    }
    
        double releasePan = 0.0;
        double releasePanRate = 0.02; // Kecepatan sweep pelan
        void releaseSearchBall()
        {
            // Sweep horizontal pelan: -0.8 sampai 0.8
            releasePan += releasePanRate;
            if (releasePan >= 0.8 || releasePan <= -0.8) {
                releasePanRate *= -1;
            }
            // Clamp
            if (releasePan > 0.8) releasePan = 0.8;
            if (releasePan < -0.8) releasePan = -0.8;
            
            // Gerakkan kepala (tilt tetap -1.2, cukup melihat depan-bawah)
            headMove(releasePan, -1.2);
            
            // SUPPRESS data bola agar sistem menganggap bola tidak terdeteksi
            // → walking engine tetap full speed
            Ball_X = -1;
            Ball_Y = -1;
        }
        
        
    NodeStatus release()
    {
        if (Release)
        {
            printf("........................release\n");

            if (!releaseInitDone) {
                deltaPos_X = 0;
                deltaPos_Y = 0;
                cnt_move_to_grid = 0;
                posRotateNew = false;
                doneMoved = false;
                releaseInitDone = true;
            }
            
	   
            // Update posisi robot berdasarkan odometri yang sudah di-reset
            robotPos_X = deltaPos_X + initialPos_X;
            robotPos_Y = deltaPos_Y + initialPos_Y;

            if (doneMoved)
            {
                motion("0");
                reset_velocity();
                stateCondition = 0;
                Pickup = false;
                printf("........................release SELESAI\n");
                return NodeStatus::SUCCESS;
            } else 
            {
                // Kepala bergerak natural (sweep pelan) tapi data bola di-suppress
                // agar walking engine tidak melambat saat kamera menangkap bola
                releaseSearchBall();
                new_out_pos(-400, 0, true);
            }
            stateCondition = 0;
            return NodeStatus::FAILURE;
        }
        return NodeStatus::SUCCESS;
    }
/*
   NodeStatus release()
    {
        if (Release)
        {
            printf("........................release\n");
            if (msg_yaw > 0) // dari sisi kiri
            {
                initialPos_X = -250;
                initialPos_Y = -305;
            } else if (msg_yaw < 0) //dari sisi kanan
            {
                initialPos_X = -250;
                initialPos_Y = 305;
            }

            // Update posisi robot untuk lokalisasi ZMQ
            //robotPos_X = deltaPos_X + initialPos_X;
            //robotPos_Y = deltaPos_Y + initialPos_Y;

            // Reset state goalkeeper SEKALI saja (saat pertama kali masuk release)
            if (!releaseInitDone) {
                stateCondition = 0;
                doneBanting = false;
                tendang = false;
                robotDirection = false;
                cnt_move_to_grid = 0;
                posRotateNew = false;
                releaseInitDone = true;
            }

            if (doneMoved)
            {
                
                threeSearchBall();
                motion("0");
                reset_velocity();
                printf("........................release SELESAI\n");
                Pickup = false;
                stateCondition = 8;
                
            } else 
            {
                //threeSearchBall(); 
                new_out_pos(-400, 0, true);
                
            }
            
            delayWaitBall = 0;
            printf("........................2548\n");
	    return NodeStatus::FAILURE;
            
        } 
        /*else 
        {
            trackBall();
            if (delayWaitBall > 20)
            {
                Pickup = false;
            } else 
            {
                delayWaitBall++;
            }
        }
        return NodeStatus::SUCCESS;
    }
*/    
    
   NodeStatus Search()
    {
        //PROTEKSI GC 2026: PENALTY KICK
        if (StatePenaltyKick() == NodeStatus::SUCCESS) {
            motion("0");
            trackBall();
            return NodeStatus::FAILURE;
        }

        // reset semua variabel goalkeeper saat masuk Set (setelah goal/restart)
        stateCondition = 0;
        doneBanting = false;
        tendang = false;
        ballPos = false;
        robotDirection = false;
        doneMoved = false;
        posRotateNew = false;
        releaseInitDone = false;
        tunggu = 0;
        delay = 0;
        delayWaitBall = 0;
        goalKickKicked = false;
        goalKickPhase = 0;
        resetCase0();

        if (ballLost(20))
        {
            motion("0");
            searchBallRectang(-1.6, -1.6, -0.8, 1.6);
        }
        else
        {
            motion("0");
            trackBall();
        }
        Pickup = false;
        return NodeStatus::FAILURE;
    }

    NodeStatus GameControllerInit()
    {
        // printf("...GameControllerInit\n");
        if (State == 0) {
            motion("0");
            Pickup = false;
            // printf("...GcInit::SUCCESS\n");
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus GameControllerReady()
    {
        
        // printf("...GameControllerReady\n");
        if (State == 1) {
           if(Pickup) {
            motion("0");
            return NodeStatus::FAILURE;
        } 
            // Saat Stopped=1: SEMUA robot WAJIB diam, di STATE APAPUN
            // Tidak boleh locomote, bahkan bangun dari jatuh
            if (Stopped == 1) {
                motion("0");  // FREEZE: stop walk + disable fall check
                printf("robot diam");
                return NodeStatus::SUCCESS;
            }

            // printf("...GcReady::SUCCESS\n");
            foundBall = 0;
            stateCondition = 272;
            ballDistance = 999;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus GameControllerSet()
    /*{
	
        // printf("...GameControllerSet\n");
        if (State == 2) {
        if(Pickup) {
            motion("0");
            return NodeStatus::FAILURE;
        }
            motion("0");
            // printf("...GcSet::SUCCESS\n");
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }*/
    {
    
    //if (State == 2 && msg_peluit_hz >= 3000.0) {//new_out_pos(-50, 100,true);
    if(Pickup) {
        motion("0");
        return NodeStatus::FAILURE;
    }

    if (State == 2 && msg_peluit_hz >= 3000.0 && KickOff == barelang_color) {//new_out_pos(-50, 100,true);
        bypass_peluit_aktif = true;
    }

    if (State == 2 && bypass_peluit_aktif == true) {
        if(Pickup){
            motion("0");
            return NodeStatus::FAILURE;
        }
        return NodeStatus::FAILURE;
    }
    
    if(State == 2){
        motion("0");
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::FAILURE; // Jika bukan State 2, gagal.
    }
    
    bool bypass_peluit_aktif = false;
    NodeStatus GameControllerPlay()
    /*{
        // printf("...GameControllerPlay\n");
        if (State == 3) {

            if (Stopped == 1) {
            motion("0");  // FREEZE: stop walk + disable fall check
            printf("robot freeze\n");
            return NodeStatus::FAILURE;
        }
            // printf("...GcPlay::SUCCESS\n");
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }*/
    {
   
    if (State == 2 && msg_peluit_hz >= 3500.0) {//new_out_pos(-50, 100,true);
        bypass_peluit_aktif = true; 
    }

    if (State == 3 || bypass_peluit_aktif) { 

        if (State < 2) { 
            bypass_peluit_aktif = false; 
        }

        return NodeStatus::SUCCESS;
    }
    
    // Jika tidak ada Play dari GC dan tidak ada peluit, tetap diam
    return NodeStatus::FAILURE;
    }

    NodeStatus GameControllerFinish()
    {
        // printf("...GameControllerFinish\n");
        if (State == 4) {
            // printf("...GcFinish::SUCCESS\n");
            motion("0");
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    bool finishFirstKick = false;
    bool doneFirstKick = false;
    int finishKick = 0;
    bool kickOffDone = false;
    int executeHeadMove = 0;
    NodeStatus StateKickOff()
    {
    //trackBall();
    static int timeoutHilangBola = 0;
/*
    if (ballLost(20)) 
    {
        panSearchBall(-1.2);   
        timeoutHilangBola++;
        printf("...ini bolanya lagi hilang...\n");
        if (timeoutHilangBola > 300) 
        {
            
            timeoutHilangBola = 0; 
            finishFirstKick = true;
            FirstKicked = true;
            return NodeStatus::SUCCESS; 
        }
    } 
    else 
    {
        printf("...stuck dikondisi timeout...\n");    
        timeoutHilangBola = 0;
    
    }*/
        if(State == 3 && Stopped ==1 && secondaryInfo[0] == 0) {
           motion("8");
           return NodeStatus::FAILURE;
           
           }
           
        if (secondaryInfo[0] == 1 || secondaryInfo[0] == 2 || secondaryInfo[0] == 3 || secondaryInfo[0] == 4 || secondaryInfo[0] == 5 || secondaryInfo[0] == 6)
        {
           return RotateToGoal();
         }
         
        if (finishFirstKick)
        {
            robotKick = 1;
            return NodeStatus::SUCCESS;
        }
        
        if (ballLost(35)){
            searchBallBreak();
        }
        
        else{
        // Kickoff GC26: jika sudah ball free (setPlay NONE + play tidak stopped),
        // langsung akhiri fase kickoff agar semua robot masuk normal play.
        //if (State == 3 && secondaryInfo[0] == 0 && Stopped == 0)
        /*if (State == 3 && secondaryInfo[0] == 0 && (SecondaryTime <= 0 || KickOff == 255))
        {
            finishFirstKick = true;
            FirstKicked = true;
            doneFirstKick = false;
            kickOffDone = false;
            action_kick = false;
            action_afterKick = false;
            isKicked = false;
            robotDirection = false;
            ballPos = false;
            tendang = false;
            motion("0");
            return NodeStatus::SUCCESS;
        }*/
 // PASTIKAN variabel tunggu dideklarasikan sebagai static
        // Taruh ini di awal fungsi atau pastikan ini tidak ter-reset ke 0
        if (doneFirstKick)
        {
            //if (abs(abs(koorRobotX) - abs(robotPos_X)) > 50)
            if (robotPos_X > 0)
            {
                finishFirstKick = true;
            } 
        }
    
       
        if (FirstKicked)
        {
            if (role == 0)
            {
                if (robot1BackIn == 1 || robot2BackIn == 1 || robot3BackIn == 1 || robot4BackIn == 1 || robot5BackIn == 1)
                {
                    bodyTracked = true;
                    finishFirstKick = true;
                }
    
                if (ballLost(20))
                {
                    //walkGrid(27, -50, 50);
                    walkTarget(-50, 0);
                    printf("...BallLost when kick off...\n");
                    robotDirection = ballPos = tendang = false;
                    //finishFirstKick = true;   --RC24
                    //finishKick = 15;          --RC24
                    //stateCondition = 272;
                    //foundBall = 0;
                    //ballDistance = 999;
                }
                else
                {
                    if (body_tracked)
                    {
                        trackBall();
                        if (delayWaitBall > 10)
                        {
                            tracked = true;
                        } else 
                        {
                            delayWaitBall++;
                            Walk(0.0, 0.0, 0.0);
                            printf("...Wait ball...\n");
                        }
    
                        if (tracked)
                        {
                            if (headTilt >= cAktif) // && headPan >= -0.6 && headPan <= 0.6) //+0.1
                            {
                                if (robotDirection)
                                {
                                    printf("...kick\n");
                                    if (tendang)
                                    {
                                       
                                        isKicked = action_walk = finishFirstKick = true;
                                        searchKe = delayWaitBall = 0;
                                        robotDirection = ballPos = tendang = false;
                                        printf("...Tendang Kick Off...\n");
                                       
                                    } else 
                                    {
                                        //motion("0");
                                        doneMoved =  false;
                                        kick(modeKick);
                                    }
                                } else
                                { 
                                    if (msg_strategy == 1)
                                    {
                                        modeKick = 5;
                                        Imu(0, cSekarang); //Imu(30, cSekarang);
                                    } else if (msg_strategy == 2)
                                    {
                                        modeKick = 6;
                                        Imu(0, cSekarang);
                                    } else 
                                    {	
                                        modeKick = 7;
                                        Imu(0, cSekarang);
                                    }
                                }
                            } else {
                                followBall(0);
                                robotDirection = ballPos = tendang = false;
                                //if (headPan > (panTengah + 0.2) || headPan < (panTengah - 0.2)) {
                                //    body_tracked = false;
                                //}
                            }
                        }
                    } else 
                    {
                        trackBall();
                        if (delayWaitBall > 20)
                        {
                            newBodyTracking();
                        } else 
                        {
                            delayWaitBall++;
                        }
                    }
                }
            } else if (role == 1)
            {
                if (robot1BackIn == 1 || robot2BackIn == 1 || robot3BackIn == 1 || robot4BackIn == 1 || robot5BackIn == 1)
                {
                    finishFirstKick = true;
                } else 
                {
                    if (ballLost(20))
                    {
                        foundBall = 0;
                        ballDistance = 999;
                        delayWaitBall = 0;
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                    } else 
                    {
                        trackBall();
                        if (delayWaitBall > 20)
                        {
                            foundBall = 1;
                            // if (BallGrid <= 17)
                            if (ballDistance <= 85)
                            {
                                finishFirstKick = true;
                            }
                        } else 
                        {
                            delayWaitBall++;
                        }
                    }
                }
            }
        }
        }
static int tunggu = 0; 
static int waktukickoff = 0;
        if (KickOff == barelang_color ) //15 sebanrnya 15
        {
            // Tim kita yang kickoff
            //FirstKicked = true; 
            FirstKicked = finishFirstKick = true;
        } 
        else 
        {
            setWaktu();
            if (ballLost(20))
            {
                searchBallBreak(); 
                motion("0");
            } 
            else 
            {
                
                trackBall();
                static float initialHeadPan = 0.0;

                if (tunggu == 0) 
                {
                    //FirstKicked = true;
                    initialHeadPan = headPan;
                    printf("...Mengunci posisi awal bola: %f...\n", initialHeadPan);
                }


                if (tunggu <= 1000) 
                {

                    float panThreshold = 0.8 ; 
                    
                    if (abs(headPan - initialHeadPan) > panThreshold || waktukickoff > 250) 
                    {
                        printf("...BOLA BERGERAK! Batal Kickoff...\n", headPan);
                        FirstKicked = finishFirstKick = true;
                        tunggu = 0;
                       // waktukickoff = 0; // Reset untuk state berikutnya
                    } 
                    else 
                    {

                        if (tunggu % 100 == 0) {
                            printf("...Mengawasi pergerakan lawan... (Tick: %d/1000)\n", tunggu);
                        }
                        motion("0");
                        tunggu++;
                        waktukickoff++;
                    }
                } 
                else 
                {

                    printf("...Waktu 10 Detik Habis! Free Play...\n");
                    FirstKicked = finishFirstKick = true;
                    tunggu = 0;
                    waktukickoff =0; // Reset untuk state berikutnya
                }
            }
        }
    /*
        if (KickOff == barelang_color)
        {
            FirstKicked = true;
        } else 
        {
            if (SecondaryTime <= 2)
            {
                if (useKickOffGoal)
                {
                    finishFirstKick = true;
                }
                FirstKicked = true;
                cekWaktu(5);
                if (timer)
                {
                    if (robotNumber == 2)
                    {
                        if (role == 1 && robot3FBall == 0)
                        {
                            //walkGrid(15, 0, 25);
                            finishFirstKick = true;
                        }
                    } else if (robotNumber == 3)
                    {
                        if (role == 1 && robot2FBall == 0)
                        {
                            //walkGrid(15, 0, 25);
                            finishFirstKick = true;
                        }
                    }
                }
            } else 
            {
                setWaktu();
                if (ballLost(20))
                {
                    searchBallBreak(); //searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                    motion("0");
                } else 
                {
                    trackBall();
                    
                    // Variabel static untuk mempertahankan nilai di antara iterasi loop node ROS
                    static float initialHeadPan = 0.0;
                    static bool panLocked = false;
    
                    // Asumsi Loop Behavior Tree berjalan di 100 Hz.
                    // 400 tick = 4 detik, 1000 tick = 10 detik.
                    if (tunggu < 1000) 
                    {
                        // Hitung mundur 4 detik agar pan robot stabil di 0,0 lapangan
                        motion("0");
                        tunggu++;
                        panLocked = false; 
                    } 
                    else if (tunggu >= 400 && tunggu <= 1000) 
                    {
                        // Kunci nilai pan pada detik ke-4
                        if (!panLocked) 
                        {
                            initialHeadPan = headPan;
                            panLocked = true;
                        }
    
                        // Threshold 0.3 radian untuk mendeteksi bola ditendang lawan
                        float panThreshold = 0.3; 
                        
                        if (abs(headPan - initialHeadPan) > panThreshold) 
                        {
                            FirstKicked = finishFirstKick = true;
                            
                            // Reset untuk siklus state berikutnya
                            tunggu = 0;
                            panLocked = false;
                        } 
                        else 
                        {
                            motion("0");
                            tunggu++;
                        }
                    } 
                    else 
                    {
                        // Melewati 10 detik, set flag menjadi true dan reset
                        FirstKicked = finishFirstKick = true;
                        tunggu = 0;
                        panLocked = false;
                    }
                }
            }
        }*/
    
        // if (!finishFirstKick)
        // {
        //     if (robotPos_X > 50)
        //     {
        //         finishFirstKick = true;
        //     }
        // }
        
        
        return NodeStatus::FAILURE;
    }
    
    int getClosestGridID(int targetX, int targetY) 
{
    // 1. DEFINISIKAN UKURAN LAPANGAN & GRID (SESUAIKAN DENGAN REALITA LAPANGANMU)
    // Asumsi total 54 grid berasal dari 9 kolom dan 6 baris
    const int NUM_COLS = 9;      
    const int NUM_ROWS = 6;      
    
    // Asumsi ukuran 1 kotak grid (misalnya 100 cm x 100 cm)
    const int GRID_WIDTH = 100;  
    const int GRID_HEIGHT = 100; 

    // 2. GESER KOORDINAT (OFFSET)
    // Di robotika (terutama robot soccer), titik (0,0) biasanya ada di TENGAH lapangan.
    // Rumus pembagian array mengharuskan (0,0) ada di ujung (kiri bawah/atas).
    // Jadi, kita geser dulu koordinat bola agar nilainya positif semua.
    int shiftedX = targetX + ((NUM_COLS * GRID_WIDTH) / 2);
    int shiftedY = targetY + ((NUM_ROWS * GRID_HEIGHT) / 2);

    // 3. CARI BOLA ADA DI KOLOM DAN BARIS BERAPA
    int col = shiftedX / GRID_WIDTH;
    int row = shiftedY / GRID_HEIGHT;

    // 4. CLAMPING (MENCEGAH ERROR JIKA BOLA KELUAR LAPANGAN)
    // Jika nilai targetX / targetY melampaui ukuran lapangan, paksa masuk ke grid paling pinggir.
    if (col < 0) col = 0;
    if (col >= NUM_COLS) col = NUM_COLS - 1;
    if (row < 0) row = 0;
    if (row >= NUM_ROWS) row = NUM_ROWS - 1;

    // 5. KONVERSI KOLOM & BARIS MENJADI ID GRID (0 - 53)
    // Asumsi penomoran grid berurutan dari kiri ke kanan, lalu lanjut ke baris berikutnya.
    int bestGrid = (row * NUM_COLS) + col;

    return bestGrid;
}


bool isValidBallTeamData(int status, int found, int distance, int x, int y)
{
    if (status != 1 || found != 1) return false;
    if (distance <= 0 || distance >= 999 || distance == 232) return false;
    if (x == 999 || y == 999 || x == -28 || y == -28) return false;
    return true;
}

bool getBestTeamBallData(int &outX, int &outY, int &outDistance, int &outRobot, bool includeSelf)
{
    bool hasData = false;
    outDistance = 999;
    outRobot = 0;

    auto consider = [&](int id, int status, int found, int distance, int x, int y) {
        if (!isValidBallTeamData(status, found, distance, x, y)) return;
        if (!hasData || distance < outDistance || (distance == outDistance && id < outRobot)) {
            outX = x;
            outY = y;
            outDistance = distance;
            outRobot = id;
            hasData = true;
        }
    };

    if (includeSelf) {
        consider(robotNumber, 1, foundBall, ballDistance, BallX, BallY);
    }

    if (robotNumber != 1) consider(1, robot1Status, robot1FBall, robot1DBall, robot1XBall, robot1YBall);
    if (robotNumber != 2) consider(2, robot2Status, robot2FBall, robot2DBall, robot2XBall, robot2YBall);
    if (robotNumber != 3) consider(3, robot3Status, robot3FBall, robot3DBall, robot3XBall, robot3YBall);
    if (robotNumber != 4) consider(4, robot4Status, robot4FBall, robot4DBall, robot4XBall, robot4YBall);
    if (robotNumber != 5) consider(5, robot5Status, robot5FBall, robot5DBall, robot5XBall, robot5YBall);
    if (robotNumber != 6) consider(6, robot6Status, robot6FBall, robot6DBall, robot6XBall, robot6YBall);

    return hasData;
}
bool getTeamBallDataByRobotId(int id, int &outX, int &outY, int &outDistance)
    {
        if (id == robotNumber) {
            if (!isValidBallTeamData(1, foundBall, ballDistance, BallX, BallY)) return false;
            outX = BallX;
            outY = BallY;
            outDistance = ballDistance;
            return true;
        }

        auto assignIfValid = [&](int status, int found, int distance, int x, int y) {
            if (!isValidBallTeamData(status, found, distance, x, y)) return false;
            outX = x;
            outY = y;
            outDistance = distance;
            return true;
        };

        if (id == 1) return assignIfValid(robot1Status, robot1FBall, robot1DBall, robot1XBall, robot1YBall);
        if (id == 2) return assignIfValid(robot2Status, robot2FBall, robot2DBall, robot2XBall, robot2YBall);
        if (id == 3) return assignIfValid(robot3Status, robot3FBall, robot3DBall, robot3XBall, robot3YBall);
        if (id == 4) return assignIfValid(robot4Status, robot4FBall, robot4DBall, robot4XBall, robot4YBall);
        if (id == 5) return assignIfValid(robot5Status, robot5FBall, robot5DBall, robot5XBall, robot5YBall);
        if (id == 6) return assignIfValid(robot6Status, robot6FBall, robot6DBall, robot6XBall, robot6YBall);
        return false;
    }

/*
    bool isNearestExecutorForThrowIn()
    {
        int bestDistance = 999;
        int bestRobot = 0;
        int dummyX = 0, dummyY = 0;

        if (!getBestTeamBallData(dummyX, dummyY, bestDistance, bestRobot, true)) {
            return false;
        }

        return bestRobot == robotNumber;
    }*/
    bool isRobotActiveByIdForSetPlay(int id)
{
    if (id == robotNumber) return true;
    if (id == 1) return robot1Status == 1;
    if (id == 2) return robot2Status == 1;
    if (id == 3) return robot3Status == 1;
    if (id == 4) return robot4Status == 1;
    if (id == 5) return robot5Status == 1;
    if (id == 6) return robot6Status == 1;
    return false;
}
bool isNearestExecutorForThrowIn()
{
    const int setPlay = secondaryInfo[0];
    const bool isSetPlayExec = (setPlay == 1 || setPlay == 2 || setPlay == 3 || setPlay == 4 || setPlay == 5 || setPlay == 6);
    const bool ourSetPlay = isSetPlayExec && (KickOff == barelang_color);

    // Kalau bukan giliran kita, semua jadi support
    if (!ourSetPlay) {
        return false;
    }

    // ======================================================
    // SISTEM BARU V2: GUNAKAN DATA TIM, TIDAK HARUS LIHAT SENDIRI
    // Robot yang sedang defend bisa jadi executor kalau dia paling dekat!
    // ======================================================
    
    int myDistance = 999;
    bool iSeeBall = false;
    
    // Cek apakah aku lihat bola sendiri
    if (!ballLost(50)) {
        myDistance = ballDistance;
        iSeeBall = true;
        printf("[SetPlay %d] Robot%d: Aku LIHAT bola, jarak=%d\n", setPlay, robotNumber, myDistance);
    } else {
        // Aku tidak lihat bola, tapi coba ambil data dari tim
        int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;
        if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
            // Ada data bola dari tim, hitung jarak aku ke bola
            int dx = teamBallX - robotPos_X;
            int dy = teamBallY - robotPos_Y;
            myDistance = (int)sqrt(dx * dx + dy * dy);
            printf("[SetPlay %d] Robot%d: Tidak lihat bola, tapi estimasi jarak=%d dari posisi tim\n", 
                   setPlay, robotNumber, myDistance);
        } else {
            // Tidak ada data bola sama sekali dari tim
            printf("[SetPlay %d] Robot%d: TIDAK ADA DATA BOLA -> SUPPORT\n", setPlay, robotNumber);
            return false;
        }
    }

    // Sekarang bandingkan dengan robot lain
    int nearestRobot = robotNumber;
    int nearestDistance = myDistance;
    
    // Cek robot 2-6 (SKIP KIPER robot 1)
    for (int id = 2; id <= 6; ++id) {
        if (id == robotNumber) continue; // skip diri sendiri
        if (!isRobotActiveByIdForSetPlay(id)) continue; // skip robot mati

        int bx = 0, by = 0, bd = 999;
        // Cek apakah robot lain lihat bola
        if (getTeamBallDataByRobotId(id, bx, by, bd)) {
            printf("[SetPlay %d] Robot%d: Robot%d lihat bola jarak=%d\n", 
                   setPlay, robotNumber, id, bd);
            
            // Bandingkan jarak
            // Prioritas: yang lihat bola langsung > estimasi dari posisi
            // Kalau sama-sama lihat atau sama-sama estimasi, pilih yang lebih dekat
            bool otherSeeBall = true; // robot lain pasti lihat karena ada di getTeamBallDataByRobotId
            
            if (otherSeeBall && !iSeeBall) {
                // Robot lain lihat langsung, aku cuma estimasi -> dia prioritas
                if (bd < nearestDistance + 50) { // toleransi 50cm
                    nearestDistance = bd;
                    nearestRobot = id;
                }
            } else if (bd < nearestDistance || (bd == nearestDistance && id < nearestRobot)) {
                // Bandingkan jarak murni
                nearestDistance = bd;
                nearestRobot = id;
            }
        }
    }

    // KEPUTUSAN: Aku executor kalau aku yang paling dekat
    bool iAmExecutor = (nearestRobot == robotNumber);
    
    if (iAmExecutor) {
        printf("[SetPlay %d] ✓✓✓ Robot%d JADI EXECUTOR! (jarak=%d, %s) ✓✓✓\n", 
               setPlay, robotNumber, myDistance, iSeeBall ? "LIHAT LANGSUNG" : "ESTIMASI");
    } else {
        printf("[SetPlay %d] Robot%d jadi SUPPORT, executor=Robot%d (jarak=%d)\n", 
               setPlay, robotNumber, nearestRobot, nearestDistance);
    }
    
    return iAmExecutor;
}

// Determine executor during normal play (State == 3)
// Mirrors isNearestExecutorForThrowIn but used when playing.
bool isNearestExecutorForPlay()
{
    if (State != 3) return false;

    int myDistance = 999;
    bool iSeeBall = false;

    if (!ballLost(50)) {
        myDistance = ballDistance;
        iSeeBall = true;
        printf("[Play] Robot%d sees ball, dist=%d\n", robotNumber, myDistance);
    } else {
        int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;
        if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
            int dx = teamBallX - robotPos_X;
            int dy = teamBallY - robotPos_Y;
            myDistance = (int)sqrt(dx*dx + dy*dy);
            printf("[Play] Robot%d estimate dist from team ball=%d\n", robotNumber, myDistance);
        } else {
            printf("[Play] Robot%d: no ball data -> support\n", robotNumber);
            return false;
        }
    }

    int nearestRobot = robotNumber;
    int nearestDistance = myDistance;

    for (int id = 2; id <= 6; ++id) {
        if (id == robotNumber) continue;
        if (!isRobotActiveByIdForSetPlay(id)) continue;

        int bx = 0, by = 0, bd = 999;
        if (getTeamBallDataByRobotId(id, bx, by, bd)) {
            printf("[Play] Robot%d: Robot%d sees ball dist=%d\n", robotNumber, id, bd);
            bool otherSeeBall = true;

            if (otherSeeBall && !iSeeBall) {
                if (bd < nearestDistance + 50) {
                    nearestDistance = bd;
                    nearestRobot = id;
                }
            } else if (bd < nearestDistance || (bd == nearestDistance && id < nearestRobot)) {
                nearestDistance = bd;
                nearestRobot = id;
            }
        }
    }

    bool iAmExecutor = (nearestRobot == robotNumber);
    if (iAmExecutor) {
        printf("[Play] ✓✓✓ Robot%d IS EXECUTOR (dist=%d) ✓✓✓\n", robotNumber, myDistance);
    } else {
        printf("[Play] Robot%d SUPPORT, executor=Robot%d (dist=%d)\n", robotNumber, nearestRobot, nearestDistance);
    }
    return iAmExecutor;
}
/*
int getDeterministicSetPlayExecutorFallback()
{
    // Fallback darurat: utamakan non-kiper (2..6), kiper jadi opsi terakhir.
    for (int id = 2; id <= 6; ++id) {
        if (isRobotActiveByIdForSetPlay(id)) {
            return id;
        }
    }
    for (int id = 1; id <= 1; ++id) {
        if (isRobotActiveByIdForSetPlay(id)) {
            return id;
        }
    }
    return robotNumber;
}

bool isNearestExecutorForThrowIn()
{
    static int latchedSetPlay = -1;
    static int latchedExecutor = 0;
    static int latchTtl = 0;
    static unsigned int seenTick = 0;
    static unsigned int firstSeenStamp[7] = {0, 0, 0, 0, 0, 0, 0};

    const int setPlay = secondaryInfo[0];
    const bool isSetPlayExec = (setPlay == 4 || setPlay == 5 || setPlay == 6);
    const bool ourSetPlay = isSetPlayExec && (KickOff == barelang_color);

    if (!ourSetPlay) {
        latchedSetPlay = -1;
        latchedExecutor = 0;
        latchTtl = 0;
        seenTick = 0;
        for (int id = 1; id <= 6; ++id) firstSeenStamp[id] = 0;
        return false;
    }

    int bestDistance = 999;
    int bestRobot = 0;
    int dummyX = 0, dummyY = 0;
    const bool hasBallOwner = getBestTeamBallData(dummyX, dummyY, bestDistance, bestRobot, true);

    if (latchedSetPlay != setPlay) {
        seenTick = 0;
        for (int id = 1; id <= 6; ++id) firstSeenStamp[id] = 0;
    }

    seenTick++;

    for (int id = 1; id <= 6; ++id) {
        int bx = 0, by = 0, bd = 999;
        if (firstSeenStamp[id] == 0 && getTeamBallDataByRobotId(id, bx, by, bd)) {
            firstSeenStamp[id] = seenTick;
        }
    }

    int firstSeenExecutor = 0;
    unsigned int firstSeenTime = 0;
    for (int id = 2; id <= 6; ++id) {
        if (!isRobotActiveByIdForSetPlay(id)) continue;
        if (firstSeenStamp[id] == 0) continue;

        if (firstSeenExecutor == 0 || firstSeenStamp[id] < firstSeenTime ||
            (firstSeenStamp[id] == firstSeenTime && id < firstSeenExecutor)) {
            firstSeenExecutor = id;
            firstSeenTime = firstSeenStamp[id];
        }
    }

    const bool needRelatch =
        (latchedSetPlay != setPlay) ||
        (latchedExecutor <= 0) ||
        (latchTtl <= 0) ||
        !isRobotActiveByIdForSetPlay(latchedExecutor);

    if (needRelatch) {
        if (firstSeenExecutor > 0) {
            latchedExecutor = firstSeenExecutor;
        } else if (hasBallOwner && bestRobot > 1) {
            latchedExecutor = bestRobot;
        } else {
            latchedExecutor = getDeterministicSetPlayExecutorFallback();
        }
        latchedSetPlay = setPlay;
        latchTtl = 60;
    } else {
        if (firstSeenExecutor > 0) {
            // Prioritaskan robot yang pertama melihat bola selama set-play berjalan.
            if (latchedExecutor != firstSeenExecutor) {
                latchedExecutor = firstSeenExecutor;
            }
            latchTtl = 60;
        } else if (hasBallOwner && bestRobot > 1) {
            // Jika belum ada first-seen, pakai nearest non-keeper sebagai transisi cepat.
            if (bestRobot == latchedExecutor || bestDistance < 120) {
                latchedExecutor = bestRobot;
                latchTtl = 60;
            } else {
                latchTtl--;
            }
        } else {
            latchTtl--;
        }
    }

    if (latchedExecutor <= 0) {
        latchedExecutor = getDeterministicSetPlayExecutorFallback();
    }

    return latchedExecutor == robotNumber;
}
*/
    void getThrowInDefendTarget(int ballX, int ballY, int &defendX, int &defendY)
    {
        // Defender mundur ke area gawang sendiri dengan offset dari posisi bola tim.
        defendX = ballX - 220;
        if (defendX < -350) defendX = -350;
        if (defendX > -120) defendX = -120;

        defendY = ballY;
        if (defendY < -250) defendY = -250;
        if (defendY > 250) defendY = 250;
    }
    /*
    void trackHeadWithTeamBall()
    {
        int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;
        if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
            const double dx = (double)teamBallX - robotPos_X;
            const double dy = (double)teamBallY - robotPos_Y;
            double relativeDeg = atan2(dy, dx) * 180.0 / PI - msg_yaw;
            while (relativeDeg > 180.0) relativeDeg -= 360.0;
            while (relativeDeg < -180.0) relativeDeg += 360.0;

            double headPanTarget = relativeDeg * PI / 180.0;
            if (headPanTarget > 1.6) headPanTarget = 1.6;
            if (headPanTarget < -1.6) headPanTarget = -1.6;

            headMove(headPanTarget, -1.2);
            return;
        }

        if (!ballLost(50)) {
            trackBall();
        } else {
            panSearchBall(-1.2);
        }
    }*/
    void getThrowInMarkerTarget(int ballX, int ballY, int &targetX, int &targetY)
    {
        // Marker lawan: jaga jarak sekitar 1.5 meter dari titik bola throw-in.
        int markerBackX = 150;
        if (ballX > 150) {
            markerBackX = 180;
        } else if (ballX < -150) {
            markerBackX = 130;
        }

        targetX = ballX - markerBackX;
        targetY = ballY;

        if (targetX < -440) targetX = -440;
        if (targetX > 440) targetX = 440;
        if (targetY < -300) targetY = -300;
        if (targetY > 300) targetY = 300;
    }

    void trackHeadToFieldPoint(int pointX, int pointY)
    {
        const double dx = (double)pointX - robotPos_X;
        const double dy = (double)pointY - robotPos_Y;
        double relativeDeg = atan2(dy, dx) * 180.0 / PI - msg_yaw;
        while (relativeDeg > 180.0) relativeDeg -= 360.0;
        while (relativeDeg < -180.0) relativeDeg += 360.0;

        double headPanTarget = relativeDeg * PI / 180.0;
        if (headPanTarget > 1.6) headPanTarget = 1.6;
        if (headPanTarget < -1.6) headPanTarget = -1.6;

        headMove(headPanTarget, -1.2);
    }

    void trackHeadWithTeamBall()
    {
        int teamBallX = 0, teamBallY = 0, teamBallDist = 999, teamBallOwner = 0;
        if (getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, true)) {
            trackHeadToFieldPoint(teamBallX, teamBallY);
            return;
        }

        if (!ballLost(50)) {
            trackBall();
        } else {
            panSearchBall(-1.2);
        }
    }
    
    bool trackHeadWithBestVisibleTeamBall(bool includeSelf = false)
    {
        int teamBallX = 0;
        int teamBallY = 0;
        int teamBallDist = 999;
        int teamBallOwner = 0;

        if (!getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, includeSelf)) {
            return false;
        }

        trackHeadToFieldPoint(teamBallX, teamBallY);
        return true;
    }
    
    bool runTeamBallDefendSupport()
    {
        if (!ballLost(10)) {
            return false;
        }

        int teamBallX = 0;
        int teamBallY = 0;
        int teamBallDist = 999;
        int teamBallOwner = 0;
        if (!getBestTeamBallData(teamBallX, teamBallY, teamBallDist, teamBallOwner, false)) {
            return false;
        }

        trackHeadToFieldPoint(teamBallX, teamBallY);
        const double bodyDeg = atan2((double)teamBallY - robotPos_Y, (double)teamBallX - robotPos_X) * 180.0 / PI;
        rotateBodyImuNew((int)bodyDeg);

        const int defendX = -200;
        const int advanceX = -100;
        int defendY = (robotPos_Y < 0.0) ? -50 : 50;
        if (robotPos_Y <= -100.0) {
            defendY = -50;
        } else if (robotPos_Y >= 100.0) {
            defendY = 50;
        }

        if (robotPos_X >= 150.0) {
            new_out_pos(defendX, defendY, true);
            return true;
        }

        const bool atDefendX = abs((int)robotPos_X - defendX) <= 20;
        const bool atDefendY = abs((int)robotPos_Y - defendY) <= 20;

        if (!atDefendX || !atDefendY) {
            new_out_pos(defendX, defendY, true);
            return true;
        }

        new_out_pos(advanceX, defendY, true);
        return true;
    }

    void trackHeadWithThrowInAnchor()
    {
        if (throwInAnchorLocked) {
            trackHeadToFieldPoint(throwInAnchorBallX, throwInAnchorBallY);
            return;
        }

        int anchorX = throwInAnchorBallX;
        int anchorY = throwInAnchorBallY;
        int anchorDist = 999;

        if (throwInAnchorOwner > 0 &&
            getTeamBallDataByRobotId(throwInAnchorOwner, anchorX, anchorY, anchorDist)) {
            trackHeadToFieldPoint(anchorX, anchorY);
            return;
        }

        trackHeadWithTeamBall();
    }
    
    //-------------------------RC2024---------------------------
    /*NodeStatus StateKickOff()
    {
        if (finishFirstKick)
        {
            robotKick = 1;
            return NodeStatus::SUCCESS;
        }

        if (KickOff == barelang_color)
        {
            //finishFirstKick = true;
            FirstKicked = true;
        } else 
        {
            //finishFirstKick = true;
            if (SecondaryTime <= 2)
            {
                if (useKickOffGoal)
                {
                    finishFirstKick = true;
                    finishKick = 1;
                }
                FirstKicked = true;
            } else 
            {
            	setWaktu();
                if (ballLost(35))
                {
                    //searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                    searchBallBreak();
                    motion("0");
                } else 
                {
                    trackBall();
                    if (tunggu > 100)
                    {
                    	hitungGerakBola();
                    	if (abs(deltaY) <= valBolaGerak)
                    	{
                    		motion("0");
                    	} else 
                    	{
                    		FirstKicked = finishFirstKick = true; 
                    		finishKick = 2;
                    	}
                    } else 
                    {
                    	motion("0");
                    	tunggu++;
                    }
                }
            }
        }

        // if (!finishFirstKick)
        // {
        //     if (robotPos_X > 50)
        //     {
        //         finishFirstKick = true;
        //     }
        // }
        
        if (doneFirstKick)
        {
        	//if (abs(abs(koorRobotX) - abs(robotPos_X)) > 50)
        	if (robotPos_X > 0)
        	{
        		finishFirstKick = true;
        		finishKick = 3;
        	} 
        }

       
        if (FirstKicked)
        {
            if (role == 0)
            {
                if (robot1BackIn == 1 || robot2BackIn == 1 || robot3BackIn == 1 || robot4BackIn == 1 || robot5BackIn == 1)
                {
                    bodyTracked = true;
                    finishFirstKick = true;
                    finishKick = 4;
                }

                if (ballLost(10))
                {
                    searchBallBreak(); //--testrobocup
                    //Walk(0.0, 0.0, 0.0); ---testrobocup
                    //walkGrid(27, 0, 50);
                    printf("QQQQQQQQQQQQQQQQQQQQ\n");
                    robotDirection = ballPos = tendang = false;
                    finishFirstKick = true;
                    finishKick = 15;
                    //stateCondition = 272;
                    //foundBall = 0;
                    //ballDistance = 999;
                }
                else
                {   
                    //stateCondition = 232;
                    //foundBall = 1;
                    //ballDistance = 1; 
                    /*trackBall(); 
                    if (headTilt >= cAktif && headPan >= -0.6 && headPan <= 0.6) //+0.1
                    {
                        if (robotDirection)
                        {
                            if (tendang)
                            {
                                //finishFirstKick = true;
                                saveKoordinatRobot();
                                //motion("0");
                                doneFirstKick = true;
                                isKicked = true;
                                //isKicked = action_walk = true;
                            } else 
                            {
                                kick(modeKick);
                                robotDirection = ballPos = tendang = false;
                            }
                        } else 
                        {
                            if (msg_strategy == 0)
                            {
                                modeKick = 7; //
                            } 
                            
                            else if (msg_strategy == 1)
                            {
                                modeKick = 6; //
                            }
                             
                            else if (msg_strategy == 2)
                            {
                                modeKick = 5; //
                            }
                            
                            else 
                            {
                                modeKick = 4; //
                            }
                            Imu(0, cSekarang);
                        }
                    } else 
                    {
                        isKicked = robotDirection = ballPos = tendang = false;
                        followBall(0);
                    } //tambahkan (* & /)
                    if (body_tracked)
		    {
		        if (ballLost(10))
		        {
		            Walk(0.0, 0.0, 0.0);
		            delayWaitBall = 0;
		            tracked = false;
		            printf("WWWWWWWWWWWWWWWWWWWWWWW\n");
		            finishFirstKick = true;
		            finishKick = 16;
		        } else 
		        {
		            trackBall();
		            if (delayWaitBall > 10)
		            {
		                tracked = true;
		            } else 
		            {
		                delayWaitBall++;
		                Walk(0.0, 0.0, 0.0);
		                printf("RRRRRRRRRRRRRRRRRRRRR\n");
		            }

		            if (tracked)
		            {
		            	if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4 && msg_yaw > -5 && msg_yaw < 5) {
		            	    robotDirection = true;
		            	}
		            	if (headTilt >= cAktif && headPan >= -0.6 && headPan <= 0.6) //+0.1
                        	{
		                    if (robotDirection)
		                    {
		                        printf("...kick\n");
		                        if (tendang)
		                        {
		                            if (modeKick == 6)
		                            {
		                                headMove(-1.6, -1.6);
		                            } else if (modeKick == 5 || modeKick == 7)
		                            {
		                                headMove(1.6, -1.6);
		                            } else 
		                            {
		                            	headMove(0.0, -1.6);
		                            }
		                            isKicked = action_walk = finishFirstKick = true;
		                            searchKe = delayWaitBall = 0;
		                            robotDirection = ballPos = tendang = false;
		                            printf("TTTTTTTTTTTTTTTTTTTTT\n");
		                        } else 
		                        {
		                            //motion("0");
		                            kick(modeKick);
		                            
		                        }
		                    } else
		                    { 
		                    	if (msg_strategy == 0)
				        {
				            modeKick = 7; //
				        } 
				            
				        else if (msg_strategy == 1)
				        {
				            modeKick = 6; //
				        }
				             
				        else if (msg_strategy == 2)
				        {
				            modeKick = 5; //
				        }
				            
				        else 
				        {
				            modeKick = 4; //
				        }
		                    	Imu(0, cSekarang);
		                    }
		            	} else {
		            		followBall(0);
		            		robotDirection = ballPos = tendang = false;
		            	}
		            }
		        }
		    } else 
		    {
		        if (ballLost(35))
		        {
		            delayWaitBall = 0;
		            Walk(0.0, 0.0, 0.0);
		            searchBallRectang(-1.6, -1.6, -0.8, 1.6);
		            finishFirstKick = true;
		            finishKick = 17;
		        } else 
		        {
		            trackBall();
		            if (delayWaitBall > 20)
		            {
		                newBodyTracking();
		            } else 
		            {
		                delayWaitBall++;
		            }
		        }
		    }
                }
            } else if (role == 1)
            {
                if (robot1BackIn == 1 || robot2BackIn == 1 || robot3BackIn == 1 || robot4BackIn == 1 || robot5BackIn == 1)
                {
                    //finishFirstKick = true;
                    finishKick = 5;
                } else 
                {
                	cekWaktu(5);
                	if (timer)
                	{
                		finishFirstKick = true;
                		finishKick = 6;
                	}
                    if (ballLost(35))
                    {
                        foundBall = 0;
                        ballDistance = 999;
                        delayWaitBall = 0;
                        searchBallBreak();
                    } else 
                    {
                        trackBall();
                        if (delayWaitBall > 20)
                        {
                            foundBall = 1;
                            if (ballDistance <= 85)
                            {
                                finishFirstKick = true;
                                finishKick = 7;
                            }
                        } else 
                        {
                            delayWaitBall++;
                        }
                    }
                }
            }
        }
        return NodeStatus::FAILURE;
    }*///-----------------------RC2024---------------------------

    int GridTarget = 0, GridX = 0, GridY = 0;
    int xTarget = 0, yTarget = 0;
    int role = 0; // 0 : attacker
    // YAML-configurable values for InitialPosition and RobotPositioning
    int init_play_left_yaw_threshold = 70;
    int init_play_left_x = -250;
    int init_play_left_y = -320;
    int init_play_right_x = -250;
    int init_play_right_y = 320;

    int robocup_yaw = 0;
    int init_robocup_left_x = -100;
    int init_robocup_left_y = -307;
    int init_robocup_right_x = -100;
    int init_robocup_right_y = 307;

    int kickoff_kita_release_left_x = -50;
    int kickoff_kita_release_left_y = 0;
    int kickoff_kita_release_right_x = -50;
    int kickoff_kita_release_right_y = 0;
    int musuh_kickoff_left_x = -100;
    int musuh_kickoff_left_y = 0;
    int musuh_kickoff_right_x = -100;
    int musuh_kickoff_right_y = 0;

    int defend_x = -150;
    int defend_y = 0;

    int release1_x = -300;
    int release1_y = 0;
    int release2_x = -75;
    int release2_y = 0;
    int release3_x = 100;
    int release3_y = 0;
    
    NodeStatus InitialPosition()
    {
         //printf("...InitialPosition\n");
        if (lockInitPos < 10)
        {
            if (State == 3) { //pickup pas playing
                if (msg_yaw >= 70) // dari sisi kiri dia strker
                {

                	initialPos_X = -250; //-250;
                	initialPos_Y = -307;
                	role = 0;
                    /*if (robot5Status == 1) {
                        initialPos_X = -250; //-250;
                        initialPos_Y = -315;
                        role = 1;
                    }
                    else {
                        initialPos_X = -150; //-250; //-200
                        initialPos_Y = -315;
                        role = 0;
                    }*/                    //if (msg_yaw >= 45) {
                    //	role = 0;
                   // } else {
                    	//role = 1;
                    //}
                } else if (msg_yaw < 70) //dari sisi kanan belakang
                {
                	initialPos_X = -250; //-250;
                        initialPos_Y = 307;
                        role = 1;
                    /*if (robot5Status == 1) {
                        initialPos_X = -250; //-250;
                        initialPos_Y = 315;
                        role = 1;
                    }
                    else {
                        initialPos_X = -150; //-250;
                        initialPos_Y = 315;
                        role = 0;
                    }*/
                    //if (msg_yaw <= -45) {
                    	//role = 0;
                    //} else {
                    	//role = 1;
                    //}
                }
            } else if (State == 0 || State == 1 || Release) { //initial || ready || release
                // PERBAIKAN: Tambah condition untuk Release
                // Ketika robot di-release, analisis posisi robot seperti di ready state
                if (modePlay == 0) //nasional
                {
                    if (msg_yaw > -90 && msg_yaw < 90) //hadap depan
                    {
                        initialPos_X = -360;
                        initialPos_Y = 0;
                        role = 0;
                    } else if (msg_yaw > 90)
                    {
                        initialPos_X = -360;
                        initialPos_Y = 130;
                        role = 1;
                    } else if (msg_yaw < -90)
                    {
                        initialPos_X = -360;
                        initialPos_Y = -130;
                        role = 1;
                    }
                } else if (modePlay == 1) //robocup
                {
                    if (msg_yaw >= robocup_yaw) // dari sisi kiri
                    {
                    	//initialPos_X = -250; //-250;
                    	//initialPos_Y = -320;
                    	initialPos_X = init_robocup_left_x;
	                initialPos_Y = init_robocup_left_y;
                    	role = 0;
                        /*if (robot5Status == 1) { 
                        	initialPos_X = -250; //-250;
                        	initialPos_Y = -315;
                        	role = 1; 
                        }
                        else {
                        	initialPos_X = -150; //-250; //-200
                        	initialPos_Y = -315; 
                        	role = 0; 
                        }*/
                    } else if (msg_yaw < robocup_yaw) //dari sisi kanan
                    {
                    	//initialPos_X = -250; //-250;
                        //initialPos_Y = 320;
                        initialPos_X = init_robocup_right_x;
                        initialPos_Y = init_robocup_right_y;
                        role = 1;
                        /*if (robot5Status == 1) { 
                        	initialPos_X = -250; //-250;
                        	initialPos_Y = 315;
                        	role = 1; 
                        }
                        else { 
                        	initialPos_X = -150; //-250;
                        	initialPos_Y = 315;
                        	role = 0; 
                        }*/
                        //test grid
                        /*initialPos_X = -356;
                        initialPos_Y = 0;
                        role = 1;*/
                    }
                    /*else if (msg_yaw >= -10 && msg_yaw <= 10) // dari sisi kiri
                    {
                        initialPos_X = -100;
                        initialPos_Y = -310;
                        role = 1;
                    }*/
                }
            }
            lockInitPos++;
        }
        // printf("...InitialPosition::SUCCESS\n");
        return NodeStatus::SUCCESS;
    }
    
    void publishRRTVisualization()
    {
    // Jalur RRT
    nav_msgs::msg::Path pathMsg;
    pathMsg.header.stamp = this->now();
    pathMsg.header.frame_id = "field";
    for (const auto& wp : rrtPath)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = pathMsg.header;
        ps.pose.position.x = wp.x;
        ps.pose.position.y = wp.y;
        ps.pose.position.z = 0.0;
        pathMsg.poses.push_back(ps);
    }
    rrtPathPub_->publish(pathMsg);

    // Obstacle: format flat [x, y, r, x, y, r, ...]
    std_msgs::msg::Float32MultiArray obsMsg;
    for (const auto& obs : localObstacles)
    {
        obsMsg.data.push_back((float)obs.x);
        obsMsg.data.push_back((float)obs.y);
        obsMsg.data.push_back((float)obs.radius);
    }
    rrtObstaclePub_->publish(obsMsg);
    }

    enum class PlanState { IDLE, PLANNING, EXECUTING, DONE };
    PlanState rrtState = PlanState::IDLE;
    std::vector<Point2D> rrtPath;
    size_t rrtWaypointIdx = 0;
    std::vector<Obstacle> localObstacles;

    int gridKe = 0;
    int jalanGrid = 0;
    bool testgrid = false;
    NodeStatus testGrid()
    {
    printf("Test Grid - RRT Motion Planning\n");
    
    // --- Spesifikasi #1 & #3: posisi awal & tujuan ---
    const Point2D startPoint = {0.0, 0.0};
    const Point2D goalPoint  = {300.0, 0.0};   // ganti sesuai titik tujuan yang diinginkan

    // --- Spesifikasi #2: peta lokal obstacle didefinisikan manual ---
    if (localObstacles.empty())
    {
        // radius = radius fisik robot lawan (~25cm) + margin keamanan robot sendiri (~20cm)
        localObstacles.push_back({150.0,   0.0, 45.0});
        // tambah/ubah sesuai skenario uji
    }

    switch (rrtState)
    {
        case PlanState::IDLE:
        {
            RRTPlanner planner(/*minX*/-500, /*maxX*/500,
                                /*minY*/-350, /*maxY*/350,
                                /*stepSize*/20.0, /*maxIter*/3000,
                                /*goalTol*/15.0);

            rrtPath = planner.plan(startPoint, goalPoint, localObstacles);
            if (rrtPath.empty())
            {
                printf("[RRT] Gagal menemukan jalur bebas obstacle!\n");
                return NodeStatus::FAILURE;
            }

            rrtPath = smoothPath(rrtPath, localObstacles);
            printf("[RRT] Jalur ditemukan: %zu waypoint\n", rrtPath.size());

            rrtWaypointIdx = 1;  // index 0 = titik start, mulai jalan ke waypoint berikutnya
            rrtState = PlanState::EXECUTING;
            break;
        }

        case PlanState::EXECUTING:
	{
	    if (rrtWaypointIdx >= rrtPath.size())
	    {
		rrtState = PlanState::DONE;
		break;
	    }
	    const Point2D& target = rrtPath[rrtWaypointIdx];
	    new_out_pos_norotate((int)target.x, (int)target.y);   // <-- diganti dari new_out_pos(...)
	    if (doneMoved)
	    {
		printf("[RRT] Waypoint %zu/%zu tercapai (%.0f, %.0f)\n",
		       rrtWaypointIdx, rrtPath.size() - 1, target.x, target.y);
		doneMoved = false;
		cnt_move_to_grid = 0;
		// posRotateNew tidak lagi relevan di fungsi ini, tapi tetap aman direset
		posRotateNew = false;
		rrtWaypointIdx++;
	    }
	    break;
	}

        case PlanState::DONE:
        {
            printf("[RRT] Robot sampai di tujuan akhir!\n");
            motion("0");
            // reset supaya bisa dites ulang dari awal kalau node dipanggil lagi
            rrtState = PlanState::IDLE;
            localObstacles.clear();
            return NodeStatus::SUCCESS;
        }
    }
    publishRRTVisualization();
    return NodeStatus::FAILURE;  // BT tetap dianggap "running" sampai benar-benar SUCCESS
}
    
    NodeStatus gridPosition()
    {
        if (lockInitPos < 10)
        {
            //initialPos_X = -50;
            //initialPos_Y = 0;
            initialPos_X = 0; //-240;
            initialPos_Y = 0;
            role = 0;
            lockInitPos++;
        }
        return NodeStatus::SUCCESS;
    }
    
    int state_rotate = 0, last_state_rotate = 0;
    int angle_to_rotate = 0, reset_rotate = 0, last_angle_rotate = 0, reset_scan = 0;
    bool start_rotate = false;
    void rotateBodySearchGrid()
    {
        if (last_state_rotate != state_rotate)
        {
            posRotateNew = false;
            sabar = 0;
            searchKe = 0;
        }

        if (robotNumber == 1)
        {
            if (robot2DBall == 1) {new_out_grid(robot2GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot3DBall == 1) {new_out_grid(robot3GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot4DBall == 1) {new_out_grid(robot4GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot5DBall == 1) {new_out_grid(robot5GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
        } else if (robotNumber == 2)
        {
            if (robot1DBall == 1) {new_out_grid(robot1GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot3DBall == 1) {new_out_grid(robot3GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot4DBall == 1) {new_out_grid(robot4GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot5DBall == 1) {new_out_grid(robot5GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
        } else if (robotNumber == 3)
        {
            if (robot1DBall == 1) {new_out_grid(robot1GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot2DBall == 1) {new_out_grid(robot2GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot4DBall == 1) {new_out_grid(robot4GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot5DBall == 1) {new_out_grid(robot5GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
        } else if (robotNumber == 4)
        {
            if (robot1DBall == 1) {new_out_grid(robot1GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot2DBall == 1) {new_out_grid(robot2GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot3DBall == 1) {new_out_grid(robot3GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot5DBall == 1) {new_out_grid(robot5GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
        } else if (robotNumber == 5)
        {
            if (robot1DBall == 1) {new_out_grid(robot1GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot2DBall == 1) {new_out_grid(robot2GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot3DBall == 1) {new_out_grid(robot3GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot4DBall == 1) {new_out_grid(robot4GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
        }


        if (angle_to_rotate != 272)// && diffTheta > 90)
        {
            start_rotate = true;
        } else 
        {
            start_rotate = false;
        }

        // printf("...reset_rotate : %d\n", reset_rotate);

        // searchBallRectang(-1.6, -1.6, -0.8, 1.6);
        if (start_rotate)
        {
            if (reset_rotate > 20)
            {
                // printf("...startRotate : %d\n", start_rotate);
                // printf("...lastAngleRotate : %d\n", last_angle_rotate);
                // printf("...angleToRotate : %d\n", angle_to_rotate);
                // printf("...diffAngle : %d\n", abs(last_angle_rotate - angle_to_rotate));
                if (posRotateNew)
                {
                    motion("0");
                    if (abs(last_angle_rotate - angle_to_rotate) > 45)
                    {
                        reset_rotate = 0;
                    }
                } else
                {
                    rotateBodyImuNew(angle_to_rotate);
                }
            } else 
            {
                last_angle_rotate = angle_to_rotate;
                posRotateNew = false;
                reset_rotate++;
            }
        } else 
        {
            motion("0");
        }
    }

    int last_state_grid = 0;
    void walkGrid(int grid, int offx, int offy)
    {
        if (state_move_grid != last_state_grid)
        {
            searchKe = 0;
            sabar = 0;
            stateSearchBall = 0;
            refreshMoveGrid();
            last_state_grid = state_move_grid;
        }

        /*printf("...searchKe: %d\n", searchKe);
        printf("...sabar: %d\n", sabar);
        printf("...state_move_grid: %d\n", state_move_grid);
        //printf("...last_state_grid: %d\n", last_state_grid);*/

        switch (state_move_grid)
        {
            
            case 0:
            	motion("0");
            	if (action_walk)
            	{
            		if (searchKe >= 2) {
            		    action_walk = false;
            		} else {
            		    tiltSearchBall(0.0);
            		}
            	} else {
            		state_move_grid = 1;
            	}
            break;
            
            case 1:
            	if (stateSearchBall >= 5) //5 kalau pakai searchBallBreak
            	{
            		state_move_grid = 2;	
            	} else 
            	{
            		if (lastPidTilt >= -0.8)
            		{
		        		Walk(-0.05, 0.0, 0.0);
		        		searchBallBreak();
		        	} else 
		        	{
		        		state_move_grid = 2;
		        	}
            	}
            break;
            
            case 2:
            	if (stateSearchBall >= 10) //10 kalau pakai searchBallBreak
            	{
            		state_move_grid = 3;
            	} else 
            	{
            		motion("0");
            		searchBallBreak();
            	}
            break;
            
            case 3:
                if (posRotateNew)
                {
                    state_move_grid = 4;
                } else 
                {
                    if (stateSearchBall >= 10) //10 kalau pakai searchBallBreak
                    {
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        rotateBodyImuNew(90);
                    } else
                    {
                        motion("0");
                        searchBallBreak();
                    }
                }
            break;

            case 4:
                if (posRotateNew)
                {
                    state_move_grid = 5;
                } else 
                {
                    if (stateSearchBall >= 10) //10 kalau pakai searchBallBreak
                    {
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        rotateBodyImuNew(-90);
                    } else
                    {
                        motion("0");
                        searchBallBreak();
                    }
                }
            break;
            
            case 5:
            	if (stateSearchBall >= 10) //10 kalau pakai searchBallBreak
            	{
            		state_move_grid = 6;
            	} else 
            	{
            		motion("0");
            		searchBallBreak();
            	}
            break;
            
            
            case 6:
            	if (doneMoved)
                {
                    state_move_grid = 7;
                } else 
                {
                    searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                    new_out_grid(grid, offx, offy, true);
                }
            break;
            
            case 7:
                if (posRotateNew)
                {
                    state_move_grid = 8;
                } else 
                {
                    if (stateSearchBall >= 10) //10 kalau pakai searchBallBreak
                    {
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        rotateBodyImuNew(90);
                    } else
                    {
                        motion("0");
                        searchBallBreak();
                    }
                }
            break;

            case 8:
                if (posRotateNew)
                {
                    state_move_grid = 7;
                } else 
                {
                    if (stateSearchBall >= 10) //10 kalau pakai searchBallBreak
                    {
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        rotateBodyImuNew(-90);
                    } else
                    {
                        motion("0");
                        searchBallBreak();
                    }
                }
            break;
        }
    }
    
    void walkTarget(int offx, int offy)
    {
        if (state_move_grid != last_state_grid)
        {
            searchKe = 0;
            sabar = 0;
            stateSearchBall = 0;
            refreshMoveGrid();
            last_state_grid = state_move_grid;
        }

        /*printf("...searchKe: %d\n", searchKe);
        printf("...sabar: %d\n", sabar);
        printf("...state_move_grid: %d\n", state_move_grid);
        //printf("...last_state_grid: %d\n", last_state_grid);*/

        switch (state_move_grid)
        {
            
            case 0:
            	motion("0");
            	if (action_walk)
            	{
            		if (searchKe >= 2) {
            		    action_walk = false;
            		} else {
            		    tiltSearchBall(0.0);
            		}
            	} else {
            		state_move_grid = 1;
            	}
            break;
            
            case 1:
            	if (stateSearchBall >= 5) //5 kalau pakai searchBallBreak
            	{
            		state_move_grid = 2;	
            	} else 
            	{
            		if (lastPidTilt >= -0.8)
            		{
		        		Walk(-0.05, 0.0, 0.0);
		        		searchBallBreak();
		        	} else 
		        	{
		        		state_move_grid = 2;
		        	}
            	}
            break;
            
            case 2:
            	if (stateSearchBall >= 10) //10 kalau pakai searchBallBreak
            	{
            		state_move_grid = 3;
            	} else 
            	{
            		motion("0");
            		searchBallBreak();
            	}
            break;
            
            case 3:
                if (posRotateNew)
                {
                    state_move_grid = 4;
                } else 
                {
                    if (stateSearchBall >= 10) //10 kalau pakai searchBallBreak
                    {
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        rotateBodyImuNew(90);
                    } else
                    {
                        motion("0");
                        searchBallBreak();
                    }
                }
            break;

            case 4:
                if (posRotateNew)
                {
                    state_move_grid = 5;
                } else 
                {
                    if (stateSearchBall >= 10) //10 kalau pakai searchBallBreak
                    {
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        rotateBodyImuNew(-90);
                    } else
                    {
                        motion("0");
                        searchBallBreak();
                    }
                }
            break;
            
            case 5:
            	if (stateSearchBall >= 10) //10 kalau pakai searchBallBreak
            	{
            		state_move_grid = 6;
            	} else 
            	{
            		motion("0");
            		searchBallBreak();
            	}
            break;
            
            
            case 6:
            	if (doneMoved)
                {
                    state_move_grid = 7;
                } else 
                {
                    searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                    //new_out_grid(grid, offx, offy, true);
                    new_out_pos(offx, offy, true);
                }
            break;
            
            case 7:
                if (posRotateNew)
                {
                    state_move_grid = 8;
                } else 
                {
                    if (stateSearchBall >= 10) //10 kalau pakai searchBallBreak
                    {
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        rotateBodyImuNew(90);
                    } else
                    {
                        motion("0");
                        searchBallBreak();
                    }
                }
            break;

            case 8:
                if (posRotateNew)
                {
                    state_move_grid = 7;
                } else 
                {
                    if (stateSearchBall >= 10) //10 kalau pakai searchBallBreak
                    {
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        rotateBodyImuNew(-90);
                    } else
                    {
                        motion("0");
                        searchBallBreak();
                    }
                }
            break;
        }
    }

    int stateSearchBall = 0, last_state_search = 0;
    double timeBreak = 0.5;	//0.5
    void searchBallBreak()
    {
        if (stateSearchBall != last_state_search)
        {
            setWaktu();
            last_state_search = stateSearchBall;
        }

        switch (stateSearchBall)
        { //if(robotFall){
            //	motion("0");
            	//sleep(1);}
            	//else{
            case 0:
            	//printf("...case %d \n", stateSearchBall);
            	if (lastPidPan > 0)
            	{
            		headMove(1.6, -0.9);
            	}
                else if(lastPidPan <0) 
                {
                	headMove(-1.6, -0.9);
                }
                else{
                	headMove(0,-0.9);
                }
                cekWaktu(5);
                if (second > timeBreak)
                { stateSearchBall = 1; }
            break;

            case 1:
            	//printf("...case %d \n", stateSearchBall);
                if (lastPidPan > 0)
            	{
            		headMove(0.8, -0.9);
            	}
                else 
                {
                	headMove(-0.8, -0.9);
                }
                cekWaktu(5);
                if (second > timeBreak)
                { stateSearchBall = 2; }
            break;

            case 2:
            	//printf("...case %d \n", stateSearchBall);
            	headMove(0.0, -0.9);
                cekWaktu(5);
                if (second > timeBreak)
                { stateSearchBall = 3; }
            break;

            case 3:
            	//printf("...case %d \n", stateSearchBall);
                if (lastPidPan > 0)
            	{
            		headMove(-0.8, -0.9);
            	}
                else 
                {
                	headMove(0.8, -0.9);
                }
                cekWaktu(5);
                if (second > timeBreak)
                { stateSearchBall = 4; }
            break;

            case 4:
            	//printf("...case %d \n", stateSearchBall);
                if (lastPidPan > 0)
            	{
            		headMove(-1.6, -0.9);
            	}
                else 
                {
                	headMove(1.6, -0.9);
                }
                cekWaktu(5);
                if (second > timeBreak)
                { stateSearchBall = 5; }
            break;

            case 5:
            	//printf("...case %d \n", stateSearchBall);
                if (lastPidPan > 0)
            	{
            		headMove(-1.6, -1.6);
            	}
                else 
                {
                	headMove(1.6, -1.6);
                }
                cekWaktu(5);
                if (second > timeBreak)
                { stateSearchBall = 6; }
            break;
            
            case 6:
            	//printf("...case %d \n", stateSearchBall);
                if (lastPidPan > 0)
            	{
            		headMove(-0.8, -1.6);
            	}
                else 
                {
                	headMove(0.8, -1.6);
                }
                cekWaktu(5);
                if (second > timeBreak)
                { stateSearchBall = 7; }
            break;
            
            case 7:
            	//printf("...case %d \n", stateSearchBall);
            	headMove(0.0, -1.6);
                cekWaktu(5);
                if (second > timeBreak)
                { stateSearchBall = 8; }
            break;
            
            case 8:
            	//printf("...case %d \n", stateSearchBall);
                if (lastPidPan > 0)
            	{
            		headMove(0.8, -1.6);
            	}
                else 
                {
                	headMove(-0.8, -1.6);
                }
                cekWaktu(5);
                if (second > timeBreak)
                { stateSearchBall = 9; }
            break;
            
            case 9:
            	//printf("...case %d \n", stateSearchBall);
                if (lastPidPan > 0)
            	{
            		headMove(1.6, -1.6);
            	}
                else 
                {
                	headMove(-1.6, -1.6);
                }
                cekWaktu(5);
                if (second > timeBreak)
                { stateSearchBall = 10; }
            break;
            
            default:
            	stateSearchBall = 0;
            break;
        }
       //}
    }

    bool action_relax = false;
    bool stateWalkSearch = false;
    bool done_calibrate = false, dont_calibrate = false;
    int RobotPositioningEntry = 0, delay_calibrate = 0, RobotPosCondition = 0;
    NodeStatus RobotPositioning()
    {
         //printf("...RobotPositioning\n");
         
            // Saat Stopped=1 & State=Playing: SEMUA robot WAJIB diam
            // Tidak boleh locomote, bahkan bangun dari jatuh
    if (Stopped == 1) {
        motion("0");  // berdiri diam
        printf("Berhentii Jangan lanjutt!");
        return NodeStatus::FAILURE;  // BLOCK semua node di bawah
    }

        if (RobotPositioningEntry > 5)
        {
        
            if (secondaryInfo[0] == 3)
            {
                // Penalty Kick Positioning
                if (KickOff == barelang_color) {
                    // Tim kita tendang
                    if (isNearestExecutorForThrowIn()) {
                        // Striker/Kicker
                        xTarget = 200;
                        yTarget = 0;
                    } else {
                        // Robot pendukung kita
                        if      (robotNumber == 2) { xTarget = -150; yTarget = -200; }
                        else if (robotNumber == 4) { xTarget = -150; yTarget =  0; }
                        else if (robotNumber == 5) { xTarget =  -50; yTarget = -200; }
                        else if (robotNumber == 1) { xTarget = 200; yTarget =  0; }
                        else if (robotNumber == 6) { xTarget =  -250; yTarget =  100; }
                        else                       { xTarget = -200; yTarget =    0; }
                    }
                } else {
                    // Lawan tendang (defend)
                    if (robotNumber == 3) {
                        // Goalkeeper
                        xTarget = -420;
                        yTarget = 0;
                    } else {
                        // Robot defend kita
                        if      (robotNumber == 2) { xTarget = -100; yTarget = -200; }
                        else if (robotNumber == 4) { xTarget = -100; yTarget = -20; }
                        else if (robotNumber == 5) { xTarget =    0; yTarget = -200; }
                        else if (robotNumber == 3) { xTarget = -100; yTarget =  200; }
                        else if (robotNumber == 6) { xTarget =    0; yTarget =  200; }
                        else                       { xTarget =  -50; yTarget =    0; }
                    }
                }
            }
            else if (role == 0)
        {
            // Target attacker dalam koordinat lapangan (X,Y),
            // msg_yaw >= 0: masuk dari sisi kiri
            // msg_yaw < 0 : masuk dari sisi kanan
            if (KickOff == barelang_color || Release)
            {
                if (msg_yaw >= robocup_yaw)
                {
                    // Kiri: kickoff kita/release
                   // xTarget = -100;
                   // yTarget = -100;
                    xTarget = kickoff_kita_release_left_x;
                    yTarget = kickoff_kita_release_left_y;
                }
                else
                {
                    // Kanan: kickoff kita/release
                  //  xTarget = -150;
                  //  yTarget = 100;
                    xTarget = kickoff_kita_release_right_x;
                    yTarget = kickoff_kita_release_right_y;
                }
            }
            else // musuh kickoff
            {
                if (msg_yaw >= robocup_yaw)
                {
                    // Kiri: lawan kickoff
                    //xTarget = -100;
                    //yTarget = -100;
                    xTarget = musuh_kickoff_left_x;
                    yTarget = musuh_kickoff_left_y;
                    
                }
                else
                {
                    // Kanan: lawan kickoff
                    //xTarget = -100;
                    //yTarget = 100;
                    xTarget = musuh_kickoff_right_x;
                    yTarget = musuh_kickoff_right_y;
                }
            }
        }
        else if (role == 1)
        {
            // Target defender default dalam koordinat lapangan (X,Y),
            // msg_yaw >= 0: sisi kiri, msg_yaw < 0: sisi kanan.
            if (msg_yaw <= robocup_yaw)
            {
               // xTarget = -100;
               // yTarget = 100;
                xTarget = defend_x;
                yTarget = defend_y;
            }
        }
       
        if (Release && secondaryInfo[0] != 3)
        {
            // Saat release, posisi tujuan mengikuti button strategy.
            if (msg_strategy == 1)
            {
                //xTarget = -300;
                //yTarget = 0;
                xTarget = release1_x;
                yTarget = release1_y;
            }
            else if (msg_strategy == 2)
            {
                //xTarget = -75;
                //yTarget = 0;
                xTarget = release2_x;
                yTarget = release2_y;
            }
            else if (msg_strategy == 3)
            {
                //xTarget = 100;
                //yTarget = 0;
                xTarget = release3_x;
                yTarget = release3_y;
            }
        }

        if (Pickup || Remaining == 600 || Remaining == 300)
        {
        	dont_calibrate = true;
                if (done_calibrate)
                {
                    Pickup = false;
                    if (ballLost(35)) //(goalLost(20))
                    {
                    	panSearchBall(-1.55);
                    	delay_calibrate = 0;
                    } else
                    {
                    	//trackGoal();
                    	trackBall();
		                if (delay_calibrate > 50)
		                {
		                	if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
		                	{
		                		initialPos_X = initialPos_Y = 0;
		                		deltaPos_X = cam_x;
		                		deltaPos_Y = cam_y;
		                	}
		                	done_calibrate = true;
		                } else
		                {
		                	delay_calibrate++;
		                }
		    }
                    //RobotPosCondition = 1;
                    return NodeStatus::SUCCESS;
                }

                if (doneMoved)
                {
                    motion("0");
                    if (ballLost(35)) //(goalLost(20))
                    {
                    	panSearchBall(-1.55);
                    	delay_calibrate = 0;
                    	cekWaktu(15);
                    	if (timer)
                    	{
                    		done_calibrate = true;
                    	}
                    } else
                    {
                    	//trackGoal();
                    	trackBall();
		                if (delay_calibrate > 50)
		                {
		                	if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
		                	{
		                		initialPos_X = initialPos_Y = 0;
		                		deltaPos_X = cam_x;
		                		deltaPos_Y = cam_y;
		                	}
		                	done_calibrate = true;
		                } else
		                {
		                	delay_calibrate++;
		                }
		    }
                } else
                {
		            if (ballLost(15)) //35
		            {
		                searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        	delayWaitBall = 0;
		            } else
		            {
		                trackBall();

		                // PERBAIKAN: Deteksi bola saat release dan langsung masuk ke play
		                // Kurangi threshold agar lebih responsif (dari 100 jadi 10)
		/*                if (Release && releaseGoToGrid && delayWaitBall > 10)
		                {
		                	Release = false; // reset flag release
		                	releaseGoToGrid = false; // tidak perlu ke grid lagi
		                	Pickup = false; // untuk langsung kejar bola kalau lihat bola pas release
		                	foundBall = 1; // set foundBall agar masuk ke play mode
		                	printf("...RobotPositioning: Ball detected during release, switching to PLAY mode!\n");
		                	return NodeStatus::SUCCESS; // untuk langsung kejar bola kalau lihat bola pas release
		                }*/
		                		                // Gunakan counter khusus untuk memastikan bola terdeteksi konsisten ~0.5 detik (threshold=25)
		              if (Release && releaseGoToGrid && !ballLost(35))
		                {
		                    releaseBallDetectCounter++; // increment counter ketika bola terdeteksi
		                    if (releaseBallDetectCounter > 100) // ~0.5 detik (50Hz * 0.5s)
		                    {
		                        releaseBallDetectCounter = 0; // reset counter
		                        Release = false; // reset flag release
		                        releaseGoToGrid = false; // tidak perlu ke grid lagi
		                        Pickup = false; // untuk langsung kejar bola kalau lihat bola pas release
		                        foundBall = 1; // set foundBall agar masuk ke play mode
		                        printf("...RobotPositioning: Ball consistently detected for 0.5s during release, switching to PLAY mode!\n");
		                        return NodeStatus::SUCCESS; // untuk langsung kejar bola kalau lihat bola pas release
		                    }
		                } else 
		                {
		                    releaseBallDetectCounter = 0; // reset counter kalau bola hilang atau tidak ada release
		                    }

		               /* if (delayWaitBall > 100)//50
		                {
		                	if (headTilt >= cAktif + 0.2)
		                	{
		                		Pickup = false;
		                	}
		                	//Pickup = false; //untuk langsung kejar bola kalau lihat bola pas release
		                	//return NodeStatus::SUCCESS; //untuk langsung kejar bola kalau lihat bola pas release
		                	//RobotPosCondition = 3;
		                } else
		                {
		                    delayWaitBall++;
		                }*/
		            }
		    //RobotPosCondition = 1;
                    //new_out_grid(GridTarget, GridX, GridY, true);
                    //new_out_pos(xTarget, yTarget, true);
                    int tolerance_x = abs(xTarget - (int)robotPos_X);
            int tolerance_y = abs(yTarget - (int)robotPos_Y);
            if (secondaryInfo[0] == 3 && KickOff != barelang_color && tolerance_x <= 15 && tolerance_y <= 15) {
              int targetYaw = 0;
              if (yTarget > 0) targetYaw = -90;
              else if (yTarget < 0) targetYaw = 90;
              rotateBodyImuNew(targetYaw);
              if (posRotateNew) motion("0");
            } else {
              new_out_pos(xTarget, yTarget, true);
            }
                }
            } else
            {
                if (doneMoved)
                {
                    motion("0");
                    if (ballLost(50))   //goalLost(20))
                    {
                    	panSearchBall(-1.55);
                    	delay_calibrate = 0;
                    } else 
                    {
                   	 trackBall();
                    	//trackGoal();
		                if (delay_calibrate > 50)
		                {
		                	if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
		                	{
		                		initialPos_X = initialPos_Y = 0;
		                		deltaPos_X = cam_x;
		                		deltaPos_Y = cam_y;
		                	}	
		                	done_calibrate = true;
		                } else 
		                {
		                	delay_calibrate++;
		                }
		     }
                    //RobotPosCondition = 2;
                    return NodeStatus::SUCCESS;
                    
                } else
                {
                    // PERBAIKAN: Hanya rotasi di awal (SecondaryTime < 2)
                    // Setelah itu langsung positioning tanpa rotasi lagi
                    if (SecondaryTime < 2)
                    {
                        // Fase awal: Rotasi body ke orientasi 0 derajat
                        if (posRotateNew)
                        {
                            motion("0");
                            doneMoved = true;
                        } else
                        {
                            rotateBodyImuNew(0);
                        }
                    } else
                    {
                        // Setelah rotasi selesai, langsung jalan ke grid tanpa rotasi lagi
                        if (ballLost(35))
                        {
                            panSearchBall(-1.55);
                        }
                        else
                        {
                            trackBall();

                            // PERBAIKAN: Kalau di pertengahan jalan ke grid si robot nampak bola
                            // dan robot sudah di-release, langsung masuk ke state play
                            if (releaseGoToGrid && !ballLost(35) && delayWaitBall > 50)
                            {
                                Release = false; // reset flag release
                                releaseGoToGrid = false; // sudah tidak perlu ke grid lagi
                                Pickup = false;
                                foundBall = 1; // set foundBall agar masuk ke play mode
                                return NodeStatus::SUCCESS; // langsung masuk ke play
                            }

                            if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
			{
				initialPos_X = initialPos_Y = 0;
				if (cekArah())
					{
					    	deltaPos_X = cam_x * -1;
						deltaPos_Y = cam_y * -1;
					}
				else
					{
						deltaPos_X = cam_x;
						deltaPos_Y = cam_y;
					}
			}
                        }
                        //RobotPosCondition = 2;
                        //new_out_grid(GridTarget, GridX, GridY, true);
                        //new_out_pos(xTarget, yTarget, true);
                        int tolerance_x = abs(xTarget - (int)robotPos_X);
              int tolerance_y = abs(yTarget - (int)robotPos_Y);
              if (secondaryInfo[0] == 3 && KickOff != barelang_color && tolerance_x <= 15 && tolerance_y <= 15) {
                int targetYaw = 0;
                if (yTarget > 0) targetYaw = -90;
                else if (yTarget < 0) targetYaw = 90;
                rotateBodyImuNew(targetYaw);
                if (posRotateNew) motion("0");
              } else {
                new_out_pos(xTarget, yTarget, true);
              }
                    }
                }
            }
        } else
        {
            Walk(0.0, 0.0, 0.0);
            sabar = 0;
            setWaktu();
            refreshMoveGrid();
            delay_calibrate = 0;
            done_calibrate = false;
            RobotPositioningEntry++;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus OdomUpdate()
    {
        if (done_calibrate)
        {
            Pickup = false;
            return NodeStatus::SUCCESS;
        }

        if (goalLost(20))
        {
            panSearchGoal(-2.0);
            delay_calibrate = 0;
        } else 
        {
            trackGoal();
            if (delay_calibrate > 50)
            {
                if (cam_x != 999 && cam_y != 999 && cam_x < 0)
                {
                    initialPos_X = initialPos_Y = 0;
                    deltaPos_X = cam_x;
                    deltaPos_Y = cam_y;
                }   
                done_calibrate = true;
            } else 
            {
                delay_calibrate++;
            }
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus ResetVar()
    {
        // printf("...ResetVar\n");
        //defend_lock = false;
	//defend_arrived = false;
	//defend_lost_counter = 0;
        refreshMoveGrid();
        //done_calibrate = Release = Pickup = action_relax = false;
        done_calibrate = action_relax = false;
        action_kick = action_walk = action_afterKick = false;
        tracked = bodyTracked = robotDirection = ballPos = tendang = false;
        tracked = bodyTracked = action_kick = action_walk = action_afterKick = robotDirection = tendang = ballPos = doneBanting = false;
        //tracked = bodyTracked = action_kick = action_walk = action_afterKick = robotDirection = tendang = ballPos = false;
        delayWaitBall = delay_calibrate = 0;
        if (State == 2)
        {
            resetCommunication();
            tunggu = robotKick = 0;
            dont_calibrate = doneFirstKick = FirstKicked = finishFirstKick = false;
            setWaktu();
        }
        return NodeStatus::FAILURE;
    }

    int teamBallFoundDebounce = 0;
    bool lockRelax = false;
    int relaxEntry = 0;
    NodeStatus Relax()
    {
    	//printf("...Relax\n");
    	//xTarget = 150, yTarget = -100;
        if (relaxEntry > 5)
        {
            //printf("...Relax\n");
            // motion("0");
            //printf("...waitingTurn!!!\n");
            ballDistance = 999;
            stateCondition = 272;
            if (ballLost(35))
            {
                ballAround();
                searchBallBreak();
                //searchBallRectang(-1.6, -1.6, -1, 1.6);
            } else 
            {
                // printf("...tracked!!!!\n");
                trackBall();
                //newBodyTracking();
                walkRelax(xTarget, yTarget);
                if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
		        {
		        	initialPos_X = initialPos_Y = 0;
		        	deltaPos_X = cam_x;
		        	deltaPos_Y = cam_y;
		        }	
            }
        } else 
        {
            // motion("0"); // ganti 
            //Walk(0.0, 0.0, 0.0);
            walkTarget(-200, -25);
            refreshMoveGrid();
            setWaktu();
            WalkSearchBallEntry = 0;
            reset_rotate = 0;
            walkBack = waitRelax = 0;
            doneWalk = false;
            done_rotate = false;
            body_tracked = false;
            RobotPositioningEntry = BallApproachEntry = 0;
            relaxEntry++;
        }

        return NodeStatus::FAILURE;
    }

    NodeStatus RelaxLocked()
    {
        if (lockRelax)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    bool doneWalk = false;
    int waitRelax = 0, walkBack = 0, errorX = 0, errorY = 0;
    double moveX = 0.0, moveY = 0.0, moveA = 0.0;
    void walkRelax(int targetRobotPos_X, int targetRobotPos_Y) //Strategi untuk kondisi robot yang gak ngejar bola....
    {
        int tolerance_x = abs((int)targetRobotPos_X - (int)robotPos_X);
        int tolerance_y = abs((int)targetRobotPos_Y - (int)robotPos_Y);
        errorX = (int)targetRobotPos_X - ((int)robotPos_X);
        errorY = (int)targetRobotPos_Y - ((int)robotPos_Y);

        //jalan X
        moveX = errorX * (0.065);
        if(moveX > jalan) {
            bodyXImu = jalan;
        } else if (moveX < -0.03) {
            bodyXImu = -0.03;
        } else {
            bodyXImu = moveX;
        }

        //jalan Y
        moveY = errorY * (-0.03);
        if(moveY > 0.04) {
            bodyYImu = 0.04;
        } else if(moveY < -0.04){
            bodyYImu = -0.04;
        } else {
            bodyYImu = moveY;
        }
        
        //putar
        moveA = posPan * 1.50;
        
        if(ballLost(30))
        {
            alfaImu = 0.0;
            if(doneWalk)
            {
                walkTarget(targetRobotPos_X, targetRobotPos_Y);
                printf("...Jalan ke Grid...\n");
            }
        } else
        {
            trackBall();
            alfaImu = moveA;
            if (headPan > (panTengah + 0.2) || headPan < (panTengah - 0.2)) {
                body_tracked = false;
            }

            if(doneWalk) {
                if (body_tracked){
                    motion("0");
                } else {
                    //trackBall();
                    if (delayWaitBall > 20)
                    {
                        newBodyTracking();
                    } else 
                    {
                        delayWaitBall++;
                    }
                }
            }
        }

        if(!doneWalk) 
        {
            state_move_grid = 2;
            if(!useOmnidirection)
            {
                /*if(waitRelax > 15)
                {
                    if(walkBack > (robotPos_X + 15)) {
                        doneWalk = true;
                    } else {
                        jalanDirection(-0.03, 0.0, 0.0);
                    	printf("...Jalan Mundur...\n");
                    }
                } else {
                    walkBack = robotPos_X;
                    waitRelax++;
                }*/
                walkTarget(targetRobotPos_X, targetRobotPos_Y);
                printf("...Jalan ke Grid...\n");
            } else {
                if (tolerance_x <= 10 && tolerance_y <= 10) {
                    doneWalk = true;
                } else {
                    jalanDirection(bodyXImu, bodyYImu, alfaImu);
                    printf("...Jalan Omnidirection...\n");
                }
            }
        }
    }
    
    int ballDirection = 0;
    double predictBallDistance = 0;
    int targetBallPos_X = 0;
    int targetBallPos_Y = 0;
    void ballAround()
    {
        if (robotNumber == 1) { 
            if(robot2State == 232) { targetBallPos_X = robot2XBall, targetBallPos_Y = robot2YBall;}
            else if(robot3State == 232) { targetBallPos_X = robot3XBall, targetBallPos_Y = robot3YBall; }
            else if(robot4State == 232) { targetBallPos_X = robot4XBall, targetBallPos_Y = robot4YBall; }
            else if(robot5State == 232) { targetBallPos_X = robot5XBall, targetBallPos_Y = robot5YBall; }
        }
        else if (robotNumber == 2) { 
            if(robot1State == 232) { targetBallPos_X = robot1XBall, targetBallPos_Y = robot1YBall; }
            else if(robot3State == 232) { targetBallPos_X = robot3XBall, targetBallPos_Y = robot3YBall; }
            else if(robot4State == 232) { targetBallPos_X = robot4XBall, targetBallPos_Y = robot4YBall; }
            else if(robot5State == 232) { targetBallPos_X = robot5XBall, targetBallPos_Y = robot5YBall; }
        }
        else if (robotNumber == 3) { 
            if(robot1State == 232) { targetBallPos_X = robot1XBall, targetBallPos_Y = robot1YBall; }
            else if(robot2State == 232) { targetBallPos_X = robot2XBall, targetBallPos_Y = robot2YBall; }
            else if(robot4State == 232) { targetBallPos_X = robot4XBall, targetBallPos_Y = robot4YBall; }
            else if(robot5State == 232) { targetBallPos_X = robot5XBall, targetBallPos_Y = robot5YBall; }
        }
        else if (robotNumber == 4) { 
            if(robot1State == 232) { targetBallPos_X = robot1XBall, targetBallPos_Y = robot1YBall; }
            else if(robot2State == 232) { targetBallPos_X = robot2XBall, targetBallPos_Y = robot2YBall; }
            else if(robot3State == 232) { targetBallPos_X = robot3XBall, targetBallPos_Y = robot3YBall; }
            else if(robot5State == 232) { targetBallPos_X = robot5XBall, targetBallPos_Y = robot5YBall; }
        }
        else if (robotNumber == 5) { 
            if(robot1State == 232) { targetBallPos_X = robot1XBall, targetBallPos_Y = robot1YBall; }
            else if(robot2State == 232) { targetBallPos_X = robot2XBall, targetBallPos_Y = robot2YBall; }
            else if(robot3State == 232) { targetBallPos_X = robot3XBall, targetBallPos_Y = robot3YBall; }
            else if(robot4State == 232) { targetBallPos_X = robot4XBall, targetBallPos_Y = robot4YBall; }
        }

        //targetBallPos_X = convertGridX(ballDirection, 0);
        //targetBallPos_Y = convertGridY(ballDirection, 0);
        //predictBallDistance = sqrt(pow(targetBallPos_X - robotPos_X, 2) + pow(targetBallPos_Y - robotPos_Y, 2));

        rotate_to_grid();
    }

    bool done_rotate = false;
    double theta2 = 0.0;
    int diffTheta2 = 0, cnt_move_to_grid2 = 0;
    void rotate_to_grid()
    {
        // Calculate the angle between the two points in degrees
        theta2 = atan2(targetBallPos_Y - robotPos_Y, targetBallPos_X - robotPos_X) * 180 / PI;
        //printf("...distance = %d Cm\n", (int)distance);
        //printf("...angle = %d Deg\n", (int)theta);
        diffTheta2 = abs(abs((int)theta2) - abs(msg_yaw));

        if (cnt_move_to_grid2 > 10)
        {
            if(done_rotate)
            {
                walkRelax(xTarget, yTarget);
            } else 
            {
                searchBallBreak();
                if (diffTheta2 < 30)
                {
                    motion("0");
                    done_rotate = true;
                } else 
                {
                    jalanDirection(0.0, 0.0, theta2);
                    printf("putar di tempat\n");
                }
            }
        }
        else
        {
            reset_velocity();
            cnt_move_to_grid2++;
        }
    }
    
       int WalkSearchBallEntry = 0;
        NodeStatus WalkSearchBall()
    {
        // Cek apakah ada executor aktif
        bool executorExists = (robot1State == 232 || robot2State == 232 ||
                               robot3State == 232 || robot4State == 232 ||
                               robot5State == 232 || robot6State == 232);
        int executorId = 0;
        if (robot1State == 232) executorId = 1;
        else if (robot2State == 232) executorId = 2;
        else if (robot3State == 232) executorId = 3;
        else if (robot4State == 232) executorId = 4;
        else if (robot5State == 232) executorId = 5;
        else if (robot6State == 232) executorId = 6;

        // Jika ada executor dan robot ini bukan executor -> bertahan
        if (executorExists && executorId != robotNumber) {
            return Defend();
        }

        // Tidak ada executor -> lakukan walk search ball seperti biasa
        if (WalkSearchBallEntry > 5) {
            if (role == 0) {
                walkTarget(200, -25);
            } else if (role == 1) {
                if (robot4Status == 1)
                    walkTarget(0, -25);
                else
                    walkTarget(-200, -25);
            }
        } else {
            saveSudutImu();
            stateSearchBall = RobotPositioningEntry = BallApproachEntry = sabar = relaxEntry = 0;
            //lockRelax = false;
            WalkSearchBallEntry++;
        }
        return NodeStatus::FAILURE;
    }
    
/*
    int WalkSearchBallEntry = 0;
    NodeStatus WalkSearchBall()
    {
        if (WalkSearchBallEntry > 5)
        {
        	//printf("walkSearchBall\n");
            if (role == 0)
            {
                //walkGrid(39, 0, 25);
                walkTarget(200, -25);
            } else if (role == 1)
            {
                if (robot4Status == 1)
                {
                    //walkGrid(27, 0, 25);
                    walkTarget(0, -25);
                } else 
                {
                    //walkGrid(15, 0, 25);
                    walkTarget(-200, -25);
                }
            }
        } else 
        {
            //setWaktu();
            saveSudutImu();
            stateSearchBall = RobotPositioningEntry = BallApproachEntry = sabar = relaxEntry = 0;
            WalkSearchBallEntry++;
        }
        return NodeStatus::FAILURE;
    }
*/
    NodeStatus BallTracking()
    {
        // printf("...BallTracking\n");
        if (ballLost(10))
        {
            searchBallRectang(-1.6, -1.6, -1, 1.6);
            //headMove(0.0, cSekarang);
            //tiltSearchBall(0.0);
            //threeSearchBall();
            delayWaitBall = 0;
        } else 
        {
            trackBall();
        }
        return NodeStatus::FAILURE;
    }
    
        int delayExecutor = 0;
    NodeStatus Executor()
    {
    	int currentExecutors = 0;
    	if (robot1State == 232) currentExecutors++;
    	if (robot2State == 232) currentExecutors++;
    	if (robot3State == 232) currentExecutors++;
    	if (robot4State == 232) currentExecutors++;
    	if (robot5State == 232) currentExecutors++;
    	if (robot6State == 232) currentExecutors++;
    	
    	if (currentExecutors >=2) {
    	   return NodeStatus::FAILURE;
    	   }
        // printf("...Executor\n");
        // printf("...seme : %d\n", ballDistance);
        // printf("...robot1dBall = %d\n...robot2dBall = %d\n...robot3dBall = %d\n...robot4dBall = %d\n...robot5dBall = %d\n", robot1DBall, robot2DBall, robot3DBall, robot4DBall, robot5DBall);
        
        if (robotNumber != 1) {
            if (robot1Status == 0 || robot1DBall == 232) { robot1DBall = 999; }
        }

        if (robotNumber != 2) {
            if (robot2Status == 0 || robot2DBall == 232) { robot2DBall = 999; }
        }

        if (robotNumber != 3) {
            if (robot3Status == 0 || robot3DBall == 232) { robot3DBall = 999; }
        }

        if (robotNumber != 4) {
            if (robot4Status == 0 || robot4DBall == 232) { robot4DBall = 999; }
        }

        if (robotNumber != 5) {
            if (robot5Status == 0 || robot5DBall == 232) { robot5DBall = 999; }
        }
        
        if (robotNumber != 6) {
            if (robot6Status == 0 || robot6DBall == 232) { robot6DBall = 999; }
        }

        // if (lockRelax)
        // {
        //     if (delayExecutor > 50)
        //     {
        //         lockRelax = false;
        //     } else 
        //     {
        //         delayExecutor++;
        //     }
        // } else 
        // {
        //     delayExecutor = 0;
        // }

        if (useCoordination) {
            if (robotNumber == 1) {
                if (robot2State == 232 || robot3State == 232 || robot4State == 232 || robot5State == 232 || robot6State == 232)
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    ((ballDistance) < robot2DBall) &&
                    ((ballDistance) < robot3DBall) &&
                    ((ballDistance) < robot4DBall) &&
                    ((ballDistance) < robot5DBall) &&
                    ((ballDistance) < robot6DBall)
                    ) {
                    //ballExecutorRobot = robotNumber; // update lokal agar Defend() sync
                    // printf("...Executor::SUCCESS\n");
                    return NodeStatus::SUCCESS;
                }
            } else if (robotNumber == 2) {
                if (robot1State == 232 || robot3State == 232 || robot4State == 232 || robot5State == 232 || robot6State == 232)
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    ((ballDistance) < robot1DBall) &&
                    ((ballDistance) < robot3DBall) &&
                    ((ballDistance) < robot4DBall) &&
                    ((ballDistance) < robot5DBall) &&
                    ((ballDistance) < robot6DBall)
                    ) {
                   // ballExecutorRobot = robotNumber; // update lokal agar Defend() sync
                    // printf("...Executor Robot 2...\n");
                    return NodeStatus::SUCCESS;
                }
            } else if (robotNumber == 3) {
                if (robot1State == 232 || robot2State == 232 || robot4State == 232 || robot5State == 232 || robot6State == 232)
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    ((ballDistance) < robot1DBall) &&
                    ((ballDistance) < robot2DBall) &&
                    ((ballDistance) < robot4DBall) &&
                    ((ballDistance) < robot5DBall) &&
                    ((ballDistance) < robot6DBall)
                    ) {
                   // ballExecutorRobot = robotNumber; // update lokal agar Defend() sync
                    // printf("...Executor::SUCCESS\n");
                    //printf("...Executor Robot 33...\n");
                    return NodeStatus::SUCCESS;
                }
            } else if (robotNumber == 4) {
                if (robot1State == 232 || robot2State == 232 || robot3State == 232 || robot5State == 232 || robot6State == 232)
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    ((ballDistance) < robot1DBall) &&
                    ((ballDistance) < robot2DBall) &&
                    ((ballDistance) < robot3DBall) &&
                    ((ballDistance) < robot5DBall) &&
                    ((ballDistance) < robot6DBall)
                    ) {
                   // ballExecutorRobot = robotNumber; // update lokal agar Defend() sync
                    // printf("...Executor::SUCCESS\n");
                    return NodeStatus::SUCCESS;
                }
            } else if (robotNumber == 5) {
                if (robot1State == 232 || robot2State == 232 || robot3State == 232 || robot4State == 232 || robot6State == 232)
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    ((ballDistance) < robot1DBall) &&
                    ((ballDistance) < robot2DBall) &&
                    ((ballDistance) < robot3DBall) &&
                    ((ballDistance) < robot4DBall) &&
                    ((ballDistance) < robot6DBall)
                    ) {
                   // ballExecutorRobot = robotNumber; // update lokal agar Defend() sync
                    // printf("...Executor::SUCCESS\n");    
                    return NodeStatus::SUCCESS;
                }
            } else if (robotNumber == 6) {
                if (robot1State == 232 || robot2State == 232 || robot3State == 232 || robot4State == 232 || robot5State == 232)
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    ((ballDistance) < robot1DBall) &&
                    ((ballDistance) < robot2DBall) &&
                    ((ballDistance) < robot3DBall) &&
                    ((ballDistance) < robot4DBall) &&
                    ((ballDistance) < robot5DBall)
                    ) {
                   // ballExecutorRobot = robotNumber; // update lokal agar Defend() sync
                    // printf("...Executor::SUCCESS\n");    
                    return NodeStatus::SUCCESS;
                }
            }
        } else {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }






    int lockInitPos = 0;
    bool Pickup = false, Release = false;
    bool releaseGoToGrid  = false; // flag: sedang navigate ke grid setelah di-release wasit
    bool releaseArrived   = false; // flag: sudah sampai di grid tujuan setelah release
    int releaseBallDetectCounter = 0; // counter untuk memastikan bola terdeteksi konsisten ~0.5 detik saat release
    NodeStatus StatePickup()
    {
        // GC26: Check untuk semua violation/penalty yang memerlukan PICKUP:
        // Pelanggaran yang perlu robot di-pickup dan ditaruh di pinggir lapangan:
        // - 1: IllegalPositioning
        // - 3: LocalGameStuck
        // - 4: IncapableRobot
        // - 5: PickedUp (Request for pick-up dari wasit)
        // - 6: BallHolding
        // - 7: LeavingTheField

        bool requiresPickup = false;

        // Check tim kita (timNumber1 == team)
        if (timNumber1 == team && (Penalty1 == 1 || Penalty1 == 2 || Penalty1 == 3 || Penalty1 == 4 ||
                                    Penalty1 == 5 || Penalty1 == 6 || Penalty1 == 7 || Penalty1 == 8 || Penalty1 == 9 || Penalty1 == 10 || Penalty1 == 11 || Penalty1 == 12 || Penalty1 == 13)) {
            requiresPickup = true;
        }

        // Check tim lawan (timNumber2 == team)
        if (timNumber2 == team && (Penalty2 == 1 || Penalty2 == 2 || Penalty2 == 3 || Penalty2 == 4 ||
                                    Penalty2 == 5 || Penalty2 == 6 || Penalty2 == 7 || Penalty2 == 8 || Penalty2 == 9 || Penalty2 == 10 || Penalty2 == 11 || Penalty2 == 12 || Penalty2 == 13)) {
            requiresPickup = true;
        }

        if (requiresPickup) {
            Pickup = true;
            motion("8"); // diam saat diangkat wasit / ditempatkan di pinggir lapangan
        }

        if (Pickup) {
            finishFirstKick = true;
            return NodeStatus::SUCCESS;
        }

        return NodeStatus::FAILURE;
    }
    
    NodeStatus StateRelease()
    {
        // GC26: Release ketika penalty sudah bersih (tidak ada lagi violation yang memerlukan pickup)
        // Penalty bersih jika:
        // - timNumber1 == team && Penalty1 BUKAN (1, 3, 4, 5, 6, 7)
        // - atau timNumber2 == team && Penalty2 BUKAN (1, 3, 4, 5, 6, 7)

        if (msg_yaw >= 0) // dari sisi kiri dia strker
        {
            initialPos_X = -300;
            initialPos_Y = -307;
            

        } else if (msg_yaw < 0) //dari sisi kanan belakang
        {
            initialPos_X = -300;
            initialPos_Y = 307;
            
        }	
        
        bool penaltyCleared = false;

        // Check tim kita (timNumber1 == team)
        if (timNumber1 == team && !(Penalty1 == 1 || Penalty1 == 2 || Penalty1 == 3 || Penalty1 == 4 ||
                                    Penalty1 == 5 || Penalty1 == 6 || Penalty1 == 7 || Penalty1 == 8 || Penalty1 == 9 || Penalty1 == 10 || Penalty1 == 11 || Penalty1 == 12 || Penalty1 == 13)) {
            penaltyCleared = true;
        }

        // Check tim lawan (timNumber2 == team)
        if (timNumber2 == team && !(Penalty2 == 1 || Penalty2 == 2 || Penalty2 == 3 || Penalty2 == 4 ||
                                    Penalty2 == 5 || Penalty2 == 6 || Penalty2 == 7 || Penalty2 == 8 || Penalty2 == 9 || Penalty2 == 10 || Penalty2 == 11 || Penalty2 == 12 || Penalty2 == 13)) {
            penaltyCleared = true;
        }
/*
        if (penaltyCleared) {
            Release = true; // reset flag
            return NodeStatus::SUCCESS;
        } else {
            motion("8"); // masih dalam kondisi harus pickup, tetap diam
        }*/
        if (penaltyCleared) {
            // PERBAIKAN: Set Pickup = true agar robot masuk ke RobotPositioning flow
            // seperti di ready state, bukan langsung ke WalkSearchBall
            Pickup = true; // set pickup agar masuk ke RobotPositioning flow
            Release = true; // set flag release agar masuk ke positioning
            releaseGoToGrid = true; // flag untuk navigasi ke grid setelah release
            releaseArrived = false; // reset flag arrived
            // JANGAN set foundBall = 0 karena akan membuat robot masuk ke WalkSearchBall
            // foundBall = 0; // DIHAPUS
            // stateCondition = 272; // DIHAPUS
            // ballDistance = 999; // DIHAPUS
            return NodeStatus::SUCCESS;
        } else {
            motion("8"); // masih dalam kondisi harus pickup, tetap diam
        }

        return NodeStatus::FAILURE;
    }

        NodeStatus IsKickingTeam()
    {
        if (KickOff == barelang_color) {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    // GC sent stopped=1: all robots must immediately stop locomotion
    NodeStatus StateStopped()
    {
        if (Stopped == 1) {
            motion("0"); //0
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }
    
    
    int delay_koordinasi = 0;
    NodeStatus Communication()
    {
        // printf("...Communication\n");
        if (delay_koordinasi > 20)
        {
            sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, pos_X, pos_Y, foundBall, ballDistance, BallX, BallY, robotKick);
            delay_koordinasi = 0;
        } else 
        {
            delay_koordinasi++;
        }
        return NodeStatus::SUCCESS;
    }
    
    NodeStatus BallFoundTeam()
    {
        if (runTeamBallDefendSupport()) {
            RobotPositioningEntry = 0;
            return NodeStatus::SUCCESS;
        }
        
        trackHeadWithBestVisibleTeamBall(false);
        if (robotNumber == 1)
        {
            if (robot2FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot3FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot4FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot5FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
        } else if (robotNumber == 2)
        {
            if (robot1FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot3FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot4FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot5FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
        } else if (robotNumber == 3)
        {
            if (robot1FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot2FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot4FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot5FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
        } else if (robotNumber == 4)
        {
            if (robot1FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot2FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot3FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot5FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
        } else if (robotNumber == 5)
        {
            if (robot1FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot2FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot3FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot4FBall == 1) {RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
        }
        return NodeStatus::FAILURE;
    }

    int state_move_grid = 0, last_move = 0, cnt_move_to_grid = 0;
    double theta, sagital, lateral, walkSagital, walkLateral;
    int outTheta = 0,  diffTheta = 0;
    void new_out_grid(int targetRobotGrid, int targetRobotGridOffsetX, int targetRobotGridOffsetY, bool isGoToGrid)
    {
        double targetRobotPos_X = convertGridX(targetRobotGrid, targetRobotGridOffsetX);
        double targetRobotPos_Y = convertGridY(targetRobotGrid, targetRobotGridOffsetY);
        // printf("...msg_yaw = %d\n", msg_yaw);
        //printf("...robotPos = %f, %f\n", robotPos_X, robotPos_Y);
        //printf("...target = %f, %f\n", targetRobotPos_X, targetRobotPos_Y);
        // Calculate the distance between the two points
        double distance = sqrt(pow(targetRobotPos_X - robotPos_X, 2) + pow(targetRobotPos_Y - robotPos_Y, 2));
        // Calculate the angle between the two points in degrees
        theta = atan2(targetRobotPos_Y - robotPos_Y, targetRobotPos_X - robotPos_X) * 180 / PI;
        //printf("...distance = %d Cm\n", (int)distance);
        //printf("...angle = %d Deg\n", (int)theta);
        outTheta = int(theta) + int(msg_yaw);
        diffTheta = abs(abs((int)theta) - abs(msg_yaw));
        //printf("diffTheta = %d\n", diffTheta);
        int tolerance_x = abs((int)targetRobotPos_X - (int)robotPos_X);
        int tolerance_y = abs((int)targetRobotPos_Y - (int)robotPos_Y);
        // printf("...tolerance = %d, %d\n", tolerance_x, tolerance_y);
        if (isGoToGrid)
        {
            if (tolerance_x <= 15 && tolerance_y <= 15)
            // if (distance < 30)
            {
                if (posRotateNew)
                {
                    // Walk(0.0, 0.0, 0.0);
                    motion("0");
                    printf("...SELESAI!!!!\n");
                    doneMoved = true;
                    gridKe++;
                } else 
                {
                    rotateBodyImuNew(0);
                }                
            }
            else
            {   
                posRotateNew = false;
                if (cnt_move_to_grid > 10)
                {
                    // printf("...jalan theta !!! \n");
                    if (diffTheta < 30)
                    {
                        jalanDirection(jalan, 0.0, theta); //test grid - jalan ke grid
                    } else 
                    {
                        jalanDirection(0.0, 0.0, theta);
                    }
                }
                else
                {
                    reset_velocity();
                    cnt_move_to_grid++;
                }
            }
        }
    }
    
    int vizTargetGrid = 0;
    int vizTargetOffsetX = 0, vizTargetOffsetY = 0;
    double vizTargetPosX = 0.0, vizTargetPosY = 0.0;
    bool vizTargetValid = false;

    void setVizTargetFromPos(double targetX, double targetY)
    {
        vizTargetGrid = 0;
        vizTargetOffsetX = 0;
        vizTargetOffsetY = 0;
        vizTargetPosX = targetX;
        vizTargetPosY = targetY;
        vizTargetValid = true;
    }

    void clearVizTarget()
    {
        vizTargetGrid = 0;
        vizTargetOffsetX = 0;
        vizTargetOffsetY = 0;
        vizTargetPosX = 0.0;
        vizTargetPosY = 0.0;
        vizTargetValid = false;
    }
        void new_out_pos(int targetRobotPos_X, int targetRobotPos_Y, bool isGoToGrid)
    {
    	setVizTargetFromPos(targetRobotPos_X, targetRobotPos_Y);
        // printf("...msg_yaw = %d\n", msg_yaw);
        //printf("...robotPos = %f, %f\n", robotPos_X, robotPos_Y);
        //printf("...target = %f, %f\n", targetRobotPos_X, targetRobotPos_Y);
        // Calculate the distance between the two points
        double distance = sqrt(pow(targetRobotPos_X - robotPos_X, 2) + pow(targetRobotPos_Y - robotPos_Y, 2));
        // Calculate the angle between the two points in degrees
        theta = atan2(targetRobotPos_Y - robotPos_Y, targetRobotPos_X - robotPos_X) * 180 / PI;
        //printf("...distance = %d Cm\n", (int)distance);
        //printf("...angle = %d Deg\n", (int)theta);
        outTheta = int(theta) + int(msg_yaw);
        
        // Menghitung selisih sudut yang benar dengan wrap-around -180 s/d 180
        int angle_diff = ((int)theta - (int)msg_yaw) % 360;
        if (angle_diff < -180) angle_diff += 360;
        if (angle_diff > 180) angle_diff -= 360;
        diffTheta = abs(angle_diff);
        //printf("diffTheta = %d\n", diffTheta);
        int tolerance_x = abs((int)targetRobotPos_X - (int)robotPos_X);
        int tolerance_y = abs((int)targetRobotPos_Y - (int)robotPos_Y);
        // printf("...tolerance = %d, %d\n", tolerance_x, tolerance_y);
        if (isGoToGrid)
        {
            if (tolerance_x <= 15 && tolerance_y <= 15)
            // if (distance < 30)
            {
                if (posRotateNew)
                {
                    // Walk(0.0, 0.0, 0.0);
                    motion("0");
                    printf("...SELESAI!!!!\n");
                    doneMoved = true;
                    gridKe++;
                } else 
                {
                    rotateBodyImuNew(0);
                }                
            }
            else
            {   
                posRotateNew = false;
                if (cnt_move_to_grid > 10)
                {
                    // printf("...jalan theta !!! \n");
                    if (diffTheta < 30)
                    {
                        jalanDirection(jalan, 0.0, theta); //test grid - jalan ke grid
                        printf("...jalan speed biasa !!! \n");
                    } else 
                    {
                        jalanDirection(0.0, 0.0, theta);
                        printf("...jalan 0.0000 !!! \n");
                    }
                }
                else
                {
                    reset_velocity();
                    cnt_move_to_grid++;
                }
            }
        }
    }
    
    void new_out_pos_norotate(int targetRobotPos_X, int targetRobotPos_Y)
{
    setVizTargetFromPos(targetRobotPos_X, targetRobotPos_Y);

    double distance = sqrt(pow(targetRobotPos_X - robotPos_X, 2) + pow(targetRobotPos_Y - robotPos_Y, 2));
    theta = atan2(targetRobotPos_Y - robotPos_Y, targetRobotPos_X - robotPos_X) * 180 / PI;

    int angle_diff = ((int)theta - (int)msg_yaw) % 360;
    if (angle_diff < -180) angle_diff += 360;
    if (angle_diff > 180) angle_diff -= 360;
    diffTheta = abs(angle_diff);

    int tolerance_x = abs((int)targetRobotPos_X - (int)robotPos_X);
    int tolerance_y = abs((int)targetRobotPos_Y - (int)robotPos_Y);

    if (tolerance_x <= 15 && tolerance_y <= 15)
    {
        // Posisi sudah tercapai -> LANGSUNG berhenti, TANPA memaksa yaw ke 0
        motion("0");
        printf("...WAYPOINT TERCAPAI (tanpa align yaw)!!!\n");
        doneMoved = true;
    }
    else
    {
        if (cnt_move_to_grid > 10)
        {
            if (diffTheta < 30)
            {
                jalanDirection(jalan, 0.0, theta);
                printf("...jalan speed biasa !!! \n");
            }
            else
            {
                jalanDirection(0.0, 0.0, theta);
                printf("...jalan 0.0000 !!! \n");
            }
        }
        else
        {
            reset_velocity();
            cnt_move_to_grid++;
        }
    }
}

    
/*
    void new_out_pos(int targetRobotPos_X, int targetRobotPos_Y, bool isGoToGrid)
    {
        // printf("...msg_yaw = %d\n", msg_yaw);
        //printf("...robotPos = %f, %f\n", robotPos_X, robotPos_Y);
        //printf("...target = %f, %f\n", targetRobotPos_X, targetRobotPos_Y);
        // Calculate the distance between the two points
        double distance = sqrt(pow(targetRobotPos_X - robotPos_X, 2) + pow(targetRobotPos_Y - robotPos_Y, 2));
        // Calculate the angle between the two points in degrees
        theta = atan2(targetRobotPos_Y - robotPos_Y, targetRobotPos_X - robotPos_X) * 180 / PI;
        //printf("...distance = %d Cm\n", (int)distance);
        //printf("...angle = %d Deg\n", (int)theta);
        outTheta = int(theta) + int(msg_yaw);
        diffTheta = abs(abs((int)theta) - abs(msg_yaw));
        //printf("diffTheta = %d\n", diffTheta);
        int tolerance_x = abs((int)targetRobotPos_X - (int)robotPos_X);
        int tolerance_y = abs((int)targetRobotPos_Y - (int)robotPos_Y);
        // printf("...tolerance = %d, %d\n", tolerance_x, tolerance_y);
        if (isGoToGrid)
        {
            if (tolerance_x <= 15 && tolerance_y <= 15)
            // if (distance < 30)
            {
                if (posRotateNew)
                {
                    // Walk(0.0, 0.0, 0.0);
                    motion("0");
                    printf("...SELESAI!!!!\n");
                    doneMoved = true;
                    gridKe++;
                } else 
                {
                    rotateBodyImuNew(0);
                }                
            }
            else
            {   
                posRotateNew = false;
                if (cnt_move_to_grid > 10)
                {
                    // printf("...jalan theta !!! \n");
                    if (diffTheta < 30)
                    {
                        jalanDirection(jalan, 0.0, theta); //test grid - jalan ke grid
                        printf("...jalan speed biasa !!! \n");
                    } else 
                    {
                        jalanDirection(0.0, 0.0, theta);
                        printf("...jalan 0.0000 !!! \n");
                    }
                }
                else
                {
                    reset_velocity();
                    cnt_move_to_grid++;
                }
            }
        }
    }
*/
    double get_slope(int xG, int yG, int xB, int yB)
    {
        double slope = (xG - xB) / (yG - yB);
        return slope;
    }

    int xTar = 0, yTar = 0;
    void calculate_target(int xG, int yG, int xB, int yB)
    {   
        xTar = xB - 25;
        yTar = get_slope(xG, yG, xB, yB) * (xTar - xB) + yB;
       // printf("...Target : %d, %d\n", xTar, yTar);
    }

    void resetCommunication()
    {
        robot1Id = 0; robot1Status = 0; robot1State = 0; robot1GridPosition = 88; robot1XPosition = 999; robot1YPosition = 999; robot1FBall = 0; robot1DBall = 999; robot1XBall = 999; robot1YBall = 999; robot1BackIn = 0; robot1Voltage = 0;
        robot2Id = 0; robot2Status = 0; robot2State = 0; robot2GridPosition = 88; robot2XPosition = 999; robot2YPosition = 999; robot2FBall = 0; robot2DBall = 999; robot2XBall = 999; robot2YBall = 999; robot2BackIn = 0; robot2Voltage = 0;
        robot3Id = 0; robot3Status = 0; robot3State = 0; robot3GridPosition = 88; robot3XPosition = 999; robot3YPosition = 999; robot3FBall = 0; robot3DBall = 999; robot3XBall = 999; robot3YBall = 999; robot3BackIn = 0; robot3Voltage = 0;
        robot4Id = 0; robot4Status = 0; robot4State = 0; robot4GridPosition = 88; robot4XPosition = 999; robot4YPosition = 999; robot4FBall = 0; robot4DBall = 999; robot4XBall = 999; robot4YBall = 999; robot4BackIn = 0; robot4Voltage = 0;
        robot5Id = 0; robot5Status = 0; robot5State = 0; robot5GridPosition = 88; robot5XPosition = 999; robot5YPosition = 999; robot5FBall = 0; robot5DBall = 999; robot5XBall = 999; robot5YBall = 999; robot5BackIn = 0; robot5Voltage = 0;
        robot6Id = 0; robot6Status = 0; robot6State = 0; robot6GridPosition = 88; robot6XPosition = 999; robot6YPosition = 999; robot6FBall = 0; robot6DBall = 999; robot6XBall = 999; robot6YBall = 999; robot6BackIn = 0; robot6Voltage = 0;
    }

    void resetVariable()
    {
        resetCommunication();
        refreshMoveGrid();
        reset_velocity();
        setWaktu();
        lockRelax = false;
        teamBallFoundDebounce = 0;
        WalkSearchBallEntry = BallFoundEntry = BodyTrackEntry = BallApproachEntry = RotateToGoalEntry = KickEntry = FirstKickEntry = 0;
        stateSearchBall = delay  = state_move_grid = stateCondition = cnt_move_to_grid = cnt_sbr = 0;
        Ball_X = Ball_Y = Goal_X = Goal_Y = -1;
        B_pole_X = T_pole_X = B_pole_Y = T_pole_Y = -1;
        doneBanting = done_calibrate = tracked = bodyTracked = action_afterKick = action_kick = action_relax = action_walk = doneSaveData = doneGetData = passed = doneWalk = false;
        //done_calibrate = tracked = bodyTracked = action_afterKick = action_kick = action_relax = action_walk = doneSaveData = doneGetData = passed = doneWalk = false;
        delay_calibrate = lockInitPos = sumWalkX = 0;
        robotPos_X = robotPos_Y = deltaPos_X = deltaPos_Y = field_x = field_y = x_pos = y_pos = 0.0;
        filtered_imu_yaw = filtered_x_vel = filtered_y_vel = 0;
        gridKe = object_count = 0;
    }

    void displayBT()
    {
        printf("Penalty1, timColour1 : %d, %d\n", Penalty1, timColour1);
        printf("Penalty2, timColour2 : %d, %d\n", Penalty2, timColour2);
        printf("timNumber1, timNumber2 : %d, %d\n", timNumber1, timNumber2);
        printf("strategy = %d, play = %d\n", msg_strategy, msg_kill);
        // printf("Grid = %d\n", Grid);
        printf("robotPos = %f, %f\n", robotPos_X, robotPos_Y);
        	//printf("sudut = %d\n", msg_yaw);
        printf("head = %.2f, %.2f\n", headPan, headTilt);
        //printf("head(offset) = %.2f, %.2f\n", PAN, TILT);
        	printf("stateGameController = %d\n\n", State);
        //printf("Grid : %d\n", Grid);
        	//printf("RobotPos : %.2f, %.2f\n", robotPos_X, robotPos_Y);
        	//printf("XYTarget : %d, %d\n", xTarget, yTarget);
        	printf("BallPos : %.2f, %.2f\n", ball_pose_x, ball_pose_y);
        	printf("BallDis : %.2f\n", jarak_bola);
        	//printf("targetBall = %d, %d\n", targetBallPos_X, targetBallPos_Y);
        //printf("ball = %d, %d\n", Ball_X, Ball_Y);
        //printf("BallDis : %d\n", ballDistance);
        //printf("BallPos : %.2f, %.2f\n", ball_pose_x, ball_pose_y);
        //printf("delay_koordinasi : %d\n", delay_koordinasi);
        //printf("GridTarget : %d, %d, %d\n", GridTarget, GridX, GridY);
        //printf("Pickup : %d\n", Pickup);
        //printf("Release : %d\n", Release);
        //printf("finishKick : %d\n", finishKick);
        	printf("SecondaryTime : %d\n", SecondaryTime);
        //printf("sudutTendang : %d\n", sudutTendang);
        	printf("stateSecondary = %d\n", SecondaryState);
        	printf("SecondaryInfo_0 = %d\n", secondaryInfo[0]);
        //printf("SecondaryInfo_1 = %d\n", secondaryInfo[1]);
        //printf("SecondaryInfo_2 = %d\n", secondaryInfo[2]);
        //printf("SecondaryInfo_3 = %d\n", secondaryInfo[3]);
        //printf("PlayerTeam = %d\n", PlayerTeam);
        //printf("KickOff, barelang_color : %d, %d \n", KickOff, barelang_color);
        //printf("stateSearch = %d\n", stateSearchBall);
        // printf("stateCondition = %d\n", stateCondition);
        	//printf("ball = %d, %d\n", Ball_X, Ball_Y);
        // printf("actual walk = %d mm, %d mm, %f m, %f m\n", vx, vy, robotWalkX, robotWalkY);
        // printf("walk active, support leg = %d, %d\n", walkActive, supportLeg);
        // printf("class id = %s\n", class_id.c_str());
        // printf("knee Current = %d\n", kneeCurr);
        // printf("stabilize state = %d\n", stabilize_state);
        // printf("second = %d\n", second);
        // printf("ObjectCount = %d\n", object_count);

        /*printf("CamPos : %.2f, %.2f\n", cam_x, cam_y);
        printf("RobotPos : %.2f, %.2f\n", robotPos_X, robotPos_Y);
        printf("BallDis : %d\n", ballDistance);
        printf("BallPos : %.2f, %.2f\n", ball_pose_x, ball_pose_y);
        printf("TarPos : %d, %d\n", xTar, yTar);
        printf("KickOff, barelang_color : %d, %d \n", KickOff, barelang_color);
        printf("Penalty1, timColour1 : %d, %d\n", Penalty1, timColour1);
        printf("Penalty2, timColour2 : %d, %d\n", Penalty2, timColour2);
        printf("Pickup : %d\n", Pickup);
        printf("Release : %d\n", Release);
        printf("FirstKicked : %d\n", FirstKicked);
        printf("sudutTendang : %d\n", sudutTendang);
        printf("role : %d\n", role);
        printf("robotKick : %d\n", robotKick);
        printf("deltaY : %f\n", deltaY);
        printf("deltaX : %f\n", deltaX);

        printf("delay_koordinasi : %d\n", delay_koordinasi);

        */
        //printf(" Aruku %f : %f\n", jalan, arukuX6);
        printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot1Id, robot1FBall, robot1DBall, robot1State);
        printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot2Id, robot2FBall, robot2DBall, robot2State);
        printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot3Id, robot3FBall, robot3DBall, robot3State);
        printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot4Id, robot4FBall, robot4DBall, robot4State);
        printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot5Id, robot5FBall, robot5DBall, robot5State);
        printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot6Id, robot6FBall, robot6DBall, robot6State);
	
        printf("\n\n");
    }

    void timer_callback()
    {
        Grid = coordinates_to_grid(robotPos_X, robotPos_Y);
        pos_X = robotPos_X;
        pos_Y = robotPos_X;

        if (useDisplay)
        {
            displayBT();
        }

        if (msg_strategy == 4)
        {
            play = false;
            motion("8");
            stateCondition = 150;
            if (ballLost(35))
            {
                // tiltSearchBall(0.0);
                searchBallBreak();
            }
            else
            {
            	hitungGerakBola();
                trackBall();
            }
            // if (goalLost(20))
            // {
            //     panSearchGoal(msg_yaw);
            // }
            // else
            // {
            // 	trackGoal();
            // }
        }
        else
        {
            if (!play)
            {
                predictGoal(msg_yaw, -1.6);
            }
            //play = true;
        }

        if (stabilize_state == 1)
        {
            // printf("...STABILIZE!!!\n");
            if (cnt_stab > 5)
            {
                cekWaktu(1);
                if (timer)
                {
                    stabilize_state = 0;
                }
            }
            else
            {
                motion("0");
                setWaktu();
                cnt_stab++;
            }
        }
        
        /*if (cam_x != 999 && cam_y != 999 && !dont_calibrate)
		{
		   	initialPos_X = initialPos_Y = 0;
		   	if (cekArah())
		   	{
		   		deltaPos_X = cam_x * -1;
				deltaPos_Y = cam_y * -1;
		   	}
		   	else 
		   	{
				deltaPos_X = cam_x;
				deltaPos_Y = cam_y;
			}
		 }*/

        auto msg_pose = nav_msgs::msg::Odometry();
        // robotPos_X = field_x + initialPos_X;
        // robotPos_Y = field_y + initialPos_Y;
        robotPos_X = deltaPos_X + initialPos_X;
        robotPos_Y = deltaPos_Y + initialPos_Y;
        msg_pose.pose.pose.position.x = robotPos_X;
        msg_pose.pose.pose.position.y = robotPos_Y;
        msg_pose.pose.pose.position.z = 1.0;
        Odometry_->publish(msg_pose);

        zmqpp::message incoming;
        if (socket_.receive(incoming, true))
        {
            nlohmann::json j;
            j["a"] = robotPos_X;
            j["b"] = robotPos_Y;
            j["c"] = ballDistance;
            j["d"] = ball_pose_x;
            j["e"] = Pickup;
            j["f"] = play;
            j["g"] = msg_yaw;
            j["h"] = msg_strategy;
            j["i"] = role;
            j["j"] = robot5BackIn;
            j["z"] = ball_pose_y;
            j["groot"] = current_bt_status;
            j["tg"] = vizTargetGrid;
            j["tgx"] = vizTargetOffsetX;
            j["tgy"] = vizTargetOffsetY;
            j["tx"] = vizTargetPosX;
            j["ty"] = vizTargetPosY;
            j["tv"] = vizTargetValid;
            string message = j.dump();

            zmqpp::message reply;
            reply << message;
            socket_.send(reply);
        }

        auto msg_ball_stat = std_msgs::msg::Bool();
        if (ballLost(35))
        {
            msg_ball_stat.data = false;
        } else 
        {
            msg_ball_stat.data = true;
        }
        ball_status_pub->publish(msg_ball_stat);
        
        bt_tick_condition_events_.clear();
        bt_tick_action_events_.clear();
        tree.tickRoot();
        std::string bt_status_now = summarizeFromTickEvents(play);
        if (bt_status_now.empty())
        {
            bt_status_now = summarizeBehaviorTreeStatus(play);
        }
        if (!bt_status_now.empty())
        {
            current_bt_status = bt_status_now;
        }
        Ball_X = Pinalty_X = Goal_X = Xcross_X = -1;
        Ball_Y = Pinalty_Y = Goal_Y = Xcross_X = -1;
        b_pole_1_x = b_pole_1_y = b_pole_2_x = b_pole_2_y = t_pole_1_x = t_pole_1_y = t_pole_2_x = t_pole_2_y = cB_pole_X = cB_pole_Y = cT_pole_X = cT_pole_Y = -1;
        Left_X_Cross_X = Left_X_Cross_Y = Right_X_Cross_X = Right_X_Cross_Y = Left_T_Cross_X = Left_T_Cross_Y = Right_T_Cross_X = Right_T_Cross_Y = -1;
        Left_Corner_X = Left_Corner_Y = Right_Corner_X = Right_Corner_Y = Left_L_Cross_X = Left_L_Cross_Y = Right_L_Cross_X = Right_L_Cross_Y = -1;
        Left_T_Corner_X = Left_T_Corner_Y = Right_T_Corner_X = Right_T_Corner_Y = Robot_X = Robot_Y = goal_L_pole_X = goal_L_pole_Y = goal_R_pole_X = goal_R_pole_Y = -1;
        GA_L_Cross_X = GA_L_Cross_Y = PA_L_Cross_X = PA_L_Cross_Y = -1;

        x_min = -1;
        x_max = -1;
        y_min = -1;
        y_max = -1;
    }

    void resetCase0()
    {
        delayWaitBall = 0;
        saveAngle	=
		searchKe	=	//counting berapa kali search
		sabar		=
		matte		=
		tiltPos	= 0;
    }

    double coorXx, coorXy, coorYy, coorYx, sX, sY, deltaPos_X = 0, deltaPos_Y = 0;
    void mapping(double arukuX, double arukuY) // jojo
    {
        // value use minimal / 10
        if (arukuX > 0)
        {
            if (arukuX > 0 && arukuX <= 0.01)
            {
                sX = arukuX1 * arukuX / 0.01; //1.6
            }
            else if (arukuX > 0.01 && arukuX <= 0.02)
            {
                sX = arukuX2 * arukuX / 0.02; //2.18
            }
            else if (arukuX > 0.02 && arukuX <= 0.03)
            {
                sX = arukuX3 * arukuX / 0.03; //3.46 		//3.58  //4.50
            }
            else if (arukuX > 0.03 && arukuX <= 0.04)
            {
                sX = arukuX4 * arukuX / 0.04; //4		//4
            }
            else if (arukuX > 0.04 && arukuX <= 0.05)
            {
                sX = arukuX5 * arukuX / 0.05; //5
            }
            else if (arukuX > 0.05 && arukuX <= 0.06)
            {
                sX = arukuX6 * arukuX / 0.06; //5.75	//7.23
            }
            else if (arukuX > 0.06 && arukuX <= 0.07)
            {
                sX = arukuX7 * arukuX / 0.07; //5.75   //8.23
            }
            else if (arukuX > 0.07 && arukuX <= 0.08)
            {
                sX = arukuX8 * arukuX / 0.08; //5.75 //9.25
            }
            else if (arukuX > 0.08 && arukuX <= 0.09)
            {
                sX = arukuX9 * arukuX / 0.09; //5.75  //10.26
            }
        }
        else if (arukuX < 0)
        {
            if (arukuX == -0.01)
            {
                sX = 0;
            }
            else if (arukuX == -0.02)
            {
                sX = -1.28;
            }
            else if (arukuX == -0.03)
            {
                sX = -1.91;
            }
            else
            {
                sX = -1.91 * arukuX / -0.03;
            }
        }
        else if (arukuX == 0)
        {
            //sX = 0.2;
            sX = 0;
        }

        if (arukuY > 0)
        {
            //printf("POSITIF\n"); //LEFT
            if (arukuY > 0 && arukuY <= 0.01)
            {
                sY = (arukuY1 * arukuY / 0.01) / 2; //0
            }
            else if (arukuY > 0.01 && arukuY <= 0.02)
            {
                sY = (arukuY2 * arukuY / 0.02) / 2; //3.01
            }
            else if (arukuY > 0.02 && arukuY <= 0.03)
            {
                sY = (arukuY3 * arukuY / 0.03) / 2; //4.56
            }
            else
            {
                sY = (arukuY4 * arukuY / 0.03) / 2; //4.56
            }
        }
        else if (arukuY < 0)
        {

            if (arukuY < 0 && arukuY >= -0.01)
            {
                sY = (arukuYn1 * arukuY / -0.01) / 2; //0
            }
            else if (arukuY < -0.01 && arukuY >= -0.02)
            {
                sY = (-arukuYn2 * arukuY / -0.02) / 2; //-3.41
            }
            else if (arukuY < -0.02 && arukuY >= -0.03)
            {
                sY = (-arukuYn3 * arukuY / -0.03) / 2; //-4.83
            }
            else
            {
                sY = (-arukuYn4 * arukuY / -0.03) / 2; //-4.83
            }
        }
        else if (arukuY == 0)
        {
            sY = 0;
        }

        // printf("robotWalk = %f, %f\n", sX, sY);
        //bagian X
        coorXx = (sX * (cos((abs(msg_yaw)) * PI / 180)));
        coorXy = (sX * (sin(msg_yaw * PI / 180)));
        //bagian Y
        if (msg_yaw <= 180 && msg_yaw >= -90)
        {
            coorYx = (sY * (cos((abs(msg_yaw - 90)) * PI / 180)));
            //coorYx = sY * cos(angleYm*PI/180);
            coorYy = sY * sin((msg_yaw - 90) * PI / 180);
        }
        else
        {
            coorYx = (sY * (cos((abs(msg_yaw + 270)) * PI / 180)));
            //coorYx = sY * cos(angleYm*PI/180);
            coorYy = (sY * (sin((msg_yaw + 270) * PI / 180)));
        }
        deltaPos_X = deltaPos_X + (coorXx * 1) + (1.1 * coorYx); //komen Yx
        deltaPos_Y = deltaPos_Y + (1.1 * coorXy) + coorYy;       //Xy = x1.1
        // printf("deltaPos = %f, %f\n", deltaPos_X, deltaPos_Y);
    }

    // Define constants for the filter
    const double alpha = 0.5;
    const double dt = 0.04;

    // Define variables for the filter
    double filtered_x_vel = 0.0;
    double filtered_y_vel = 0.0;
    double filtered_imu_yaw = 0.0;

    // Define variables for the robot's position and heading
    double field_x = 0.0;
    double field_y = 0.0;
    double x_pos = 0.0;
    double y_pos = 0.0;

    // Define the function to update the robot's position using filtered IMU readings
    void new_mapping(double x_vel, double y_vel, int imu_yaw) {
        // Apply the low-pass filter to the IMU readings
        filtered_x_vel = alpha * filtered_x_vel + (1 - alpha) * x_vel;
        filtered_y_vel = alpha * filtered_y_vel + (1 - alpha) * y_vel;
        filtered_imu_yaw = alpha * filtered_imu_yaw + (1 - alpha) * imu_yaw;

        // Convert the filtered IMU yaw from degrees to radians
        double imu_yaw_rad = filtered_imu_yaw * PI / 180.0;

        // Calculate the robot's new position based on its filtered velocity and heading
        x_pos += filtered_x_vel * cos(imu_yaw_rad) + filtered_y_vel * sin(imu_yaw_rad) * dt;
        y_pos += filtered_x_vel * sin(imu_yaw_rad) + filtered_y_vel * cos(imu_yaw_rad) * dt;

        // Map the robot's position to the RoboCup field coordinates in cm
        field_x = x_pos * 100;
        field_y = y_pos * 100;
    }

    void readTrackbar(const std_msgs::msg::Float64MultiArray::SharedPtr msg_trackbar_)
    {
        ball_panKP = goal_panKP = msg_trackbar_->data[0];
        ball_panKD = goal_panKD = msg_trackbar_->data[1];
        ball_tiltKP = goal_tiltKP = msg_trackbar_->data[2];
        ball_tiltKD = goal_tiltKD = msg_trackbar_->data[3];
    }

    void readButton(const bfc_msgs::msg::Button::SharedPtr msg_btn_)
    {
        msg_strategy = msg_btn_->strategy;
        msg_kill = msg_btn_->kill;
    }
    
    void readPeluit(const std_msgs::msg::Float32::SharedPtr msg_plt_)
    {
    // Simpan nilai frekuensi Hz ke variabel internal
    msg_peluit_hz = msg_plt_->data;
    }

    // Define necessary constants
    const double CUTOFF_FREQUENCY = 10.0; // Filter cutoff frequency in Hz

    // Define necessary variables
    double prev_filtered_value = 0.0;

    // Define low-pass filter function
    double low_pass_filter(double prev_filtered_value, double raw_value, double dt, double cutoff_freq) {
        double alpha = 2 * PI * cutoff_freq * dt / (2 * PI * cutoff_freq * dt + 1);
        double filtered_value = alpha * raw_value + (1 - alpha) * prev_filtered_value;
        return filtered_value;
    }

    // Define function to convert yaw angle from degrees to radians within the range of -pi to +pi
    double degrees_to_radians(double degrees) {
        double radians = degrees * PI / 180.0;
        while (radians > PI) {
            radians -= 2 * PI;
        }
        while (radians < -PI) {
            radians += 2 * PI;
        }
        return radians;
    }

    double radians_to_degrees(double radians) {
        double degrees = radians * 180.0 / M_PI;
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees < -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

    bool robotFall = false;
    void readImu(const sensor_msgs::msg::Imu::SharedPtr msg_imu_)
    {
        msg_roll = msg_imu_->angular_velocity.x;
        msg_pitch = msg_imu_->angular_velocity.y;
        msg_yaw = msg_imu_->angular_velocity.z;

        // double filtered_value = low_pass_filter(prev_filtered_value, degrees_to_radians(msg_yaw), 0.04, CUTOFF_FREQUENCY);
        // prev_filtered_value = filtered_value;
        // // printf("... msg_yaw = %d, filtered_yaw = %.2f\n", msg_yaw, radians_to_degrees(filtered_value));
        // msg_yaw = (int)radians_to_degrees(filtered_value);

        if (msg_roll >= 45 || msg_roll <= 45 || msg_pitch >= 45 || msg_pitch <= 45)
        {
            robotFall = true;
        }
        else
        {
            robotFall = false;
        }
    }

    void objCoor(const geometry_msgs::msg::PointStamped::SharedPtr msg_bbox_)
    {
        Ball_X = msg_bbox_->point.x;
        Ball_Y = msg_bbox_->point.y;
    }
    //game_controller kri

   /* void readGameControllerData(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
    {
        State = msg->data[0]; // 0 = initial, 1 = ready, 2 = set, 3 = Play, 4 = Finish
        FirstHalf = msg->data[1];
        Version = msg->data[2];
        PacketNumber = msg->data[3];
        PlayerTeam = msg->data[4];
        // GameTipe = msg->data[7];
        KickOff = msg->data[5]; // 1 = kickoff team kita, 0 = kickoff team lawan
        SecondaryState = msg->data[6];
        DropTeam = msg->data[7];
        DropTime = msg->data[8];
        Remaining = msg->data[9];      // waktu permainan 600 = 10 menit
        SecondaryTime = msg->data[10]; // jika waktu tunggu kickoff lebih dari 10 detik dan positionning 30 detik
        // ket :    1 = msg->data[]; untuk data GameController yang kiri
        //	        2 = msg->data[]; untuk data GameController yang kanan
        timNumber1 = msg->data[11]; //
        timNumber2 = msg->data[12]; //
        timColour1 = msg->data[13]; //
        timColour2 = msg->data[14];
        Score1 = msg->data[15];
        Score2 = msg->data[16];
        Penaltyshoot1 = msg->data[17];
        Penaltyshoot2 = msg->data[18];
        Singleshoot1 = msg->data[19];
        Singleshoot2 = msg->data[20];
        // Coachsequence1 = msg->data[24];
        // Coachsequence2 = msg->data[25];

        Penalty1 = msg->data[21];
        Penalty2 = msg->data[22];
        TimeUnpenalis1 = msg->data[23];
        TimeUnpenalis2 = msg->data[24];
        YellowCard1 = msg->data[30];
        YellowCard2 = msg->data[31];
        RedCard1 = msg->data[32];
        RedCard2 = msg->data[33];
        secondaryInfo[0] = msg->data[29];
        secondaryInfo[1] = msg->data[30];
        secondaryInfo[2] = msg->data[31];
        secondaryInfo[3] = msg->data[32];
        
        
        
    }
    */
    //game_controller robocup 26
    void readGameControllerData(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
    {
        // === SAFETY CHECK: Publisher mengirim 19 elemen ===
        if (msg->data.size() < 19) {
            RCLCPP_WARN(this->get_logger(), "Data GC tidak sinkron! Diterima %zu elemen, butuh 19", msg->data.size());
            return;
        }

        // === PARSING SESUAI LAYOUT PUBLISHER (game_controller.cpp) ===
        // [0]=State [1]=FirstHalf [2]=Version [3]=PacketNumber
        // [4]=PlayerTeam [5]=KickOff [6]=SecondaryState(gamePhase)
        // [7]=Remaining [8]=SecondaryTime
        // [9]=timNumber1 [10]=timNumber2 [11]=timColour1 [12]=timColour2
        // [13]=Score1 [14]=Score2 [15]=Stopped
        // [16]=Penalty1 [17]=Penalty2 [18]=setPlay

        State           = msg->data[0];  // 0=Init, 1=Ready, 2=Set, 3=Play, 4=Finish
        FirstHalf       = msg->data[1];
        Version         = msg->data[2];
        PacketNumber    = msg->data[3];
        PlayerTeam      = msg->data[4];
        KickOff         = msg->data[5];  // team number of kicking team, 255=none
        SecondaryState  = msg->data[6];  // gamePhase: 0=Normal,1=PenaltyShoot,dll
        Remaining       = msg->data[7];  // detik tersisa
        SecondaryTime   = msg->data[8];  // waktu sekunder

        timNumber1      = msg->data[9];
        timNumber2      = msg->data[10];
        timColour1      = msg->data[11];
        timColour2      = msg->data[12];
        Score1          = msg->data[13];
        Score2          = msg->data[14];
        Stopped         = msg->data[15]; // 1 = play is stopped

        Penalty1        = msg->data[16];
        Penalty2        = msg->data[17];
        secondaryInfo[0]= msg->data[18]; // setPlay (0=None,1=DirectFK,2=IndirectFK,3=Penalty,4=ThrowIn,5=GoalKick,6=CornerKick)

        // Field yang tidak dikirim publisher → set default
        DropTeam = DropTime = 0;
        Penaltyshoot1 = Penaltyshoot2 = Singleshoot1 = Singleshoot2 = 0;
        TimeUnpenalis1 = TimeUnpenalis2 = 0;
        YellowCard1 = YellowCard2 = RedCard1 = RedCard2 = 0;
        secondaryInfo[1] = secondaryInfo[2] = secondaryInfo[3] = 0;

        // --- Print GC26 v20 status setiap 2 detik ---
        static int gcPrintCounter = 0;
        if (++gcPrintCounter >= 50) { // 50 * 40ms = 2 detik
            gcPrintCounter = 0;
            auto penaltyName = [](int p) -> const char* {
                switch(p) {
                    case 0:  return "None";
                    case 1:  return "IllegalPositioning";
                    case 2:  return "MotionInSet";
                    case 3:  return "MotionInStop";
                    case 4:  return "LocalGameStuck";
                    case 5:  return "IncapableRobot";
                    case 6:  return "PickedUp";
                    case 7:  return "BallHolding";
                    case 8:  return "LeavingTheField";
                    case 9:  return "PlayingWithArmsHands";
                    case 10: return "Pushing";
                    case 11: return "Cautioned";
                    case 12: return "SentOff";
                    case 13: return "Substitute";
                    default: return "Unknown";
                }
            };
            auto setPlayName = [](int sp) -> const char* {
                switch(sp) {
                    case 0: return "None";
                    case 1: return "DirectFreeKick";
                    case 2: return "IndirectFreeKick";
                    case 3: return "PenaltyKick";
                    case 4: return "ThrowIn";
                    case 5: return "GoalKick";
                    case 6: return "CornerKick";
                    default: return "Unknown";
                }
            };
            printf("[R-GC] State=%d(%s) SetPlay=%d(%s) Stopped=%d KickOff=%d Score=%d-%d Pen1=%d(%s) Pen2=%d(%s)\n",
                   State,
                   State==0?"Init":State==1?"Ready":State==2?"Set":State==3?"Playing":State==4?"Finish":"Unknown",
                   secondaryInfo[0], setPlayName(secondaryInfo[0]),
                   Stopped, KickOff, Score1, Score2,
                   Penalty1, penaltyName(Penalty1),
                   Penalty2, penaltyName(Penalty2));
        }
        
        // --- Deteksi Goal (salah satu tim score berubah) ---
        static int prevScore1 = -1, prevScore2 = -1;
        bool goalScored = false;
        if (prevScore1 >= 0 && prevScore2 >= 0) {
            if (Score1 != prevScore1 || Score2 != prevScore2) {
                goalScored = true;
                if (Score1 > prevScore1)
                    printf("[GC26] *** GOAL: Tim0 (num=%d) score %d → %d ***\n",
                           (int)timNumber1, prevScore1, (int)Score1);
                if (Score2 > prevScore2)
                    printf("[GC26] *** GOAL: Tim1 (num=%d) score %d → %d ***\n",
                           (int)timNumber2, prevScore2, (int)Score2);
            }
        }
        prevScore1 = Score1;
        prevScore2 = Score2;

        // Reset kickoff flags ketika ada goal (berlaku untuk kedua tim)
        if (goalScored) {
            printf("[GC26] Goal terdeteksi → reset kickoff flags\n");
            finishFirstKick   = false;
            FirstKicked       = false;
            doneFirstKick     = false;
            kickOffDone       = false;
            action_kick       = false;
            action_afterKick  = false;
            isKicked          = false;
        }
        // --- Kirim Status Stopped Terus-Menerus ke walk_server (Hindari Packet Loss UDP) ---
        if (Stopped == 1) {
            motion("gc,1"); // Aktifkan freeze & matikan fall check
        } else {
            motion("gc,0"); // Normal
        }
        //sampai sini
    }


    double PAN = 0.0, TILT = 0.0;
    // Head Movement =============================================================================
    void headMove(double pan, double tilt)
    {
        if (useRos)
        { // Socket
            auto msg_head_ = bfc_msgs::msg::HeadMovement();
            msg_head_.pan = ceil(pan * 100.0) / 100.0;
            msg_head_.tilt = ceil(tilt * 100.0) / 100.0;
            cmd_head_->publish(msg_head_);
        }
        else
        { // NotePad
            FILE *fp, *outputfp;
            outputfp = fopen("../Player/HeadNote", "wb");
            fprintf(outputfp, "%.2lf,%.2lf", pan, tilt);
            fclose(outputfp);
        }

        headPan = posPan = ceil(pan * 100.0) / 100.0;
        headTilt = posTilt = ceil(tilt * 100.0) / 100.0;
        PAN = headPan - 0.15;
        TILT = headTilt;
        // printf("headPan = %.2f \t headTilt = %.2f\n", pan, tilt);
    }

    // Robot Movement ============================================================================
    void motion(const char line[2])
    {
        // printf("Motion = %s\n",line[2]);
        // Tendang Jauh Kiri		= 1
        // Tendang Jauh Kanan		= 2

        // Tendang Pelan Kiri		= 3
        // Tendang Pelan Kanan		= 4

        // Tendang Ke Samping Kiri	= 5
        // Tendang Ke Samping Kanan	= 6

        // Tendang WalkKick Kiri		= t
        // Tendang WalkKick Kanan	= y

        // Duduk				= 7
        // Berdiri			= 8
        // Play				= 9
        // Stop				= 0

        // Saat brief stop: BLOCK semua motion kecuali "8" (berdiri) dan "0" (stop)
        // Bahkan get-up motion harus di-block saat Stop Play
        /*
        if (Stopped == 1) {
            if (line[0] != '8' && line[0] != '0') {
                printf("[MOTION-BLOCKED] Stopped=1, blocking motion '%c'\n", line[0]);
                return;  // BLOCK! Jangan kirim perintah
            }
        }
        
        if (Stopped == 1) {
            if (strncmp(line, "gc,", 3) != 0) {
                printf("[MOTION-BLOCKED] Stopped=1, blocking motion '%s'\n", line);
                return;  // BLOCK semua kecuali "gc," command
            }
            }*/
        if (useRos)
        { // Socket
            auto msg_mot_ = std_msgs::msg::String();
            msg_mot_.data = &line[0];
            cmd_mot_->publish(msg_mot_);
        }
        else
        { // NotePad
            FILE *outputfp;
            outputfp = fopen("../Player/WalkNote", "wb");
            fprintf(outputfp, "%s", &line[0]);
            fclose(outputfp);
        }
    }

    // Walk Controller ===========================================================================
    double walkX = 0.0,
           walkY = 0.0,
           walkA = 0.0;
    double Walk(double x, double y, double a)
    {
    	        // Saat brief stop: BLOCK semua walk command
        if (Stopped == 1) {
            printf("[WALK-BLOCKED] Stopped=1, blocking Walk(%.3f,%.3f,%.3f)\n", x, y, a);
            return 0.0;
        }
        char line[50];

        motion("9");

        walkX = x;
        walkY = y;
        walkA = a;

        if (useRos)
        {
            auto msg_walk_ = geometry_msgs::msg::Twist();
            msg_walk_.linear.x = x + erorrXwalk;
            msg_walk_.linear.y = y + erorrYwalk;
            msg_walk_.linear.z = a + erorrAwalk;
            cmd_vel_->publish(msg_walk_);
        }
        else
        { // NotePad
            FILE *outputfp;
            strcpy(line, "walk");
            outputfp = fopen("../Player/WalkNote", "wb");
            fprintf(outputfp, "%s,%.2lf,%.2lf,%.2lf", &line[0], x + erorrXwalk, y, a); // setting jalan ditempat default
            fclose(outputfp);
             printf("walk : %g,%g,%g\n",x,y,a);
        }

        return (walkX, walkY, walkA);
    }

    // Acceleration & Decceleration
    const double velX_Kp = 0.005, velY_Kp = 0.01, velA_Kp = 0.1;
    double velocityX = 0.0, velocityY = 0.0, velocityA = 0.0;
    void set_velocity(double vWalkX, double vWalkY, double vWalkA)
    {
        double error_X = velX_Kp * vWalkX;
        double error_Y = velY_Kp * vWalkY;
        double error_A = velA_Kp * vWalkA;
        velocityX = velocityX + error_X;
        velocityY = velocityY + error_Y;
        velocityA = velocityA + error_A;
        // if (abs(vy) > 0.02 || abs(va) > 0.3)
        // {
        //     if (vWalkX > kejarMid)
        //     {
        //         vWalkX = kejarMid; // reduce speed for stability
        //     }
        // }
        if (vWalkX > 0 && velocityX > vWalkX)
        {
            velocityX = vWalkX;
        }
        else if (vWalkX < 0 && velocityX < vWalkX)
        {
            velocityX = vWalkX;
        }
        if (vWalkY > 0 && velocityY > vWalkY)
        {
            velocityY = vWalkY;
        }
        else if (vWalkY < 0 && velocityY < vWalkY)
        {
            velocityY = vWalkY;
        }
        if (vWalkA > 0 && velocityA > vWalkA)
        {
            velocityA = vWalkA;
        }
        else if (vWalkA < 0 && velocityA < vWalkA)
        {
            velocityA = vWalkA;
        }   

        if (vWalkX == 0.0)
        {
            velocityX = 0;
        }
        // printf("ERROR = %f,%f,%f\n", error_X, error_Y, error_A);
        // printf("NEW VELOCITY = %.2f, %.2f, %.2f \n", velocityX, velocityY, velocityA);
        
        if (stabilize_state == 0)
        {
            if (cnt_stab2 <= 0)
            {
                // Walk(velocityX, velocityY, velocityA);
                Walk(velocityX, vWalkY, vWalkA);
            }
            else
            {
                Walk(0.0, 0.0, 0.0);
                reset_velocity();
                cnt_stab2--;
            }
        }
        else
        {
            reset_velocity();
        }
    }

    void reset_velocity()
    {
        velocityX = velocityY = velocityA = 0;
    }

    void declareParameters()
    {
    	this->declare_parameter("enableAvoid", false);
        this->declare_parameter("robotNumber", 0);
        this->declare_parameter("useDribbleMode", true);
        this->declare_parameter("nomorpickup", 0);
        this->declare_parameter("frame_X", 640);
        this->declare_parameter("frame_Y", 640);
        this->declare_parameter("useBanting", false);
        this->declare_parameter("pPanTendangKanan", -0.20);
        this->declare_parameter("pTiltTendangKanan", -0.57);
        this->declare_parameter("pPanTendangKiri", -0.20);
        this->declare_parameter("pTiltTendangKiri", -0.57);
        this->declare_parameter("ballPositioningSpeed", 0.12);
        this->declare_parameter("cSekarang", -0.60);
        this->declare_parameter("cAktif", -1.40);
        this->declare_parameter("tiltBolaJauh", -1.70);
        this->declare_parameter("tiltBolaDekat", -1.40);
        this->declare_parameter("posTiltLocal", -1.90);
        this->declare_parameter("posTiltGoal", -1.80);
        this->declare_parameter("ball_panKP", 0.075);
        this->declare_parameter("ball_panKD", 0.0000505);
        this->declare_parameter("ball_tiltKP", 0.05);
        this->declare_parameter("ball_tiltKD", 0.0000755);
        this->declare_parameter("goal_panKP", 0.10);
        this->declare_parameter("goal_panKD", 0.000050);
        this->declare_parameter("goal_tiltKP", 0.05);
        this->declare_parameter("goal_tiltKD", 0.000050);
        this->declare_parameter("errorXwalk", 0.0);
        this->declare_parameter("errorYwalk", 0.0);
        this->declare_parameter("errorAwalk", 0.0);
        this->declare_parameter("jalanGrid", 0);
        this->declare_parameter("aruku", 0.050);
        this->declare_parameter("jalan", 0.040);
        this->declare_parameter("lari", 0.050);
        this->declare_parameter("kejar", 0.060);
        this->declare_parameter("kejarMid", 0.070);
        this->declare_parameter("kejarMax", 0.089);
        this->declare_parameter("tendangJauh", 1);
        this->declare_parameter("tendangSamping", 3);
        this->declare_parameter("tendangDekat", 5);
        this->declare_parameter("sudutTengah", 0);
        this->declare_parameter("sudutKanan", 30);
        this->declare_parameter("sudutKiri", -30);
        this->declare_parameter("rotateGoal_x", -0.0005);
        this->declare_parameter("rotateGoal_y", 0.017);
        this->declare_parameter("rotateGoal_a", 0.14);
        this->declare_parameter("myAccrX", 0.3);
        this->declare_parameter("myAccrY", 0.0);
        this->declare_parameter("tinggiRobot", 48);
        this->declare_parameter("outputSudutY1", 9.0);
        this->declare_parameter("inputSudutY1", -0.40);
        this->declare_parameter("outputSudutY2", 60.0);
        this->declare_parameter("inputSudutY2", -1.43);
        this->declare_parameter("panSaveKanan", -0.30);
        this->declare_parameter("panSaveKiri", 0.30);
        this->declare_parameter("outputSudutX1", 0.0);
        this->declare_parameter("inputSudutX1", 0.0);
        this->declare_parameter("outputSudutX2", 45.0);
        this->declare_parameter("inputSudutX2", -0.8);
        this->declare_parameter("arukuX1", 0.15);
        this->declare_parameter("arukuX2", 2.46);
        this->declare_parameter("arukuX3", 3.90);
        this->declare_parameter("arukuX4", 5.43);
        this->declare_parameter("arukuX5", 6.0);
        this->declare_parameter("arukuX6", 6.7);
        this->declare_parameter("arukuX7", 7.5);
        this->declare_parameter("arukuX8", 8.2);
        this->declare_parameter("arukuX9", 8.2);
        this->declare_parameter("arukuY1", 0.15);
        this->declare_parameter("arukuY2", 0.15);
        this->declare_parameter("arukuY3", 0.15);
        this->declare_parameter("arukuY4", 0.15);
        this->declare_parameter("arukuYn1", 0.15);
        this->declare_parameter("arukuYn2", 0.15);
        this->declare_parameter("arukuYn3", 0.15);
        this->declare_parameter("arukuYn4", 0.15);
        this->declare_parameter("usePenaltyStrategy", false);
        this->declare_parameter("useVision", false);
        this->declare_parameter("useImu", false);
        this->declare_parameter("useRos", false);
        this->declare_parameter("useGameController", false);
        this->declare_parameter("useCoordination", false);
        this->declare_parameter("useLocalization", false);
        this->declare_parameter("useFollowSearchGoal", false);
        this->declare_parameter("useSearchGoal", false);
        this->declare_parameter("useDribble", false);
        this->declare_parameter("dribbleOnly", false);
        this->declare_parameter("useSideKick", false);
        this->declare_parameter("useLastDirection", false);
        this->declare_parameter("useNearFollowSearchGoal", false);
        this->declare_parameter("firstStateCondition", 0);
        this->declare_parameter("firstStateLocalization", 0);
        this->declare_parameter("barelang_color", 0);
        this->declare_parameter("dropball", 0);
        this->declare_parameter("team", 0);
        this->declare_parameter("useDisplay", false);
        this->declare_parameter("useOmnidirection", false);
        this->declare_parameter("useWalkKick", false);
        this->declare_parameter("tree_path", "");
        this->declare_parameter("max_current", 15);
        this->declare_parameter("modePlay", 0);
        this->declare_parameter("useFollowExecutor", false);
        this->declare_parameter("forceKanan", false);
        this->declare_parameter("forceKiri", false);
        this->declare_parameter("useBodyTracking", false);
        this->declare_parameter("useKickOffGoal", false);
        this->declare_parameter("valBolaGerak", 10);
        // InitialPosition values
        this->declare_parameter("init_play_left_yaw_threshold", 70);
        this->declare_parameter("init_play_left_x", -250);
        this->declare_parameter("init_play_left_y", -320);
        this->declare_parameter("init_play_right_x", -250);
        this->declare_parameter("init_play_right_y", 320);

        this->declare_parameter("robocup_yaw", 0);
        this->declare_parameter("init_robocup_left_x", -100);
        this->declare_parameter("init_robocup_left_y", -307);
        this->declare_parameter("init_robocup_right_x", -100);
        this->declare_parameter("init_robocup_right_y", 307);

        // RobotPositioning values
        this->declare_parameter("kickoff_kita_release_left_x", -50);
        this->declare_parameter("kickoff_kita_release_left_y", 0);
        this->declare_parameter("kickoff_kita_release_right_x", -50);
        this->declare_parameter("kickoff_kita_release_right_y", 0);
        this->declare_parameter("musuh_kickoff_left_x", -100);
        this->declare_parameter("musuh_kickoff_left_y", 0);
        this->declare_parameter("musuh_kickoff_right_x", -100);
        this->declare_parameter("musuh_kickoff_right_y", 0);

        this->declare_parameter("defend_x", -150);
        this->declare_parameter("defend_y", 0);

        this->declare_parameter("release1_x", -300);
        this->declare_parameter("release1_y", 0);
        this->declare_parameter("release2_x", -75);
        this->declare_parameter("release2_y", 0);
        this->declare_parameter("release3_x", 100);
        this->declare_parameter("release3_y", 0);
        
        this->declare_parameter("cornerKickAngleLeft",   45);  // pojok kiri → serong ke kanan-dalam
        this->declare_parameter("cornerKickAngleRight", -45); 
        
        this->declare_parameter("goalAreaMinX",        325);
        this->declare_parameter("goalCornerLeftMaxY",  -130);
        this->declare_parameter("goalCornerRightMinY",  130);
        this->declare_parameter("leftSideMaxY",        -100);
        this->declare_parameter("rightSideMinY",        100);
        this->declare_parameter("angleCornerLeft",       80);
        this->declare_parameter("angleCornerRight",     -80);
        this->declare_parameter("angleGoalLeft",         10);
        this->declare_parameter("angleGoalRight",       -10);
        this->declare_parameter("angleSideLeftOpp",      40);
        this->declare_parameter("angleSideLeftOwn",      35);
        this->declare_parameter("angleSideRightOpp",    -40);
        this->declare_parameter("angleSideRightOwn",    -35);
    }

    std::string tree_path = "";
    int valBolaGerak = 0;
    void getParameters()
    {
        ball_panKP = this->get_parameter("ball_panKP").as_double();
        ball_panKD = this->get_parameter("ball_panKD").as_double();
        ball_tiltKP = this->get_parameter("ball_tiltKP").as_double();
        ball_tiltKD = this->get_parameter("ball_tiltKD").as_double();
        goal_panKP = this->get_parameter("goal_panKP").as_double();
        goal_panKD = this->get_parameter("goal_panKD").as_double();
        goal_tiltKP = this->get_parameter("goal_tiltKP").as_double();
        goal_tiltKD = this->get_parameter("goal_tiltKD").as_double();
        useDribbleMode = this->get_parameter("useDribbleMode").as_bool();
        enableAvoid = this->get_parameter("enableAvoid").as_bool();
        frame_X = this->get_parameter("frame_X").as_int();
        frame_Y = this->get_parameter("frame_Y").as_int();
        tiltBolaDekat = this->get_parameter("tiltBolaDekat").as_double();
        tiltBolaJauh = this->get_parameter("tiltBolaJauh").as_double();
        robotNumber = this->get_parameter("robotNumber").as_int();
        pPanTendangKanan = this->get_parameter("pPanTendangKanan").as_double();
        pTiltTendangKanan = this->get_parameter("pTiltTendangKanan").as_double();
        pPanTendangKiri = this->get_parameter("pPanTendangKiri").as_double();
        pTiltTendangKiri = this->get_parameter("pTiltTendangKiri").as_double();
        ballPositioningSpeed = this->get_parameter("ballPositioningSpeed").as_double();
        cSekarang = this->get_parameter("cSekarang").as_double();
        cAktif = this->get_parameter("cAktif").as_double();
        posTiltLocal = this->get_parameter("posTiltLocal").as_double();
        posTiltGoal = this->get_parameter("posTiltGoal").as_double();
        erorrXwalk = this->get_parameter("errorXwalk").as_double();
        erorrYwalk = this->get_parameter("errorYwalk").as_double();
        erorrAwalk = this->get_parameter("errorAwalk").as_double();
        jalan = this->get_parameter("jalan").as_double();
        arukuX1 = this->get_parameter("arukuX1").as_double();
        arukuX2 = this->get_parameter("arukuX2").as_double();
        arukuX3 = this->get_parameter("arukuX3").as_double();
        arukuX4 = this->get_parameter("arukuX4").as_double();
        arukuX5 = this->get_parameter("arukuX5").as_double();
        arukuX6 = this->get_parameter("arukuX6").as_double();
        arukuX7 = this->get_parameter("arukuX7").as_double();
        arukuX8 = this->get_parameter("arukuX8").as_double();
        arukuX9 = this->get_parameter("arukuX9").as_double();
        arukuY1 = this->get_parameter("arukuY1").as_double();
        arukuY2 = this->get_parameter("arukuY2").as_double();
        arukuY3 = this->get_parameter("arukuY3").as_double();
        arukuY4 = this->get_parameter("arukuY4").as_double();
        arukuYn1 = this->get_parameter("arukuYn1").as_double();
        arukuYn2 = this->get_parameter("arukuYn2").as_double();
        arukuYn3 = this->get_parameter("arukuYn3").as_double();
        arukuYn4 = this->get_parameter("arukuYn4").as_double();
        lari = this->get_parameter("lari").as_double();
        aruku = this->get_parameter("aruku").as_double();
        kejar = this->get_parameter("kejar").as_double();
        kejarMid = this->get_parameter("kejarMid").as_double();
        kejarMax = this->get_parameter("kejarMax").as_double();
        jalanGrid = this->get_parameter("jalanGrid").as_int();
        tendangJauh = this->get_parameter("tendangJauh").as_int();
        tendangSamping = this->get_parameter("tendangSamping").as_int();
        tendangDekat = this->get_parameter("tendangDekat").as_int();
        sudutTengah = this->get_parameter("sudutTengah").as_int();
        sudutKanan = this->get_parameter("sudutKanan").as_int();
        sudutKiri = this->get_parameter("sudutKiri").as_int();
        nomorpickup = this->get_parameter("nomorpickup").as_int();
        rotateGoal_x = this->get_parameter("rotateGoal_x").as_double();
        rotateGoal_y = this->get_parameter("rotateGoal_y").as_double();
        rotateGoal_a = this->get_parameter("rotateGoal_a").as_double();
        useBanting = this->get_parameter("useBanting").as_bool();
        myAccrX = this->get_parameter("myAccrX").as_double();
        myAccrY = this->get_parameter("myAccrY").as_double();
        tinggiRobot = this->get_parameter("tinggiRobot").as_int();
        outputSudutY1 = this->get_parameter("outputSudutY1").as_double();
        inputSudutY1 = this->get_parameter("inputSudutY1").as_double();
        outputSudutY2 = this->get_parameter("outputSudutY2").as_double();
        inputSudutY2 = this->get_parameter("inputSudutY2").as_double();
        panSaveKanan = this->get_parameter("panSaveKanan").as_double();
        panSaveKiri = this->get_parameter("panSaveKiri").as_double();
        outputSudutX1 = this->get_parameter("outputSudutX1").as_double();
        inputSudutX1 = this->get_parameter("inputSudutX1").as_double();
        outputSudutX2 = this->get_parameter("outputSudutX2").as_double();
        inputSudutX2 = this->get_parameter("inputSudutX2").as_double();
        useRos = this->get_parameter("useRos").as_bool();
        usePenaltyStrategy = this->get_parameter("usePenaltyStrategy").as_bool();
        useVision = this->get_parameter("useVision").as_bool();
        useImu = this->get_parameter("useImu").as_bool();
        useGameController = this->get_parameter("useGameController").as_bool();
        useCoordination = this->get_parameter("useCoordination").as_bool();
        useLocalization = this->get_parameter("useLocalization").as_bool();
        useFollowSearchGoal = this->get_parameter("useFollowSearchGoal").as_bool();
        useSearchGoal = this->get_parameter("useSearchGoal").as_bool();
        useDribble = this->get_parameter("useDribble").as_bool();
        dribbleOnly = this->get_parameter("dribbleOnly").as_bool();
        useSideKick = this->get_parameter("useSideKick").as_bool();
        useLastDirection = this->get_parameter("useLastDirection").as_bool();
        useNearFollowSearchGoal = this->get_parameter("useNearFollowSearchGoal").as_bool();
        firstStateCondition = this->get_parameter("firstStateCondition").as_int();
        firstStateLocalization = this->get_parameter("firstStateLocalization").as_int();
        barelang_color = this->get_parameter("barelang_color").as_int();
        team = this->get_parameter("team").as_int();
        dropball = this->get_parameter("dropball").as_int();
        useDisplay = this->get_parameter("useDisplay").as_bool();
        useOmnidirection = this->get_parameter("useOmnidirection").as_bool();
        useWalkKick = this->get_parameter("useWalkKick").as_bool();
        tree_path = this->get_parameter("tree_path").as_string();
        max_current = this->get_parameter("max_current").as_int();
        modePlay = this->get_parameter("modePlay").as_int();
        useFollowExecutor = this->get_parameter("useFollowExecutor").as_bool();
        forceKanan = this->get_parameter("forceKanan").as_bool();
        forceKiri = this->get_parameter("forceKiri").as_bool();
        useBodyTracking = this->get_parameter("useBodyTracking").as_bool();
        useKickOffGoal = this->get_parameter("useKickOffGoal").as_bool();
        valBolaGerak = this->get_parameter("valBolaGerak").as_int();
        // InitialPosition values
        init_play_left_yaw_threshold = this->get_parameter("init_play_left_yaw_threshold").as_int();
        init_play_left_x = this->get_parameter("init_play_left_x").as_int();
        init_play_left_y = this->get_parameter("init_play_left_y").as_int();
        init_play_right_x = this->get_parameter("init_play_right_x").as_int();
        init_play_right_y = this->get_parameter("init_play_right_y").as_int();

        robocup_yaw = this->get_parameter("robocup_yaw").as_int();
        init_robocup_left_x = this->get_parameter("init_robocup_left_x").as_int();
        init_robocup_left_y = this->get_parameter("init_robocup_left_y").as_int();
        init_robocup_right_x = this->get_parameter("init_robocup_right_x").as_int();
        init_robocup_right_y = this->get_parameter("init_robocup_right_y").as_int();

        // RobotPositioning values
        kickoff_kita_release_left_x = this->get_parameter("kickoff_kita_release_left_x").as_int();
        kickoff_kita_release_left_y = this->get_parameter("kickoff_kita_release_left_y").as_int();
        kickoff_kita_release_right_x = this->get_parameter("kickoff_kita_release_right_x").as_int();
        kickoff_kita_release_right_y = this->get_parameter("kickoff_kita_release_right_y").as_int();
        musuh_kickoff_left_x = this->get_parameter("musuh_kickoff_left_x").as_int();
        musuh_kickoff_left_y = this->get_parameter("musuh_kickoff_left_y").as_int();
        musuh_kickoff_right_x = this->get_parameter("musuh_kickoff_right_x").as_int();
        musuh_kickoff_right_y = this->get_parameter("musuh_kickoff_right_y").as_int();

        defend_x = this->get_parameter("defend_x").as_int();
        defend_y = this->get_parameter("defend_y").as_int();

        release1_x = this->get_parameter("release1_x").as_int();
        release1_y = this->get_parameter("release1_y").as_int();
        release2_x = this->get_parameter("release2_x").as_int();
        release2_y = this->get_parameter("release2_y").as_int();
        release3_x = this->get_parameter("release3_x").as_int();
        release3_y = this->get_parameter("release3_y").as_int();
        
        cornerKickAngleLeft  = this->get_parameter("cornerKickAngleLeft").as_int();
        cornerKickAngleRight = this->get_parameter("cornerKickAngleRight").as_int();
        goalAreaMinX        = this->get_parameter("goalAreaMinX").as_int();
        goalCornerLeftMaxY  = this->get_parameter("goalCornerLeftMaxY").as_int();
        goalCornerRightMinY = this->get_parameter("goalCornerRightMinY").as_int();
        leftSideMaxY        = this->get_parameter("leftSideMaxY").as_int();
        rightSideMinY       = this->get_parameter("rightSideMinY").as_int();
        angleCornerLeft     = this->get_parameter("angleCornerLeft").as_int();
        angleCornerRight    = this->get_parameter("angleCornerRight").as_int();
        angleGoalLeft       = this->get_parameter("angleGoalLeft").as_int();
        angleGoalRight      = this->get_parameter("angleGoalRight").as_int();
        angleSideLeftOpp    = this->get_parameter("angleSideLeftOpp").as_int();
        angleSideLeftOwn    = this->get_parameter("angleSideLeftOwn").as_int();
        angleSideRightOpp   = this->get_parameter("angleSideRightOpp").as_int();
        angleSideRightOwn   = this->get_parameter("angleSideRightOwn").as_int();
    }

    void sendRobotCoordinationData(signed short rNumber, signed short rStatus, signed short sNumber, signed short xPos, signed short yPos, signed short fBall, signed short dBall, signed short xBall, signed short yBall, signed short backIn)
    {
        auto msg = bfc_msgs::msg::Coordination();
        msg.robot_number = rNumber;
        msg.status = rStatus;
        msg.state = sNumber;
        msg.x_position = xPos;
        msg.y_position = yPos;
        msg.found_ball = fBall;
        msg.distance_ball = dBall;
        msg.x_ball = xBall;
        msg.y_ball = yBall;
        msg.back_in = backIn;
        // publish executor and defend lock for team coordination
        msg.executor = executor;
        msg.defend_lock = defend_lock;
        robotCoordination_->publish(msg);
    }

    void readRobotCoordinationData1(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        // printf("..koordinasi r 1 masuk\n");
        // resetCommunication();
        robot1Id = message->robot_number;
        robot1Status = message->status;
        robot1State = message->state;
        robot1XPosition = message->x_position;
        robot1YPosition = message->y_position;
        robot1FBall = message->found_ball;
        robot1DBall = message->distance_ball;
        robot1XBall = message->x_ball;
        robot1YBall = message->y_ball;
        robot1BackIn = message->back_in;
        robot1KickOff = message->kick_off;
        robot1_executor = message->executor;

        if (robot1DBall == 232)	//0
        {
            robot1DBall = 999;
        }
    }

    void readRobotCoordinationData2(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        // printf("..koordinasi r 2 masuk\n");
        // resetCommunication();
        robot2Id = message->robot_number;
        robot2Status = message->status;
        robot2State = message->state;
        robot2XPosition = message->x_position;
        robot2YPosition = message->y_position;
        robot2FBall = message->found_ball;
        robot2DBall = message->distance_ball;
        robot2XBall = message->x_ball;
        robot2YBall = message->y_ball;
        robot2BackIn = message->back_in;
        robot2KickOff = message->kick_off;
        robot2_executor = message->executor;

        if (robot2DBall == 232)	//0
        {
            robot2DBall = 999;
        }
    }

    void readRobotCoordinationData3(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        // printf("..koordinasi r 3 masuk\n");
        // resetCommunication();
        robot3Id = message->robot_number;
        robot3Status = message->status;
        robot3State = message->state;
        robot3XPosition = message->x_position;
        robot3YPosition = message->y_position;
        robot3FBall = message->found_ball;
        robot3DBall = message->distance_ball;
        robot3XBall = message->x_ball;
        robot3YBall = message->y_ball;
        robot3BackIn = message->back_in;
        robot3KickOff = message->kick_off;
        robot3_executor = message->executor;
    
        if (robot3DBall == 232)	//0
        {
            robot3DBall = 999;
        }
    }

    void readRobotCoordinationData4(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        // printf("..koordinasi r 4 masuk\n");
        // resetCommunication();
        robot4Id = message->robot_number;
        robot4Status = message->status;
        robot4State = message->state;
        robot4XPosition = message->x_position;
        robot4YPosition = message->y_position;
        robot4FBall = message->found_ball;
        robot4DBall = message->distance_ball;
        robot4XBall = message->x_ball;
        robot4YBall = message->y_ball;
        robot4BackIn = message->back_in;
        robot4KickOff = message->kick_off;
        robot4_executor = message->executor;
    
        if (robot4DBall == 232)	//0
        {
            robot4DBall = 999;
        }
    }

    void readRobotCoordinationData5(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        // printf("..koordinasi r 5 masuk\n");
        // resetCommunication();
        robot5Id = message->robot_number;
        robot5Status = message->status;
        robot5State = message->state;
        robot5XPosition = message->x_position;
        robot5YPosition = message->y_position;
        robot5FBall = message->found_ball;
        robot5DBall = message->distance_ball;
        robot5XBall = message->x_ball;
        robot5YBall = message->y_ball;
        robot5BackIn = message->back_in;
        robot5KickOff = message->kick_off;
        robot5_executor = message->executor;
    
        if (robot5DBall == 232)	//0
        {
            robot5DBall = 999;
        }    
    }

    void readRobotCoordinationData6(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        // printf("..koordinasi r 6 masuk\n");
        // resetCommunication();
        robot6Id = message->robot_number;
        robot6Status = message->status;
        robot6State = message->state;
        robot6XPosition = message->x_position;
        robot6YPosition = message->y_position;
        robot6FBall = message->found_ball;
        robot6DBall = message->distance_ball;
        robot6XBall = message->x_ball;
        robot6YBall = message->y_ball;
        robot6BackIn = message->back_in;
        robot6KickOff = message->kick_off;
        robot6_executor = message->executor;
    
        if (robot6DBall == 232)	//0
        {
            robot6DBall = 999;
        }    
    }

    void readRobotCoordinationData7(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        // printf("..koordinasi r 7 masuk\n");
        // resetCommunication();
        robot7Id = message->robot_number;
        robot7Status = message->status;
        robot7State = message->state;
        robot7XPosition = message->x_position;
        robot7YPosition = message->y_position;
        robot7FBall = message->found_ball;
        robot7DBall = message->distance_ball;
        robot7XBall = message->x_ball;
        robot7YBall = message->y_ball;
        robot7BackIn = message->back_in;
        robot7KickOff = message->kick_off;
        robot7_executor = message->executor;
    
        if (robot7DBall == 232)	//0
        {
            robot7DBall = 999;
        }    
    }

    int voltage = 0, walkActive = 0, supportLeg = 0, lastSupportLeg = 0, kneeCurr = 0, stabilize_state = 0, cnt_stab = 0, cnt_stab2 = 0;
    int vx = 0, vy = 0, va = 0, sumWalkX = 0;
    double robotWalkX = 0.0, robotWalkY = 0.0, robotWalkA = 0.0;
    void readVoltageAndOdom(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        voltage = msg->data[0];
        walkActive = msg->data[1];
        supportLeg = msg->data[2];
        kneeCurr = msg->data[3];
        vx = msg->data[4];
        vy = msg->data[5];
        va = msg->data[6];

        robotWalkX = (double)vx / 1000;
        robotWalkY = (double)vy / 1000;
        robotWalkA = (double)va / 1000;

        if (walkActive == 1)
        //if (walkActive == 1 && !robotFall)
        {
            if (supportLeg != lastSupportLeg)
            {
                // new_mapping(robotWalkX, robotWalkY, msg_yaw);
                mapping(robotWalkX, robotWalkY);
                //mapping(robotWalkX, 0.0);
                sumWalkX += 1;
                lastSupportLeg = supportLeg;
            }
        }
        // printf("knee current = %d\n", kneeCurr);
        if (kneeCurr >= max_current)
        {
            cnt_stab = 0;
            cnt_stab2 = 50;
            stabilize_state = 1;
        }

        kneeCurr = 0;
    }

    int gridUpdate = 0;
    void readGrid(const std_msgs::msg::Int32::SharedPtr msg)
    {
        gridUpdate = msg->data;
        printf("...GridUpdate : %d\n", gridUpdate);
        robotPos_X = convertGridX(gridUpdate, 0);
        robotPos_Y = convertGridY(gridUpdate, 0);
    }

    int object_count = 0;

    /*void callbackFoundObject(const darknet_ros_msgs::msg::ObjectCount::SharedPtr msg)
    {
        object_count = msg->count;
        // printf("cnt = %d\n", object_count);
    }*/
    /*int x_min, x_max, y_min, y_max;
    std::string class_id;
    float b_pole_1_x, b_pole_1_y, b_pole_2_x, b_pole_2_y, cB_pole_X, cB_pole_Y;
    float t_pole_1_x, t_pole_1_y, t_pole_2_x, t_pole_2_y, cT_pole_X, cT_pole_Y;
    void callbackBoundingBox(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
    {
        std::vector<float> x_values_b, y_values_b, x_values_t, y_values_t;
        std::vector<int> id_classes_b, id_classes_t;

        for (const auto& bbox : msg->bounding_boxes)
        {
            const auto& id_class = bbox.id;
            const auto x_center = (bbox.xmin + bbox.xmax) / 2;
            const auto y_center = (bbox.ymin + bbox.ymax) / 2;

            switch (id_class)
            {
                case 0: // ball
                    Ball_X = x_center;
                    Ball_Y = y_center;
                    break;
                case 2: // b pole
                    x_values_b.push_back(x_center);
                    y_values_b.push_back(y_center);
                    id_classes_b.push_back(id_class);
                    break;
                case 2: // robot
                    Robot_X = x_center;
                    Robot_Y = y_center;
                    break;
                case 7: // pinalty
                    Pinalty_X = x_center;
                    Pinalty_Y = y_center;
                    break;
                case 5: // x_cross
                    Xcross_X = x_center;
                    Xcross_Y = y_center;
                    break;
                case 1: // t_pole
                    x_values_t.push_back(x_center);
                    y_values_t.push_back(y_center);
                    id_classes_t.push_back(id_class);
                    break;
                case 6: // goal
                    Goal_X = x_center;
                    Goal_Y = y_center;
                    break;
                case 3: // GA_L_Cross
                    GA_L_Cross_X = x_center;
                    GA_L_Cross_Y = y_center;
                    break;
                case 4: // PA_L_Cross
                    PA_L_Cross_X = x_center;
                    PA_L_Cross_Y = y_center;
                    break;
                default:
                    break; // ignore any other classes
            }
        }

        if (id_classes_b.size() == 2)
        {
            //RCLCPP_INFO(rclcpp::get_logger("B_pole detection"), "All 2 B_poles detected in one frame:");
            for (size_t i = 0; i < 2; ++i)
            {
                B_pole_X = x_values_b[i];
                B_pole_Y = y_values_b[i];
                if (id_classes_b[i] == 1)  // Check if the ID class is for a B_pole
                {
                    if (i == 0)
                    {
                        b_pole_1_x = x_values_b[i];
                        b_pole_1_y = y_values_b[i];
                    }
                    else
                    {
                        b_pole_2_x = x_values_b[i];
                        b_pole_2_y = y_values_b[i];
                    }

                    // RCLCPP_INFO(rclcpp::get_logger("bounding_box_detection"), "Detected B_poles: (%f, %f) and (%f, %f)", b_pole_1_x, b_pole_1_y, b_pole_2_x, b_pole_2_y);
                }

                // RCLCPP_INFO(rclcpp::get_logger("B_pole detection"), "B_pole %zu: (x, y) = (%f, %f)", i + 1, x_values_b[i], y_values_b[i]);
            }
        }

        if (id_classes_t.size() == 2)
        {
            //RCLCPP_INFO(rclcpp::get_logger("T_pole detection"), "All 2 T_poles detected in one frame:");
            for (size_t i = 0; i < 2; ++i)
            {
                T_pole_X = x_values_t[i];
                T_pole_Y = y_values_t[i];

                if (id_classes_t[i] == 5)  // Check if the ID class is for a T_pole
                {
                    if (i == 0)
                    {
                        t_pole_1_x = x_values_t[i];
                        t_pole_1_y = y_values_t[i];
                    }
                    else
                    {
                        t_pole_2_x = x_values_t[i];
                        t_pole_2_y = y_values_t[i];
                    }

                    // RCLCPP_INFO(rclcpp::get_logger("bounding_box_detection"), "Detected T_poles: (%f, %f) and (%f, %f)", t_pole_1_x, t_pole_1_y, t_pole_2_x, t_pole_2_y);
                }

                // RCLCPP_INFO(rclcpp::get_logger("T_pole detection"), "T_pole %zu: (x, y) = (%f, %f)", i + 1, x_values_t[i], y_values_t[i]);
            }
        }

        //RCLCPP_INFO(rclcpp::get_logger("bounding_box_detection"), "Detected B_poles: (%f, %f) and (%f, %f)", b_pole_1_x, b_pole_1_y, b_pole_2_x, b_pole_2_y);
        //RCLCPP_INFO(rclcpp::get_logger("bounding_box_detection"), "Detected T_poles: (%f, %f) and (%f, %f)", t_pole_1_x, t_pole_1_y, t_pole_2_x, t_pole_2_y);


        cB_pole_X = (b_pole_1_x + b_pole_2_x) / 2;
        cB_pole_Y = (b_pole_1_y + b_pole_2_y) / 2;       
        cT_pole_X = (t_pole_1_x + t_pole_2_x) / 2;
        cT_pole_Y = (t_pole_1_y + t_pole_2_y) / 2;

        //Goal_X = (cB_pole_X + cT_pole_X) / 2;
        //Goal_Y = (cB_pole_Y + cT_pole_Y) / 2;
    }*/
    
    int x_min, x_max, y_min, y_max;
    std::string class_id;
    float b_pole_1_x, b_pole_1_y, b_pole_2_x, b_pole_2_y, cB_pole_X, cB_pole_Y;
    float t_pole_1_x, t_pole_1_y, t_pole_2_x, t_pole_2_y, cT_pole_X, cT_pole_Y;
    /*void callbackBoundingBox(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
    {
        std::vector<float> x_values_b, y_values_b, x_values_t, y_values_t;
        std::vector<int> id_classes_b, id_classes_t;

        for (const auto& bbox : msg->bounding_boxes)
        {
            const auto& id_class = bbox.id;
            const auto x_center = (bbox.xmin + bbox.xmax) / 2;
            const auto y_center = (bbox.ymin + bbox.ymax) / 2;

            switch (id_class)
            {
                case 0:
                    Ball_X = x_center;
                    Ball_Y = y_center;
                    break;
                case 1:
                    x_values_t.push_back(x_center);
                    y_values_t.push_back(y_center);
                    id_classes_t.push_back(id_class);
                    
                    break;
                case 2:
                    x_values_b.push_back(x_center);
                    y_values_b.push_back(y_center);
                    id_classes_b.push_back(id_class);
                    //Robot_X = x_center;
                    //Robot_Y = y_center;
                    break;
                case 3:
                    //Pinalty_X = x_center;
                    //Pinalty_Y = y_center;
                    break;
                case 4:
                    
                    break;
                case 5:
                    Xcross_X = x_center;
                    Xcross_Y = y_center;
                    //x_values_t.push_back(x_center);
                    //y_values_t.push_back(y_center);
                    //id_classes_t.push_back(id_class);
                    break;
                case 6:
                    Goal_X = x_center;
                    Goal_Y = y_center;
                    break;
                case 7:
                    // Ball_X = x_center;
                    // Ball_Y = y_center;
                    break;
                case 8:
                    //Robot_X = x_center;
                    //Robot_Y = y_center;
                    break;
                case 9:
                    // goal_L_pole_X = x_center;
                    // goal_L_pole_Y = y_center;
                    break;
                case 10:
                    // goal_R_pole_X = x_center;
                    // goal_R_pole_Y = y_center;
                    break;
                default:
                    break; // ignore any other classes
            }
        }
        if (id_classes_b.size() == 2)
        {
            //RCLCPP_INFO(rclcpp::get_logger("B_pole detection"), "All 2 B_poles detected in one frame:");
            for (size_t i = 0; i < 2; ++i)
            {
                B_pole_X = x_values_b[i];
                B_pole_Y = y_values_b[i];
                if (id_classes_b[i] == 1)  // Check if the ID class is for a B_pole
                {
                    if (i == 0)
                    {
                        b_pole_1_x = x_values_b[i];
                        b_pole_1_y = y_values_b[i];
                    }
                    else
                    {
                        b_pole_2_x = x_values_b[i];
                        b_pole_2_y = y_values_b[i];
                    }

                    // RCLCPP_INFO(rclcpp::get_logger("bounding_box_detection"), "Detected B_poles: (%f, %f) and (%f, %f)", b_pole_1_x, b_pole_1_y, b_pole_2_x, b_pole_2_y);
                }

                // RCLCPP_INFO(rclcpp::get_logger("B_pole detection"), "B_pole %zu: (x, y) = (%f, %f)", i + 1, x_values_b[i], y_values_b[i]);
            }
        }

        if (id_classes_t.size() == 2)
        {
            //RCLCPP_INFO(rclcpp::get_logger("T_pole detection"), "All 2 T_poles detected in one frame:");
            for (size_t i = 0; i < 2; ++i)
            {
                T_pole_X = x_values_t[i];
                T_pole_Y = y_values_t[i];

                if (id_classes_t[i] == 5)  // Check if the ID class is for a T_pole
                {
                    if (i == 0)
                    {
                        t_pole_1_x = x_values_t[i];
                        t_pole_1_y = y_values_t[i];
                    }
                    else
                    {
                        t_pole_2_x = x_values_t[i];
                        t_pole_2_y = y_values_t[i];
                    }

                    // RCLCPP_INFO(rclcpp::get_logger("bounding_box_detection"), "Detected T_poles: (%f, %f) and (%f, %f)", t_pole_1_x, t_pole_1_y, t_pole_2_x, t_pole_2_y);
                }

                // RCLCPP_INFO(rclcpp::get_logger("T_pole detection"), "T_pole %zu: (x, y) = (%f, %f)", i + 1, x_values_t[i], y_values_t[i]);
            }
        }

        //RCLCPP_INFO(rclcpp::get_logger("bounding_box_detection"), "Detected B_poles: (%f, %f) and (%f, %f)", b_pole_1_x, b_pole_1_y, b_pole_2_x, b_pole_2_y);
        //RCLCPP_INFO(rclcpp::get_logger("bounding_box_detection"), "Detected T_poles: (%f, %f) and (%f, %f)", t_pole_1_x, t_pole_1_y, t_pole_2_x, t_pole_2_y);


        cB_pole_X = (b_pole_1_x + b_pole_2_x) / 2;
        cB_pole_Y = (b_pole_1_y + b_pole_2_y) / 2;       
        cT_pole_X = (t_pole_1_x + t_pole_2_x) / 2;
        cT_pole_Y = (t_pole_1_y + t_pole_2_y) / 2;

        //Goal_X = (cB_pole_X + cT_pole_X) / 2;
        //Goal_Y = (cB_pole_Y + cT_pole_Y) / 2;
    }*/
    
    /*void callbackMidpoints(const yolo_msgs::msg::Midpoints::SharedPtr msg) {
    if (!msg) {
        //RCLCPP_INFO(this->get_logger(), "Pesan kosong diterima!");
        return;
    }

    if (msg->midpoints.empty()) {
        RCLCPP_WARN(this->get_logger(), "Tidak ada objek terdeteksi");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Menerima %ld deteksi:", msg->midpoints.size());
    
    for (const auto& mp : msg->midpoints) {
        if (mp.midpoint_x < 0 || mp.midpoint_y < 0) {
            RCLCPP_WARN(this->get_logger(), "Koordinat invalid untuk class %d", mp.class_id);
            continue;
        }

        RCLCPP_INFO(this->get_logger(), 
            "Class ID: %d | Koordinat: (%d, %d)", 
            mp.class_id, mp.midpoint_x, mp.midpoint_y);
        
        auto id_class = mp.class_id;  
        if (id_class == "ball") {
            RCLCPP_DEBUG(this->get_logger(), "Memproses bola pada (%d,%d)", 
                mp.midpoint_x, mp.midpoint_y);
            Ball_X = mp.midpoint_x;
            Ball_Y = mp.midpoint_y;
        } else if (id_class == "field") {
            RCLCPP_DEBUG(this->get_logger(), "Memproses lapangan pada (%d,%d)", mp.midpoint_x, mp.midpoint_y);
        }
    }
}*/

    double targetPoseX = 0.0, targetPoseY = 0.0;
    void callbackTargetPose(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        targetPoseX = msg->pose.pose.position.x;
        targetPoseY = msg->pose.pose.position.y;
    }

    double landmarkDistance = 0;
    void callbackObjectDistance(const std_msgs::msg::Float32::SharedPtr msg)
    {
        landmarkDistance = msg->data;
    }
    void gkBodyTrackingBall()
    {
        //trackBall();
        if (posPan < 0.05 && posPan > -0.05)
        { // Stop(bola sudah dekat)
            countReadyKick++;
        }
        else
        { // Kejar Bola(bola masih jauh)
            countReadyKick = 0;
        }

        if (countReadyKick >= 1)
        {                              // 5
            PxMove = 0.0;              // jalan ditempat
            PyMove = errorfPan * 0.05; // 0.045
            PaMove = errorfPan * 0.20; // 0.30; //0.045
        }
        else
        {
            errorfPan = posPan - SetPointPan;
            PyMove = errorfPan * 0.20; // 0.125; //0.045
            PaMove = errorfPan * 0.30; // 0.25; //0.35; //0.045
        }

        printf("errorfPan = %f\n", errorfPan);
        printf("Move = %f, %f\n", PyMove, PaMove);

        if (errorfPan > -0.1 && errorfPan < 0.1)
        { // printf("AAAAAAAA\n");
            Walk(0.0, 0.0, PaMove);
        }
        else
        { // printf("BBBBBBBB\n");
            if (PyMove > 0)
                //Walk(0.0, PyMove, 0.015);
                //Walk(0.0, PyMove, 0.0);
                jalanDirection(0.0, PyMove, 0.0);
            else
               //Walk(0.0, PyMove, -0.015);
               //Walk(0.0, PyMove, 0.0);
               jalanDirection(0.0, PyMove, 0.0);
        }
    }
    int ballDistance = 0;
    float jarak_bola = 0.0;
    void callbackBallDistance(const std_msgs::msg::Float32::SharedPtr msg)
    {
        ballDistance = (int)msg->data;
        jarak_bola = msg->data;
        if (ballDistance == -1)
        {
            ballDistance = 999;
        }
    }

    vector<int> grid_list;
    void callbackPathFinding(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        size_t numElements = msg->data.size();
        for (int i = 0; i < numElements; i++)
        {
            grid_list.push_back(msg->data[i]);
        } 
        // print the contents of the vector
        cout << "grid_list : "  ;
        for (int i = 0; i < grid_list.size(); i++) {
            cout << grid_list[i] << " ";
        }
        cout << endl;
        cout << "size : " << grid_list.size() << endl;
    }

    double dataPanKey = 0, dataTiltKey = 0;
    bool triggerSave = false;
    void callbackTeleop(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (msg->linear.x > 0)
        {
            dataTiltKey -= 0.2;
        } else if (msg->linear.x < 0)
        {
            dataTiltKey += 0.2;
        }else if (msg->angular.z > 0)
        {
            dataPanKey += 0.2;
        } else if (msg->angular.z < 0)
        {
            dataPanKey -= 0.2;
        } else if (msg->angular.z == 0)
        {
            triggerSave = true;
        } else 
        {
            triggerSave = false;
        }
    }

    double ball_pose_x = 0, ball_pose_y = 0; 
    int robotStatus = 0, foundBall = 0, BallX = 999, BallY = 999;
    void callbackBallPose(const nav_msgs::msg::Odometry::SharedPtr msg_pose)
    {
        if (msg_pose->pose.pose.position.x == -1 && msg_pose->pose.pose.position.y == -1)
        {
            BallX = 999, BallY = 999;
            ball_pose_x = -1;
            ball_pose_y = -1;
        } else 
        {
            ball_pose_x = msg_pose->pose.pose.position.x - 450;
            ball_pose_y = (msg_pose->pose.pose.position.y - 300) * -1;
            BallX = ball_pose_x;
            BallY = ball_pose_y;
        }
    }

    double cam_x = 999.0, cam_y = 999.0;
    void callbackCameraOdom(const nav_msgs::msg::Odometry::SharedPtr msg_pose)
    {
        if (msg_pose->pose.pose.position.x != -1 && msg_pose->pose.pose.position.y != -1)
        {
            cam_x = msg_pose->pose.pose.position.x;
            cam_y = msg_pose->pose.pose.position.y;
        } else 
        {
            cam_x = cam_y = 999;
        }
    }

    // Function To Set Timer =====================================================================
    struct timeval t1,
        t2;
    int attends;
    double elapsedTime,
        second;
    bool timer = false;
    void setWaktu()
    {
        elapsedTime =
            second =
                attends = 0;
        timer = false;

        gettimeofday(&t1, NULL);
    }
    // Function For Check Timer
    void cekWaktu(double detik)
    {
        if (attends > 10)
        {
            gettimeofday(&t2, NULL);

            // compute and print the elapsed time in millisec
            elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
            elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
            second = elapsedTime / 1000.0;
            //printf ("  waktu berlangsung = %.f detik \n\n\n\n", second);

            if (second >= detik)
            {
                timer = true;
            }
            else
            {
                timer = false;
            }
        }
        else
        {
            attends++;
        }
    }

    // Checking Lost Ball ========================================================================
    int countBallLost = 0,
        countBallFound = 0,
        returnBallVal;
    int ballLost(int threshold)
    {
        if (useVision)
        {
            if (Ball_X == -1 && Ball_Y == -1)
            {
                countBallFound = 0;
                countBallLost++;
                if (countBallLost >= threshold)
                {
                    returnBallVal = 1;
                }
            }
            else
            {
                countBallLost = 0;
                countBallFound++;
                if (countBallFound > 1)
                {
                    returnBallVal = 0;
                }
            }
        }
        else
        {
            countBallFound = 0;
            countBallLost++;
            if (countBallLost >= threshold)
            {
                returnBallVal = 1;
            }
        }
        return returnBallVal;
    }

    // Checking Lost Pen ========================================================================
    int countPenLost = 0,
        countPenFound = 0,
        returnPenVal;
    int penLost(int threshold)
    {
        if (useVision)
        {
            if (Pinalty_X == -1 && Pinalty_Y == -1)
            {
                countPenFound = 0;
                countPenLost++;
                if (countPenLost >= threshold)
                {
                    returnPenVal = 1;
                }
            }
            else
            {
                countPenLost = 0;
                countPenFound++;
                if (countPenFound > 1)
                {
                    returnPenVal = 0;
                }
            }
        }
        else
        {
            countPenFound = 0;
            countPenLost++;
            if (countPenLost >= threshold)
            {
                returnPenVal = 1;
            }
        }
        return returnPenVal;
    }

    // Search ball ===============================================================================
    double tiltRate = -0.03,
           panRate = -0.03,
           searchKe = 0,

           batasKanan = -1.6,
           batasKiri = 1.6,
           batasAtas = -2.0,
           batasBawah = -0.6;

    void tiltSearchBall(double tempPosPan)
    { // printf("  tiltSearchBall\n\n");
        posPan = tempPosPan;
        posTilt += tiltRate;

        if (posTilt <= batasAtas || posTilt >= batasBawah)
        {
            tiltRate *= -1;
        }

        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
            searchKe++;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    void tiltPredictSearchBall(double tempPosPan)
    { // printf("  tiltSearchBall\n\n");
        posTilt += tiltRate;

        if (posTilt <= batasAtas || posTilt >= -1)
        {
            tiltRate *= -1;
        }

        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= -1)
        {
            posTilt = -1;
        }

        posPan = tempPosPan / 57.29; // sudut * nilai per satu sudut(nilai servo)

        if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    double cnt_pan_search_ball = 0;
    void panSearchBall(double tempPosTilt)
    { // printf("  panSearchBall\n\n");
        posTilt = tempPosTilt;
        posPan += panRate;

        if (posPan <= batasKanan || posPan >= batasKiri)
        {
            panRate *= -1;
        }

        if (headPan <= (0.0 + panRate) && headPan >= (0.0 - panRate))
        {
            cnt_pan_search_ball += 0.5;
        }

        if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }
    
    void panSearchBallWithInit(double initPan, double tempPosTilt)
    { // printf("  panSearchBall\n\n");
        posTilt = tempPosTilt;
        posPan = initPan;
        
        if(initPan < 0){
        posPan += panRate;
        }
        else{
        posPan -= panRate;
        }

        if (posPan <= batasKanan || posPan >= batasKiri)
        {
            panRate *= -1;
        }

        if (headPan <= (0.0 + panRate) && headPan >= (0.0 - panRate))
        {
            cnt_pan_search_ball += 0.5;
        }

        if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    int i = 1,
        panKe = 2,
        tiltKe = 1;
    double panSearch[5] = {1.6, 0.8, 0.0, -0.8, -1.6},
           // tiltSearch1[3] = {-0.6, -1.2, -1.8},
        tiltSearch1[3] = {-0.8, -1.4, -1.8},
           tiltSearch2[2] = {-0.8, -1.4};
    void SearchBall(int mode)
    {
        if (mode == 1)
        { // (atas-bawah)
            posTilt += tiltRate;
            if (posTilt <= batasAtas || posTilt >= batasBawah)
            {
                if (panKe == 2)
                {
                    searchKe += 1;
                }

                tiltRate *= -1;
                panKe += i;
                if (panKe >= 4 || panKe <= 0)
                {
                    i = -i;
                }
            }
            posPan = panSearch[panKe];
            //printf("count pan = %d\n", panKe);
        }
        else if (mode == 2)
        { // (kiri-kanan)
            posPan += panRate;
            if (posPan <= batasKanan || posPan >= batasKiri)
            {
                if (tiltKe == 1)
                {
                    searchKe += 1;
                }
                panRate *= -1;
                tiltKe += i;

                if (tiltKe >= 2 || tiltKe <= 0)
                {
                    i = -i;
                }
            }
            posTilt = tiltSearch1[tiltKe]; // printf("count tilt = %d\n", tiltKe);
        }
        else if (mode == 3)
        { // muter-muter
            posPan += panRate;
            if (posPan <= batasKanan || posPan >= batasKiri)
            {
                panRate *= -1;
                countTilt++;
                if (countTilt > 1)
                    countTilt = 0;
            }
            posTilt = tiltSearch2[countTilt]; // printf("count tilt = %d\n", countTilt);
        }

        if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }
        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    int sabar = 0,
        tiltPos = 0;
    void threeSearchBall()
    {
        if (sabar > 7)
        {
            posPan += panRate;
            if (posPan <= batasKanan || posPan >= batasKiri)
            {
                if (tiltPos == 2 && posPan <= -1.5)
                {
                    searchKe += 1;
                }
                panRate *= -1;
                tiltPos += i;
                if (tiltPos >= 2 || tiltPos <= 0)
                {
                    i = -i;
                }
            }
            //posTilt = tiltSearch1[tiltPos]; // printf("count tilt = %d\n", tiltPos);
            double targetTilt = tiltSearch1[tiltPos];
            double tiltStep = 0.03; // kecepatan pergerakan tilt (halus)
            
            if (posTilt > targetTilt + tiltStep) {
                posTilt -= tiltStep; // turun pelan ke target
            } else if (posTilt < targetTilt - tiltStep) {
                posTilt += tiltStep; // naik pelan ke target
            } else {
                posTilt = targetTilt; // sudah dekat, snap ke target
            }
        }
        else
        {
            posPan = 1.45;
            posTilt = -0.8;
            tiltPos = 0;
            searchKe = 0;
            i = 1;
            tiltRate = -0.05;
            panRate = -0.05;
            sabar++;
        }

        if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }
        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    void rightSearchBall()
    { // printf("  rightSearchBall\n\n");
        posTilt += tiltRate;
        if (posTilt <= batasAtas || posTilt >= batasBawah)
        {
            tiltRate *= -1;
            posPan += panRate;
            if (posPan >= 0.0 || posPan <= batasKanan)
            {
                panRate *= -1;
            }
        }

        if (posPan >= 0.0)
        {
            posPan = 0.0;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }
        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt);
    }

    void leftSearchBall()
    { // printf("  leftSearchBall\n\n");
        posTilt += tiltRate;
        if (posTilt <= batasAtas || posTilt >= batasBawah)
        {
            tiltRate *= -1;
            posPan += panRate;
            if (posPan >= batasKiri || posPan <= 0.0)
            {
                panRate *= -1;
            }
        }

        if (posPan <= 0.0)
        {
            posPan = 0.0;
        }
        else if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt);
    }

    double invPan;
    void searchBallPan(double uPan, double uTilt)
    {
        posTilt = uTilt;
        invPan = uPan * -1;

        posPan += panRate;

        if (posPan <= invPan)
        {
            posPan = invPan;
            panRate *= -1;
        }
        else if (posPan >= uPan)
        {
            posPan = uPan;
            panRate *= -1;
        }

        headMove(posPan, posTilt);
    }

    bool neckX;
    double cnt_sbr = 0;
    void searchBallRectang(double atas, double kanan, double bawah, double kiri)
    {
        if (neckX)
        {
            posPan += panRate;
            if (posPan >= kiri || posPan <= kanan)
            {
                panRate *= -1;
                cnt_sbr += 0.5;
                neckX = false;
            }
        }
        else
        {
            posTilt += tiltRate;
            if (posTilt <= atas || posTilt >= bawah)
            {
                tiltRate *= -1;
                cnt_sbr += 0.5;
                neckX = true;
            }
        }

        if (posPan >= kiri)
        {
            posPan = kiri;
        }
        else if (posPan <= kanan)
        {
            posPan = kanan;
        }
        if (posTilt <= atas)
        {
            posTilt = atas;
        }
        else if (posTilt >= bawah)
        {
            posTilt = bawah;
        }

        headMove(posPan, posTilt); // printf("pan = %f, tilt = %f\n",posPan,posTilt);
    }

    double ballPan = 0,
           ballTilt = 0;
    void saveBallLocation()
    {
        //	trackBall();
        ballPan = posPan;
        ballTilt = posTilt;
    }

    void loadBallLocation(double tilt)
    {
        // posPan = 0.0;
        // posTilt = -1;
        posPan = ballPan;
        posTilt = ballTilt + tilt;
        headMove(posPan, posTilt);
    }

    int panDegree, tiltRad = 0;
    double panRad = 0;
    void savePan()
    {
        panRad = headPan;
        tiltRad = posTilt;
        panDegree = (posPan * (180 / PI)) + msg_yaw;
        panRad = panRad * -1;
        panDegree = panDegree * -1;
    }

    double koorRobotX,
        koorRobotY = 0;
    void saveKoordinatRobot()
    {
        koorRobotX = deltaPos_X;
        koorRobotY = deltaPos_Y;
    }

    void loadKoordinatRobot()
    {
        // resetOdometry();
        // deltaPos_X = koorRobotX;
        // deltaPos_Y = koorRobotY;
        initialPos_X = koorRobotX;
        initialPos_Y = koorRobotY;
    }

    int semeh = 0;
    int koordinasiJarak()
    {
        semeh = (int)(headTilt*-100);
        return semeh;
    }

    // Ball Tracking =============================================================================
    double intPanB = 0, dervPanB = 0, errorPanB = 0, preErrPanB = 0,
           PPanB = 0, IPanB = 0, DPanB = 0,
           intTiltB = 0, dervTiltB = 0, errorTiltB = 0, preErrTiltB = 0,
           PTiltB = 0, ITiltB = 0, DTiltB = 0,
           dtB = 0.04;
    double B_Pan_err_diff, B_Pan_err, B_Tilt_err_diff, B_Tilt_err, B_PanAngle, B_TiltAngle,
        pOffsetB, iOffsetB, dOffsetB,
        errorPanBRad, errorTiltBRad,
        offsetSetPointBall, lastErrorPanB, lastErrorTiltB, pidPan, pidTilt, lastPidPan, lastPidTilt;
    void trackBall()
    {
        if (useVision)
        {
            if (Ball_X != -1 && Ball_Y != -1)
            { // printf("Tracking");
                // mode 1 ######################################################################
                /*// PID pan ==========================================================
                errorPanB  = (double)Ball_x - (frame_X / 2);//160
                PPanB  = errorPanB  * 0.00010; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya

                intPanB += errorPanB * dtB;
                IPanB = intPanB * 0.0;

                dervPanB = (errorPanB - preErrPanB) / dtB;
                DPanB = dervPanB * 0.00001;

                preErrPanB = errorPanB;

                //posPan += PPanB*-1; //dikali -1 kalau receler terbalik dalam pemasangan
                posPan += (PPanB + IPanB + DPanB) * -1;


                // PID tilt ==========================================================
                errorTiltB = (double)Ball_y - (frame_Y / 2);//120
                PTiltB = errorTiltB * 0.00010; // Tune in Kp Tilt 0.00030

                intTiltB += errorTiltB * dtB;
                ITiltB = intTiltB * 0.0;

                dervTiltB = (errorTiltB - preErrTiltB) / dtB;
                DTiltB = dervTiltB * 0.00001;

                preErrTiltB = errorTiltB;

                //posTilt += PTiltB;
                posTilt += (PTiltB + ITiltB + DTiltB);*/

                // mode 2 ######################################################################
                //  offsetSetPointBall = ((int)(posTilt * 30)+7); //+54
                //  if (offsetSetPointBall > 36) offsetSetPointBall = 36;
                //  else if (offsetSetPointBall < 0) offsetSetPointBall = 0;

                // errorPanB  = (double)Ball_X - ((frame_X / 2) + offsetSetPointBall);//160
                errorPanB = (double)Ball_X - (frame_X / 2);
                errorTiltB = (double)Ball_Y - (frame_Y / 2); // 120
                errorPanB *= -1;
                errorTiltB *= -1;
                errorPanB *= (104.6 / (double)frame_X);  // pixel per angle
                errorTiltB *= (61.6 / (double)frame_Y); // pixel per angle
                // errorPanB *= (77.32 / (double)frame_X); // pixel per angle
                // errorTiltB *= (61.93 / (double)frame_Y); // pixel per angle

                errorPanBRad = (errorPanB * PI) / 180;
                errorTiltBRad = (errorTiltB * PI) / 180;
                // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanB, errorTiltB);
                // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanBRad, errorTiltBRad);
                // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

                B_Pan_err_diff = errorPanBRad - B_Pan_err;
                B_Tilt_err_diff = errorTiltBRad - B_Tilt_err;

                // PID pan ==========================================================
                // PPanB  = B_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                PPanB = B_Pan_err * ball_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                intPanB += B_Pan_err * dtB;
                IPanB = intPanB * 0.0;
                dervPanB = B_Pan_err_diff / dtB;
                // DPanB = dervPanB * kamera.panKD;
                DPanB = dervPanB * ball_panKD;
                B_Pan_err = errorPanBRad;
                pidPan = (PPanB + IPanB + DPanB);
                if (pidPan > 0.3)
                {
                	pidPan = 0.3;
                } else if (pidPan < -0.3)
                {
                	pidPan = -0.3;
                }
                
                if (pidPan != lastPidPan)
                {
                	posPan += pidPan;
                	lastPidPan = pidPan;
                }
                //posPan += pidPan;

                // PID tilt ==========================================================
                // PTiltB = B_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
                PTiltB = B_Tilt_err * ball_tiltKP; // Tune in Kp Tilt 0.00030

                intTiltB += B_Tilt_err * dtB;
                ITiltB = intTiltB * 0.0;

                dervTiltB = B_Tilt_err_diff / dtB;
                // DTiltB = dervTiltB * kamera.tiltKD;
                DTiltB = dervTiltB * ball_tiltKD;

                preErrTiltB = errorTiltB;
                B_Tilt_err = errorTiltBRad;
                pidTilt = (PTiltB + ITiltB + DTiltB) * -1;
					
				if (pidTilt > 0.3)
                {
                	pidTilt = 0.3;
                } else if (pidTilt < -0.3)
                {
                	pidTilt = -0.3;
                }
                if (pidTilt != lastPidTilt)
                {
                	posTilt += pidTilt;
                	lastPidTilt = pidTilt;
                }
                //posTilt += pidTilt;

                /*if (posTilt >= -0.9)
                {
                    if (posPan >= 0.4)
                    {
                    	posPan = 0.4;
                    }
                    else if (posPan <= -0.4)
                    {
                    	posPan = -0.4;
                    }
                }*/ //Trackball Robocup2024
                if (posPan >= 1.6)
                {
                    posPan = 1.6;
                }
                else if (posPan <= -1.6)
                {
                    posPan = -1.6;
                }
                if (posTilt <= -2.0)
                {
                    posTilt = -2.0;
                }
                else if (posTilt >= -0.15)
                {
                    posTilt = -0.15;
                }
                
                //printf("pidPan = %.2f, pidTilt = %.2f\n", pidPan, pidTilt);
                

                headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);

                koordinasiJarak();
                saveBallLocation();
            } else {
            	printf("...ballLost...");
            }
        }
    }

    // Pen Tracking =============================================================================
    double intPanP = 0, dervPanP = 0, errorPanP = 0, preErrPanP = 0,
        PPanP = 0, IPanP = 0, DPanP = 0,
        intTiltP = 0, dervTiltP = 0, errorTiltP = 0, preErrTiltP = 0,
        PTiltP = 0, ITiltP = 0, DTiltP = 0,
        dtP = 0.04;

    double P_Pan_err_diff, P_Pan_err, P_Tilt_err_diff, P_Tilt_err, P_PanAngle, P_TiltAngle,
        pOffsetP, iOffsetP, dOffsetP,
        errorPanPRad, errorTiltPRad,
        offsetSetPointPall;

    void trackPen()
    {
        if (useVision)
        {
            if (Pinalty_X != -1 && Pinalty_Y != -1)
            { // printf("Tracking");
                // mode 1 ######################################################################
                /*// PID pan ==========================================================
                errorPanP  = (double)Pinalty_X - (frame_X / 2);//160
                PPanP  = errorPanP  * 0.00010; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya

                intPanP += errorPanP * dtB;
                IPanP = intPanP * 0.0;

                dervPanP = (errorPanP - preErrPanP) / dtB;
                DPanP = dervPanP * 0.00001;

                preErrPanP = errorPanP;

                //posPan += PPanP*-1; //dikali -1 kalau receler terbalik dalam pemasangan
                posPan += (PPanP + IPanP + DPanP) * -1;


                // PID tilt ==========================================================
                errorTiltP = (double)Pinalty_Y - (frame_Y / 2);//120
                PTiltB = errorTiltP * 0.00010; // Tune in Kp Tilt 0.00030

                intTiltP += errorTiltP * dtB;
                ITiltB = intTiltP * 0.0;

                dervTiltP = (errorTiltP - preErrTiltP) / dtB;
                DTiltB = dervTiltP * 0.00001;

                preErrTiltP = errorTiltP;

                //posTilt += PTiltB;
                posTilt += (PTiltB + ITiltB + DTiltB);*/

                // mode 2 ######################################################################
                //  offsetSetPointBall = ((int)(posTilt * 30)+7); //+54
                //  if (offsetSetPointBall > 36) offsetSetPointBall = 36;
                //  else if (offsetSetPointBall < 0) offsetSetPointBall = 0;

                // errorPanP  = (double)Pinalty_X - ((frame_X / 2) + offsetSetPointBall);//160
                errorPanP = (double)Pinalty_X - (frame_X / 2);
                errorTiltP = (double)Pinalty_Y - (frame_Y / 2); // 120
                errorPanP *= -1;
                errorTiltP *= -1;
                errorPanB *= (104.6 / (double)frame_X);  // pixel per angle
                errorTiltB *= (61.6 / (double)frame_Y); // pixel per angle
                // errorPanP *= (77.32 / (double)frame_X); // pixel per msg_yaw
                // errorTiltP *= (61.93 / (double)frame_Y); // pixel per msg_yaw

                errorPanPRad = (errorPanP * PI) / 180;
                errorTiltPRad = (errorTiltP * PI) / 180;
                // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanP, errorTiltP);
                // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanPRad, errorTiltPRad);
                // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

                P_Pan_err_diff = errorPanPRad - P_Pan_err;
                P_Tilt_err_diff = errorTiltPRad - P_Tilt_err;

                // PID pan ==========================================================
                // PPanP  = P_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                PPanP = P_Pan_err * ball_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                intPanP += P_Pan_err * dtP;
                IPanP = intPanP * 0.0;
                dervPanP = P_Pan_err_diff / dtP;
                // DPanP = dervPanP * kamera.panKD;
                DPanP = dervPanP * ball_panKD;
                P_Pan_err = errorPanPRad;
                posPan += (PPanP + IPanP + DPanP);

                // PID tilt ==========================================================
                // PTiltP = P_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
                PTiltP = P_Tilt_err * ball_tiltKP; // Tune in Kp Tilt 0.00030

                intTiltP += P_Tilt_err * dtP;
                ITiltP = intTiltP * 0.0;

                dervTiltP = P_Tilt_err_diff / dtP;
                // DTiltP = dervTiltP * kamera.tiltKD;
                DTiltP = dervTiltP * ball_tiltKD;

                preErrTiltP = errorTiltP;
                P_Tilt_err = errorTiltPRad;
                posTilt += (PTiltP + ITiltP + DTiltP) * -1;

                if (posPan >= 1.6)
                {
                    posPan = 1.6;
                }
                else if (posPan <= -1.6)
                {
                    posPan = -1.6;
                }
                if (posTilt <= -2.0)
                {
                    posTilt = -2.0;
                }
                else if (posTilt >= -0.4)
                {
                    posTilt = -0.4;
                }

                headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);

            }
        }
    }


    // Body Tracking Ball ========================================================================
    double errorBodyPosition,
        bodyP_Controller;
    int bodyTrue = 0,
        delayTrue = 0;
    int bodyTrackingBall(int threshold)
    {
        errorBodyPosition = 0 - headPan;
        bodyP_Controller = errorBodyPosition * -0.5; //-0.5

        if (ballLost(35))
        {
            Walk(0.0, 0.0, 0.0);
            bodyP_Controller = bodyTrue = delayTrue = 0;
        }
        else
        {
            if (errorBodyPosition >= -0.3 && errorBodyPosition <= 0.3)
            { // untuk hasil hadap 0.8
                // motion("0");
                Walk(0.0, 0.0, 0.0);
                delayTrue++;
            }
            else
            {
                trackBall();

                if (bodyP_Controller < 0)
                {
                    bodyP_Controller = -0.2;
                } // kanan 0.15
                else
                {
                    bodyP_Controller = 0.2;
                } // kiri 0.15

                bodyTrue = delayTrue = 0;
                Walk(0.0, 0.0, bodyP_Controller);
            }

            if (delayTrue >= threshold)
            {
                bodyTrue = 1;
            }
            else
            {
                bodyTrue = 0;
            }
        } // printf("Body Error = %.2f\t Body P Controller = %.2f\n",errorBodyPosition,bodyP_Controller);
        return bodyTrue;
    }

    // Hitung Jarak Bola berdasarkan headTilt ==============================================================
    double alphaY, // hasil derajat ketika headTilt
        betaY,
        inputY, // nilai realtime headTilt

        alphaX, // hasil derajat ketika headPan
        betaX,
        inputX, // nilai realtime headPan

        jarakBola_Y, // hasil jarak(cm) dari kalkulasi headTilt
        jarakBola_X, // hasil jarak(cm) dari kalkulasi headPan
        jarakBola;

    void kalkulasiJarakBola()
    {
        inputY = headTilt;
        inputX = headPan;

        // alphaY = -57.29 * headTilt; //(metode 1)
        alphaY = outputSudutY1 + ((outputSudutY2 - outputSudutY1) / (inputSudutY2 - inputSudutY1)) * (inputY - inputSudutY1); //(metode 2) //printf("  alphaY = %.2f,", alphaY);
        betaY = 180 - (90 + alphaY);                                                                                          // printf("  betaY = %.2f,", betaY);

        // alphaX = -57.29 * headPan; //(metode 1)
        alphaX = outputSudutX1 + ((outputSudutX2 - outputSudutX1) / (inputSudutX2 - inputSudutX1)) * (inputX - inputSudutX1); //(metode 2) //printf("  alphaX = %.2f,", alphaX);
        betaX = 180 - (90 + alphaX);                                                                                          // printf("  betaX = %.2f,", betaX);

        // sin & cos dalam c++ adalah radian, oleh karena itu harus : sin( ... * PI / 180)
        jarakBola_Y = (tinggiRobot * sin(alphaY * PI / 180)) / sin(betaY * PI / 180); // printf("  jarakBola_Y = %.f,", jarakBola_Y);
        jarakBola_X = (jarakBola_Y * sin(alphaX * PI / 180)) / sin(betaX * PI / 180); // printf("  jarakBola_X = %.f\n\n\n\n", jarakBola_X);
        jarakBola = sqrt((jarakBola_Y * jarakBola_Y) + (jarakBola_X * jarakBola_X));

        // regresi (metode 3)
        // jarakBola_Y = (-933.9*(pow(posTilt,5))) + (-5340.8*(pow(posTilt,4))) + (-12018*(pow(posTilt,3))) + (-13183*(pow(posTilt,2))) + (-7050.2*posTilt) - 1454.3;
    }

    // Untuk Kalkulasi Posisi P1
    double P1_X, P1_Y;
    void hitungKoordinatBolaP1()
    {
        // mode1--------------
        // kalkulasiJarakBola();
        // P1_X = jarakBola_X;
        // P1_Y = jarakBola_Y;
        // mode2--------------
        P1_X = Ball_Y;
        P1_Y = Ball_X;
        // printf("  P1_X = %.2f,  P1_Y = %.2f,", P1_X, P1_Y);
    }

    // Untuk Kalkulasi Posisi P2
    double P2_X, P2_Y;
    void hitungKoordinatBolaP2()
    {
        // mode1--------------
        // kalkulasiJarakBola();
        // P2_X = jarakBola_X;
        // P2_Y = jarakBola_Y;
        // mode2--------------
        P2_X = Ball_Y;
        P2_Y = Ball_X;
        // printf("  P2_X = %.2f,  P2_Y = %.2f,", P2_X, P2_Y);
    }

    // Untuk penyebut dan pembilang gradient
    double deltaY = 0, deltaX = 0;
    void hitungDeltaY()
    {
        deltaY = P2_Y - P1_Y; // mendekat positif
        // deltaY = P1_Y - P2_Y; //mendekat positif
        // printf("  deltaY = %.2f,", deltaY);
    }

    void hitungDeltaX()
    {
        deltaX = P2_X - P1_X; // mendekat negatif
        // deltaX = P1_X - P2_X; //mendekat positif
        // printf("  deltaX = %.2f,\n\n", deltaX);
    }

    // Untuk Gradient
    double gradient;
    void hitungGradient()
    {
        gradient = deltaY / deltaX;
        // printf("  gradient = %.2f,", gradient);
    }

    int cntOke1,
        cntOke2,
        cntUlang,
        kondisiBola = 0;
    bool oke = true,
         ulang = false;
    void hitungGerakBola()
    { // Bagian pengecekan pergerakan bola
        if (Ball_X == -1 && Ball_Y == -1)
        { // bola hilang
            deltaY =
                deltaX =
                    jarakBola_X =
                        jarakBola_Y = 0;
        }

        if (ulang)
        { // printf("ulang \n");
            cntOke1 =
                cntOke2 = 0;
            if (cntUlang > 5)
            {
                ulang = false;
                oke = true;
            }
            else
            {
                deltaY =
                    deltaX =
                        jarakBola_X =
                            jarakBola_Y = 0;

                cntUlang++;
            }
        }

        if (oke)
        { // printf("oke \n");
            cntUlang = 0;
            if (cntOke1 > 5)
            {
                hitungKoordinatBolaP2();

                hitungDeltaY();
                hitungDeltaX();
                // hitungGradient();

                if (cntOke2 > 10)
                {
                    oke = false;
                    ulang = true;
                }
                else
                {
                    cntOke2++;
                }
            }
            else
            {
                hitungKoordinatBolaP1();
                cntOke1++;
            }
        }

        // if ((deltaY >= 20 && deltaX <= -20) || (deltaY >= 20 && deltaX >= 20)) { //0.5 //7.0
        if (deltaY >= 10)
        { // 0.5 //7.0 // 30
            // printf("  deltaY = %.f,", deltaY);
            // printf("  Bola Menjauh\n");
            kondisiBola = 1;
            //} else if((deltaY <= -20 && deltaX <= -20) || (deltaY <= -20 && deltaX >= 20)) { //-2 //-1.4
        }
        else if (deltaY <= -10)
        { //-2 //-1.4 // -30
            // printf("  deltaY = %.f,", deltaY);
            // printf("  Bola Mendekat\n");
            kondisiBola = -1;
        }
        else if (deltaY >= -0.3 && deltaY <= 0.3 && deltaX >= -0.3 && deltaX <= 0.3)
        { //-2 //-1.4 // -5
            // printf("  Bola Diam");
            kondisiBola = 0;
        }
    }
    
    enum AvoidRobotDir { AVOID_ROB_NONE, AVOID_ROB_LEFT, AVOID_ROB_RIGHT };
    AvoidRobotDir avoidRobDir  = AVOID_ROB_NONE;
    int           avoidRobHold = 0;           // counter hold setelah robot hilang

    // Threshold bbox robot (tuning sesuai resolusi kamera)
    const float ROB_W_THRESH    = 120.0f;     // lebar   bbox agar dianggap menghalangi
    const float ROB_H_THRESH    = 160.0f;     // tinggi  bbox
    const float ROB_CENTER_ZONE = 90.0f;      // ±90px dari tengah frame = zona tengah
    const float FRAME_CX        = 320.0f;     // setengah lebar frame (640px)
    const int   AVOID_HOLD_MAX  = 25;
    
    /*
    // Follow Ball ===============================================================================
    int	countReadyKick;
    double	SetPointPan = 0,
        SetPointTilt = cAktif,
        errorfPan,
        errorfTilt,
        PyMove = 0,
        PxMove = 0,
        PaMove = 0;
    void followBall(int mode){ //0 normal, 1 sambil belok
        // trackBall();

        printf("...followBall...\n");
        if	(posTilt >= SetPointTilt) { posTilt = SetPointTilt; }
        else if	(posTilt < -2.0) { posTilt = -2.0; }

        errorfPan  = posPan - SetPointPan;
        errorfTilt = posTilt - SetPointTilt;

        if (posTilt >= SetPointTilt && Ball_X != -1 && Ball_Y != -1) { //Stop(bola sudah dekat)
            countReadyKick++;
        } else { //Kejar Bola(bola masih jauh)
            countReadyKick = 0;
        }

        if (countReadyKick >= 1) { //5
            PxMove = 0.00; //jalan ditempat
            PyMove = errorfPan * 0.040;
            PaMove = errorfPan * 0.20;
        } else {
            if (headTilt > (cAktif)) { //cAktif+0.15
                PxMove = 0.00;
                printf("diam...\n");
            } else if (headTilt > -1.4) {
                PxMove = kejar;
                printf("kejar...\n");
            } else if (headTilt > -1.6) {
                PxMove = kejarMid;
                printf("kejarmid...\n");
            } else {
                PxMove = kejarMax;
                printf("kejarmax...\n");
            }

            PyMove = errorfPan * 0.50;
            PaMove = errorfPan * 0.45;      
        }

        if (mode == 0) { // Mode differential walking
            if (errorfPan > -0.3 && errorfPan < 0.3) {
                Walk(PxMove, 0.0, PaMove);
            } else {
                Walk(0.0, 0.0, PaMove);
            }
        }
        else if (mode == 1) { // Mode omnidirectional walking
            if (errorfPan > -0.4 && errorfPan < 0.4) {
                Walk(PxMove, PyMove, PaMove);
            } else {
                Walk(0.0, 0.0, PaMove);
            }
        }
    }
*/
double lastBallPanDir = 0.0;   // pan kepala saat bola terakhir terlihat
bool   hasBallMemory  = false; // apakah sudah pernah melihat bola
const int BALL_MEMORY_TIMEOUT = 90; // ~3 detik di 30Hz
int ballLostCounter = 0;

void searchBallFromMemory()
{
    if (!hasBallMemory) {
        // Belum pernah lihat bola → search normal
        panSearchBall(-1.6);
        return;
    }

    // Arahkan kepala ke posisi terakhir bola dilihat
    posTilt = -1.4;
    posPan  = lastBallPanDir;

    // Clamp supaya tidak melebihi batas servo
    if (posPan >  batasKiri)  posPan =  batasKiri;
    if (posPan <  batasKanan) posPan =  batasKanan;

    headMove(posPan, posTilt);

    printf("[SEARCH] ingat bola terakhir di pan=%.2f (%s)\n",
           lastBallPanDir,
           lastBallPanDir < 0 ? "KANAN" : "KIRI");
}

int countReadyKick;
bool enableAvoid = false;
double SetPointPan  = 0,
       SetPointTilt = cAktif,
       errorfPan,
       errorfTilt,
       PyMove = 0,
       PxMove = 0,
       PaMove = 0;
       
int avoidRobCount = 0;
const int AVOID_COUNT_MAX = 200; 

void followBall(int mode)
{
    printf("...followBall...\n");

    // ════════════════════════════════════════════════════════════════
    // BAGIAN 1 — DETEKSI & KEPUTUSAN AVOID
    // ════════════════════════════════════════════════════════════════
    bool robotClose = enableAvoid
                   && (jarak_bola > 300.0)
                   && robot_detected
                   && (robot_w >= ROB_W_THRESH)
                   && (robot_h >= ROB_H_THRESH);

    if (robotClose && avoidRobDir == AVOID_ROB_NONE) {
        // Robot baru terdeteksi → tentukan arah & mulai counter
        if (robot_x < FRAME_CX - ROB_CENTER_ZONE) {
            avoidRobDir = AVOID_ROB_RIGHT;
        } else if (robot_x > FRAME_CX + ROB_CENTER_ZONE) {
            avoidRobDir = AVOID_ROB_LEFT;
        } else {
            avoidRobDir = (robot_x <= FRAME_CX) ? AVOID_ROB_RIGHT : AVOID_ROB_LEFT;
        }
        avoidRobCount = AVOID_COUNT_MAX;  // mulai hitung mundur
    }

    // ════════════════════════════════════════════════════════════════
    // BAGIAN 2 — EKSEKUSI AVOID (counter-based)
    // ════════════════════════════════════════════════════════════════
    if (avoidRobDir != AVOID_ROB_NONE) {
        avoidRobCount--;

        if (avoidRobCount <= 0) {
            // Counter habis → selesai avoid
            avoidRobCount = 0;
            avoidRobDir   = AVOID_ROB_NONE;
            printf("[AVOID] selesai\n");
        } else {
            // Masih dalam hitungan → eksekusi geser
            trackBall();

            if (avoidRobDir == AVOID_ROB_RIGHT) {
                printf("[AVOID] geser KANAN (sisa %d)\n", avoidRobCount);
                Walk(0.008, -0.050, 0.0);
            } else {
                printf("[AVOID] geser KIRI  (sisa %d)\n", avoidRobCount);
                Walk(0.008, 0.050, 0.0);
            }
            return;
        }
    }

    // ════════════════════════════════════════════════════════════════
    // BAGIAN 3 — FOLLOW BALL NORMAL
    // ════════════════════════════════════════════════════════════════
    if (posTilt >= SetPointTilt) { posTilt = SetPointTilt; }
    else if (posTilt < -2.0)    { posTilt = -2.0; }

    errorfPan  = posPan  - SetPointPan;
    errorfTilt = posTilt - SetPointTilt;

    if (posTilt >= SetPointTilt && Ball_X != -1 && Ball_Y != -1) {
        countReadyKick++;
    } else {
        countReadyKick = 0;
    }

    if (countReadyKick >= 1) {
        PxMove = 0.00;
        PyMove = errorfPan * 0.040;
        PaMove = errorfPan * 0.20;
    } else {
        if (headTilt > cAktif) {
            PxMove = 0.00;
            printf("diam...\n");
        } else if (headTilt > -1.4) {
            PxMove = kejar;
            printf("kejar...\n");
        } else if (headTilt > -1.6) {
            PxMove = kejarMid;
            printf("kejarmid...\n");
        } else {
            PxMove = kejarMax;
            printf("kejarmax...\n");
        }
        PyMove = errorfPan * 0.50;
        PaMove = errorfPan * 0.45;
    }

    if (mode == 0) {
        if (errorfPan > -0.3 && errorfPan < 0.3) {
            Walk(PxMove, 0.0, PaMove);
        } else {
            Walk(0.0, 0.0, PaMove);
        }
    } else if (mode == 1) {
        if (errorfPan > -0.4 && errorfPan < 0.4) {
            Walk(PxMove, PyMove, PaMove);
        } else {
            Walk(0.0, 0.0, PaMove);
        }
    }
}

/*void followBall(int mode) //tanpa durasi - done by vision
{
    printf("...followBall...\n");

    // ════════════════════════════════════════════════════════════════
    // BAGIAN 1 — DETEKSI & KEPUTUSAN AVOID
    // ════════════════════════════════════════════════════════════════
    bool robotClose = enableAvoid
    		      && (ballDistance > 300)
    		      && robot_detected
                      && (robot_w >= ROB_W_THRESH)
                      && (robot_h >= ROB_H_THRESH);

    if (robotClose) {
        // Tentukan arah hindari saat robot pertama kali terdeteksi
        if (avoidRobDir == AVOID_ROB_NONE) {
            if (robot_x < FRAME_CX - ROB_CENTER_ZONE) {
                // Robot di sisi KIRI frame → kita geser ke KANAN
                avoidRobDir = AVOID_ROB_RIGHT;
            } else if (robot_x > FRAME_CX + ROB_CENTER_ZONE) {
                // Robot di sisi KANAN frame → kita geser ke KIRI
                avoidRobDir = AVOID_ROB_LEFT;
            } else {
                // Robot tepat di tengah → pilih sisi yang lebih banyak ruang
                avoidRobDir = (robot_x <= FRAME_CX) ? AVOID_ROB_RIGHT : AVOID_ROB_LEFT;
            }
        }
        avoidRobHold = AVOID_HOLD_MAX;   // reset hold timer
    } else {
        // Robot tidak terdeteksi → kurangi hold timer
        if (avoidRobHold > 0) avoidRobHold--;
        else                   avoidRobDir = AVOID_ROB_NONE;
    }

    // ════════════════════════════════════════════════════════════════
    // BAGIAN 2 — EKSEKUSI AVOID (jika sedang menghindar)
    // ════════════════════════════════════════════════════════════════
    if (avoidRobDir != AVOID_ROB_NONE) {
        trackBall();   // kepala tetap tracking bola selama menghindar

        if (avoidRobDir == AVOID_ROB_RIGHT) {
            // Geser ke kanan sambil sedikit maju
            printf("[AVOID] geser KANAN (robot_x=%.0f w=%.0f h=%.0f)\n",
                   robot_x, robot_w, robot_h);
            Walk(0.008, -0.050, 0.0);
        } else {
            // Geser ke kiri sambil sedikit maju
            printf("[AVOID] geser KIRI  (robot_x=%.0f w=%.0f h=%.0f)\n",
                   robot_x, robot_w, robot_h);
            Walk(0.008, 0.050, 0.0);
        }
        return;   // skip logika follow normal selama menghindar
    }

    // ════════════════════════════════════════════════════════════════
    // BAGIAN 3 — FOLLOW BALL NORMAL (tidak ada robot penghalang)
    // ════════════════════════════════════════════════════════════════
    if (posTilt >= SetPointTilt) { posTilt = SetPointTilt; }
    else if (posTilt < -2.0)    { posTilt = -2.0; }

    errorfPan  = posPan  - SetPointPan;
    errorfTilt = posTilt - SetPointTilt;

    // Cek bola sudah dekat (siap tendang)
    if (posTilt >= SetPointTilt && Ball_X != -1 && Ball_Y != -1) {
        countReadyKick++;
    } else {
        countReadyKick = 0;
    }

    if (countReadyKick >= 1) {
        // Bola sudah dekat → jalan di tempat, sesuaikan posisi lateral
        PxMove = 0.00;
        PyMove = errorfPan * 0.040;
        PaMove = errorfPan * 0.20;
    } else {
        // Bola masih jauh → kejar
        if (headTilt > cAktif) {
            PxMove = 0.00;
            printf("diam...\n");
        } else if (headTilt > -1.4) {
            PxMove = kejar;
            printf("kejar...\n");
        } else if (headTilt > -1.6) {
            PxMove = kejarMid;
            printf("kejarmid...\n");
        } else {
            PxMove = kejarMax;
            printf("kejarmax...\n");
        }
        PyMove = errorfPan * 0.50;
        PaMove = errorfPan * 0.45;
    }

    // Eksekusi Walk sesuai mode
    if (mode == 0) {
        // Mode differential
        if (errorfPan > -0.3 && errorfPan < 0.3) {
            Walk(PxMove, 0.0, PaMove);
        } else {
            Walk(0.0, 0.0, PaMove);
        }
    } else if (mode == 1) {
        // Mode omnidirectional
        if (errorfPan > -0.4 && errorfPan < 0.4) {
            Walk(PxMove, PyMove, PaMove);
        } else {
            Walk(0.0, 0.0, PaMove);
        }
    }
}*/
//-------------Robocup 2024---------------
/*    void followBall(int mode){ //0 normal, 1 sambil belok
        // trackBall();

        printf("...followBall...\n");
        if	(posTilt >= SetPointTilt) { posTilt = SetPointTilt; }
        else if	(posTilt < -2.0) { posTilt = -2.0; }

        errorfPan  = posPan - SetPointPan;
        errorfTilt = posTilt - SetPointTilt;

        if (posTilt >= SetPointTilt && posPan < 0.5 && posPan > -0.5 && Ball_X != -1 && Ball_Y != -1) { //Stop(bola sudah dekat)
            countReadyKick++;
        } else { //Kejar Bola(bola masih jauh)
            countReadyKick = 0;
        }

        if (countReadyKick >= 1) { //5
            PxMove = 0.00; //jalan ditempat
            PyMove = errorfPan * 0.040; //0.045
            PaMove = errorfPan * 0.20; //0.30; //0.045
        } else {
            if (headTilt < -1.5) {
                PxMove = kejarMax; //0.08
            } else if (headTilt >= -1.5 && headTilt < -1.4) {
                PxMove = kejarMid; //0.07
            } else if (headTilt > -1.0) {
                PxMove = lari; //0.05
            } else {
                PxMove = lari; //0.06
            }
            //PxMove = errorfTilt * 0.1 * -13; //Robot besar 0.13, robot kecil 0.1
            // PxMove = 0.06 / -1.6 * posTilt; //0.04-0.06
            PyMove = errorfPan * 0.50;//0.125; //0.045
            PaMove = errorfPan * 0.45;//0.25; //0.35; //0.045            
        }

        if (mode == 0) { // Mode differential walking
            if (errorfPan > -0.3 && errorfPan < 0.3) { 	//printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            } else { 					//printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
        }
        else if (mode == 1) { // Mode omnidirectional walking
            if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("CCCCCCCC\n");
                Walk(PxMove, PyMove, PaMove);
            } else { 					//printf("DDDDDDDD\n");
                Walk(0.0, 0.0, PaMove);
            }
        }
    }*/ //------------------------------

//track bola dengan rotasi badan
    int reset_body = 0;
    double last_pan = 0;
    bool body_tracked = false;
    void newBodyTracking()
    {
        //trackBall();
        if (!body_tracked)
        {
            if (posPan < 0.1 && posPan > -0.1)
            { // Stop(bola sudah dekat)
                motion("0");
                last_pan = posPan;
                body_tracked = true;
            }
            else
            { // Kejar Bola(bola masih jauh)
                errorfPan = posPan - SetPointPan;
                PyMove = errorfPan * 0.20; // 0.125; //0.045
                PaMove = errorfPan * 0.45; // 0.25; //0.35; //0.045

                // Define smoothing factor
                double alpha = 0.2;

                // Initialize smoothed velocities
                double smoothedPxMove = PxMove;
                double smoothedPyMove = PyMove;
                double smoothedPaMove = PaMove;

                // Calculate smoothed velocities using EMA algorithm
                smoothedPxMove = alpha * PxMove + (1 - alpha) * smoothedPxMove;
                smoothedPyMove = alpha * PyMove + (1 - alpha) * smoothedPyMove;
                smoothedPaMove = alpha * PaMove + (1 - alpha) * smoothedPaMove;

                Walk(0.0, 0.0, smoothedPaMove);
                //printf("...rotate\n");
            }
        }
        else
        {
            if (abs(last_pan - posPan) > 45 * PI/180.0)
            {
                body_tracked = false;
            }
        }
    }
//kiper balik ke gawang
    bool doneGkBack = false;
    bool y_done = false, x_done = false;
    int cntBackPos = 0;
    void gkBackPos()
    {
        double y_move = (abs(odom_pose_y) * 0.15) / 0.5;

        if (posRotateNew)
        {
            if (odom_pose_y >= -0.1 && odom_pose_y <= 0.1)
            {
                doneGkBack = true;
            }
            else
            {
                if (cntBackPos > 30)
                {
                    if (odom_pose_y > 0)
                    {
                        jalanDirection(0.0, -y_move, 0.0);
                    }
                    else
                    {
                        jalanDirection(0.0, y_move, 0.0);
                    }
                }
                else
                {
                    jalanDirection(0.0, 0.0, 0.0);
                    cntBackPos++;
                }
            }
        }
        else
        {
            rotateBodyImuNew(0);
        }
    }

   int maxY = 0, maxX = 0, minY = 0, minX = 0, lastKoorX = 0;
    bool doneBanting = false, keeperNoTrack = false;
    bool releaseInitDone = false; 
    double lastBallPan = 0.0; // Simpan arah bola terakhir sebelum banting
    void banting()
    {
        if (ballLost(10))
        {
            //Walk(0.0, 0.0, 0.0);
            delayWaitBall = 0;
            threeSearchBall();
            ballPos = robotDirection = false;
            tunggu = lastKoorX = minX = minY = maxX = maxY = 0;
        }
        else
        {
            trackBall();
            delayWaitBall++;
            if (delayWaitBall > 3)
            {
                if (posTilt >= tiltBolaDekat)
                {
                    if (headPan < panSaveKanan)
                    {
                        lastBallPan = headPan; // Simpan arah bola
                        motion("3"); // Banting Kanan
                        doneBanting = true;
                    }
                    else if (headPan > panSaveKiri)
                    {
                        lastBallPan = headPan; // Simpan arah bola
                        motion("4"); // Banting Kiri
                        doneBanting = true;
                    }
                    else if (posTilt >= -1.2) 
                    {
                        lastBallPan = headPan; // Simpan arah bola
                        // Bola mendekat lurus ke arah tengah, sangat dekat
                        motion("7"); // Banting Tengah
                        headMove(0.0, -0.6);
                        doneBanting = true;
                    }
                }
            }
            else
            {
                delayWaitBall++;
            }
        }
    }

    // IMU ===========================================================================
    int setPoint1,
        setPoint2;

    double errorCPosPan,
        errorCPosTilt,
        alfaImu,
        bodyYImu,
        bodyXImu;

    //	X+ = maju
    //	X- = mundur
    //	Y+ = samping kiri
    //	Y- = samping kanan
    //	A+ = putar kiri
    //	A- = putar kanan
    void rotateDirec(int arah, double jarakTilt) //mutar IMU ketika positioning
    {
        if (posPan >= 0) {
            errorCPosPan = (posPan) - 0.1;
        } else {
            errorCPosPan = (posPan) + 0.1;
        }
        
        //errorCPosPan = posPan;                 // adalah nilai tengah pan, dan menjadi titik berhenti jika telah tepenuhi
        errorCPosTilt = posTilt - (jarakTilt); //-0.45 adalah nilai tengah tilt, robot akan jalan ditempat(tidak maju/mundur) jika nilai terpenuhi

        bodyXImu = errorCPosTilt * (-0.065);     // nilai pengali ini harus tetap bernilai negatif //besarnya kalkulasi maju/mundur yg dibutuhkan tehadap posTilt
        bodyYImu = abs(errorCPosPan / 100) + 0.025; //0.035; // 0.017;
        alfaImu = errorCPosPan * 0.80; //0.9;               // 0.75; nilai pengali ini harus tetap bernilai positif //besarnya kalkulasi rotate yg dibutuhkan tehadap posPan

        if (arah <= 0)
        { // rotate ke kanan
            // alfaImu = -0.15; bodyYImu = 0.017;
            if (bodyYImu < 0)
            {
                bodyYImu = -bodyYImu;
            }
        }
        else
        { // rotate ke kiri
            // alfaImu = 0.15; bodyYImu = -0.017;
            if (bodyYImu > 0)
            {
                bodyYImu = -bodyYImu;
            }
        }
        //printf("22222222\n");
    }
    
    void rotateParabolic(int arah, double jarakTilt)
    {
        rotateDirec(arah, jarakTilt);
        Walk(bodyXImu, bodyYImu, alfaImu);
    }
    
    bool robotDirection = false;
int eleh = 0,
    btsRotate = 0,
    btsSetPoint = 0,
    postImuCooldown = 0;          // <<< harus masuk ke dalam blok int yang sama
const int POST_IMU_COOLDOWN_MAX = 8;
    void Imu(int gawang, double jarakTilt)
{ // opsi 2, menggunakan "mode" untuk mengecek robot direction

    printf("...IMU...\n");
    if (gawang > 180)
    {
        gawang = 180;
    }
    else if (gawang < -180)
    {
        gawang = -180;
    }

    setPoint1 = 10 + gawang;
    setPoint2 = -10 + gawang;

    if (setPoint1 > 180 || setPoint2 < -180)
    {
        if (setPoint1 > 180)
        {
            btsSetPoint = setPoint1 - 360;

            if (msg_yaw >= setPoint2 || msg_yaw <= btsSetPoint)
            {
                bodyXImu = bodyYImu = alfaImu = 0.0;
                if (eleh > 10)
                {
                    robotDirection = true;
                    postImuCooldown = 0; // <<< RESET COOLDOWN
                }
                else
                {
                    eleh++;
                }
            }
            else
            {
                if (gawang >= 0)
                {
                    btsRotate = gawang - 180;
                    if ((msg_yaw <= gawang) && (msg_yaw >= btsRotate))
                    {
                        rotateDirec(-1, jarakTilt);
                    }
                    else
                    {
                        rotateDirec(1, jarakTilt);
                    }
                }
                else
                {
                    btsRotate = gawang + 180;
                    if ((msg_yaw >= gawang) && (msg_yaw <= btsRotate))
                    {
                        rotateDirec(1, jarakTilt);
                    }
                    else
                    {
                        rotateDirec(-1, jarakTilt);
                    }
                }

                eleh = 0;
                robotDirection = false;
            }
            Walk(bodyXImu, bodyYImu, alfaImu);
        }
        else
        {
            btsSetPoint = setPoint2 + 360;

            if (msg_yaw >= btsSetPoint || msg_yaw <= setPoint1)
            {
                bodyXImu = bodyYImu = alfaImu = 0.0;
                if (eleh > 10)
                {
                    robotDirection = true;
                    postImuCooldown = 0; // <<< RESET COOLDOWN
                }
                else
                {
                    eleh++;
                }
            }
            else
            {
                if (gawang >= 0)
                {
                    btsRotate = gawang - 180;
                    if ((msg_yaw <= gawang) && (msg_yaw >= btsRotate))
                    {
                        rotateDirec(-1, jarakTilt);
                    }
                    else
                    {
                        rotateDirec(1, jarakTilt);
                    }
                }
                else
                {
                    btsRotate = gawang + 180;
                    if ((msg_yaw >= gawang) && (msg_yaw <= btsRotate))
                    {
                        rotateDirec(1, jarakTilt);
                    }
                    else
                    {
                        rotateDirec(-1, jarakTilt);
                    }
                }

                eleh = 0;
                robotDirection = false;
            }
            Walk(bodyXImu, bodyYImu, alfaImu);
        }
    }
    else
    {
        if (msg_yaw >= setPoint2 && msg_yaw <= setPoint1)
        {
            bodyXImu = bodyYImu = alfaImu = 0.0;
            if (eleh > 10)
            {
                robotDirection = true;
                postImuCooldown = 0; // <<< RESET COOLDOWN
            }
            else
            {
                eleh++;
            }
        }
        else
        {
            if (gawang >= 0)
            {
                btsRotate = gawang - 180;
                if ((msg_yaw <= gawang) && (msg_yaw >= btsRotate))
                {
                    rotateDirec(-1, jarakTilt);
                }
                else
                {
                    rotateDirec(1, jarakTilt);
                }
            }
            else
            {
                btsRotate = gawang + 180;
                if ((msg_yaw >= gawang) && (msg_yaw <= btsRotate))
                {
                    rotateDirec(1, jarakTilt);
                }
                else
                {
                    rotateDirec(-1, jarakTilt);
                }
            }

            eleh = 0;
            robotDirection = false;
        }
        Walk(bodyXImu, bodyYImu, alfaImu);
    }
}
    
/*TERAKHIR DIPAKAI
    bool robotDirection = false;
    int eleh = 0,
        btsRotate = 0,
        btsSetPoint = 0;
    void Imu(int gawang, double jarakTilt)
    { // opsi 2, menggunakan "mode" untuk mengecek robot direction
        // trackBall();

        printf("...IMU...\n");
        if (gawang > 180)
        {
            gawang = 180;
        }
        else if (gawang < -180)
        {
            gawang = -180;
        }

        setPoint1 = 10 + gawang;  // 10 //15 //20 //60
        setPoint2 = -10 + gawang; // 10 //15 //20 //60

        if (setPoint1 > 180 || setPoint2 < -180)
        { // jika arah imu dibelakang
            if (setPoint1 > 180)
            { // nilai setpoint1 diubah jd negatif
                btsSetPoint = setPoint1 - 360;

                if (msg_yaw >= setPoint2 || msg_yaw <= btsSetPoint)
                {
                    bodyXImu = bodyYImu = alfaImu = 0.0;
                    if (eleh > 10)
                    {
                        robotDirection = true;
                    }
                    else
                    {
                        eleh++;
                    }
                }
                else
                {
                    if (gawang >= 0)
                    {
                        btsRotate = gawang - 180;
                        if ((msg_yaw <= gawang) && (msg_yaw >= btsRotate))
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                        else
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                    }
                    else
                    {
                        btsRotate = gawang + 180;
                        if ((msg_yaw >= gawang) && (msg_yaw <= btsRotate))
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                        else
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                    }

                    eleh = 0;
                    robotDirection = false;
                }
                //printf("...AAAAAAAA......\n");
                Walk(bodyXImu, bodyYImu, alfaImu);
            }
            else
            { // nilai setPoint2 diubah jadi positif
                btsSetPoint = setPoint2 + 360;

                if (msg_yaw >= btsSetPoint || msg_yaw <= setPoint1)
                {
                    bodyXImu = bodyYImu = alfaImu = 0.0;
                    if (eleh > 10)
                    {
                        robotDirection = true;
                    }
                    else
                    {
                        eleh++;
                    }
                }
                else
                {
                    if (gawang >= 0)
                    {
                        btsRotate = gawang - 180;
                        if ((msg_yaw <= gawang) && (msg_yaw >= btsRotate))
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                        else
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                    }
                    else
                    {
                        btsRotate = gawang + 180;
                        if ((msg_yaw >= gawang) && (msg_yaw <= btsRotate))
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                        else
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                    }

                    eleh = 0;
                    robotDirection = false;
                }
                //printf("...BBBBBBB......\n");
                Walk(bodyXImu, bodyYImu, alfaImu);
            }
        }
        else
        { // arah imu kedepan
            if (msg_yaw >= setPoint2 && msg_yaw <= setPoint1)
            {
                bodyXImu = bodyYImu = alfaImu = 0.0;
                if (eleh > 10)
                {
                    robotDirection = true;
                }
                else
                {
                    eleh++;
                }
            }
            else
            {
                if (gawang >= 0)
                {
                    btsRotate = gawang - 180;
                    if ((msg_yaw <= gawang) && (msg_yaw >= btsRotate))
                    {
                        rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                    }
                    else
                    {
                        rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                    }
                }
                else
                {
                    btsRotate = gawang + 180;
                    if ((msg_yaw >= gawang) && (msg_yaw <= btsRotate))
                    {
                        rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                    }
                    else
                    {
                        rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                    }
                }

                eleh = 0;
                robotDirection = false;
            }
            //printf("...CCCCCCCC......\n");
            Walk(bodyXImu, bodyYImu, alfaImu);
        }
    }
    */

/*void Imu(int gawang, double jarakTilt)
    { // opsi 2, menggunakan "mode" untuk mengecek robot direction
        // trackBall();
        printf("333333333\n");

        if (gawang > 180)
        {
            gawang = 180;
        }
        else if (gawang < -180)
        {
            gawang = -180;
        }

        setPoint1 = 10 + gawang;  // 10 //15 //20 //60
        setPoint2 = -10 + gawang; // 10 //15 //20 //60

        if (setPoint1 > 180 || setPoint2 < -180)
        { // jika arah imu dibelakang
            if (setPoint1 > 180)
            { // nilai setpoint1 diubah jd negatif
                btsSetPoint = setPoint1 - 360;

                if (msg_yaw >= setPoint2 || msg_yaw <= btsSetPoint)
                {
                    bodyXImu = bodyYImu = alfaImu = 0.0;
                    if (eleh > 10)
                    {
                        robotDirection = true;
                    }
                    else
                    {
                        eleh++;
                    }
                }
                else
                {
                    if (gawang >= 0)
                    {
                        btsRotate = gawang - 180;
                        if ((msg_yaw <= gawang) && (msg_yaw >= btsRotate))
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                        else
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                    }
                    else
                    {
                        btsRotate = gawang + 180;
                        if ((msg_yaw >= gawang) && (msg_yaw <= btsRotate))
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                        else
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                    }

                    eleh = 0;
                    robotDirection = false;
                }
                Walk(bodyXImu, bodyYImu, alfaImu);
            }
            else
            { // nilai setPoint2 diubah jadi positif
                btsSetPoint = setPoint2 + 360;

                if (msg_yaw >= btsSetPoint || msg_yaw <= setPoint1)
                {
                    bodyXImu = bodyYImu = alfaImu = 0.0;
                    if (eleh > 10)
                    {
                        robotDirection = true;
                    }
                    else
                    {
                        eleh++;
                    }
                }
                else
                {
                    if (gawang >= 0)
                    {
                        btsRotate = gawang - 180;
                        if ((msg_yaw <= gawang) && (msg_yaw >= btsRotate))
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                        else
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                    }
                    else
                    {
                        btsRotate = gawang + 180;
                        if ((msg_yaw >= gawang) && (msg_yaw <= btsRotate))
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                        else
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                    }

                    eleh = 0;
                    robotDirection = false;
                }
                Walk(bodyXImu, bodyYImu, alfaImu);
            }
        }
        else
        { // arah imu kedepan
            if (msg_yaw >= setPoint2 && msg_yaw <= setPoint1)
            {
                bodyXImu = bodyYImu = alfaImu = 0.0;
                if (eleh > 10)
                {
                    robotDirection = true;
                }
                else
                {
                    eleh++;
                }
            }
            else
            {
                if (gawang >= 0)
                {
                    btsRotate = gawang - 180;
                    if ((msg_yaw <= gawang) && (msg_yaw >= btsRotate))
                    {
                        rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                    }
                    else
                    {
                        rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                    }
                }
                else
                {
                    btsRotate = gawang + 180;
                    if ((msg_yaw >= gawang) && (msg_yaw <= btsRotate))
                    {
                        rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                    }
                    else
                    {
                        rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                    }
                }

                eleh = 0;
                robotDirection = false;
            }
            Walk(bodyXImu, bodyYImu, alfaImu);
        }
    }*/
    // Rotate Body with IMU ===========================================================
    bool posRotate = false;

    void rotateBodyImu(int rotate)
    {
        trackBall();

        setPoint1 = 5 + rotate;  // 20 bisa kemungkinan 180 keatas
        setPoint2 = -5 + rotate; // 20 bisa kemungkinan -180 bawah

        if (msg_yaw > setPoint2 && msg_yaw < setPoint1)
        {
            posRotate = true;
        }
        else
        {
            if (msg_yaw > rotate)
            { // putar Kiri
                Walk(0.0, 0.0, 0.2);
            }
            else
            { // putar Kanan
                Walk(0.0, 0.0, -0.2);
            }
            // posRotate = false;
        }
    }

    // Rotate Body with IMU ===========================================================
    bool posRotateNew = false;
    double IaMove, errorImuNew,
        putar;
    void rotateBodyImuNew(int rotate)
    {
        // trackBall();

        setPoint1 = 5 + rotate;  // 10 //15 //20 //60
        setPoint2 = -5 + rotate; // 10 //15 //20 //60

        if (setPoint1 > 180 || setPoint2 < -180)
        { // jika arah imu dibelakang
            if (setPoint1 > 180)
            { // nilai setpoint1 diubah jd negatif
                btsSetPoint = setPoint1 - 360;

                if (msg_yaw >= setPoint2 || msg_yaw <= btsSetPoint)
                { // misal 170 ke -170
                    PaMove = 0.00;
                    posRotateNew = true;
                }
                else
                {
                    btsRotate = rotate - 180;
                    if ((msg_yaw <= rotate) && (msg_yaw >= btsRotate))
                    {                                        // misal di range 0 - 180, maka putar kanan
                        putar = (rotate - msg_yaw) * 0.0065; // 0.0065
                        PaMove = -putar;
                    }
                    else
                    { // putar kiri
                        if (msg_yaw > rotate)
                        {
                            putar = (msg_yaw - rotate) * 0.0065;
                        } // 0.0065
                        else
                        {
                            putar = ((180 - rotate) + (180 + msg_yaw)) * 0.0065;
                        } // 0.0065
                        PaMove = putar;
                    }
                }
            }
            else
            { // nilai setPoint2 diubah jadi positif
                btsSetPoint = setPoint2 + 360;

                if (msg_yaw >= btsSetPoint || msg_yaw <= setPoint1)
                {
                    PaMove = 0.00;
                    posRotateNew = true;
                }
                else
                {
                    btsRotate = rotate + 180;
                    if ((msg_yaw >= rotate) && (msg_yaw <= btsRotate))
                    {                                           // misal di range -180 - 0, maka putar kiri
                        putar = abs(rotate - msg_yaw) * 0.0065; // 0.0033
                        PaMove = putar;
                    }
                    else
                    { // putar kanan
                        if (msg_yaw < rotate)
                        {
                            putar = (rotate - msg_yaw) * 0.0065;
                        } // 0.0033
                        else
                        {
                            putar = ((180 + rotate) + (180 - msg_yaw)) * 0.0065;
                        } // 0.0033
                        PaMove = -putar;
                    }
                }
            }
        }
        else
        { // arah imu kedepan
            if (msg_yaw >= setPoint2 && msg_yaw <= setPoint1)
            {
                PaMove = 0.00;
                posRotateNew = true;
            }
            else
            {
                if (rotate >= 0)
                {
                    btsRotate = rotate - 180;
                    if ((msg_yaw <= rotate) && (msg_yaw >= btsRotate))
                    {                                        // putar kanan
                        putar = (rotate - msg_yaw) * 0.0065; // 0.0033
                        PaMove = -putar;
                    }
                    else
                    { // putar kiri
                        if (msg_yaw > rotate)
                        {
                            putar = (msg_yaw - rotate) * 0.0065;
                        } // 0.0033
                        else
                        {
                            putar = ((180 - rotate) + (180 + msg_yaw)) * 0.0065;
                        } // 0.0033
                        PaMove = putar;
                    }
                }
                else
                {
                    btsRotate = rotate + 180;
                    if ((msg_yaw >= rotate) && (msg_yaw <= btsRotate))
                    {                                           // maka putar kiri
                        putar = abs(rotate - msg_yaw) * 0.0065; // 0.0033
                        PaMove = putar;
                    }
                    else
                    { // putar kanan
                        if (msg_yaw < rotate)
                        {
                            putar = (rotate - msg_yaw) * 0.0065;
                        } // 0.0033
                        else
                        {
                            putar = ((180 + rotate) + (180 - msg_yaw)) * 0.0065;
                        } // 0.0033
                        PaMove = -putar;
                    }
                }
            }
        }
        if (!posRotateNew)
        {
            Walk(0.0, 0.0, PaMove);
        }
    }

    //	A+ = putar kiri
    //	A- = putar kanan
    void jalanDirection(double Xwalk, double Ywalk, double rotate)
    {
        if (rotate > 180)
        {
            rotate = 180;
        }
        else if (rotate < -180)
        {
            rotate = -180;
        }
        //-175 sampai -185
        setPoint1 = 5 + rotate;  // 10 //15 //20 //60
        setPoint2 = -5 + rotate; // 10 //15 //20 //60

        if (setPoint1 > 180 || setPoint2 < -180)
        { // jika arah imu dibelakang
            if (setPoint1 > 180)
            { // nilai setpoint1 diubah jd negatif
                btsSetPoint = setPoint1 - 360;

                if (msg_yaw >= setPoint2 || msg_yaw <= btsSetPoint)
                { // misal 170 ke -170
                    PaMove = 0.00;
                }
                else
                {
                    btsRotate = rotate - 180;
                    if ((msg_yaw <= rotate) && (msg_yaw >= btsRotate))
                    {                                        // misal di range 0 - 180, maka putar kanan
                        putar = (rotate - msg_yaw) * 0.010; // 0.0033
                        PaMove = -putar;
                    }
                    else
                    { // putar kiri
                        if (msg_yaw > rotate)
                        {
                            putar = (msg_yaw - rotate) * 0.004;
                        } // 0.0033
                        else
                        {
                            putar = ((180 - rotate) + (180 + msg_yaw)) * 0.004;
                        } // 0.0033
                        PaMove = putar;
                    }
                }
            }
            else
            { // nilai setPoint2 diubah jadi positif
                btsSetPoint = setPoint2 + 360;

                if (msg_yaw >= btsSetPoint || msg_yaw <= setPoint1)
                {
                    PaMove = 0.00;
                }
                else
                {
                    btsRotate = rotate + 180;
                    if ((msg_yaw >= rotate) && (msg_yaw <= btsRotate))
                    {                                          // misal di range -180 - 0, maka putar kiri
                        putar = abs(rotate - msg_yaw) * 0.004; // 0.0033
                        PaMove = putar;
                    }
                    else
                    { // putar kanan
                        if (msg_yaw < rotate)
                        {
                            putar = (rotate - msg_yaw) * 0.004;
                        } // 0.0033
                        else
                        {
                            putar = ((180 + rotate) + (180 - msg_yaw)) * 0.004;
                        } // 0.0033
                        PaMove = -putar;
                    }
                }
            }
        }
        else
        { // arah imu kedepan
            if (msg_yaw >= setPoint2 && msg_yaw <= setPoint1)
            {
                PaMove = 0.00;
            }
            else
            {
                if (rotate >= 0)
                {
                    btsRotate = rotate - 180;
                    if ((msg_yaw <= rotate) && (msg_yaw >= btsRotate))
                    {                                       // putar kanan
                        putar = (rotate - msg_yaw) * 0.004; // 0.0033
                        PaMove = -putar;
                    }
                    else
                    { // putar kiri
                        if (msg_yaw > rotate)
                        {
                            putar = (msg_yaw - rotate) * 0.004;
                        } // 0.0033
                        else
                        {
                            putar = ((180 - rotate) + (180 + msg_yaw)) * 0.004;
                        } // 0.0033
                        PaMove = putar;
                    }
                }
                else
                {
                    btsRotate = rotate + 180;
                    if ((msg_yaw >= rotate) && (msg_yaw <= btsRotate))
                    {                                          // maka putar kiri
                        putar = abs(rotate - msg_yaw) * 0.004; // 0.0033
                        PaMove = putar;
                    }
                    else
                    { // putar kanan
                        if (msg_yaw < rotate)
                        {
                            putar = (rotate - msg_yaw) * 0.004;
                        } // 0.0033
                        else
                        {
                            putar = ((180 + rotate) + (180 - msg_yaw)) * 0.004;
                        } // 0.0033
                        PaMove = -putar;
                    }
                }
            }
        }

        if (PaMove > 0.3)
        {
            PaMove = 0.3;
        }
        else if (PaMove < -0.3)
        {
            PaMove = -0.3;
        }

        Walk(Xwalk, Ywalk, PaMove);
    }

    // Ball Positioning Using P Controller =======================================================
    double	errorPosX,
        errorPosY,
        PxMoveBallPos,
        PyMoveBallPos,
        PaMoveBallPos;
    bool	ballPos = false;
    /*void ballPositioning(double setPointX, double setPointY, double speed) {
        errorPosX = headPan - setPointX;
        errorPosY = headTilt - setPointY;

        printf("...ballPositioning...\n");
        if ((errorPosX > -0.10 && errorPosX < 0.10) && (errorPosY > -0.12)) { //&& errorPosY < 0.10)) { //sudah sesuai
            PyMoveBallPos = 0.00;
            PxMoveBallPos = 0.00;
            ballPos = true;
        } else { //belum sesuai
            ballPos = false;
            if ((headPan >= 1.0 && headTilt >= -1.2) || (headPan <= -1.0 && headTilt >= -1.2)) { //bola disamping //pan tilt kircok (polar)
                PxMoveBallPos = -0.03;
                PyMoveBallPos = errorPosX * 0.08;//0.12;
                //printf("444444444444\n");
            } else {
                if (headPan >= 0.04 && headTilt > setPointY || (headPan <= -0.04 && headTilt > setPointY)) { //bola disamping //pan tilt kircok (polar)
                    PxMoveBallPos = -0.03;
                    //printf("555555555555\n");
                } else {
                    //Xmove
                    if (headTilt > setPointY) { //> (setPointY + 0.1)) { //kelebihan
                        PxMoveBallPos = 0.00;
                    } else if (headTilt >= (setPointY - 0.1) && headTilt <= setPointY) { //<= (setPointY + 0.1)) { //sudah dalam range
                        PxMoveBallPos = 0.00;
                    } else if (headTilt >= (setPointY - 0.3) && headTilt < (setPointY - 0.1)) { //bola sudah dekat
                        PxMoveBallPos = errorPosY * -speed;
                        //PxMoveBallPos = errorPosY * -0.08;
    //					PxMoveBallPos = 0.01;
                        if (PxMoveBallPos >= 0.02) { PxMoveBallPos = 0.02; }
                        else if (PxMoveBallPos <= 0.00) { PxMoveBallPos = 0.00; }
                    } else { //bola masih jauh
                        //PxMoveBallPos = headTilt * (0.08 / -1.6); //0.05
                        PxMoveBallPos = kejar;
    //					PxMoveBallPos = 0.05;
                    }

                    //Ymove
                    if (headTilt >= (setPointY - 0.03)) { //> (setPointY + 0.1)) { //kelebihan
                        PyMoveBallPos = 0.00;
                    } else {
                        if (headPan >= (setPointX - 0.1) && headPan <= (setPointX + 0.1)) { //sudah dalam range
                            PyMoveBallPos = 0.00;
                        } else { //belum dalam range
                            PyMoveBallPos = errorPosX * 0.08;//0.08;//0.12;
                        }
                    }
                }
            }
            
        }
        //printf("...DDDDDDD......\n"); 
        Walk(PxMoveBallPos, PyMoveBallPos, 0.0);
    }*/ 
    	/*double  _fPx        = 0.0;   // filtered Walk X
	double  _fPy        = 0.0;   // filtered Walk Y
	double  _ixIntg     = 0.0;   // PID-X integral
	double  _ixPrev     = 0.0;   // PID-X prev error
	double  _iyIntg     = 0.0;   // PID-Y integral
	double  _iyPrev     = 0.0;   // PID-Y prev error
	int     _footLock   = 0;     // foot lock counter
	bool    _lastKanan  = false;  // state kaki terakhir
	
	inline void _resetBallPosState() {
	    _fPx = _fPy = 0.0;
	    _ixIntg = _ixPrev = 0.0;
	    _iyIntg = _iyPrev = 0.0;
	    _footLock = 0;
	}
	*/

        void followPen(int mode) 
    {
        if (posTilt >= SetPointTilt) 
        {
            posTilt = SetPointTilt;
        } 
        else if (posTilt < -2.0) 
        {
            posTilt = -2.0;
        }

        errorfPan = posPan - SetPointPan;
        errorfTilt = posTilt - SetPointTilt;

        if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && Pinalty_X != -1 && Pinalty_Y != -1) 
        { // Stop(bola sudah dekat)
            countReadyKick++;
        } 
        else 
        { // Kejar Bola(bola masih jauh)
            countReadyKick = 0;
        }

        if (countReadyKick >= 1) 
        { 
            PxMove = 0.0;
            PyMove = errorfPan * 0.040;
            PaMove = errorfPan * 0.20;
        } 
        else 
        {
            PxMove = kejarMax;
            PyMove = errorfPan * 0.40;
            PaMove = errorfPan * 0.30;
        }

        // Define smoothing factor
        double alpha = 0.2;

        // Initialize smoothed velocities
        double smoothedPxMove = PxMove;
        double smoothedPyMove = PyMove;
        double smoothedPaMove = PaMove;

        // Calculate smoothed velocities using EMA algorithm
        smoothedPxMove = alpha * PxMove + (1 - alpha) * smoothedPxMove;
        smoothedPyMove = alpha * PyMove + (1 - alpha) * smoothedPyMove;
        smoothedPaMove = alpha * PaMove + (1 - alpha) * smoothedPaMove;

        // Use the smoothed velocities for robot movement
        if (mode == 0) 
        {
            if (errorfPan > -0.2 && errorfPan < 0.2) 
            {
                set_velocity(smoothedPxMove, 0.0, smoothedPaMove);
            } 
            else 
            {
                set_velocity(0.0, 0.0, smoothedPaMove);
            }
        } 
        else if (mode == 1) 
        {
            if (errorfPan > -0.4 && errorfPan < 0.4) 
            {
                set_velocity(smoothedPxMove, smoothedPyMove, smoothedPaMove);
            } 
            else 
            {
                set_velocity(0.0, 0.0, smoothedPaMove);
            }
        }
    }
//TERAKHIR DI PAKAI
    void ballPositioning(double setPointX, double setPointY, double speed) {
        errorPosX = headPan - setPointX;
        errorPosY = headTilt - setPointY;

        printf("...ballPositioning...\n");
        if ((errorPosX > -0.13 && errorPosX < 0.13) && (errorPosY > -0.15)) { //&& errorPosY < 0.10)) { //sudah sesuai
            PyMoveBallPos = 0.00;
            PxMoveBallPos = 0.00;
            ballPos = true;
        } else { //belum sesuai
            ballPos = false;
            if ((headPan >= 1.5 && headTilt >= -1.7) || (headPan <= -1.5 && headTilt >= -1.7)) { //bola disamping //pan tilt kircok (polar)
                PxMoveBallPos = -0.035;
                PyMoveBallPos = errorPosX * 0.12;//0.12;
            } else {
                
                //Xmove
                if ((headTilt > setPointY) || (kanan && (headPan < (setPointX -0.20))) || (kiri && (headPan > (setPointX +0.20)))) { //kelebihan
                    printf("...Mundur...\n");
                    PxMoveBallPos = 0.0;
                } else if (headTilt >= (setPointY - 0.1) && headTilt <= setPointY) { //<= (setPointY + 0.1)) { //sudah dalam range
                    printf("...Jalan di tempat...\n");
                    PxMoveBallPos = 0.00;
                } else if (headTilt >= (setPointY - 0.3) && headTilt < (setPointY - 0.1)) { //bola sudah dekat
                    PxMoveBallPos = errorPosY * -speed;
                    if (PxMoveBallPos >= 0.03) {
                    	PxMoveBallPos = 0.03;
                    } else if (PxMoveBallPos <= 0.00) {
                    	PxMoveBallPos = 0.00;
                    }
                } else { //bola masih jauh
                    //PxMoveBallPos = (headTilt) * (0.08 / (-1.6));
                    PxMoveBallPos = 0.04; //0.09;
                }

                //Ymove
                if (headTilt >= (setPointY - 0.03)) { //> (setPointY + 0.1)) { //kelebihan
                    PyMoveBallPos = 0.00;
                } else {
                    if (headPan >= (setPointX - 0.05) && headPan <= (setPointX + 0.05)) { //sudah dalam range
                        PyMoveBallPos = 0.00;
                        printf("...Dalam range samping...");
                    } else { //belum dalam range
                        PyMoveBallPos = errorPosX * 0.15;
                        printf("...Jalan Samping...\n");
                    }
                }
                
                //-- }
            }
            
        }	
        //printf("...DDDDDDD......\n"); 
        Walk(PxMoveBallPos, PyMoveBallPos, 0.0);
    }


/*   void ballPositioning(double setPointX, double setPointY, double speed) {
    errorPosX = headPan - setPointX;
    errorPosY = headTilt - setPointY;
    printf("...ballPositioning...\n");

    if ((errorPosX > -0.10 && errorPosX < 0.10) && (errorPosY > -0.05)) {
        PyMoveBallPos = 0.00;
        PxMoveBallPos = 0.00;
        ballPos = true;
    } else {
        ballPos = false;

        if ((headPan >= 1.0 && headTilt >= -1.2) || (headPan <= -1.0 && headTilt >= -1.2)) {
            PxMoveBallPos = -0.03;
            PyMoveBallPos = errorPosX * 0.08;
        } else {
            if (headPan >= 0.04 && headTilt > setPointY || (headPan <= -0.04 && headTilt > setPointY)) {
                PxMoveBallPos = -0.03;
            } else {
                // ── XMOVE ──────────────────────────────────────────────
                if (headTilt > setPointY) {
                    PxMoveBallPos = 0.00;

                } else if (headTilt >= (setPointY - 0.10) && headTilt <= setPointY) {
                    // sudah di range setpoint, jalan ditempat
                    PxMoveBallPos = 0.00;

                } else if (headTilt >= (setPointY - 0.25) && headTilt < (setPointY - 0.10)) {
                    // FIX: zona pelan -- deket setpoint
                    PxMoveBallPos = errorPosY * -speed;
                    double brakeFactor = (headTilt - (setPointY - 0.25)) / 0.15;
                    if (brakeFactor < 0.0) brakeFactor = 0.0;
                    if (brakeFactor > 1.0) brakeFactor = 1.0;
                    PxMoveBallPos *= (1.0 - brakeFactor * 0.7);
                    if (PxMoveBallPos >= 0.012) { PxMoveBallPos = 0.012; }
                    else if (PxMoveBallPos <= 0.00) { PxMoveBallPos = 0.00; }

                } else if (headTilt >= (setPointY - 0.55) && headTilt < (setPointY - 0.25)) {
                    // FIX: zona sedang -- jangan langsung kejar penuh
                    PxMoveBallPos = 0.025;

                } else {
                    // FIX: bola masih jauh -- kejar tapi bukan 0.06 langsung
                    PxMoveBallPos = jalan; // 0.045, bukan kejar 0.06
                }

                // ── YMOVE ──────────────────────────────────────────────
                // ── YMOVE ──────────────────────────────────────────────
if (headTilt >= (setPointY - 0.25)) {
    // bola udah lumayan deket secara tilt, koreksi Y lebih bebas
    if (errorPosX >= -0.08 && errorPosX <= 0.08) {
        PyMoveBallPos = 0.00;
    } else {
        PyMoveBallPos = errorPosX * 0.08;
        if (PyMoveBallPos > 0.05)  PyMoveBallPos = 0.05;
        if (PyMoveBallPos < -0.05) PyMoveBallPos = -0.05;
    }
} else {
    // bola masih jauh, geser Y lebih agresif dikit
    if (errorPosX >= -0.08 && errorPosX <= 0.08) {
        PyMoveBallPos = 0.00;
    } else {
        PyMoveBallPos = errorPosX * 0.12;
        if (PyMoveBallPos > 0.06)  PyMoveBallPos = 0.06;
        if (PyMoveBallPos < -0.06) PyMoveBallPos = -0.06;
    }
}
               /* if (headTilt >= (setPointY - 0.03)) {
                    PyMoveBallPos = 0.00;
                } else {
                    if (headPan >= (setPointX - 0.1) && headPan <= (setPointX + 0.1)) {
                        PyMoveBallPos = 0.00;
                    } else {
                        PyMoveBallPos = errorPosX * 0.08;
                    }
                }
            }
        }
    }

    Walk(PxMoveBallPos, PyMoveBallPos, 0.0);
}
	
*/	
	double  _fPx        = 0.0;   // filtered Walk X
	double  _fPy        = 0.0;   // filtered Walk Y
	double  _ixIntg     = 0.0;   // PID-X integral
	double  _ixPrev     = 0.0;   // PID-X prev error
	double  _iyIntg     = 0.0;   // PID-Y integral
	double  _iyPrev     = 0.0;   // PID-Y prev error
	int     _footLock   = 0;     // foot lock counter
	bool    _lastKanan  = false;  // state kaki terakhir
	
	inline void _resetBallPosState() {
	    _fPx = _fPy = 0.0;
	    _ixIntg = _ixPrev = 0.0;
	    _iyIntg = _iyPrev = 0.0;
	    _footLock = 0;
	}

    
        

    // Dribble Ball ======================================================================
    int bawaBola;
    double setPointFootY, setPointFootY1, setPointFootY2;
    void dribble(int gawang, double speed)
    {
        // 1. Pastikan selalu melacak bola agar nilai headPan dan headTilt selalu update
        trackBall();

        // --- SAFETY LIMITS ---
        double maxStableSpeed = 0.09;  // Batas atas kecepatan maju
        double minStableSpeed = 0.04;  

        if (speed > maxStableSpeed) speed = maxStableSpeed;
        else if (speed < minStableSpeed) speed = minStableSpeed;

        // 2. Tentukan variabel batas toleransi bola berada di tengah (titik ideal)
        // panTolerance diperketat menjadi 0.05 agar bola BENAR-BENAR dipaksa di tengah kaki
        double panTolerance = 0.08;  
        double targetTilt = -1.30;   // Mulai ngerem dari jarak yang agak lebih jauh

        // 3. Logika Pergerakan Maju/Mundur (Sumbu X)
        if (headTilt > targetTilt) {
            PxMoveBallPos = speed; // Bola jauh, lari normal
        } else {
            // Hitung faktor pelambat. Semakin mendekati telapak kaki (-2.0), makin pelan.
            double distanceFactor = (2.0 + headTilt) / (2.0 + targetTilt); 
            
            // Limit bawah diturunkan ke 15% agar saat menempel di kaki, 
            // robot berjalan sangat pelan (hanya menyentuh/nudge bola bergantian)
            if (distanceFactor < 0.15) distanceFactor = 0.15; 
            if (distanceFactor > 1.0) distanceFactor = 1.0;
            
            PxMoveBallPos = speed * distanceFactor;    
        }

        // 4. Logika Pergerakan Menyamping (Sumbu Y) - Memaksa Bola di Tengah
        // Karena panTolerance sangat kecil (0.05), robot akan merespons cepat
        // menggeser badannya sedikit agar bola pas di tengah kedua kaki.
        if (headPan > panTolerance) {
            PyMoveBallPos = (headPan - panTolerance) * 0.15; // Agak responsif ke kiri
        } else if (headPan < -panTolerance) {
            PyMoveBallPos = (headPan + panTolerance) * 0.15; // Agak responsif ke kanan
        } else {
            PyMoveBallPos = 0.0;
        }

        // Batasi ketat maksimal kecepatan samping agar bodi tidak oleng/jatuh
        if (PyMoveBallPos > 0.025) PyMoveBallPos = 0.025;
        if (PyMoveBallPos < -0.025) PyMoveBallPos = -0.025;

        // 5. Eksekusi Jalan
        jalanDirection(PxMoveBallPos, PyMoveBallPos, gawang);
    }

    
    /*
    void dribble(int gawang, double speed)
    {
        // trackBall();

        // setPoint1 =  20 + gawang;//20
        // setPoint2 = -20 + gawang;//20

        // if (msg_yaw >= setPoint2 && msg_yaw <= setPoint1) {
        //	robotDirection = true;
        // } else  { robotDirection = false; }

        // if (robotDirection) { //printf("arah imu sudah benar\n");
        // if (headTilt <= -0.7 || bawaBola >= 300) {
        //	bawaBola = 0;
        //	robotDirection = false;
        //	stateCondition = 5;
        // } bawaBola++;

        // Ball Positioning ======================================
        // if (posPan >= 0) { setPointFootY = 0.25;  }//kiri
        // else             { setPointFootY = -0.25; }//kanan
        // errorPosX = headPan - setPointFootY;

        setPointFootY1 = 0.2;       // 0.22
        setPointFootY2 = -0.2;      //-0.22
        errorPosY = headTilt + 0.5; // 0.04;//0.05;//0.08;

        // if(errorPosX >= -0.1 && errorPosX <= 0.1) {
        if (headPan <= setPointFootY1 && headPan >= setPointFootY2)
        { // -0.2 > x < 0.2
            // if (headTilt >= -0.8) {
            PxMoveBallPos = 0.3 * speed*-1;
            //} else {
            //	PxMoveBallPos = errorPosY*speed*-1;
            //}
            // Walk(PxMoveBallPos, 0.0, 0.0);
            jalanDirection(PxMoveBallPos, 0.0, gawang);
        }
        else
        { // x < -0.2 || x > 0.2
            if (headTilt >= pTiltTendangKanan)
            {
                PxMoveBallPos = errorPosY * -speed;
                // PxMoveBallPos = -0.02;//-0.03;
                // Walk(-0.03, 0.0, 0.0);
            }
            else
            {
                PxMoveBallPos = 0.0;
            }

            if (headPan > setPointFootY1)
            { // printf("kiri bos\n");
                PyMoveBallPos = (headPan - 0.1) * 0.06;
            }
            else if (headPan < setPointFootY2)
            { // printf("kanan bos\n");
                PyMoveBallPos = (headPan + 0.1) * 0.06;
            }

            // Walk(PxMoveBallPos, PyMoveBallPos, 0.0);
            jalanDirection(PxMoveBallPos, PyMoveBallPos, gawang);

            // if (headTilt >= (cSekarang - 0.2)) {
            //	Walk(-0.03, 0.0, 0.0);
            // } else {
            //	if (headPan > setPointFootY1) { //printf("kiri bos\n");
            //		PyMoveBallPos = (headPan - 0.1) * 0.06;
            //		Walk(0.0, PyMoveBallPos, 0.0);
            //	} else if (headPan < setPointFootY2) { //printf("kanan bos\n");
            //		PyMoveBallPos = (headPan + 0.1) * 0.06;
            //		Walk(0.0, PyMoveBallPos, 0.0);
            //	}
            // }

            // if(errorPosY >= -0.1) { // XmoveBackWard
            //	Walk(-0.03, 0.0, 0.0);
            // } else { // Ymove
            //	PyMoveBallPos = errorPosX*0.06;
            //	Walk(0.0, PyMoveBallPos, 0.0);
            // }
        }
        //} else { //printf("cari arah imu\n");
        //	bawaBola = 0;
        //	if (posTilt > -0.8 && posPan > -0.5 && posPan < 0.5) {//bola dekat
        //		Imu(gawang, cSekarang);
        //	} else {//bola masih jauh
        //		followBall(0);
        //	}
        //}
    }
*/
    // Checking Lost Goal ========================================================================
    int countGoalLost = 0,
        countGoalFound = 0,
        returnGoalVal;
    int goalLost(int threshold)
    {
        if (useVision)
        {
            if (Goal_X == -1 && Goal_Y == -1)
            {
                countGoalFound = 0;
                countGoalLost++;
                if (countGoalLost >= threshold)
                {
                    returnGoalVal = 1;
                }
            }
            else
            {
                if (headTilt < -1.0)
                {
                    countGoalLost = 0;
                    countGoalFound++;
                    if (countGoalFound > 1)
                    {
                        returnGoalVal = 0;
                    }
                }
            }
        }
        else
        {
            countGoalFound = 0;
            countGoalLost++;
            if (countGoalLost >= threshold)
            {
                returnGoalVal = 1;
            }
        }
        return returnGoalVal;
    }

    // Goal Tracking =============================================================================
    double intPanG = 0, dervPanG = 0, errorPanG = 0, preErrPanG = 0,
           PPanG = 0, IPanG = 0, DPanG = 0,
           intTiltG = 0, dervTiltG = 0, errorTiltG = 0, preErrTiltG = 0,
           PTiltG = 0, ITiltG = 0, DTiltG = 0,
           dtG = 0.04;
    double G_Pan_err_diff, G_Pan_err, G_Tilt_err_diff, G_Tilt_err, G_PanAngle, G_TiltAngle,
        pOffsetG, iOffsetG, dOffsetG,
        errorPanGRad, errorTiltGRad;
    int offsetSetPointGoal;
    void trackGoal()
    {
        if (useVision)
        {
            if (Goal_X != -1 && Goal_Y != -1)
            { // printf("Tracking");
                // mode 1 ######################################################################
                // PID pan ==========================================================
                /*errorPanG  = (double)Goal_X - (frame_X / 2);//160
                PPanG  = errorPanG  * 0.00010; //Tune in Kp Pan  0.00035 //kalau kepala msh goyang2, kurangin nilainya

                intPanG += errorPanG * dtG;
                IPanG = intPanG * 0.0;

                dervPanG = (errorPanG - preErrPanG) / dtG;
                DPanG = dervPanG * 0.00001;

                preErrPanG = errorPanG;

                //posPan += PPanG*-1; //dikali -1 kalau receler terbalik dalam pemasangan
                posPan += (PPanG + IPanG + DPanG) * -1;


                //PID tilt ==========================================================
                errorTiltG = (double)Goal_Y - (frame_Y / 2);//120
                PTiltG = errorTiltG * 0.00010; //Tune in Kp Tilt 0.00030

                intTiltG += errorTiltG * dtG;
                ITiltG = intTiltG * 0.0;

                dervTiltG = (errorTiltG - preErrTiltG) / dtG;
                DTiltG = dervTiltG * 0; //0.00001;

                preErrTiltG = errorTiltG;

                //posTilt += PTiltG;
                posTilt += (PTiltG + ITiltG + DTiltG);*/

                // mode 2 ######################################################################
                //  offsetSetPointGoal = (int)((posTilt * 30) + 54);
                //  if (offsetSetPointGoal > 36) offsetSetPointGoal = 36;
                //  else if (offsetSetPointGoal < 0) offsetSetPointGoal = 0;

                // errorPanG  = (double)Goal_X - ((frame_X / 2) + offsetSetPointGoal);//160
                errorPanG = (double)Goal_X - (frame_X / 2);
                errorTiltG = (double)Goal_Y - (frame_Y / 2); // 120

                errorPanG *= -1;
                errorTiltG *= -1;
                errorPanG *= (90 / (double)frame_X);     // pixel per msg_yaw
                errorTiltG *= (60 / (double)frame_Y);    // pixel per msg_yaw
                errorPanG *= (77.32 / (double)frame_X);  // pixel per msg_yaw
                errorTiltG *= (61.93 / (double)frame_Y); // pixel per msg_yaw

                errorPanGRad = (errorPanG * PI) / 180;
                errorTiltGRad = (errorTiltG * PI) / 180;
                // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanG, errorTiltG);
                // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanGRad, errorTiltGRad);
                // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

                G_Pan_err_diff = errorPanGRad - G_Pan_err;
                G_Tilt_err_diff = errorTiltGRad - G_Tilt_err;

                // PID pan ==========================================================
                // PPanG  = G_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                PPanG = G_Pan_err * goal_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                intPanG += G_Pan_err * dtG;
                IPanG = intPanG * 0.0;
                dervPanG = G_Pan_err_diff / dtG;
                // DPanG = dervPanG * kamera.panKD;
                DPanG = dervPanG * goal_panKD;
                G_Pan_err = errorPanGRad;
                posPan += (PPanG + IPanG + DPanG);

                // PID tilt ==========================================================
                // PTiltG = G_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
                PTiltG = G_Tilt_err * goal_tiltKP; // Tune in Kp Tilt 0.00030

                intTiltG += G_Tilt_err * dtG;
                ITiltG = intTiltG * 0.0;

                dervTiltG = G_Tilt_err_diff / dtG;
                // DTiltG = dervTiltG * kamera.tiltKD;
                DTiltG = dervTiltG * goal_tiltKD;

                preErrTiltG = errorTiltG;
                G_Tilt_err = errorTiltGRad;
                posTilt += (PTiltG + ITiltG + DTiltG) * -1;

                if (posPan >= 1.6)
                {
                    posPan = 1.6;
                }
                else if (posPan <= -1.6)
                {
                    posPan = -1.6;
                }
                if (posTilt <= -2.0)
                {
                    posTilt = -2.0;
                }
                else if (posTilt >= -0.4)
                {
                    posTilt = -0.4;
                }

                headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
            }
        }
    }
    
    // Follow Goal ===============================================================================
    int countReadyStop = 0;
    void followGoal(double Xwalk, double SetPan, int mode)
    { // 0 normal, 1 sambil belok
        errorfPan = posPan - SetPan;

        if (posTilt < -2.0 && posPan < 0.4 && posPan > -0.4 && Goal_X != -1 && Goal_Y != -1)
        { // Stop
            countReadyStop++;
        }
        else
        { // Follow
            countReadyStop = 0;
        }

        if (countReadyStop >= 5)
        {
            PxMove = 0.00;              // jalan ditempat
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.30; //0.045
        }
        else
        {
            PxMove = Xwalk;             // 0.08
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.35; //0.045
        }

        if (mode == 0)
        { // pake alfa
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            }
            else
            { // printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
        }
        else if (mode == 1)
        { // tanpa alfa
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("CCCCCCCC\n");
                Walk(PxMove, PyMove, 0.0);
            }
            else
            { // printf("DDDDDDDD\n");
                Walk(0.0, PyMove, 0.0);
            }
        }
    }

    // Body Tracking Goal ========================================================================
    double errorBodyPositionG,
        bodyP_ControllerG;
    int bodyTrueG = 0,
        delayTrueG = 0;
    int bodyTrackingGoal(int threshold)
    {
        //	trackGoal();

        errorBodyPositionG = 0 - headPan;
        bodyP_ControllerG = errorBodyPositionG * -0.5; //-0.5

        if (errorBodyPositionG >= -0.1 && errorBodyPositionG <= 0.1)
        { // untuk hasil hadap 0.8
            // motion("0");
            Walk(0.0, 0.0, 0.0);
            delayTrueG++;
        }
        else
        {
            trackGoal();

            bodyTrueG =
                delayTrueG = 0;

            if (bodyP_ControllerG < 0)
            { // kanan
                // Walk(rotateGoal_x, abs(rotateGoal_y), -abs(rotateGoal_a));
                Walk(rotateGoal_x, rotateGoal_y, -rotateGoal_a);
            }
            else
            { // kiri
                // Walk(rotateGoal_x, -abs(rotateGoal_y), abs(rotateGoal_a));
                Walk(rotateGoal_x, -rotateGoal_y, rotateGoal_a);
            }
        }

        if (delayTrueG >= threshold)
        {
            bodyTrueG = 1;
        }
        else
        {
            bodyTrueG = 0;
        }
        return bodyTrueG;
    }

    // Search Goal =====================================================================================================
    double goalPan = 0;
    void saveGoalLocation()
    {
        //	trackGoal();
        goalPan = headPan;
    }

    // Landmark Tracking =============================================================================
    double intPanL = 0, dervPanL = 0, errorPanL = 0, preErrPanL = 0,
           PPanL = 0, IPanL = 0, DPanL = 0,
           intTiltL = 0, dervTiltL = 0, errorTiltL = 0, preErrTiltL = 0,
           PTiltL = 0, ITiltL = 0, DTiltL = 0,
           dtL = 0.04;
    double L_Pan_err_diff, L_Pan_err, L_Tilt_err_diff, L_Tilt_err,
        errorPanLRad, errorTiltLRad;
    int offsetSetPointLand;
    void trackLand()
    {
        if (useVision)
        {
            if (Xcross_LX != -1 && Xcross_LY != -1)
            { // printf("Tracking");
                // mode 1 ######################################################################
                // PID pan ==========================================================
                /*errorPanL  = (double)Goal_X - (frame_X / 2);//160
                PPanL  = errorPanL  * 0.00010; //Tune in Kp Pan  0.00035 //kalau kepala msh goyang2, kurangin nilainya

                intPanL += errorPanL * dtL;
                IPanL = intPanL * 0.0;

                dervPanL = (errorPanL - preErrPanL) / dtL;
                DPanL = dervPanL * 0.00001;

                preErrPanL = errorPanL;

                //posPan += PPanL*-1; //dikali -1 kalau receler terbalik dalam pemasangan
                posPan += (PPanL + IPanL + DPanL) * -1;


                //PID tilt ==========================================================
                errorTiltL = (double)Goal_Y - (frame_Y / 2);//120
                PTiltL = errorTiltL * 0.00010; //Tune in Kp Tilt 0.00030

                intTiltL += errorTiltL * dtL;
                ITiltL = intTiltL * 0.0;

                dervTiltL = (errorTiltL - preErrTiltL) / dtL;
                DTiltL = dervTiltL * 0; //0.00001;

                preErrTiltL = errorTiltL;

                //posTilt += PTiltL;
                posTilt += (PTiltL + ITiltL + DTiltL);*/

                // mode 2 ######################################################################
                offsetSetPointLand = (int)((posTilt * 30) + 54);
                if (offsetSetPointLand > 36)
                    offsetSetPointLand = 36;
                else if (offsetSetPointLand < 0)
                    offsetSetPointLand = 0;

                if ((Xcross_LX != -1 && Xcross_LY != -1) && (Xcross_RX != -1 && Xcross_RY != -1))
                {
                    errorPanL = (double)Xcross_RX - ((frame_X / 2) + offsetSetPointLand); // 160
                    errorTiltL = (double)Xcross_RY - (frame_Y / 2);                       // 120
                }
                else
                {
                    errorPanL = (double)Xcross_LX - ((frame_X / 2) + offsetSetPointLand); // 160
                    errorTiltL = (double)Xcross_LY - (frame_Y / 2);                       // 120
                }

                errorPanL *= -1;
                errorTiltL *= -1;
                errorPanL *= (90 / (double)frame_X);     // pixel per msg_yaw
                errorTiltL *= (60 / (double)frame_Y);    // pixel per msg_yaw
                errorPanL *= (77.32 / (double)frame_X);  // pixel per msg_yaw
                errorTiltL *= (61.93 / (double)frame_Y); // pixel per msg_yaw

                errorPanLRad = (errorPanL * PI) / 180;
                errorTiltLRad = (errorTiltL * PI) / 180;
                // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanL, errorTiltL);
                // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanLRad, errorTiltLRad);
                // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

                L_Pan_err_diff = errorPanLRad - L_Pan_err;
                L_Tilt_err_diff = errorTiltLRad - L_Tilt_err;

                // PID pan ==========================================================
                // PPanL  = L_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                PPanL = L_Pan_err * goal_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                intPanL += L_Pan_err * dtL;
                IPanL = intPanL * 0.0;
                dervPanL = L_Pan_err_diff / dtL;
                // DPanL = dervPanL * kamera.panKD;
                DPanL = dervPanL * goal_panKD;
                L_Pan_err = errorPanLRad;
                posPan += (PPanL + IPanL + DPanL);

                // PID tilt ==========================================================
                // PTiltL = L_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
                PTiltL = L_Tilt_err * goal_tiltKP; // Tune in Kp Tilt 0.00030

                intTiltL += L_Tilt_err * dtL;
                ITiltL = intTiltL * 0.0;

                dervTiltL = L_Tilt_err_diff / dtL;
                // DTiltL = dervTiltL * kamera.tiltKD;
                DTiltL = dervTiltL * goal_tiltKD;

                preErrTiltL = errorTiltL;
                L_Tilt_err = errorTiltLRad;
                posTilt += (PTiltL + ITiltL + DTiltL) * -1;

                if (posPan >= 1.6)
                {
                    posPan = 1.6;
                }
                else if (posPan <= -1.6)
                {
                    posPan = -1.6;
                }
                if (posTilt <= -2.0)
                {
                    posTilt = -2.0;
                }
                else if (posTilt >= -0.4)
                {
                    posTilt = -0.4;
                }

                headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
            }
        }
    }

    // Follow Landmark ===============================================================================
    int countReadyStopL = 0;
    void followLand(double Xwalk, double SetPan, int mode)
    { // 0 normal, 1 sambil belok
        errorfPan = posPan - SetPan;

        if (posTilt < -2.0 && posPan < 0.4 && posPan > -0.4 && Xcross_LX != -1 && Xcross_LY != -1)
        { // Stop
            countReadyStopL++;
        }
        else
        { // Follow
            countReadyStopL = 0;
        }

        if (countReadyStopL >= 5)
        {
            PxMove = 0.00;              // jalan ditempat
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.30; //0.045
        }
        else
        {
            PxMove = Xwalk;             // 0.08
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.35; //0.045
        }

        if (mode == 0)
        { // pake alfa
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            }
            else
            { // printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
        }
        else if (mode == 1)
        { // tanpa alfa
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("CCCCCCCC\n");
                Walk(PxMove, PyMove, 0.0);
            }
            else
            { // printf("DDDDDDDD\n");
                Walk(0.0, PyMove, 0.0);
            }
        }
    }

    void saveSudutImu()
    {
        //	sudut();
        saveAngle = msg_yaw;
    }

    int prediksiGoalPan = 0;
    double Rotate = 0;
    void prediksiArahGoal()
    {
        prediksiGoalPan = (int)(57.29 * headPan);
        Rotate = headPan * (8 / 1.57); // 90 derajat = 8 detik //waktu rotate
    }

    int count = 0;
    bool goalSearch = false,
         searchGoalFinish = false;
    void panSearchGoal(double arah)
    { // printf("  panSearchBall\n\n");
        if (panRate < 0)
        {
            panRate = -panRate;
        }

        if (arah < 0)
        {
            headPan += panRate;
            if (headPan >= batasKiri)
            {
                prediksiGoalPan = 0;
                saveAngle = 0;
                Rotate = 0;
                goalSearch = true;
            }
        }
        else
        {
            headPan -= panRate;
            if (headPan <= batasKanan)
            {
                prediksiGoalPan = 0;
                saveAngle = 0;
                Rotate = 0;
                goalSearch = true;
            }
        }

        if (headPan >= batasKiri)
        {
            headPan = batasKiri;
        }
        else if (headPan <= batasKanan)
        {
            headPan = batasKanan;
        }

        headMove(headPan, -2.0); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    // +predictPanGoal = kiri
    // -predictPanGoal = kanan

    //	A+ = putar kiri
    //	A- = putar kanan
    double predictGoalPan;
    void predictGoal(double alpha, double tilt)
    {
        predictGoalPan = alpha / 57.29; // sudut * nilai per satu sudut(nilai servo)

        if (predictGoalPan <= -1.6)
        {
            predictGoalPan = -1.6;
        }
        else if (predictGoalPan >= 1.6)
        {
            predictGoalPan = 1.6;
        }
        // printf("  predict = %f\n\n",predictGoalPan);

        headMove(predictGoalPan, tilt);
    }

    void predictGoalTeam(double alpha, double tilt)
    {
        int setPointPan1, setPointPan2,
            btsSetPointPan, btsRotatePan;
        double putarPan;

        if (alpha > 180)
        {
            alpha = 180;
        }
        else if (alpha < -180)
        {
            alpha = -180;
        }

        setPointPan1 = 5 + alpha;
        setPointPan2 = -5 + alpha;

        if (setPointPan1 > 180 || setPointPan2 < -180)
        { // jika arah imu dibelakang
            if (setPointPan1 > 180)
            { // nilai setPointPan1 diubah jd negatif
                btsSetPointPan = setPointPan1 - 360;

                if (msg_yaw >= setPointPan2 || msg_yaw <= btsSetPointPan)
                { // misal 170 ke -170
                    predictGoalPan = 0.00;
                }
                else
                {
                    btsRotatePan = alpha - 180;
                    if ((msg_yaw <= alpha) && (msg_yaw >= btsRotatePan))
                    { // misal di range 0 - 180, maka putarPan kanan
                        putarPan = (alpha - msg_yaw) / 57.29;
                        predictGoalPan = -putarPan;
                    }
                    else
                    { // putarPan kiri
                        if (msg_yaw > alpha)
                        {
                            putarPan = (msg_yaw - alpha) / 57.29;
                        } // 0.0033
                        else
                        {
                            putarPan = ((180 - alpha) + (180 + msg_yaw)) / 57.29;
                        } // 0.0033
                        predictGoalPan = putarPan;
                    }
                }
            }
            else
            { // nilai setPointPan2 diubah jadi positif
                btsSetPointPan = setPointPan2 + 360;

                if (msg_yaw >= btsSetPointPan || msg_yaw <= setPointPan1)
                {
                    predictGoalPan = 0.00;
                }
                else
                {
                    btsRotatePan = alpha + 180;
                    if ((msg_yaw >= alpha) && (msg_yaw <= btsRotatePan))
                    {                                            // misal di range -180 - 0, maka putarPan kiri
                        putarPan = abs(alpha - msg_yaw) / 57.29; // 0.0033
                        predictGoalPan = putarPan;
                    }
                    else
                    { // putarPan kanan
                        if (msg_yaw < alpha)
                        {
                            putarPan = (alpha - msg_yaw) / 57.29;
                        } // 0.0033
                        else
                        {
                            putarPan = ((180 + alpha) + (180 - msg_yaw)) / 57.29;
                        } // 0.0033
                        predictGoalPan = -putarPan;
                    }
                }
            }
        }
        else
        { // arah imu kedepan
            if (msg_yaw >= setPointPan2 && msg_yaw <= setPointPan1)
            {
                predictGoalPan = 0.00;
            }
            else
            {
                if (alpha >= 0)
                {
                    btsRotatePan = alpha - 180;
                    if ((msg_yaw <= alpha) && (msg_yaw >= btsRotatePan))
                    {                                         // putarPan kanan
                        putarPan = (alpha - msg_yaw) / 57.29; // 0.0033
                        predictGoalPan = -putarPan;
                    }
                    else
                    { // putarPan kiri
                        if (msg_yaw > alpha)
                        {
                            putarPan = (msg_yaw - alpha) / 57.29;
                        } // 0.0033
                        else
                        {
                            putarPan = ((180 - alpha) + (180 + msg_yaw)) / 57.29;
                        } // 0.0033
                        predictGoalPan = putarPan;
                    }
                }
                else
                {
                    btsRotatePan = alpha + 180;
                    if ((msg_yaw >= alpha) && (msg_yaw <= btsRotatePan))
                    { // maka putarPan kiri
                        putarPan = abs(alpha - msg_yaw) / 57.29;
                        ; // 0.0033
                        predictGoalPan = putarPan;
                    }
                    else
                    { // putarPan kanan
                        if (msg_yaw < alpha)
                        {
                            putarPan = (alpha - msg_yaw) / 57.29;
                        } // 0.0033
                        else
                        {
                            putarPan = ((180 + alpha) + (180 - msg_yaw)) / 57.29;
                        } // 0.0033
                        predictGoalPan = -putarPan;
                    }
                }
            }
        }

        if (predictGoalPan <= -1.6)
        {
            predictGoalPan = -1.6;
        }
        else if (predictGoalPan >= 1.6)
        {
            predictGoalPan = 1.6;
        }

        headMove(predictGoalPan, tilt);
    }

    int goalSide = 0;
    double arahPandang = 0;
    int cekArah()
    {
        arahPandang = msg_yaw - (57.29 * headPan);
        if (arahPandang >= -90 && arahPandang <= 90)
        { // gawang lawan
            goalSide = 0;
        }
        else
        { // gawang sendiri
            goalSide = 1;
        }
        return goalSide;
    }

    // Tendang ===================================================================================
    bool tendang = false;
    double panTengah = (((pPanTendangKanan) - (pPanTendangKiri)) / 2) + (pPanTendangKiri);
    /*
    void kick(int mode)
    {
        //	trackBall();
        //if (mode == 1 || mode == 2 || mode == 3 || mode == 4 || mode == 33 || mode == 44 || mode == 5 || mode == 6 || mode == 7)
        
        // tendang samping--
        if (mode == 3 || mode == 4 || mode == 33 || mode == 44 || mode == 5 || mode == 6 || mode == 7)
        {
            if (mode == 1 || mode == 3 || mode == 33 || mode == 5 || mode == 7 || mode == 88 || mode == 99)
            {
                kanan = true;
                kiri = false;
                tengah = false;
            } // arah kanan
            else if (mode == 2 || mode == 4 || mode == 44 || mode == 6)
            {
                kiri = true;
                kanan = false;
                tengah = false;
            } // arah kiri
        }
       /* else if(mode == 5 || mode == 6)
        {
        	kiri = false;
            kanan = false;
            tengah = true;
        }
        else
        {
            if (forceKanan && forceKiri)
            {
                if (posPan > panTengah) //= 0 && kanan == false && kiri == false)
                { // kiri
                    kiri = true;
                    kanan = false;
                    tengah = false;
                }
                else //if (posPan < = 0 && kanan == false && kiri == false)
                // else
                { // kanan
                    kanan = true;
                    kiri = false;
                    tengah = false;
                }
            } else
            {
                if (forceKanan)
                {
                    kanan = true;
                    kiri = false;
                    tengah = false;
                } else if (forceKiri)
                {
                    kiri = true;
                    kanan = false;
                    tengah = false;
                }
            }
        }

        if (tendang)
        {
            kanan = kiri = tengah = false;
        }
        else
        {
		if (kiri)
		{ // kiri
		    // if (posPan >= 0) { //kiri
		    if (ballPos)
		    { // printf("ball pos left true\n");
		        // motion("0");
		        if (mode == 1 || mode == 2)
		        {
		            motion("0");
		            sleep(1);
		            motion("1"); //longleft
		            tendang = true;
		        }
		        else if (mode == 3 || mode == 4)
		        {
		            motion("0");
		            sleep(1); // 10
		            motion("4"); //90kiri
		            tendang = true;
		        }
		        else if (mode == 33 || mode == 44)
		        {
		            motion("0");
		            sleep(1); // 10
		            motion("@");
		            tendang = true;
		        } 
		        else if (mode == 5 || mode == 6)
		        {
		            motion("0");
		            sleep(1);
		            motion("6"); //45kiri
		            tendang = true;
		        } 
		    }
		    else
		    {
		        ballPositioning(pPanTendangKiri, pTiltTendangKiri, ballPositioningSpeed); // 0.15
		    }
		}
		else if (kanan)
		{ // kanan
		    //} else { //kanan
		    if (ballPos)
		    { // printf("ball pos right true\n");
		        // motion("0");
		        if (mode == 1 || mode == 2)
		        {
		            motion("0");
		            sleep(1); // 7
		            motion("2"); //long right
		            tendang = true;
		        }
		        else if (mode == 3 || mode == 4)
		        {
		            motion("0");
		            sleep(1); // 10
		            motion("3"); //90kanan
		            tendang = true;
		        }
		        else if (mode == 33 || mode == 44)
		        {
		            motion("0");
		            sleep(1); // 10
		            motion("!"); 
		            tendang = true;
		        }
		        else if (mode == 5 || mode == 6)
		        {
		            motion("0");
		            sleep(1); // 7
		            motion("5"); //45kanan
		            tendang = true;
		        }
		        else if (mode == 7)
		        {
		            motion("0");
		            sleep(1); // 7
		            motion("7"); //short right
		            tendang = true;
		        }
		        else if (mode == 88)
		        {
		            motion("0");
		            sleep(1); // 7
		            motion("@"); //meter
		            tendang = true;
		        }
		        else if (mode == 99)
		        {
		            motion("0");
		            sleep(1); // 7
		            motion("#"); //high kick
		            tendang = true;
		        }
		    }
		    else
		    {
		        ballPositioning(pPanTendangKanan, pTiltTendangKanan, ballPositioningSpeed); // 0.15
		    }
		} else if (tengah)
		{
				if (ballPos)
				{
					if (mode == 5)
					{
						motion("0");
						sleep(1);
						motion("2");
						tendang = true;
					} else if (mode == 6)
					{
						motion("0");
						sleep(1);
						motion("6");
						tendang = true;
					}
				} else 
				{
					ballPositioning(-0.05, pTiltTendangKanan, ballPositioningSpeed); // 0.15
				}        
		}
	}
    }
*/
    void kick(int mode) {

    // ── A. TENTUKAN KAKI ──────────────────────────────────────────────────
    const double HYST        = 0.12;  // zona hysteresis pan
    const int    LOCK_FRAMES = 25;    // frame sebelum boleh ganti kaki

    if (mode == 3 || mode == 4 || mode == 33 || mode == 44 ||
        mode == 5 || mode == 6 || mode == 7)
    {
        // Mode eksplisit → kaki dari mode
        bool wantKanan = (mode==1||mode==3||mode==33||mode==5||mode==7||mode==88||mode==99);
        bool wantKiri  = (mode==2||mode==4||mode==44||mode==6);

        if (wantKanan && !kanan) {
            kanan=true; kiri=false; tengah=false;
            _resetBallPosState();
        } else if (wantKiri && !kiri) {
            kiri=true; kanan=false; tengah=false;
            _resetBallPosState();
        }
    }
    else
    {
        // Mode auto → pilih kaki dari posisi headPan dengan hysteresis
        if (forceKanan && forceKiri)
        {
            // Auto-select dengan lock counter
            if (_footLock > 0) {
                _footLock--;  // pertahankan kaki saat ini
            } else {
                bool wantKanan = (headPan < -HYST);
                bool wantKiri  = (headPan >  HYST);
                // zona tengah (-HYST..+HYST): tidak ganti kaki

                if (wantKanan && !kanan) {
                    kanan=true; kiri=false; tengah=false;
                    _footLock = LOCK_FRAMES;
                    _resetBallPosState();
                    printf("[Kick] AUTO → KANAN (pan=%.3f)\n", headPan);
                } else if (wantKiri && !kiri) {
                    kiri=true; kanan=false; tengah=false;
                    _footLock = LOCK_FRAMES;
                    _resetBallPosState();
                    printf("[Kick] AUTO → KIRI (pan=%.3f)\n", headPan);
                }
                // jika masih di zona tengah → pertahankan pilihan sebelumnya
            }
        }
        else if (forceKanan && !kanan) {
            kanan=true; kiri=false; tengah=false;
            _resetBallPosState();
        }
        else if (forceKiri && !kiri) {
            kiri=true; kanan=false; tengah=false;
            _resetBallPosState();
        }
    }

    if (tendang) { kanan=kiri=tengah=false; return; }

    // ── B. EKSEKUSI ───────────────────────────────────────────────────────
    if (kiri) {
        if (ballPos) {
            motion("0"); sleep(1);
            if      (mode==1||mode==2)    motion("1");
            else if (mode==3||mode==4)    motion("4");
            else if (mode==33||mode==44)  motion("@");
            else if (mode==5||mode==6)    motion("6");
            else                          motion("1");
            tendang = true;
        } else {
            ballPositioning(pPanTendangKiri, pTiltTendangKiri, ballPositioningSpeed);
            //dribble(arahGoal, ballPositioningSpeed);
        }

    } else if (kanan) {
        if (ballPos) {
            motion("0"); sleep(1);
            if      (mode==1||mode==2)    motion("2");
            else if (mode==3||mode==4)    motion("3");
            else if (mode==33||mode==44)  motion("!");
            else if (mode==5||mode==6)    motion("5");
            else if (mode==7)             motion("7");
            else if (mode==88)            motion("@");
            else if (mode==99)            motion("#");
            else                          motion("2");
            tendang = true;
        } else {
            ballPositioning(pPanTendangKanan, pTiltTendangKanan, ballPositioningSpeed);
        }

    } else if (tengah) {
        if (ballPos) {
            motion("0"); sleep(1);
            if      (mode==5) motion("2");
            else if (mode==6) motion("6");
            tendang = true;
        } else {
            ballPositioning(-0.05, pTiltTendangKanan, ballPositioningSpeed);
        }
    }
}

    void kickNoSudut(int mode) {
    //trackBall();
        if (headTilt >= -1.5) {
		if (robotDirection && headPan >= -0.2 && headPan <= 0.6) {
				if (ballPos) { //printf("ball pos left true\n");
					motion("0");
					if (mode == 1 || mode == 2) {
						usleep(900000); //6
						motion("2");
					} else if (mode == 3 || mode == 4) {
						usleep(300000); //10
						sleep(1); //10
						motion("4");
						//motion("4");
					} else if (mode == 5 || mode == 6) {
						usleep(850000);
						motion("6");
					}
					tendang = true;
				} else {
					ballPositioning(pPanTendangKanan, pTiltTendangKanan, ballPositioningSpeed); //0.15
				}			
		} else {
			if (headTilt >= (cAktif  + 0.2) && headPan >= -0.2 && headPan <= 0.6) {
				//Imu(sudut, cSekarang - 0.20);
				robotDirection = true;
			} else {
				robotDirection = false;
				followBall(0);
			}
			
		}
	} else {
		if	(posTilt >= SetPointTilt) { posTilt = SetPointTilt; }
		else if	(posTilt < -2.0) { posTilt = -2.0; }

		errorfPan  = posPan - SetPointPan;
		errorfTilt = posTilt - SetPointTilt;

		if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && Ball_X != -1 && Ball_Y != -1) { //Stop(bola sudah dekat)
			PxMove = 0.0; //jalan ditempat
			PyMove = errorfPan * 0.040; //0.045
			PaMove = errorfPan * 0.20; //0.30; //0.045
		} else { //Kejar Bola(bola masih jauh)
			PxMove = kejarMax; //0.06
			PyMove = errorfPan * 0.045; //0.045
			PaMove = errorfPan * 0.25; //0.35; //0.045
		}

		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("AAAAAAAA\n");
			Walk(PxMove, 0.0, PaMove);
		} else { 					//printf("BBBBBBBB\n");
			Walk(0.0, 0.0, PaMove);
		}
		//followBall(0);
	}
}

    void rotateKickOffImu(int sudut, int mode)
    {
        if (headTilt >= cSekarang)
        {
            if (robotDirection && headPan >= -0.1 && headPan <= 0.6)
            {
                if (sudut >= 0)
                { // kiri
                    if (ballPos)
                    { // printf("ball pos left true\n");
                        motion("0");
                        if (mode == 1 || mode == 2)
                        {
                            sleep(1); // 6
                            motion("1");
                        }
                        else if (mode == 3 || mode == 4)
                        {
                            sleep(1); // 10
                            motion("0");
                        }
                        else if (mode == 5 || mode == 6)
                        {
                            sleep(1);
                            motion("5");
                        }
                        tendang = true;
                    }
                    else
                    {
                        ballPositioning(pPanTendangKiri, pTiltTendangKiri, ballPositioningSpeed); // 0.15
                        //dribble(arahGoal, ballPositioningSpeed);
                    }
                }
                else
                { // kanan
                    if (ballPos)
                    { // printf("ball pos right true\n");
                        motion("0");
                        if (mode == 1 || mode == 2)
                        {
                            sleep(1); // 7
                            motion("2");
                        }
                        else if (mode == 3 || mode == 4)
                        {
                            sleep(1); // 10
                            motion("0");
                        }
                        else if (mode == 5 || mode == 6)
                        {
                            sleep(1); // 7
                            motion("6");
                        }
                        tendang = true;
                    }
                    else
                    {
                        ballPositioning(pPanTendangKanan, pTiltTendangKanan, ballPositioningSpeed); // 0.15
                    }
                }
            }
            else
            {
                if (headTilt >= cAktif && headPan >= -0.1 && headPan <= 0.6)
                {                          // +
                    Imu(sudut, cSekarang); //- 0.20
                }
                else
                {
                    robotDirection = false;
                    followBall(0);
                }
            }
        }
        else
        {
            if (posTilt >= SetPointTilt)
            {
                posTilt = SetPointTilt;
            }
            else if (posTilt < -2.0)
            {
                posTilt = -2.0;
            }

            errorfPan = posPan - SetPointPan;
            errorfTilt = posTilt - SetPointTilt;

            if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && Ball_X != -1 && Ball_Y != -1)
            {                               // Stop(bola sudah dekat)
                PxMove = 0.0;               // jalan ditempat
                PyMove = errorfPan * 0.040; // 0.045
                PaMove = errorfPan * 0.20;  // 0.30; //0.045
            }
            else
            {                              // Kejar Bola(bola masih jauh)
                PxMove = kejarMax;         // 0.06
                PyMove = errorfPan * 0.45; // 0.045
                PaMove = errorfPan * 0.25; // 0.35; //0.045
            }

            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            }
            else
            { // printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
            // followBall(0);
        }
    }

    void rotateKickOff(double timeRotate, int mode)
    {
        trackBall();

        // if (headTilt >= -1.0) {
        if (headTilt >= -0.8 && headPan >= -0.1 && headPan <= 0.6)
        {
            if (reset > 5)
            { // printf("set......................!!!\n");
                cekWaktu(abs(timeRotate));
                if (timer)
                { // printf("true......................!!!\n");
                    if (posPan >= 0)
                    { // kiri
                        if (ballPos)
                        { // printf("ball pos left true\n");
                            motion("0");
                            if (mode == 1 || mode == 2)
                            {
                                sleep(2); // 6
                                motion("1");
                            }
                            else if (mode == 3 || mode == 4)
                            {
                                // usleep(1000000); //10
                                sleep(2); // 10
                                motion("3");
                                // motion("4");
                            }
                            else if (mode == 5 || mode == 6)
                            {
                                sleep(2);
                                motion("5");
                            }
                            tendang = true;
                        }
                        else
                        {
                            ballPositioning(pPanTendangKiri, pTiltTendangKiri, ballPositioningSpeed); // 0.15
                            //dribble(arahGoal, ballPositioningSpeed);
                        }
                    }
                    else
                    { // kanan
                        if (ballPos)
                        { // printf("ball pos right true\n");
                            motion("0");
                            if (mode == 1 || mode == 2)
                            {
                                sleep(2); // 7
                                motion("2");
                            }
                            else if (mode == 3 || mode == 4)
                            {
                                // usleep(1000000); //10
                                sleep(2); // 10
                                // motion("4");
                                motion("4");
                            }
                            else if (mode == 5 || mode == 6)
                            {
                                sleep(2); // 7
                                motion("6");
                            }
                            tendang = true;
                        }
                        else
                        {
                            ballPositioning(pPanTendangKanan, pTiltTendangKanan, ballPositioningSpeed); // 0.15
                        }
                    }
                }
                else
                {
                    // rotateParabolic(timeRotate, cSekarang);
                    if (timeRotate < 0)
                    { // kanan
                        Walk(rotateGoal_x, rotateGoal_y, -rotateGoal_a);
                    }
                    else
                    { // kiri
                        Walk(rotateGoal_x, -rotateGoal_y, rotateGoal_a);
                    }
                }
            }
            else
            { // printf("cek......................!!!\n");
                Walk(0.0, 0.0, 0.0);
                setWaktu();
                reset++;
            }
        }
        else
        {
            reset = 0;
            followBall(0);
        }
    }

    /* int convertGridX(int valueGrid, int valueOffSetX)
    { // Konvert grid posisi robot jadi nilai koordinat x
        int tempCoorX, tempGridtoX;

        if (valueGrid % 6 == 0)
        {
            // printf("POPO\n");
            tempGridtoX = valueGrid / 6;
        }
        else
        {
            // printf("PIPI\n");
            tempGridtoX = (valueGrid / 6) + 1;
        }

        if (tempGridtoX <= 1)
        {
            tempGridtoX = 1;
        }
        else if (tempGridtoX >= 9)
        {
            tempGridtoX = 9;
        }

        tempCoorX = (((tempGridtoX * 100) - 500) + valueOffSetX);

        return tempCoorX;
    }

    int convertGridY(int valueGrid, int valueOffSetY)
    { // Konvert grid posisi robot jadi nilai koordinat y
        int tempCoorY, tempGridtoY;
        if (valueGrid % 6 == 0)
        {
            // printf("POPO\n");
            tempGridtoY = 6;
        }
        else
        {
            // printf("PIPI\n");
            tempGridtoY = (valueGrid - ((valueGrid / 6) * 6));
        }

        if (tempGridtoY <= 1)
        {
            tempGridtoY = 1;
        }
        else if (tempGridtoY >= 6)
        {
            tempGridtoY = 6;
        }

        tempCoorY = (((tempGridtoY * 100) - 350) + valueOffSetY);

        return tempCoorY;
    } */

    int tempCoorX, tempGridtoX;
    int tempCoorY, tempGridtoY;

    int convertGridX(int valueGrid, int valueOffSetX) {
        //convert Grid To Koordinat X
        tempGridtoX = (valueGrid / 6) + 1;
        if(tempGridtoX <= 1){
            tempGridtoX = 1;
        }
        else if(tempGridtoX >= 9){
            tempGridtoX = 9;
        }
        tempCoorX = ((((int)(tempGridtoX)*100)-500))+valueOffSetX;
        return tempCoorX;
    }

    int convertGridY(int valueGrid, int valueOffsetY)
    {
        //convert Grid To Koordinat Y
        if(valueGrid == 2 || valueGrid == 8 || valueGrid == 14 || valueGrid == 20 || valueGrid == 26 || valueGrid == 32 || valueGrid == 38 || valueGrid == 44 || valueGrid == 50){
            tempGridtoY = 3;
            if (valueOffsetY <= -300)
            {
                valueOffsetY = -300;
            } else if (valueOffsetY >= 200)
            {
                valueOffsetY = 200;
            } 
        }
        else if(valueGrid == 3 || valueGrid == 9 || valueGrid == 15 || valueGrid == 21 || valueGrid == 27 || valueGrid == 33 || valueGrid == 39 || valueGrid == 45 || valueGrid == 51){
            tempGridtoY = 4;
            if (valueOffsetY <= -200)
            {
                valueOffsetY = -200;
            } else if (valueOffsetY >= 300)
            {
                valueOffsetY = 300;
            }
        }
        else if(valueGrid == 1 || valueGrid == 7 || valueGrid == 13 || valueGrid == 19 || valueGrid == 25 || valueGrid == 31 || valueGrid == 37 || valueGrid == 43 || valueGrid == 49){
            tempGridtoY = 2;
            if (valueOffsetY <= -400)
            {
                valueOffsetY = -400;
            } else if (valueOffsetY >= 100)
            {
                valueOffsetY = 100;
            } 
        }
        else if(valueGrid == 4 || valueGrid == 10 || valueGrid == 16 || valueGrid == 22 || valueGrid == 28 || valueGrid == 34 || valueGrid == 40 || valueGrid == 46 || valueGrid == 52){
            tempGridtoY = 5;
            if (valueOffsetY <= -100)
            {
                valueOffsetY = -100;
            } else if (valueOffsetY >= 400)
            {
                valueOffsetY = 400;
            }
        }
        else if(valueGrid == 0 || valueGrid == 6 || valueGrid == 12 || valueGrid == 18 || valueGrid == 24 || valueGrid == 30 || valueGrid == 36 || valueGrid == 42 || valueGrid == 48){
            tempGridtoY = 1;
            if (valueOffsetY >= 0)
            {
                valueOffsetY = 0;
            }
        }
        else{
            tempGridtoY = 6;
            if (valueOffsetY <= 0)
            {
                valueOffsetY = 0;
            }
        }
        tempCoorY = (-(((int)(tempGridtoY)*100)-350))+valueOffsetY;
        return tempCoorY;
    }

    bool doneMoved = false,
         setGrid1 = false,
         setGrid2 = false;

    int countMoveGrid1 = 0, //count walk ditempat
        countMoveGrid2 = 0, //count rotate
        countMoveGrid3 = 0; //count walk x,y

    double rotateMoveGrid = 0;
    bool udah = false;

    void moveGrid(int valueGrid, int valueOffSetX, int valueOffSetY)
    { //Bergerak menuju grid yang ditentukan
        double c, s, sn, x, y, r, rotate, speedX, speedY, speedrX, speedrY, nilaiSudut;

        c = cos(msg_yaw);
        s = sin(msg_yaw);
        sn = sin(msg_yaw) * -1;

        if (msg_yaw < 0)
        {
            nilaiSudut = msg_yaw + 360;
        }
        else
        {
            nilaiSudut = msg_yaw;
        }
        // printf("robotPos = %f, %f\n", robotPos_X, robotPos_Y);
        // printf("Nilai Sudut = %.2lf\n", nilaiSudut);
        x = robotPos_X - convertGridX(valueGrid, valueOffSetX); //-240--240	= 0
        y = robotPos_Y - convertGridY(valueGrid, valueOffSetY); //-320--0	= -320
        r = sqrt((x * x) + (y * y));
        // printf("NILAI R = %.2lf\n",r);
        //printf("ADA APA\n");
        if (robotPos_X >= convertGridX(valueGrid, valueOffSetX))
        { //Target ada dibelakang
            // printf("MASUK HAHA\n");
            if (robotPos_Y >= convertGridY(valueGrid, valueOffSetY))
            { //saat ini sebelah kanan
                rotate = -180 + asin(y / r) * (180 / PI);
            }
            else if (robotPos_Y < convertGridY(valueGrid, valueOffSetY))
            { //saat ini sebelah kiri
                rotate = 180 + asin(y / r) * (180 / PI);
            }
        }
        else if (robotPos_X < convertGridX(valueGrid, valueOffSetX))
        { //Target ada didepan
            // printf("MASUK HIHI\n");
            if (robotPos_Y < convertGridY(valueGrid, valueOffSetY))
            { //saat ini sebelah kiri
                //printf("MASUK HUHU\n");
                rotate = 0 - asin(y / r) * (180 / PI);
                //printf("%.2lf\n",valueRotateBody);
            }
            else if (robotPos_Y >= convertGridY(valueGrid, valueOffSetY))
            { //saat ini sebalah kanan
                //printf("MASUK HOHO\n");
                rotate = 0 - asin(y / r) * (180 / PI);
            }
        }
        rotateMoveGrid = rotate;
        // printf("rotate move grid = %.2lf\n", rotateMoveGrid);
        if (robotPos_X >= (convertGridX(valueGrid, valueOffSetX) - 15) && robotPos_X < (convertGridX(valueGrid, valueOffSetX) + 15) &&
            robotPos_Y >= (convertGridY(valueGrid, valueOffSetY) - 15) && robotPos_Y < (convertGridY(valueGrid, valueOffSetY) + 15))
        {
            countMoveGrid1 =
                countMoveGrid2 =
                    countMoveGrid3 = 0;

            posRotateNew =
                setGrid1 =
                    setGrid2 = false;
            doneMoved = true;
        }
        else
        {
            if (countMoveGrid1 >= 5)
            {
                //RY < TY => RY = -30 TY = -25 DY = -30
                if (r < 30)
                {
                    //printf("TIME TO SHOWWWWWW>>>>>> \n");
                    if (countMoveGrid2 >= 5)
                    {
                        if (nilaiSudut > 270 || nilaiSudut <= 90)
                        {
                            //printf("MASUK 0 0 0 0\n");
                            if (posRotateNew)
                            {
                                //printf("SELESAI ROTATE 0\n");
                                if (countMoveGrid3 >= 5)
                                {
                                    if (!setGrid1 && !setGrid2)
                                    {
                                        if (robotPos_X >= convertGridX(valueGrid, valueOffSetX))
                                        {
                                            setGrid1 = true; //RX = -300 TX = -400 DX = -300 -(-400) = 100
                                                             //RX = 30 TX = -400 DX = 30 -(-400) = 430
                                        }
                                        else if (robotPos_X < convertGridX(valueGrid, valueOffSetX))
                                        {
                                            setGrid2 = true; //RX = 300 TX = 400 DX = 300 - 400 = -100
                                                             //RX = -30 TX = -25 DX = -30 -(-25) = -5
                                        }
                                    }
                                    else
                                    {
                                        if (setGrid1)
                                        {
                                            speedX = x * 0.02;
                                            //printf("SET GRID 1\n");
                                        }
                                        else if (setGrid2)
                                        {
                                            speedX = x * -0.02;
                                            //printf("SET GRID 2\n");
                                        }
                                    }

                                    speedY = y * 0.02;

                                    if (speedX >= kejarMax)
                                    {
                                        speedX = kejarMax;
                                    }
                                    else if (speedX <= -0.03)
                                    {
                                        speedX = -0.03;
                                    }
                                    if (speedY >= 0.04)
                                    {
                                        speedY = 0.04;
                                    }
                                    else if (speedY <= -0.04)
                                    {
                                        speedY = -0.04;
                                    }

                                    Walk(speedX, speedY, 0.0);
                                }
                                else
                                {
                                    Walk(0.0, 0.0, 0.0);
                                    countMoveGrid3++;
                                }
                                udah = true;
                            }
                            else
                            {
                                rotateBodyImuNew(0);
                            }
                        }
                        else if (nilaiSudut > 90 || nilaiSudut <= 270)
                        {
                            //printf("MASUK 180 180 180 180\n");
                            if (posRotateNew)
                            {
                                //printf("SELESAI ROTATE 0\n");
                                if (countMoveGrid3 >= 5)
                                {
                                    if (!setGrid1 && !setGrid2)
                                    {
                                        if (robotPos_X >= convertGridX(valueGrid, valueOffSetX))
                                        {
                                            setGrid1 = true; //RX = -300 TX = -400 DX = -300 -(-400) = 100
                                                             //RX = 30 TX = -400 DX = 30 -(-400) = 430
                                        }
                                        else if (robotPos_X < convertGridX(valueGrid, valueOffSetX))
                                        {
                                            setGrid2 = true; //RX = 300 TX = 400 DX = 300 - 400 = -100
                                                             //RX = -30 TX = -25 DX = -30 -(-25) = -5
                                        }
                                    }
                                    else
                                    {
                                        if (setGrid1)
                                        {
                                            speedX = x * 0.02;
                                            //printf("SET GRID 1\n");
                                        }
                                        else if (setGrid2)
                                        {
                                            speedX = x * -0.02;
                                            //printf("SET GRID 2\n");
                                        }
                                    }
                                    speedY = y * 0.02;

                                    if (speedX >= 0.06)
                                    {
                                        speedX = 0.06;
                                    }
                                    else if (speedX <= -0.03)
                                    {
                                        speedX = -0.03;
                                    }
                                    if (speedY >= 0.04)
                                    {
                                        speedY = 0.04;
                                    }
                                    else if (speedY <= -0.04)
                                    {
                                        speedY = -0.04;
                                    }

                                    Walk(speedX, -speedY, 0.0);
                                }
                                else
                                {
                                    Walk(0.0, 0.0, 0.0);
                                    countMoveGrid3++;
                                }
                                udah = true;
                            }
                            else
                            {
                                rotateBodyImuNew(0);
                            }
                        }
                    }
                    else
                    {
                        posRotateNew = false;
                        Walk(0.0, 0.0, 0.0);
                        countMoveGrid2++;
                    }
                }
                else
                {
                    setGrid1 = false;
                    setGrid2 = false;
                    posRotateNew = false;
                    countMoveGrid2 = 0;
                    countMoveGrid3 = 0;
                    jalanDirection(kejarMax, 0.0, rotate);
                }
            }
            else
            {
                Walk(0.0, 0.0, 0.0);
                countMoveGrid1++;
            }
        }
    }

    void refreshMoveGrid()
    {
        posRotateNew = false;
        doneMoved = false;
        udah = false;
        countMoveGrid1 =
            countMoveGrid2 =
                countMoveGrid3 = 
                    cnt_move_to_grid = 0;
        
    }

    int Grid = 0;
    int pos_X = 0;
    int pos_Y = 0;
    const double CAMERA_HEIGHT = 0.6; // meters
    const double HEAD_TILT_ZERO_POS = -PI/2.0; // radians
    const double CAMERA_FOV_VERTICAL = 51.0; // degrees
    const double PIXELS_PER_DEGREE = 7.11; // adjust this value based on your camera

    // Calculate the distance to the landmark given the distance to the object and the head tilt angle
    double calculateDistance(double objectDistance, double headTiltAngle) {
        // Convert the tilt angle from degrees to radians and adjust for the zero position
        double tiltAngle = headTiltAngle * PI/180.0 - HEAD_TILT_ZERO_POS;

        // Calculate the angle between the camera and the landmark using trigonometry
        double angleToLandmark = std::atan2(CAMERA_HEIGHT, objectDistance) - tiltAngle;

        // Calculate the distance to the landmark using trigonometry
        double landmarkDistance = CAMERA_HEIGHT / std::tan(angleToLandmark);

        return landmarkDistance;
    }

    // Calculate the number of pixels per meter based on the camera's field of view and the image size
    double calculatePixelsPerMeter(int imageHeight) {
        double fovRadians = CAMERA_FOV_VERTICAL * PI/180.0;
        double pixelsPerMeter = imageHeight / (2.0 * CAMERA_HEIGHT * std::tan(fovRadians/2.0));
        return pixelsPerMeter;
    }

    // Calculate the head tilt angle given the position of the servo
    double calculateHeadTiltAngle(int servoPosition) {
        // Adjust for the zero position of the tilt servo
        double headTiltAngle = servoPosition - HEAD_TILT_ZERO_POS;

        return headTiltAngle * 180.0/PI;
    }

    // Calculate the robot's x and y coordinates on the field
    void calculateRobotPosition(double xL, double yL, double d, double theta) {
        // Map the angle to the range of -180 to 180 degrees
        theta = fmod(theta + 540.0, 360.0) - 180.0;

        // Convert the orientation angle from degrees to radians
        double theta_rad = theta * PI / 180.0;

        // Calculate the robot's x and y coordinates on the field
        double xR = xL + d * cos(theta_rad);
        double yR = yL + d * sin(theta_rad);

        // Print the results
        std::cout << "Robot position: (" << xR << ", " << yR << ")" << std::endl;
        robotPos_X = xR;
        robotPos_Y = yR;
    }

    void updateCoordinate()
    {
        double objectDistance = landmarkDistance*100; // meters
        double servoPosition = headTilt; // radians
        int imageHeight = frame_Y; // pixels
        double landmarkCenter = Pinalty_Y; // pixels

        double headTiltAngle = calculateHeadTiltAngle(servoPosition);
        double pixelsPerMeter = calculatePixelsPerMeter(imageHeight);
        double pixelsFromTop = landmarkCenter - imageHeight/2.0;
        double landmarkDistance = calculateDistance(objectDistance, headTiltAngle)*100;
        double landmarkHeight = pixelsFromTop / pixelsPerMeter;

        std::cout << "Object distance: " << objectDistance << " cm" << std::endl;
        std::cout << "Head tilt angle: " << headTiltAngle << " degrees" << std::endl;
        std::cout << "Pixels per meter: " << pixelsPerMeter << " pixels/meter" << std::endl;
        std::cout << "Landmark distance: " << landmarkDistance << " cm" << std::endl;
        std::cout << "Landmark height: " << landmarkHeight << " meters" << std::endl;

        double orientationAngle = (msg_yaw*-1) + (headPan *180/PI); 
        calculateRobotPosition(-350, 0, objectDistance, orientationAngle);
    }

    int coordinates_to_grid(int GridX, int GridY) 
    {
    // Convert the input coordinates to be relative to the center of the grid
    GridX += 450;  // adjust for the center at 4.5m
    GridY = 300 - GridY;  // adjust for the center at 3m

    // Clamp the input coordinates
    GridX = std::max(std::min(GridX, 900), 0);
    GridY = std::max(std::min(GridY, 600), 0);

    // Convert coordinates to grid positions
    int tempGridX = GridX / 100;
    int tempGridY = GridY / 100;

    // Calculate grid position
    int Grid_Pose = tempGridY % 6 + 6 * tempGridX;

    return Grid_Pose;
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr peluit_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscriber_detections;
    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr trackbarSubscription_;
    rclcpp::Subscription<bfc_msgs::msg::Button>::SharedPtr button_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr bounding_boxes_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot1Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot2Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot3Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot4Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot5Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot6Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot7Subscription_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr gameControllerSubscription_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr voltage_n_odometry;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr GridSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr TargetPoseSub_;
    //rclcpp::Subscription<yolo_msgs::msg::Midpoints>::SharedPtr subscriber_yolo;
    //rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr subscriber_darknet;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr object_distance;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ball_distance;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr path_finding_subscription_;
    //rclcpp::Subscription<darknet_ros_msgs::msg::ObjectCount>::SharedPtr subscriber_object_count;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_teleop;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ball_pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr camera_odom_sub;
    rclcpp::Publisher<bfc_msgs::msg::Coordination>::SharedPtr robotCoordination_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_mot_;
    rclcpp::Publisher<bfc_msgs::msg::HeadMovement>::SharedPtr cmd_head_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr Odometry_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Update_coor_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ball_status_pub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr obstacle_distance_sub; //obstacle run
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rrtPathPub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rrtObstaclePub_;

    BehaviorTreeFactory factory;
    Tree tree;
    PublisherZMQ *pubZ;
    zmqpp::context context_;
    zmqpp::socket socket_;

    int odom_pose_x, odom_pose_y, odom_pose_z, firstStateLocalization, stateLocalization;
    int msg_strategy, msg_kill, msg_roll, msg_pitch, msg_yaw, modePlay;
    int robotKick = 0;
    
    float msg_peluit_hz = 0.0;

    int robot1Id, robot1Status, robot1State, robot1GridPosition, robot1XPosition, robot1YPosition, robot1FBall, robot1DBall, robot1BackIn, robot1Voltage, robot1KickOff, robot1GridBall, robot1XBall, robot1YBall;
    int robot2Id, robot2Status, robot2State, robot2GridPosition, robot2XPosition, robot2YPosition, robot2FBall, robot2DBall, robot2BackIn, robot2Voltage, robot2KickOff, robot2GridBall, robot2XBall, robot2YBall;
    int robot3Id, robot3Status, robot3State, robot3GridPosition, robot3XPosition, robot3YPosition, robot3FBall, robot3DBall, robot3BackIn, robot3Voltage, robot3KickOff, robot3GridBall, robot3XBall, robot3YBall;
    int robot4Id, robot4Status, robot4State, robot4GridPosition, robot4XPosition, robot4YPosition, robot4FBall, robot4DBall, robot4BackIn, robot4Voltage, robot4KickOff, robot4GridBall, robot4XBall, robot4YBall;
    int robot5Id, robot5Status, robot5State, robot5GridPosition, robot5XPosition, robot5YPosition, robot5FBall, robot5DBall, robot5BackIn, robot5Voltage, robot5KickOff, robot5GridBall, robot5XBall, robot5YBall;
    int robot6Id, robot6Status, robot6State, robot6GridPosition, robot6XPosition, robot6YPosition, robot6FBall, robot6DBall, robot6BackIn, robot6Voltage, robot6KickOff, robot6GridBall, robot6XBall, robot6YBall;
    int robot7Id, robot7Status, robot7State, robot7GridPosition, robot7XPosition, robot7YPosition, robot7FBall, robot7DBall, robot7BackIn, robot7Voltage, robot7KickOff, robot7GridBall, robot7XBall, robot7YBall;

    int Ball_X, Ball_Y, Ball_W, Ball_H, Ball_D, Pinalty_X, Pinalty_Y, Xcross_X, Xcross_Y;
    int B_pole_X, B_pole_Y, T_pole_X, T_pole_Y;
    int Goal_X, Goal_Y, Goal_W, Goal_H, Goal_LH, Goal_RH, Goal_C, Goal_LD, Goal_RD;
    int Goal_LX, Goal_LY, Goal_RX, Goal_RY, Xcross_RX, Xcross_RY, Xcross_LX, Xcross_LY;
    int Pinalty_D, Lcross_LD, Lcross_RD, Xcross_LD, Xcross_RD, Tcross_LD, Tcross_RD;
    int Left_X_Cross_X, Left_X_Cross_Y, Right_X_Cross_X, Right_X_Cross_Y, Left_T_Cross_X, Left_T_Cross_Y, Right_T_Cross_X, Right_T_Cross_Y;
    int Left_Corner_X, Left_Corner_Y, Right_Corner_X, Right_Corner_Y, Left_L_Cross_X, Left_L_Cross_Y, Right_L_Cross_X, Right_L_Cross_Y;
    int Left_T_Corner_X, Left_T_Corner_Y, Right_T_Corner_X, Right_T_Corner_Y, Robot_X, Robot_Y, goal_L_pole_X, goal_L_pole_Y, goal_R_pole_X, goal_R_pole_Y;
    int Left_T_Pole, Right_T_Pole, Left_B_Pole, Right_B_Pole;
    
    int GA_L_Cross_X, GA_L_Cross_Y, PA_L_Cross_X, PA_L_Cross_Y; 

    int State, Player, Team,
        FirstHalf,
        Version,
        PacketNumber,
        PlayerTeam,
        GameTipe,
        KickOff,
        SecondaryState,
        DropTeam,
        DropTime,
        Remaining,
        SecondaryTime,
        // ket : 1 = untuk data GameController yang kiri
        //	 2 = untuk data GameController yang kanan
        timNumber1,
        timNumber2,
        timColour1,
        timColour2,
        Score1,
        Score2,
        Penaltyshoot1,
        Penaltyshoot2,
        Singleshoot1,
        Singleshoot2,
        Coachsequence1,
        Coachsequence2,

        Penalty1,
        Penalty2,
        TimeUnpenalis1,
        TimeUnpenalis2,
        YellowCard1,
        YellowCard2,
        RedCard1,
        RedCard2,
        // GC26 new fields
        Stopped   = 0, // 1 = play stopped (during set plays)
        GamePhase = 0; // 0=Normal 1=PenaltyShoot 2=ExtraTime 3=Timeout
        
        int secondaryInfo[4];
        
        int messageBudget1 = 0, messageBudget2 = 0;
        int goalkeeperNum1 = 0, goalkeeperNum2 = 0;
        int goalkeeperColour1 = 0, goalkeeperColour2 = 0;
        int CompetitionType = 0;

    int team, barelang_color, dropball, max_current;
    /////////////////////////////////////////////////////////
    ///////////////////Variable Global///////////////////////
    /////////////////////////////////////////////////////////
    int robotNumber,
        stateCondition = 0,
        firstStateCondition = 0, // switch strategy

        stateGameController = 0,

        state,     // kill n run
        lastState, // kill n run
        wait = 0,
        delay = 0,
        delayWaitBall = 0,
        delayBolaJauh = 0,
        countBearing = 0,
        countDribble = 0,
        tunda = 0,
        tunggu = 0,
        waiting = 0,
        waitTracking = 0,
        reset = 0,
        matte = 0,
        chotto = 0,
        sudutTendang = 0,
        modeTendang = 0,

        modeKick = 1,

        saveAngle = 0,
        lastDirection = 0,

        countHilang = 0,

        confirmsBall = 0,
        countTilt = 0,
        sumTilt = 0;

    double headPan, // f.Kepala
        headTilt,   // f.Kepala
        posPan,
        posTilt,
        errorPan,  // f.Kepala
        errorTilt, // f.Kepala
        PPan,      // f.Kepala
        PTilt,     // f.Kepala

        ball_panKP,  // PID trackBall
        ball_panKD,  // PID trackBall
        ball_tiltKP, // PID trackBall
        ball_tiltKD, // PID trackBall

        goal_panKP,  // PID trackGoal
        goal_panKD,  // PID trackGoal
        goal_tiltKP, // PID trackGoal
        goal_tiltKD,

        land_panKP,  // PID trackLand
        land_panKD,  // PID trackLand
        land_tiltKP, // PID trackLand
        land_tiltKD, // PID trackLand

        panMax, panMin,

        RollCM,
        PitchCM,
        YawCM; // PID trackGoall

    double ballPositioningSpeed,
        pTiltTendangKanan,
        pPanTendangKanan,
        pTiltTendangKiri,
        pPanTendangKiri,
        pTiltOper,
        pPanOper,
        tiltBolaJauh,
        tiltBolaDekat,
        cSekarang,
        cAktif,
        posTiltLocal,
        posTiltGoal,
        erorrXwalk,
        erorrYwalk,
        erorrAwalk,
        aruku,
        jalan,
        lari,
        kejar,
        kejarMid,
        kejarMax,
        tinggiRobot,
        outputSudutY1,
        inputSudutY1,
        outputSudutY2,
        inputSudutY2,
        outputSudutX1,
        inputSudutX1,
        outputSudutX2,
        inputSudutX2,
        frame_X,
        frame_Y,
        rotateGoal_x,
        rotateGoal_y,
        rotateGoal_a,
        arukuX1,
        arukuX2,
        arukuX3,
        arukuX4,
        arukuX5,
        arukuX6,
        arukuX7,
        arukuX8,
        arukuX9,
        arukuY1,
        arukuY2,
        arukuY3,
        arukuY4,
        arukuYn1,
        arukuYn2,
        arukuYn3,
        arukuYn4,
        
        sendAngle,

        robotPos_X,   // om
        initialPos_X, // ov
        robotPos_Y,   // om
        initialPos_Y, // om
        initialAlpha,

        kurama = 0,
        panSaveKanan = -0.30,
        panSaveKiri = 0.30,
        offsetPan = 0,
        myAccrX = 0,
        myAccrY = 0;

    int sudutTengah,
        nomorpickup,
        arahGoal,
        sudutKanan,
        sudutKiri,
        tendangJauh,
        tendangDekat,
        tendangSamping;
        
    int goalAreaMinX        = 325,   // Min X area gawang lawan
        goalCornerLeftMaxY  = -130,  // Max Y pojok kiri gawang
        goalCornerRightMinY =  130,  // Min Y pojok kanan gawang
        leftSideMaxY        = -100,  // Batas pinggir kiri lapangan
        rightSideMinY       =  100;  // Batas pinggir kanan lapangan
    int angleCornerLeft     =  80,   // Sudut tendang pojok kiri gawang lawan
        angleCornerRight    = -80,   // Sudut tendang pojok kanan gawang lawan
        angleGoalLeft       =  10,   // Sudut tendang depan gawang sisi kiri
        angleGoalRight      = -10,   // Sudut tendang depan gawang sisi kanan
        angleSideLeftOpp    =  40,   // Pinggir kiri area lawan (serong ke tengah)
        angleSideLeftOwn    =  35,   // Pinggir kiri area sendiri (clear)
        angleSideRightOpp   = -40,   // Pinggir kanan area lawan (serong ke tengah)
        angleSideRightOwn   = -35;   // Pinggir kanan area sendiri (clear)
        
    int cornerKickAngleLeft  =  45,  // Pojok kiri → tendang serong ke kanan-dalam lapangan
        cornerKickAngleRight = -45;

    bool play,
        kanan,
        kiri,
        tengah,
        forceKanan,
        forceKiri,
        exeCutor = false,
        useVision, // f.setting
        useRos,
        useGameController,       // f.setting
        useCoordination,         // f.setting
        useLocalization,         // f.setting
        useFollowSearchGoal,     // f.setting
        useImu,                  // f.setting
        useDribble,              // f.setting
        dribbleOnly,             // f.setting
        useSearchGoal,           // f.setting
        useSideKick,             // f.setting
        useLastDirection,        // f.setting
        useNearFollowSearchGoal, // f.setting
        useUpdateCoordinate,     // f.setting
        usePenaltyStrategy,
        useDisplay,
        useWalkKick,
        useFollowExecutor,
        useOmnidirection,
        useKickOffGoal;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<main_strategy>();
    node->run();
    // while (rclcpp::ok())
    // {
    //     node->run();
    // }
    rclcpp::shutdown();
    return 0;
}
