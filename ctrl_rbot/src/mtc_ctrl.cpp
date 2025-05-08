#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Task Constructor headers
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace mtc = moveit::task_constructor;

class MTCTaskCtrl : public rclcpp::Node
{
public:
    explicit MTCTaskCtrl(const rclcpp::NodeOptions &node_options);
    ~MTCTaskCtrl() = default;

    void initialize();
    void run();

private:
    void setupCollisionObjects();
    void planAndExecute();
    void drawText(const std::string &text);

    // Task Constructor related methods
    bool createPoseGoalTask();
    bool executeTaskSolution();
    void setupTaskSolver();

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::thread executor_thread_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    // Task Constructor related members
    std::unique_ptr<mtc::Task> task_;
    std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner_;
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;

    // Action client for Task Constructor execution
    rclcpp_action::Client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SharedPtr execute_task_client_;
    const std::string LOGNAME = "moveit_ctrl";
};

MTCTaskCtrl::MTCTaskCtrl(const rclcpp::NodeOptions &node_options)
    : Node("MTCTask", node_options)
{
    RCLCPP_INFO(this->get_logger(), "大家好，我是MTCTask.");
}

void MTCTaskCtrl::initialize()
{
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "robot_6_dof_manipulator");
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        shared_from_this(), "base_link",
        rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface_->getRobotModel());
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Initialize Task Constructor action client
    execute_task_client_ = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
        shared_from_this(), "/execute_task_solution");

    // 初始化执行器线程
    executor_.add_node(shared_from_this());
    executor_thread_ = std::thread([this]()
                                   { executor_.spin(); });

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    // Setup Task Constructor solvers
    setupTaskSolver();
}

void MTCTaskCtrl::setupTaskSolver()
{
    // Initialize solvers
    pipeline_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    pipeline_planner_->setProperty("goal_joint_tolerance", 1e-4);
    pipeline_planner_->setProperty("planning_attempts", 2);

    // Setup cartesian planner for Cartesian motions
    cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>(shared_from_this());
    cartesian_planner_->setMaxVelocityScalingFactor(0.5);
    cartesian_planner_->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner_->setStepSize(0.01);
}

void MTCTaskCtrl::setupCollisionObjects()
{
    moveit_msgs::msg::CollisionObject collision_object;

    collision_object.header.frame_id = move_group_interface_->getPlanningFrame();
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_->applyCollisionObject(collision_object);
}

bool MTCTaskCtrl::createPoseGoalTask()
{
    task_ = std::make_unique<mtc::Task>("pose_goal_task");
    task_->loadRobotModel(shared_from_this());

    const auto &arm_group_name = "robot_6_dof_manipulator";

    // Set task properties
    task_->setProperty("group", arm_group_name);
    task_->setProperty("ik_frame", "tool0"); // Adjust based on your robot's end effector frame

    // Disable warnings for this cross-reference in PropertyMap
    task_->stages()->setProperty("ignore_virtual_joint", true);

    // Create stages for the task

    // Stage 1: Current state
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current_state");
    task_->add(std::move(current_state));

    // Stage 2: Move to target pose
    auto stage_move_to_pose = std::make_unique<mtc::stages::MoveTo>("move_to_pose", pipeline_planner_);
    stage_move_to_pose->setGroup(arm_group_name);

    // Define target pose (same as in original planAndExecute)
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = move_group_interface_->getPlanningFrame();
    target_pose.pose.orientation.x = 0.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 1.0;
    target_pose.pose.position.x = 0.28;
    target_pose.pose.position.y = 0.4;
    target_pose.pose.position.z = 0.5;

    stage_move_to_pose->setPoseGoal(target_pose);
    task_->add(std::move(stage_move_to_pose));

    return task_->init();
}

bool MTCTaskCtrl::executeTaskSolution()
{
    if (!task_->plan(5))
    {
        RCLCPP_ERROR(get_logger(), "Task planning failed");
        return false;
    }

    // Get the solution from the task
    auto solutions = task_->solutions();
    if (solutions.empty())
    {
        RCLCPP_ERROR(get_logger(), "No solutions found");
        return false;
    }

    // Use the best solution (usually the first one in the sorted solutions)
    const auto &solution = solutions.front();

    // Create a goal for the execute task client
    auto execute_goal = moveit_task_constructor_msgs::action::ExecuteTaskSolution::Goal();
    execute_goal.solution = *solution;

    if (!execute_task_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(get_logger(), "Execute task action server not available");
        return false;
    }

    // Visualize solution trajectory if visual tools available
    if (visual_tools_)
    {
        auto jmg = move_group_interface_->getRobotModel()->getJointModelGroup(
            "robot_6_dof_manipulator");

        for (const auto &trajectory : solution->sub_trajectory)
        {
            if (!trajectory.trajectory.joint_trajectory.points.empty())
            {
                visual_tools_->publishTrajectoryLine(trajectory.trajectory, jmg);
            }
        }
        visual_tools_->trigger();
    }

    auto send_goal_options = rclcpp_action::Client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SendGoalOptions();

    // Send goal to the action server
    auto goal_handle_future = execute_task_client_->async_send_goal(execute_goal, send_goal_options);

    // Wait for the goal to be accepted
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to send goal to execute task action server");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        return false;
    }

    // Wait for the result
    auto result_future = execute_task_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to get task execution result");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Task execution completed");
    return true;
}

void MTCTaskCtrl::planAndExecute()
{
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.28;
    target_pose.position.y = 0.4;
    target_pose.position.z = 0.5;
    move_group_interface_->setPoseTarget(target_pose);

    drawText("Planning");
    visual_tools_->prompt("Press 'next' to plan");
    visual_tools_->trigger();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface_->plan(plan));

    if (success)
    {
        auto jmg = move_group_interface_->getRobotModel()->getJointModelGroup("robot_6_dof_manipulator");
        visual_tools_->publishTrajectoryLine(plan.trajectory_, jmg);
        visual_tools_->trigger();

        visual_tools_->prompt("Press 'next' to execute");
        drawText("Executing");
        visual_tools_->trigger();
        move_group_interface_->execute(plan);
    }
    else
    {
        drawText("Planning Failed!");
        visual_tools_->trigger();
        RCLCPP_ERROR(get_logger(), "Planning failed!");
    }
}

void MTCTaskCtrl::drawText(const std::string &text)
{
    auto text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools_->publishText(text_pose, text,
                               rviz_visual_tools::WHITE,
                               rviz_visual_tools::XLARGE);
}

void MTCTaskCtrl::run()
{
    // Setup environment
    setupCollisionObjects();

    // Choose whether to use Task Constructor or the original method
    bool use_task_constructor = true;

    if (use_task_constructor)
    {
        // Use Task Constructor
        drawText("Using Task Constructor");
        visual_tools_->trigger();

        if (createPoseGoalTask())
        {
            visual_tools_->prompt("Press 'next' to execute task");
            drawText("Executing Task");
            visual_tools_->trigger();
            executeTaskSolution();
        }
        else
        {
            drawText("Task Creation Failed!");
            visual_tools_->trigger();
            RCLCPP_ERROR(get_logger(), "Failed to create task!");
        }
    }
    else
    {
        // Use original method
        planAndExecute();
    }

    rclcpp::shutdown();
    if (executor_thread_.joinable())
    {
        executor_thread_.join();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<MTCTaskCtrl>(
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    node->initialize();

    node->run();
    return 0;
}