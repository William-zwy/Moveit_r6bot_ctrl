#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

class MoveitCtrl : public rclcpp::Node
{
public:
  explicit MoveitCtrl(const rclcpp::NodeOptions &node_options);
  ~MoveitCtrl() = default;

  void initialize();
  void run();

private:

  void setupCollisionObjects(); 
  void planAndExecute();
  void drawText(const std::string& text);

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::thread executor_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};


MoveitCtrl::MoveitCtrl(const rclcpp::NodeOptions &node_options) 
  : Node("ctrl_rbot", node_options)
{
  RCLCPP_INFO(this->get_logger(), "大家好，我是ctrl_rbot.");
}

void MoveitCtrl::initialize()
{
  move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "robot_6_dof_manipulator");
visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
    shared_from_this(), "base_link", 
    rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface_->getRobotModel());
planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

// 初始化执行器线程
executor_.add_node(shared_from_this());
executor_thread_ = std::thread([this]() { executor_.spin(); });


  visual_tools_->deleteAllMarkers();
  visual_tools_->loadRemoteControl();
}

void MoveitCtrl::setupCollisionObjects() 
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

void MoveitCtrl::planAndExecute() 
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

  if (success) {

    auto jmg = move_group_interface_->getRobotModel()->getJointModelGroup("robot_6_dof_manipulator");
    visual_tools_->publishTrajectoryLine(plan.trajectory_, jmg);
    visual_tools_->trigger();

    visual_tools_->prompt("Press 'next' to execute");
    drawText("Executing");
    visual_tools_->trigger();
    move_group_interface_->execute(plan);
  } else {
    drawText("Planning Failed!");
    visual_tools_->trigger();
    RCLCPP_ERROR(get_logger(), "Planning failed!");
  }
}

void MoveitCtrl::drawText(const std::string& text) 
{
  auto text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools_->publishText(text_pose, text, 
                           rviz_visual_tools::WHITE, 
                           rviz_visual_tools::XLARGE);
}

void MoveitCtrl::run() 
{
  // setupCollisionObjects();
  planAndExecute();
  
  rclcpp::shutdown();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<MoveitCtrl>(
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    
    node->initialize();  

    node->run();
    return 0;
}
