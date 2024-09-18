#include "autonomy_node.hpp"

using namespace std::chrono_literals;

const std::string bt_xml_dir =
    ament_index_cpp::get_package_share_directory("cart_park_autonomy") + "/bt_xml";

AutonomyNode::AutonomyNode(const std::string &nodeName) : Node(nodeName)
{
  this->declare_parameter("location_file","none");

  RCLCPP_INFO(get_logger(), "Init done");
}

void AutonomyNode::setup()
{
  RCLCPP_INFO(get_logger(), "Setting up");
  create_behavior_tree();
  RCLCPP_INFO(get_logger(), "BT created");

  const auto timer_period = 500ms;
  timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&AutonomyNode::update_behavior_tree, this));

  rclcpp::spin(shared_from_this());
  rclcpp::shutdown();
}

void AutonomyNode::create_behavior_tree()
{
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<FirstLocSetter>("FirstLocSetter");
  factory.registerNodeType<Retry>("Retry");

  // register bt node
  BT::NodeBuilder builder_patrol =
      [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<Patrol>(name, config, shared_from_this());
  };
  factory.registerBuilder<Patrol>("Patrol", builder_patrol);

  BT::NodeBuilder builder_gotocart =
      [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<GoToCart>(name, config, shared_from_this());
  };
  factory.registerBuilder<GoToCart>("GoToCart", builder_gotocart);

  BT::NodeBuilder builder_dockcart =
      [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<DockCart>(name, config, shared_from_this());
  };
  factory.registerBuilder<DockCart>("DockCart", builder_dockcart);

  BT::NodeBuilder builder_parkcart =
      [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<ParkCart>(name, config, shared_from_this());
  };
  factory.registerBuilder<DockCart>("ParkCart", builder_parkcart);
  
  BT::NodeBuilder builder_opengripper =
      [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<OpenGripper>(name, config, shared_from_this());
  };
  factory.registerBuilder<OpenGripper>("OpenGripper", builder_opengripper);

  BT::NodeBuilder builder_spinback =
      [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<SpinBack>(name, config, shared_from_this());
  };
  factory.registerBuilder<SpinBack>("SpinBack", builder_spinback);

  RCLCPP_INFO(get_logger(), bt_xml_dir.c_str());
  tree_ = factory.createTreeFromFile(bt_xml_dir + "/tree.xml");
}

void AutonomyNode::update_behavior_tree()
{
  BT::NodeStatus tree_status = tree_.tickRoot();

  if (tree_status == BT::NodeStatus::RUNNING)
  {
    return;
  }
  else if (tree_status == BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Finished Navigation");
  }
  else if (tree_status == BT::NodeStatus::FAILURE)
  {
    RCLCPP_INFO(this->get_logger(), "Navigation Failed");
    timer_->cancel();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomyNode>("autonomy_node");
  node->setup();

  return 0;
}

