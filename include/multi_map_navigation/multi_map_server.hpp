#ifndef MULTI_MAP_SERVER_HPP_
#define MULTI_MAP_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "multi_map_navigation/action/navigate_to_map.hpp"
#include "sqlite3.h"

namespace multi_map_navigation
{
class MultiMapServer : public rclcpp::Node
{
public:
  using NavigateToMap = multi_map_navigation::action::NavigateToMap;
  using GoalHandleNavigateToMap = rclcpp_action::ServerGoalHandle<NavigateToMap>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit MultiMapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MultiMapServer();

private:
  rclcpp_action::Server<NavigateToMap>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

  sqlite3* db_;
  std::string current_map_ = "room1"; // Start by assuming we are in room1

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToMap::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToMap> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle);

  // This is the line that was missing before
  bool navigateToPose(geometry_msgs::msg::PoseStamped pose, std::shared_ptr<GoalHandleNavigateToMap> goal_handle);

  void execute(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle);
};

}  // namespace multi_map_navigation

#endif  // MULTI_MAP_SERVER_HPP_
