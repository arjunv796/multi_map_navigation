#include "multi_map_navigation/multi_map_server.hpp"
#include <memory>
#include <thread>
#include <string>
#include <cstdlib>
#include <vector>
#include <fstream>

namespace multi_map_navigation
{

// Helper struct to hold wormhole data
struct Wormhole {
    geometry_msgs::msg::PoseStamped pose;
};

// Callback function for SQLite to process query results
static int db_callback(void *data, int argc, char **argv, char **azColName) {
    auto wormhole_optional = static_cast<std::optional<Wormhole>*>(data);
    Wormhole wormhole;
    wormhole.pose.header.frame_id = "map";
    for (int i = 0; i < argc; i++) {
        std::string colName(azColName[i]);
        if (colName == "pos_x") wormhole.pose.pose.position.x = atof(argv[i]);
        else if (colName == "pos_y") wormhole.pose.pose.position.y = atof(argv[i]);
        else if (colName == "orientation_z") wormhole.pose.pose.orientation.z = atof(argv[i]);
        else if (colName == "orientation_w") wormhole.pose.pose.orientation.w = atof(argv[i]);
    }
    *wormhole_optional = wormhole;
    return 0;
}

MultiMapServer::MultiMapServer(const rclcpp::NodeOptions & options)
: Node("multi_map_server", options)
{
  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<NavigateToMap>(
    this,
    "navigate_to_map",
    std::bind(&MultiMapServer::handle_goal, this, _1, _2),
    std::bind(&MultiMapServer::handle_cancel, this, _1),
    std::bind(&MultiMapServer::handle_accepted, this, _1));

  this->nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // IMPORTANT: Use the absolute path to your database
  std::string db_path = "/home/arjun/ros2_ws_clean/src/multi_map_navigation/config/wormholes.db";

  if (sqlite3_open(db_path.c_str(), &db_)) {
      RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db_));
  } else {
      RCLCPP_INFO(this->get_logger(), "Opened database successfully");
  }

  RCLCPP_INFO(this->get_logger(), "Multi Map Navigation Server has been started.");
}

MultiMapServer::~MultiMapServer()
{
    sqlite3_close(db_);
}

rclcpp_action::GoalResponse MultiMapServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToMap::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request for map: %s", goal->target_map.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MultiMapServer::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToMap> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  nav_to_pose_client_->async_cancel_all_goals();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MultiMapServer::handle_accepted(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&MultiMapServer::execute, this, _1), goal_handle}.detach();
}

std::optional<Wormhole> getWormholePose(sqlite3* db, const std::string& current_map) {
    std::string sql = "SELECT pos_x, pos_y, orientation_z, orientation_w FROM wormholes WHERE map_name = '" + current_map + "';";
    std::optional<Wormhole> wormhole;
    char *zErrMsg = 0;
    int rc = sqlite3_exec(db, sql.c_str(), db_callback, &wormhole, &zErrMsg);
    if (rc != SQLITE_OK) {
        sqlite3_free(zErrMsg);
        return std::nullopt;
    }
    return wormhole;
}

bool MultiMapServer::navigateToPose(geometry_msgs::msg::PoseStamped pose, std::shared_ptr<GoalHandleNavigateToMap> goal_handle) {
    if (!this->nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available after waiting");
        return false;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback = [this](GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        // Optional: Forward Nav2 feedback to our own action feedback
    };

    auto goal_handle_future = this->nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send goal to Nav2");
        return false;
    }

    auto nav2_goal_handle = goal_handle_future.get();
    if (!nav2_goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 goal was rejected by server");
        return false;
    }

    auto result_future = nav_to_pose_client_->async_get_result(nav2_goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get result from Nav2");
        return false;
    }

    auto status = result_future.get().code;
    return status == rclcpp_action::ResultCode::SUCCEEDED;
}

void MultiMapServer::execute(const std::shared_ptr<GoalHandleNavigateToMap> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<NavigateToMap::Result>();
  auto feedback = std::make_shared<NavigateToMap::Feedback>();

  // ==== CORE LOGIC ====
  if (goal->target_map == current_map_) {
      RCLCPP_INFO(this->get_logger(), "Goal is in the current map. Navigating directly.");
      feedback->status = "Navigating to goal in " + current_map_;
      goal_handle->publish_feedback(feedback);

      if (navigateToPose(goal->goal_pose, goal_handle)) {
          result->success = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal reached successfully.");
      } else {
          result->success = false;
          goal_handle->abort(result);
          RCLCPP_ERROR(this->get_logger(), "Failed to reach goal.");
      }
  } else {
      RCLCPP_INFO(this->get_logger(), "Goal is in a different map. Navigating via wormhole.");

      // 1. Navigate to the wormhole in the current map
      feedback->status = "Finding wormhole in " + current_map_;
      goal_handle->publish_feedback(feedback);
      auto wormhole = getWormholePose(db_, current_map_);

      if (!wormhole) {
          RCLCPP_ERROR(this->get_logger(), "Could not find a wormhole from map '%s'", current_map_.c_str());
          result->success = false;
          goal_handle->abort(result);
          return;
      }

      feedback->status = "Navigating to wormhole in " + current_map_;
      goal_handle->publish_feedback(feedback);
      if (!navigateToPose(wormhole->pose, goal_handle)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to navigate to wormhole.");
          result->success = false;
          goal_handle->abort(result);
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Arrived at wormhole.");

      // 2. Switch the map
      feedback->status = "Switching map from " + current_map_ + " to " + goal->target_map;
      goal_handle->publish_feedback(feedback);

      // This is a simple but effective way for this project. It is not robust for production.
      RCLCPP_INFO(this->get_logger(), "Stopping current Nav2 stack...");
      system("pkill -f 'nav2_bringup.*launch.py' && sleep 5");

      RCLCPP_INFO(this->get_logger(), "Starting new Nav2 stack with map: %s", goal->target_map.c_str());
      std::string launch_command = "ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=~/ros2_ws_clean/src/multi_map_navigation/maps/" + goal->target_map + ".yaml &";
      system(launch_command.c_str());
      current_map_ = goal->target_map;

      RCLCPP_INFO(this->get_logger(), "Waiting for new Nav2 stack to initialize...");
      std::this_thread::sleep_for(std::chrono::seconds(15)); // Wait for Nav2 to come up

      // 3. Navigate from the wormhole to the final goal in the new map
      feedback->status = "Navigating to final goal in " + current_map_;
      goal_handle->publish_feedback(feedback);
      if (navigateToPose(goal->goal_pose, goal_handle)) {
          result->success = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal reached successfully in new map.");
      } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to reach goal in new map.");
          result->success = false;
          goal_handle->abort(result);
      }
  }
}

}  // namespace multi_map_navigation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(multi_map_navigation::MultiMapServer)
