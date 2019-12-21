#include <crane_msgs/msg/world_model.hpp>
#include <eigen3/Eigen/Core>
#include <vector>

struct Pose2D {
  Eigen::Vector2f pos;
  float theta;
};

struct RobotInfo {
  uint8_t id;
  Pose2D pos;
  Pose2D vel;
};

struct TeamInfo{
  Rect defense_area;
  std::vector<RobotInfo> robots;
};

struct Ball {
  Eigen::Vector2f pos;
  Eigen::Vector2f vel;
  bool is_curve;
};

Rect {
  Eigen::Vector2f min;
  Eigen::Vector2f max;
};

class WorldModel {
public:
  WorldModel() {}
  void update(crane_msgs::msg::WorldModel::SharedPtr world_model) {}

public:
  TeamInfo ours;
  TeamInfo theirs;
  Eigen::Vector2f field_size;
  Ball ball;
};