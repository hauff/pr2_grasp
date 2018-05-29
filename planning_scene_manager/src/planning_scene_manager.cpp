#include <planning_scene_manager/planning_scene_manager.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <eigen_conversions/eigen_msg.h>

namespace planning_scene_manager
{

PlanningSceneManager::PlanningSceneManager()
{
  client_get_scene_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  client_set_scene_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
}

void PlanningSceneManager::allowCollision(const std::string& id)
{
  moveit_msgs::AllowedCollisionMatrix acm;
  getAllowedCollisionMatrix(acm);

  if (std::find(acm.default_entry_names.begin(), acm.default_entry_names.end(), id) !=
      acm.default_entry_names.end())
  {
    acm.default_entry_names.push_back(id);
    acm.default_entry_values.push_back(1);

    moveit_msgs::PlanningScene scene;
    scene.allowed_collision_matrix = acm;

    setPlanningScene(scene);
  }
}

void PlanningSceneManager::addBoxCollisionObject(const std::string& frame_id, const std::string& object_id,
  const Eigen::Vector3d& dimensions, const Eigen::Affine3d& pose)
{
  moveit_msgs::CollisionObject object;
  initHeader(ros::Time::now(), frame_id, object.header);
  object.id = object_id;

  object.primitives.resize(1);
  std::vector<double> dims(dimensions.data(), dimensions.data() + dimensions.size());
  initSolidPrimitive(shape_msgs::SolidPrimitive::BOX, dims, object.primitives[0]);

  object.primitive_poses.resize(1);
  tf::poseEigenToMsg(pose, object.primitive_poses[0]);

  addCollisionObject(object);
}

void PlanningSceneManager::removeCollisionObject(const std::string& frame_id, const std::string& object_id)
{
  moveit_msgs::CollisionObject object;
  object.header.frame_id = "odom_combined";
  object.id = object_id;
  object.operation = moveit_msgs::CollisionObject::REMOVE;

  addCollisionObject(object);
}

void PlanningSceneManager::getAllowedCollisionMatrix(moveit_msgs::AllowedCollisionMatrix& acm)
{
  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  client_get_scene_.call(srv);

  acm = srv.response.scene.allowed_collision_matrix;
}

void PlanningSceneManager::setPlanningScene(const moveit_msgs::PlanningScene& scene)
{
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = scene;
  srv.request.scene.is_diff = true;

  client_set_scene_.call(srv);
}

void PlanningSceneManager::addCollisionObject(const moveit_msgs::CollisionObject& object)
{
  moveit_msgs::PlanningScene scene;
  scene.world.collision_objects.push_back(object);

  setPlanningScene(scene);
}

void PlanningSceneManager::initHeader(const ros::Time& stamp, const std::string& frame_id,
  std_msgs::Header& header)
{
  header.stamp = stamp;
  header.frame_id = frame_id;
}

void PlanningSceneManager::initSolidPrimitive(const int& type,
  const std::vector<double>& dimensions, shape_msgs::SolidPrimitive& primitive)
{
  primitive.type = type;
  primitive.dimensions = dimensions;
}

}