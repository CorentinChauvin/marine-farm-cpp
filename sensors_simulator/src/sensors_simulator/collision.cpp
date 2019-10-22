/**
 * @file
 *
 * \brief  Definition of collision functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "camera_nodelet.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace mfcpp {

CameraNodelet::OverlapCallback::OverlapCallback(CameraNodelet *parent):
  rp3d::OverlapCallback(),
  parent_(parent)
{

}


CameraNodelet::RaycastCallback::RaycastCallback(CameraNodelet *parent):
  rp3d::RaycastCallback(),
  parent_(parent)
{

}


void CameraNodelet::OverlapCallback:: notifyOverlap(
  rp3d::CollisionBody *body)
{
  for (unsigned int k = 0; k < parent_->algae_bodies_.size(); k++) {
    if (parent_->algae_bodies_[k] == body) {
      rp3d::Transform transform = body->getTransform();
      rp3d::CollisionBody* new_body = parent_->ray_world_.createCollisionBody(transform);
      parent_->ray_bodies_.emplace_back(new_body);

      rp3d::Vector3 half_extents = parent_->algae_shapes_[k]->getExtent();
      box_shape_ptr new_shape = box_shape_ptr(new rp3d::BoxShape(half_extents));

      parent_->ray_shapes_.emplace_back(std::move(new_shape));
      new_body->addCollisionShape(parent_->ray_shapes_.back().get(), rp3d::Transform::identity());

      return;
    }
  }
}


rp3d::decimal CameraNodelet::RaycastCallback::notifyRaycastHit(
  const rp3d::RaycastInfo& info) {
  // Display the world hit point coordinates
  // std::cout << "---" << std::endl;
  // std::cout << "Hit point : " <<
  //   info.worldPoint.x << " ; " <<
  //   info.worldPoint.y << " ; " <<
  //   info.worldPoint.z <<
  //   std::endl;

  // std::cout << info.body->getAABB().getMin().x << " ; "
  //           << info.body->getAABB().getMin().y << " ; "
  //           << info.body->getAABB().getMin().z << std::endl;
  // std::cout << info.body->getAABB().getMax().x << " ; "
  //           << info.body->getAABB().getMax().y << " ; "
  //           << info.body->getAABB().getMax().z << std::endl;

  for (unsigned int k = 0; k < parent_->ray_bodies_.size(); k++) {
    if (parent_->ray_bodies_[k] == info.body) {
      std::cout << "k=" << k << std::endl;
    }
  }

  // Return a fraction of 1.0 to gather all hits
  return rp3d::decimal(0.0);
}


void CameraNodelet::overlap_fov()
{
  // Update fov body transform
  rp3d::Vector3 pos(camera_tf_.transform.translation.x,
    camera_tf_.transform.translation.y,
    camera_tf_.transform.translation.z);
  rp3d::Quaternion orient(camera_tf_.transform.rotation.x,
    camera_tf_.transform.rotation.y,
    camera_tf_.transform.rotation.z,
    camera_tf_.transform.rotation.w);

  rp3d::Transform transform(pos, orient);
  fov_body_->setTransform(transform);

  // Check for overlaps between fov body and algae and update ray_world_
  unsigned int n = ray_bodies_.size();
  for (unsigned int k = 0; k < n; k++) {
    ray_world_.destroyCollisionBody(ray_bodies_[k]);
  }
  ray_bodies_.resize(0);
  ray_shapes_.resize(0);

  coll_world_.testOverlap(fov_body_, &overlap_cb_);
}


bool CameraNodelet::raycast_alga(const tf2::Vector3 &aim_pt, float &disease,
  tf2::Vector3 &hit_pt)
{
  tf2::Vector3 tf_origin(0, 0, 0);

  // Transform the ray into fixed frame
  geometry_msgs::Pose p_origin;
  geometry_msgs::Pose p_p;
  tf2::toMsg(tf_origin, p_origin.position);
  tf2::toMsg(aim_pt, p_p.position);
  p_origin.orientation.w = 1;
  p_p.orientation.w = 1;

  geometry_msgs::Pose origin;
  geometry_msgs::Pose p;
  tf2::doTransform(p_origin, origin, camera_tf_);
  tf2::doTransform(p_p, p, camera_tf_);

  // Raycast
  rp3d::Vector3 start_point(origin.position.x, origin.position.y, origin.position.z);
  rp3d::Vector3 end_point(p.position.x, p.position.y, p.position.z);
  rp3d::Ray ray(start_point, end_point);

  ray_world_.raycast(ray, &raycast_cb_);

  return true;
}


}  // mfcpp
