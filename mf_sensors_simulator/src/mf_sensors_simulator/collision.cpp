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
      parent_->corr_algae_.emplace_back(k);

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
  const rp3d::RaycastInfo& info)
{
  for (unsigned int k = 0; k < parent_->ray_bodies_.size(); k++) {
    if (parent_->ray_bodies_[k] == info.body) {
      alga_hit_ = true;
      hit_pt_ = tf2::Vector3(info.worldPoint.x, info.worldPoint.y,
        info.worldPoint.z);
      alga_idx_ = k;
    }
  }

  // Stop the raycasting
  return rp3d::decimal(0.0);
}


void CameraNodelet::overlap_fov()
{
  // Update fov body transform
  rp3d::Vector3 pos(fixed_camera_tf_.transform.translation.x,
    fixed_camera_tf_.transform.translation.y,
    fixed_camera_tf_.transform.translation.z);
  rp3d::Quaternion orient(fixed_camera_tf_.transform.rotation.x,
    fixed_camera_tf_.transform.rotation.y,
    fixed_camera_tf_.transform.rotation.z,
    fixed_camera_tf_.transform.rotation.w);

  rp3d::Transform transform(pos, orient);
  fov_body_->setTransform(transform);

  // Check for overlaps between fov body and algae and update ray_world_
  unsigned int n = ray_bodies_.size();
  for (unsigned int k = 0; k < n; k++) {
    ray_world_.destroyCollisionBody(ray_bodies_[k]);
  }
  ray_bodies_.resize(0);
  ray_shapes_.resize(0);
  corr_algae_.resize(0);

  coll_world_.testOverlap(fov_body_, &overlap_cb_);
}


void CameraNodelet::get_ray_algae_carac(
  std::vector<float> &w_algae, std::vector<float> &h_algae,
  std::vector<float> &inc_y3,  std::vector<float> &inc_z3,
  std::vector<geometry_msgs::TransformStamped> &tf_algae)
{
  unsigned int n = w_algae.size();
  float n_height = heatmaps_[0].size();    // height of the heatmap
  float n_width = heatmaps_[0][0].size();  // width of the heatmap

  for (int k = 0; k < n; k++) {
    // Compute the alga inverse transform
    rp3d::Transform inverse_tf = ray_bodies_[k]->getTransform().getInverse();
    rp3d::Vector3 pos = inverse_tf.getPosition();
    rp3d::Quaternion quati = inverse_tf.getOrientation();
    tf_algae[k].transform.translation.x = pos.x;
    tf_algae[k].transform.translation.y = pos.y;
    tf_algae[k].transform.translation.z = pos.z;
    tf_algae[k].transform.rotation.x = quati.x;
    tf_algae[k].transform.rotation.y = quati.y;
    tf_algae[k].transform.rotation.z = quati.z;
    tf_algae[k].transform.rotation.w = quati.w;

    // Get dimensions and increments
    w_algae[k] = 2*ray_shapes_[k]->getExtent().y;
    h_algae[k] = 2*ray_shapes_[k]->getExtent().z;

    inc_y3[k] = w_algae[k] / n_width;
    inc_z3[k] = h_algae[k] / n_height;
  }
}


bool CameraNodelet::raycast_alga(const tf2::Vector3 &aim_pt, tf2::Vector3 &hit_pt,
  int &alga_idx)
{
  // Transform the ray into fixed frame
  tf2::Vector3 tf_origin(0, 0, 0);
  geometry_msgs::Pose p_origin;
  geometry_msgs::Pose p_p;
  tf2::toMsg(tf_origin, p_origin.position);
  tf2::toMsg(aim_pt, p_p.position);
  p_origin.orientation.w = 1;
  p_p.orientation.w = 1;

  geometry_msgs::Pose origin;
  geometry_msgs::Pose p;
  tf2::doTransform(p_origin, origin, fixed_camera_tf_);
  tf2::doTransform(p_p, p, fixed_camera_tf_);

  // Raycast
  raycast_cb_.alga_hit_ = false;
  rp3d::Vector3 start_point(origin.position.x, origin.position.y, origin.position.z);
  rp3d::Vector3 end_point(p.position.x, p.position.y, p.position.z);
  rp3d::Ray ray(start_point, end_point);

  ray_world_.raycast(ray, &raycast_cb_);

  if (raycast_cb_.alga_hit_) {
    hit_pt = raycast_cb_.hit_pt_;
    alga_idx = raycast_cb_.alga_idx_;

    return true;
  }
  else {
    return false;
  }
}


}  // mfcpp
