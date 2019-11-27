/**
 * @file
 *
 * \brief  Definition of collision functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "camera_nodelet.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

using namespace std;


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


void CameraNodelet::multi_fov_body(const vector<geometry_msgs::Pose> &poses,
  rp3d::CollisionBody* body, std::unique_ptr<rp3d::BoxShape> &shape)
{
  // Create the collision body
  rp3d::Vector3 pos(fixed_camera_tf_.transform.translation.x,
    fixed_camera_tf_.transform.translation.y,
    fixed_camera_tf_.transform.translation.z);
  rp3d::Quaternion orient(fixed_camera_tf_.transform.rotation.x,
    fixed_camera_tf_.transform.rotation.y,
    fixed_camera_tf_.transform.rotation.z,
    fixed_camera_tf_.transform.rotation.w);

  rp3d::Transform transform(pos, orient);
  body->setTransform(transform);

  // Adding FOV shapes for each pose
  unsigned int nbr_poses = poses.size();
  std::vector<box_shape_ptr> shapes(nbr_poses);

  float x = sensor_width_ * fov_distance_
          / (2 * sqrt(pow(focal_length_, 2) + pow(sensor_width_, 2)/4));
  float y = sensor_width_ * fov_distance_
          / (2 * sqrt(pow(focal_length_, 2) + pow(sensor_height_, 2)/4));
  rp3d::Vector3 half_extents(x, y, fov_distance_/2);

  for (unsigned int k = 0; k < nbr_poses; k++) {
    shapes[k] = unique_ptr<rp3d::BoxShape>(new rp3d::BoxShape(half_extents));

    rp3d::Vector3 fov_pos(poses[k].position.x, poses[k].position.y, poses[k].position.z + fov_distance_/2);
    rp3d::Quaternion fov_orient(poses[k].orientation.x, poses[k].orientation.y, poses[k].orientation.y, poses[k].orientation.z);
    rp3d::Transform fov_transform(fov_pos, fov_orient);
    body->addCollisionShape(shapes[k].get(), fov_transform);
  }

  // Merge all these shapes into a single AABB
  rp3d::AABB merged_shape = body->getAABB();

  for (unsigned int k = 0; k < nbr_poses; k++) {
    body->removeCollisionShape(body->getProxyShapesList());
  }

  pos = merged_shape.getCenter();
  orient = rp3d::Quaternion(0, 0, 0, 1);
  transform = rp3d::Transform(pos, orient);
  body->setTransform(transform);

  half_extents = merged_shape.getExtent();
  shape = unique_ptr<rp3d::BoxShape>(new rp3d::BoxShape(half_extents));
  body->addCollisionShape(shape.get(), rp3d::Transform::identity());
}


void CameraNodelet::update_fov_pose()
{
  rp3d::Vector3 pos(fixed_camera_tf_.transform.translation.x,
    fixed_camera_tf_.transform.translation.y,
    fixed_camera_tf_.transform.translation.z);
  rp3d::Quaternion orient(fixed_camera_tf_.transform.rotation.x,
    fixed_camera_tf_.transform.rotation.y,
    fixed_camera_tf_.transform.rotation.z,
    fixed_camera_tf_.transform.rotation.w);

  rp3d::Transform transform(pos, orient);
  fov_body_->setTransform(transform);
}


void CameraNodelet::overlap_fov(rp3d::CollisionBody* body)
{
  // Check for overlaps between fov body and algae and update ray_world_
  unsigned int n = ray_bodies_.size();
  for (unsigned int k = 0; k < n; k++) {
    ray_world_.destroyCollisionBody(ray_bodies_[k]);
  }
  ray_bodies_.resize(0);
  ray_shapes_.resize(0);
  corr_algae_.resize(0);
  coll_world_.testOverlap(body, &overlap_cb_);
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


tf2::Vector3 CameraNodelet::get_aim_pt(int pxl_h, int pxl_w, int n_pixel_h, int n_pixel_w)
{
  if (n_pixel_h <= 0 || n_pixel_w <= 0) {
    n_pixel_h = n_pxl_height_;
    n_pixel_w = n_pxl_width_;
  }

  tf2::Vector3 a(sensor_width_*(-1./2 + float(pxl_w)/(n_pixel_w-1)),
                 sensor_height_*(-1./2 + float(pxl_h)/(n_pixel_h-1)),
                 focal_length_);
  tf2::Vector3 origin(0, 0, 0);
  tf2::Vector3 aim_pt = fov_distance_ / tf2::tf2Distance(a, origin) * a;

  return aim_pt;
}


tf2::Vector3 CameraNodelet::apply_transform(const tf2::Vector3 &in_vector,
  const geometry_msgs::TransformStamped &transform)
{
  geometry_msgs::Pose pose;
  tf2::toMsg(in_vector, pose.position);
  pose.orientation.w = 1;

  geometry_msgs::Pose transf_pose;
  tf2::doTransform(pose, transf_pose, transform);

  return tf2::Vector3(transf_pose.position.x, transf_pose.position.y, transf_pose.position.z);
}


bool CameraNodelet::raycast_alga(const tf2::Vector3 &aim_pt, tf2::Vector3 &hit_pt,
  int &alga_idx, const tf2::Vector3 &origin)
{
  // Transform the ray into fixed frame
  tf2::Vector3 tf_origin = apply_transform(origin, fixed_camera_tf_);
  tf2::Vector3 tf_aim_pt = apply_transform(aim_pt, fixed_camera_tf_);

  // Raycast
  raycast_cb_.alga_hit_ = false;
  rp3d::Ray ray(tf2_to_rp3d(tf_origin), tf2_to_rp3d(tf_aim_pt));

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


bool CameraNodelet::raycast_alga(const tf2::Vector3 &aim_pt, float &distance,
  const tf2::Vector3 &origin)
{
  // Perform raycast
  int alga_idx;
  tf2::Vector3 hit_pt;
  bool alga_hit = raycast_alga(aim_pt, hit_pt, alga_idx, origin);

  // Get distance to the hit alga
  if (alga_hit) {
    distance = tf2::tf2Distance(hit_pt, apply_transform(origin, fixed_camera_tf_));
    return true;
  } else {
    return false;
  }
}


}  // mfcpp
