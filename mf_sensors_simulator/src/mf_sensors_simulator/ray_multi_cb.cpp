/*
 * Dichotomic version of ray_multi_cb function 
 */


bool CameraNodelet::ray_multi_cb(mf_sensors_simulator::MultiPoses::Request &req,
  mf_sensors_simulator::MultiPoses::Response &res)
{
  // Prepare the output
  int nbr_poses = req.pose_array.poses.size();
  res.results.resize(nbr_poses);

  for (int k = 0; k < nbr_poses; k++) {
    res.results[k].pxl_info.resize(4);
  }

  // Transform the poses in camera frame
  if (!world_init_ || !get_tf()) {
    res.is_success = false;
    return true;
  }

  vector<geometry_msgs::Pose> poses(nbr_poses);

  for (unsigned int k = 0; k < nbr_poses; k++) {
    geometry_msgs::Pose transf_pose;
    tf2::doTransform(req.pose_array.poses[k], transf_pose, camera_robot_tf_);

    poses[k] = transf_pose;
  }

  // Creating a collision body for field of view at all poses
  rp3d::CollisionBody* body = coll_world_.createCollisionBody(rp3d::Transform::identity());
  unique_ptr<rp3d::BoxShape> shape;
  multi_fov_body(poses, body, shape);

  // Selects algae that are in field of view of the camera
  coll_mutex_.lock();
  overlap_fov(false, body);

  // Get their position and dimension, and compute their axes
  int nbr_fov = ray_bodies_.size();
  vector<float> w_algae(nbr_fov);  // width of algae
  vector<float> h_algae(nbr_fov);  // height of algae
  vector<float> inc_y3(nbr_fov);   // increment along y3 axis of algae
  vector<float> inc_z3(nbr_fov);   // increment along z3 axis of algae
  vector<geometry_msgs::TransformStamped> tf_algae(nbr_fov);  // transforms of local frames

  get_ray_algae_carac(w_algae, h_algae, inc_y3,  inc_z3, tf_algae);

  cout << "---" << endl;
  cout << "Starting raycasting..." << endl;

  // Raycast for each pose
  for (int k = 0; k < nbr_poses; k++) {
    geometry_msgs::TransformStamped robot_vp_tf = pose_to_tf(req.pose_array.poses[k]);  // transform from robot to view point

    int x_corner[4] = {0, 0, n_pxl_height_-1, n_pxl_height_-1};  // position of the 4 corners in height direction
    int y_corner[4] = {0, n_pxl_width_-1, n_pxl_width_-1, 0};    // position of the 4 corners in width direction
    int x_dir[4] = {1, 1, -1, -1};  // direction of search for the 4 corners along x direction
    int y_dir[4] = {1, -1, -1, 1};  // direction of search for the 4 corners along y direction

    // For each corner, raycast the corner pixel. If the alga is not hit in the
    // corner, a dichotomy search is executed to find the extreme position of
    // the alga.
    // During the dichotomy, the distance variable is updated each time an alga
    // is hit. Therefore, it gets closer and closer to the searched one, and
    // is eventually equal.
    for (int l = 0; l < 4; l++) {
      bool corner_checked = false;
      bool alga_found = false;  // whether the alga has been hit
      bool search_over = false;
      float distance;
      int lb_x = 0;  // pixel index lower bound along x axis for dichotomy
      int ub_x = n_pxl_height_-1;  // pixel index upper bound along x axis for dichotomy
      int lb_y = 0;  // pixel index lower bound along x axis for dichotomy
      int ub_y = n_pxl_width_-1;  // pixel index upper bound along x axis for dichotomy
      int i, j;  // coordinates to check

      while (!search_over) {
        // Select pixel to check
        if (!corner_checked) {
          i = x_corner[l];
          j = y_corner[l];
        } else {
          if (x_dir[l] < 0)
            i = ceil((lb_x + ub_x) / 2.0);
          else
            i = (lb_x + ub_x) / 2;

          if (y_dir[l] < 0)
            j = ceil((lb_y + ub_y) / 2.0);
          else
            j = (lb_y + ub_y) / 2;
        }

        // cout << "i=" << i << " ; j=" << j << endl;
        // printf("(lb_x=%d, ub_x=%d) ; (lb_y=%d, ub_y=%d) \n", lb_x, ub_x, lb_y, ub_y);

        // Transform aim point into camera frame
        tf2::Vector3 aim_pt1 = get_aim_pt(i, j);  // aim point in view point camera frame
        tf2::Vector3 aim_pt2 = apply_transform(aim_pt1, robot_camera_tf_);  // aim point in view point frame
        tf2::Vector3 aim_pt3 = apply_transform(aim_pt2, robot_vp_tf);       // aim point in robot frame
        tf2::Vector3 aim_pt  = apply_transform(aim_pt3, camera_robot_tf_);  // aim point in camera frame

        // Perform raycast
        float curr_dist;
        tf2::Vector3 origin(poses[k].position.x, poses[k].position.y, poses[k].position.z);
        bool alga_hit = raycast_alga(aim_pt, curr_dist, origin);

        if (alga_hit)
          distance = curr_dist;  // update the distance

        // Initial corner check
        if (alga_hit && !corner_checked) {
          alga_found = true;
          search_over = true;
        } else if (!corner_checked)
          corner_checked = true;

        // Check for dichotomy end condition
        if (alga_hit && ub_x-lb_x == 1 && ub_y-lb_y == 1)
          alga_found = true;

        if (lb_x == ub_x && lb_y == ub_y) {
          alga_found = alga_hit;
          search_over = true;
        }

        // Move the bounds in a dychotomic fashion
        if (!alga_hit) {
          if (x_dir[l] > 0)
            lb_x = i + 1;
          else
            ub_x = i - 1;

          if (y_dir[l] > 0)
            lb_y = j + 1;
          else
            ub_y = j - 1;
        } else {
          if (x_dir[l] > 0)
            ub_x = i;
          else
            lb_x = i;

          if (y_dir[l] > 0)
            ub_y = j;
          else
            lb_y = j;
        }
      }

      // Fill output
      res.results[k].alga_hit = alga_found;

      if (alga_found) {
        res.results[k].pxl_info[l].x = i;
        res.results[k].pxl_info[l].y = j;
        res.results[k].pxl_info[l].distance = distance;
      }
    }

    // Transform it back to robot pose
    // ... TODO
  }

  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // Display the corners in Rviz to check whether it works in weird situations
  visualization_msgs::Marker ray_marker;
  ray_marker.header.stamp = ros::Time::now();
  ray_marker.header.frame_id = camera_frame_;
  ray_marker.ns = "Rays";
  ray_marker.lifetime = ros::Duration(1/camera_freq_);
  ray_marker.color.r = 1.0;
  ray_marker.color.g = 0.0;
  ray_marker.color.b = 0.0;
  ray_marker.color.a = 1.0;
  ray_marker.type = visualization_msgs::Marker::LINE_LIST;
  ray_marker.action = visualization_msgs::Marker::ADD;
  ray_marker.scale.x = 0.05;
  ray_marker.points.reserve(4*nbr_poses);

  for (int k = 0; k < nbr_poses; k++) {
    if (res.results[k].alga_hit) {
      geometry_msgs::TransformStamped robot_vp_tf = pose_to_tf(req.pose_array.poses[k]);  // transform from robot to view point

      for (int l = 0; l < 4; l++) {
        int i = res.results[k].pxl_info[l].x;
        int j = res.results[k].pxl_info[l].y;

        tf2::Vector3 aim_pt1 = get_aim_pt(i, j);  // aim point in view point camera frame
        tf2::Vector3 aim_pt2 = apply_transform(aim_pt1, robot_camera_tf_);  // aim point in view point frame
        tf2::Vector3 aim_pt3 = apply_transform(aim_pt2, robot_vp_tf);       // aim point in robot frame
        tf2::Vector3 aim_pt  = apply_transform(aim_pt3, camera_robot_tf_);  // aim point in camera frame

        tf2::Vector3 origin(poses[k].position.x, poses[k].position.y, poses[k].position.z);
        add_line_to_marker(ray_marker, origin, aim_pt);
      }
    }

    test_pub_.publish(ray_marker);
  }
  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


  // Destroy the collision body
  coll_world_.destroyCollisionBody(body);


  coll_mutex_.unlock();

  res.is_success = true;

  cout << res << endl;


  return true;
}


}  // namespace mfcpp
