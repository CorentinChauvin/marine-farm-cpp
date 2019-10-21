/**
 * @file
 *
 * \brief  Implementation of functions publishing farm data
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "farm_nodelet.hpp"
#include "rviz_visualisation.hpp"
#include "farm_common.hpp"
#include "perlin_noise.hpp"
#include "farm_simulator/Alga.h"
#include "farm_simulator/Algae.h"
#include <std_msgs/Float32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;


namespace mfcpp {

void FarmNodelet::pub_rviz_markers(float duration) const
{
  // Initialise common marker data
  MarkerArgs args;
  args.stamp = ros::Time::now();
  args.duration = ros::Duration(duration);
  args.frame_id = "/world";
  args.color.r = 0.8;
  args.color.g = 0.8;
  args.color.b = 0.8;
  args.color.a = 1.0;

  visualization_msgs::MarkerArray markers;
  unsigned int nbr_assets = nbr_lines_ * (5 + nbr_algae_);
  markers.markers.reserve(nbr_assets);

  // Add anchor markers
  args.ns = "anchors";
  for (unsigned int i = 0; i < nbr_lines_; i++) {
    const AlgaeLine *al = &algae_lines_[i];  // for convenience

    markers.markers.emplace_back(
      rviz_marker_cylinder(al->anchor1, al->anchors_diameter, al->anchors_height, args)
    );
    markers.markers.emplace_back(
      rviz_marker_cylinder(al->anchor2, al->anchors_diameter, al->anchors_height, args)
    );
  }

  // Add buoys
  args.ns = "buoys";
  visualization_msgs::Marker buoys_marker;
  pop_buoys_marker(buoys_marker, args);
  markers.markers.emplace_back(buoys_marker);

  // Add ropes
  args.ns = "ropes";
  visualization_msgs::Marker line_marker;
  pop_ropes_marker(line_marker, args);
  markers.markers.emplace_back(line_marker);

  // Add algae
  args.ns = "algae";
  visualization_msgs::Marker rect_marker;
  pop_algae_marker(rect_marker, args);
  markers.markers.emplace_back(rect_marker);

  // Add disease heatmaps
  if (disp_disease_) {
    args.ns = "heatmaps";
    visualization_msgs::Marker img_marker;
    pop_algae_heatmaps(img_marker, args);
    markers.markers.emplace_back(img_marker);
  }

  // Publish the markers
  pop_marker_ids(markers);
  rviz_pub_.publish(markers);

}

void FarmNodelet::pop_buoys_marker(visualization_msgs::Marker &marker,
  MarkerArgs args) const
{
  marker = rviz_marker_spheres(buoys_diameter_, args);
  marker.points.reserve(nbr_lines_ * nbr_buoys_);

  for (unsigned int i = 0; i < nbr_lines_; i++) {
    const AlgaeLine *al = &algae_lines_[i];  // for convenience
    geometry_msgs::Point point;
    tf2::Vector3 X1 = al->floating_rope.p1;
    tf2::Vector3 X2 = al->floating_rope.p2;
    unsigned int n = al->nbr_buoys;

    for (unsigned int k = 0; k < n; k++) {
      tf2::Vector3 X = X1 + float(k)/(n-1) * (X2-X1);
      marker.points.emplace_back(tf2::toMsg(X, point));
    }
  }
}


void FarmNodelet::pop_ropes_marker(visualization_msgs::Marker &marker,
  MarkerArgs args) const
{
  marker = rviz_marker_line(thickness_ropes_, args);
  marker.points.reserve(3 * nbr_lines_);

  for (unsigned int i = 0; i < nbr_lines_; i++) {
    const AlgaeLine *al = &algae_lines_[i];
    geometry_msgs::Point point;

    marker.points.emplace_back(tf2::toMsg(al->anchor1, point));
    marker.points.emplace_back(tf2::toMsg(al->floating_rope.p1, point));
    marker.points.emplace_back(tf2::toMsg(al->floating_rope.p2, point));
    marker.points.emplace_back(tf2::toMsg(al->anchor2, point));
    marker.points.emplace_back(tf2::toMsg(al->floating_rope.p1, point));
    marker.points.emplace_back(tf2::toMsg(al->floating_rope.p2, point));
    marker.points.emplace_back(tf2::toMsg(al->line.p1, point));
    marker.points.emplace_back(tf2::toMsg(al->line.p2, point));

    // Ropes between the buoys and the algae line
    unsigned int n = al->nbr_buoys;
    tf2::Vector3 X1 = al->floating_rope.p1;
    tf2::Vector3 X2 = al->floating_rope.p2;
    tf2::Vector3 Y1 = al->line.p1;
    tf2::Vector3 Y2 = al->line.p2;

    for (unsigned int k = 1; k < n-1; k++) {
      tf2::Vector3 X = X1 + float(k)/(n-1) * (X2-X1);
      tf2::Vector3 Y = Y1 + float(k)/(n-1) * (Y2-Y1);
      marker.points.emplace_back(tf2::toMsg(X, point));
      marker.points.emplace_back(tf2::toMsg(Y, point));
    }
  }
}


void FarmNodelet::pop_algae_marker(visualization_msgs::Marker &marker,
  MarkerArgs args) const
{
  marker = rviz_marker_triangles(args);
  marker.points.reserve(2 * nbr_lines_ * nbr_algae_);

  for (unsigned int i = 0; i < nbr_lines_; i++) {
    const AlgaeLine *al = &algae_lines_[i];

    tf2::Vector3 X1 = al->line.p1;
    tf2::Vector3 X2 = al->line.p2;
    tf2::Vector3 z0(0, 0, 1);
    tf2::Vector3 y0(X2.getX()-X1.getX(), X2.getY()-X1.getY(), 0);
    y0 /= tf2::tf2Distance(y0, tf2::Vector3(0, 0, 0));
    float delta = asin((X2.getZ()-X1.getZ()) / length_lines_);

    tf2::Vector3 z1 = cos(delta)*z0 - sin(delta)*y0;
    tf2::Vector3 y1 = (X2-X1) / tf2::tf2Distance(X1, X2);
    tf2::Vector3 x1 = tf2::tf2Cross(y1, z1);

    for (unsigned int k = 0; k < al->algae.size(); k++) {
      tf2::Vector3 X = al->algae[k].position;
      float psi = al->algae[k].orientation;
      float H = al->algae[k].length;
      float W = al->algae[k].width;

      tf2::Vector3 p1 = X - W/2*y1;
      tf2::Vector3 p2 = X + W/2*y1;
      tf2::Vector3 p3 = p2 - H*(cos(psi)*z1 - sin(psi)*x1);
      tf2::Vector3 p4 = p1 - H*(cos(psi)*z1 - sin(psi)*x1);

      // Add the rectangular alga as two triangles
      geometry_msgs::Point point;
      marker.points.emplace_back(tf2::toMsg(p1, point));  // first triangle
      marker.points.emplace_back(tf2::toMsg(p2, point));
      marker.points.emplace_back(tf2::toMsg(p4, point));

      marker.points.emplace_back(tf2::toMsg(p2, point));  // second triangle
      marker.points.emplace_back(tf2::toMsg(p3, point));
      marker.points.emplace_back(tf2::toMsg(p4, point));
    }
  }

  std_msgs::ColorRGBA color;
  color.r = 0.1;
  color.g = 0.8;
  color.b = 0.1;
  color.a = 1.0;
  marker.colors.resize(marker.points.size()/3, color);
}


void FarmNodelet::pop_algae_heatmaps(visualization_msgs::Marker &marker,
  MarkerArgs args) const
{
  marker = rviz_marker_rect(args);
  marker.points.reserve(nbr_lines_ * nbr_algae_
    * 2 * height_disease_heatmap_ * width_disease_heatmap_);
  marker.colors.reserve(nbr_lines_ * nbr_algae_
    * 2 * height_disease_heatmap_ * width_disease_heatmap_);

  for (int k = 0; k < nbr_lines_; k++) {
    const AlgaeLine *al = &algae_lines_[k];

    tf2::Vector3 X1 = al->line.p1;
    tf2::Vector3 X2 = al->line.p2;
    tf2::Vector3 z(0, 0, 1);
    tf2::Vector3 y = (X2-X1) / tf2::tf2Distance(X1, X2);
    tf2::Vector3 x = tf2::tf2Cross(y, z);

    for (int l = 0; l < al->algae.size(); l++) {
      // Find the coordinates of the corners of the alga
      tf2::Vector3 X = al->algae[l].position;
      float psi = al->algae[l].orientation;
      float H = al->algae[l].length;
      float W = al->algae[l].width;

      vector<tf2::Vector3> coord(4);
      coord[0] = X - W/2*y;
      coord[1] = X + W/2*y;
      coord[2] = coord[1] - H*(cos(psi)*z - sin(psi)*x);
      coord[3] = coord[0] - H*(cos(psi)*z - sin(psi)*x);

      // Display its disease heatmap
      pop_img_marker(marker, al->algae[l].disease_heatmap, coord);
    }
  }

}


void FarmNodelet::pop_img_marker(visualization_msgs::Marker &marker,
  vector<vector<float>> img, const vector<tf2::Vector3> &coord) const
{
  int height = img.size();
  int width;

  if (height > 0)
    width = img[0].size();
  else {
    NODELET_ERROR("[farm_output] Dimensions of image not valid");
    return;
  }

  // coord[0] | coord[1]   --> y
  // -------------------   |
  // coord[3] | coord[2]   V x
  tf2::Vector3 x = (coord[3]-coord[0])/height;
  tf2::Vector3 y = (coord[1]-coord[0])/width;

  for (unsigned int i = 0; i < height; i++) {
    tf2::Vector3 p = coord[0] + i*x;  // current point

    for (unsigned int j = 0; j < width; j++) {
      // Add the points corresponding to the triangles representing the pixel
      geometry_msgs::Point point;
      marker.points.emplace_back(tf2::toMsg(p, point));  // first triangle
      marker.points.emplace_back(tf2::toMsg(p+y, point));
      marker.points.emplace_back(tf2::toMsg(p+y+x, point));

      marker.points.emplace_back(tf2::toMsg(p, point));  // second triangle
      marker.points.emplace_back(tf2::toMsg(p+x, point));
      marker.points.emplace_back(tf2::toMsg(p+y+x, point));

      // Add the color
      std_msgs::ColorRGBA color;
      color.r = img[i][j];
      color.g = img[i][j];
      color.b = img[i][j];
      color.a = 1.0;
      marker.colors.emplace_back(color);
      marker.colors.emplace_back(color);

      // Increment the current point
      p += y;
    }
  }
}


void FarmNodelet::pub_algae()
{
  farm_simulator::Algae algae;
  algae.algae.reserve(nbr_lines_ * nbr_algae_);

  for (unsigned int k = 0; k < nbr_lines_; k++) {
    const AlgaeLine *al = &algae_lines_[k];

    tf2::Vector3 X1 = al->line.p1;
    tf2::Vector3 X2 = al->line.p2;
    tf2::Vector3 z0(0, 0, 1);
    tf2::Vector3 y0(X2.getX()-X1.getX(), X2.getY()-X1.getY(), 0);
    y0 /= tf2::tf2Distance(y0, tf2::Vector3(0, 0, 0));
    float delta = asin((X2.getZ()-X1.getZ()) / length_lines_);

    tf2::Vector3 z1 = cos(delta)*z0 - sin(delta)*y0;
    tf2::Vector3 y1 = (X2-X1) / tf2::tf2Distance(X1, X2);
    tf2::Vector3 x1 = tf2::tf2Cross(y1, z1);


    for (unsigned int l = 0; l < al->algae.size(); l++) {
      farm_simulator::Alga alga;

      // Set the orientation of the alga
      float psi = al->algae[l].orientation;
      float H = al->algae[l].length;
      float W = al->algae[l].width;

      tf2::Vector3 x3 = -cos(psi)*x1 + sin(psi)*y1;
      tf2::Vector3 y3 = y1;
      tf2::Vector3 z3 = -cos(psi)*z1 - sin(psi)*x1;

      tf2::Matrix3x3 rotation;
      rotation.setValue(x3.getX(), y3.getX(), z3.getX(),
                        x3.getY(), y3.getY(), z3.getY(),
                        x3.getZ(), y3.getZ(), z3.getZ());
      tf2::Quaternion quati;
      rotation.getRotation(quati);
      alga.orientation = tf2::toMsg(quati);

      // Set the position of the alga
      tf2::Vector3 X = al->algae[l].position;
      alga.position = vector3_to_point32(X + H/2*z3);

      // Set the dimensions of the alga
      alga.dimensions.x = thickness_algae_;
      alga.dimensions.y = W;
      alga.dimensions.z = H;

      // Set the disease heatmap of the alga
      int n_height = height_disease_heatmap_;
      int n_width = width_disease_heatmap_;

      alga.disease_heatmap.resize(n_height);
      for (unsigned int i = 0; i < n_height; i++)
        alga.disease_heatmap[i].array.resize(n_width);

      for (unsigned int i = 0; i < n_height; i++) {
        for (unsigned int j = 0; j < n_width; j++) {
          alga.disease_heatmap[i].array[j] = al->algae[l].disease_heatmap[i][j];
        }
      }

      // Add the alga
      algae.algae.emplace_back(alga);
    }
  }

  algae_pub_.publish(algae);
}


}  // namespace mfcpp
