/**
 * @file
 *
 * \brief  Implementation of functions for initialising a farm
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "farm_nodelet.hpp"
#include "farm_common.hpp"
#include "mf_common/perlin_noise.hpp"
#include <vector>

using namespace std;


namespace mfcpp {

void FarmNodelet::init_anchors(AlgaeLine &line, unsigned int i)
{
  line.anchor1[0] = i * offset_lines_;
  line.anchor1[1] = 0;
  line.anchor1[2] = -depth_water_;

  line.anchor2[0] = i * offset_lines_;
  line.anchor2[1] = length_lines_;
  line.anchor2[2] = -depth_water_;

  line.anchors_diameter = anchors_diameter_;
  line.anchors_height = anchors_height_;
}


void FarmNodelet::init_ropes(AlgaeLine &line)
{
  float l = depth_water_;      // distance seafloor - floating line
  float h = l - depth_lines_;  // distance seafloor - algae line
  float L = length_lines_;

  // Initialise floating rope
  line.thickness_ropes = thickness_ropes_;
  float phi = phi_lines_;
  float theta = theta_lines_;

  if (randomise_lines_) {
    phi += rand_uniform(-bnd_phi_lines_, bnd_phi_lines_);
    theta += rand_uniform(-bnd_theta_lines_, bnd_theta_lines_);
  }

  float x1 = l * sin(theta) * cos(phi);
  float y1 = l * sin(theta) * sin(phi);
  float z1 = l * cos(theta);

  float    D2 = pow(x1, 2) + pow(L-y1, 2);
  float     d = sqrt(D2 + pow(z1, 2));
  float alpha = atan2(x1, L - y1);
  float  beta = atan2(z1, sqrt(D2));
  float delta = -asin((pow(d, 2) + pow(l, 2) - pow(L, 2)) / (2*d*l));
  float gamma = -acos(1/(cos(beta)*cos(delta)) * (z1/l + sin(beta)*sin(delta)));

  gamma = copysign(gamma, y1);
  if (randomise_lines_)
    gamma += rand_uniform(-bnd_gamma_lines_, bnd_gamma_lines_);

  float ca = cos(alpha);
  float sa = sin(alpha);
  float cgcd = cos(gamma)*cos(delta);
  float expr = sin(beta)*cgcd + cos(beta)*sin(delta);
  float sgcd = sin(gamma)*cos(delta);

  float x2 = -l * ( sa*(expr) - ca*sgcd);
  float y2 = -l * (-ca*(expr) - sa*sgcd);
  float z2 = l * (cos(beta)*cgcd - sin(beta)*sin(delta));

  line.floating_rope.p1 = line.anchor1 + tf2::Vector3(x1, y1, z1);
  line.floating_rope.p2 = line.anchor2 + tf2::Vector3(x2, y2, z2);

  // Initialise algae rope
  x1 = h * sin(theta) * cos(phi);
  y1 = h * sin(theta) * sin(phi);
  z1 = h * cos(theta);
  x2 = -h * ( sa*(expr) - ca*sgcd);
  y2 = -h * (-ca*(expr) - sa*sgcd);
  z2 = h * (cos(beta)*cgcd - sin(beta)*sin(delta));

  line.line.p1 = line.anchor1 + tf2::Vector3(x1, y1, z1);
  line.line.p2 = line.anchor2 + tf2::Vector3(x2, y2, z2);
}


void FarmNodelet::init_algae(AlgaeLine &line)
{
  line.algae.reserve(nbr_algae_);
  unsigned int n = nbr_algae_;
  tf2::Vector3 X1 = line.line.p1;
  tf2::Vector3 X2 = line.line.p2;

  perlin_.configure(height_disease_heatmap_, width_disease_heatmap_,
    height_grid_heatmap_, width_grid_heatmap_, random_seed_);
  perlin_.randomise_gradients();


  for (unsigned int k = 1; k <= nbr_algae_; k++) {
    if (!rand_bernoulli(alga_miss_rate_)) {
      Alga alga;

      // Initialise pose and dimensions of alga
      alga.position = X1 + float(k)/(n+1) * (X2 - X1);

      if (randomise_algae_) {
        alga.orientation = rand_gaussian(psi_algae_, std_psi_algae_);
        alga.length = abs(rand_gaussian(length_algae_, std_length_algae_));
        alga.width = abs(rand_gaussian(width_algae_, std_width_algae_));
      } else {
        alga.orientation = psi_algae_;
        alga.length = length_algae_;
        alga.width = width_algae_;
      }

      // Initialise disease heatmap of alga
      alga.disease_heatmap.resize(
        height_disease_heatmap_,
        vector<float>(width_disease_heatmap_, 0)
      );

      perlin_.generate(random_seed_);

      for (int i = 0; i < height_disease_heatmap_; i++) {
        for (int j = 0; j < width_disease_heatmap_; j++) {
          float value = perlin_.evaluate(i, j);
          alga.disease_heatmap[i][j] = disease_ratio_*value;
        }
      }

      // Add the alga
      line.algae.emplace_back(alga);
    }
  }
}




}  // namespace mfcpp
