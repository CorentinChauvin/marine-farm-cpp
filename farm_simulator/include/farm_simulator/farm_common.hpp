/**
 * @file
 *
 * \brief  Declaration of common structures and functions for farm simulator
 * \author Corentin Chauvin-Hamea
 * \date   2019
 */

#ifndef FARM_COMMON_HPP
#define FARM_COMMON_HPP

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

namespace mfcpp {
	/**
	 * \brief  Rope tensed between two 3D points
	 */
	struct Rope {
		/// Position of the first extremity of the rope
		tf2::Vector3 extremity1;

		/// Position of the second extremity of the rope
		tf2::Vector3 extremity2;

		/// Thickness of the rope
		float thickness;
	};

	/**
	 * \brief  Alga hanging on a line
	 */
	struct Alga {
		/**
		 * \brief  Position of the alga on the line
		 *
		 * 0.0 -> center of the alga on one extremity of the line <br/>
		 * 1.0 -> center of the alga on the other extremity of the line
		 */
		tf2::Vector3 position;

		/// Orientation of the alga
		float orientation;

		/// Length of the alga
		float length;

		/// Width of the alga
		float width;

		// heatmap color, image texture
		// TODO
	};

	/**
	 * \brief  Line on which algae grow
	 */
	struct Algae_line {
		/// Rope

		/// List of algae hanging on the line
		std::vector<Alga> algae;

		// TODO: function mapping (0, 1) to (extremity1, extremity2)
		// TODO: function giving the orientation of the line (to get the orientation of one alga)
	};


} // namespace mfcpp

#endif
