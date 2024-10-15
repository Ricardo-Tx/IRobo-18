#include <rrt_planner/collision_detector.h>

namespace rrt_planner {

    CollisionDetector::CollisionDetector(costmap_2d::Costmap2DROS* costmap) {
        costmap_ = costmap->getCostmap();
        resolution_ = costmap_->getResolution();
        origin_x_ = costmap_->getOriginX();
        origin_y_ = costmap_->getOriginY();
    }

    bool CollisionDetector::inFreeSpace(const double* world_pos) {
        /**************************
         * Implement your code here
         **************************/
        // Get the x and y positions in the costmap coordinate frame
        double costmap_x = (world_pos[0] - origin_x_) / resolution_;
        double costmap_y = (world_pos[1] - origin_y_) / resolution_;

        // Ensure the point is within the bounds of the costmap
        if (costmap_x < 0 || costmap_x >= costmap_->getSizeInCellsX() ||
            costmap_y < 0 || costmap_y >= costmap_->getSizeInCellsY()) {
            return false; // Out of bounds
        }

        // Get the cost at the specified cell
        unsigned char cost = costmap_->getCost(static_cast<unsigned int>(costmap_x), static_cast<unsigned int>(costmap_y));

        // Check if the cell is free (commonly represented by a cost of 0)
        return (cost == costmap_2d::FREE_SPACE);
    }

    bool CollisionDetector::obstacleBetween(const double* point_a, const double* point_b) {
        double dist = computeDistance(point_a, point_b);

        if (dist < resolution_) {
            return (!inFreeSpace(point_b)) ? true : false;
        } else {
            int num_steps = static_cast<int>(floor(dist / resolution_));
            double point_i[2];

            for (int n = 1; n <= num_steps; n++) {
                point_i[0] = point_a[0] + n * (point_b[0] - point_a[0]) / num_steps;
                point_i[1] = point_a[1] + n * (point_b[1] - point_a[1]) / num_steps;

                if (!inFreeSpace(point_i)) return true;
            }
            return false;
        }
    }

}; // End of namespace rrt_planner
