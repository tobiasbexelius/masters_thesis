#ifndef LIB_INCLUDE_APM_UTIL_HDERP_
#define LIB_INCLUDE_APM_UTIL_HDERP_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

namespace internal {

void NearestNeighbours(std::vector<cv::Point2f>& points, int num_neighbours, std::vector<std::vector<int>>& neighbour_list_out);

} /* namespace internal */
} /* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_APM_UTIL_HDERP_ */
