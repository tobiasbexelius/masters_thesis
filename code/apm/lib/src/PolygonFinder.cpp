#include "../include/PolygonFinder.h"

namespace automatic_package_measuring {

PolygonFinder::PolygonFinder(double error_tolerance, bool convex_only) :
		error_tolerance(error_tolerance), convex_only(convex_only) {
}


PolygonFinder::~PolygonFinder() {
}

std::vector<std::vector<cv::Point2i>> PolygonFinder::findPolygons(
		std::vector<std::vector<cv::Point2i>> contours) {

	std::vector<std::vector<cv::Point2i>> polygons;
	for (auto contour : contours) {
		std::vector<cv::Point2i> polygon;
		double curve_length = cv::arcLength(contour, true);
		cv::approxPolyDP(contour, polygon, curve_length * error_tolerance,
				true);
		if (!convex_only || cv::isContourConvex(polygon))
			polygons.push_back(polygon);
	}
	return polygons;
}

} /* namespace automatic_package_measuring */
