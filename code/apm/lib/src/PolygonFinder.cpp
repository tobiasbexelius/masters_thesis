#include "../include/PolygonFinder.h"

namespace automatic_package_measuring {

PolygonFinder::PolygonFinder(double error_tolerance, bool convex_only,
		double convexity_defect_tolerance) :
		error_tolerance(error_tolerance), convex_only(convex_only), convexity_defect_tolerance(
				convexity_defect_tolerance) {
}

PolygonFinder::~PolygonFinder() {
}

std::vector<std::vector<cv::Point>> PolygonFinder::findPolygons(
		std::vector<std::vector<cv::Point>> contours) {

	std::vector<std::vector<cv::Point>> polygons;
	for (auto contour : contours) {
		std::vector<cv::Point> polygon;
		double curve_length = cv::arcLength(contour, true);
		cv::approxPolyDP(contour, polygon, curve_length * error_tolerance,
				true);
		if (!convex_only || isConvex(polygon, curve_length))
			polygons.push_back(polygon);
	}
	return polygons;
}

bool PolygonFinder::isConvex(std::vector<cv::Point> &polygon,
		double curve_length) {
	std::vector<int> hull;
	cv::convexHull(polygon, hull);

	if (polygon.size() < 4 || hull.size() < 4)
			return false;
	std::vector<cv::Vec4i> defects;
	cv::convexityDefects(polygon, hull, defects);
	double sum_defects = 0;
	for (auto defect : defects)
		sum_defects += defect[3];
	return sum_defects / curve_length <= convexity_defect_tolerance;

}

} /* namespace automatic_package_measuring */
