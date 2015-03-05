#include "../include/PackageMeasurer.h"
#include "../include/CannyEdgeDetector.h"
#include "../include/Morphology.h"
#include "../include/ContourExtractor.h"
#include "../include/PolygonFinder.h"
#include "../include/A4PaperFinder.h"

namespace automatic_package_measuring {

PackageMeasurer::PackageMeasurer(int canny_low_threshold, int morph_ele_radius,
		int morph_iterations, double poly_approx_error_margin,
		double canny_ratio) :
		canny_low_threshold(canny_low_threshold), morph_ele_radius(morph_ele_radius), morph_iterations(
				morph_iterations), poly_approx_error_margin(
				poly_approx_error_margin), canny_ratio(canny_ratio) {
}

PackageMeasurer::~PackageMeasurer() {
}

void PackageMeasurer::analyzeImage(cv::Mat image) {
	src = image;
	cv::cvtColor(src, src_gray, CV_BGR2GRAY);

	findReferenceObject();
}

std::vector<cv::Point2i>& PackageMeasurer::getReferenceObject() {
	return reference_object;
}

std::vector<cv::Point2i>& PackageMeasurer::getPackage() {
	return package;
}

void PackageMeasurer::findReferenceObject() {

	CannyEdgeDetector canny = CannyEdgeDetector(canny_low_threshold,
			canny_ratio * canny_low_threshold);
	Morphology morphology = Morphology(morph_ele_radius, morph_iterations);
	cv::Mat edges = canny.detectEdges(src_gray);
	edges = morphology.close(edges);

	ContourExtractor contour_extractor;
	std::vector<std::vector<cv::Point2i>> contours =
			contour_extractor.extractContours(edges);

	PolygonFinder polygon_finder = PolygonFinder(poly_approx_error_margin);
	std::vector<std::vector<cv::Point2i>> polygons = polygon_finder.findPolygons(
			contours);

	A4PaperFinder paper_finder;
	reference_object = paper_finder.findObject(src, polygons);
}

cv::Vec3d PackageMeasurer::getMeasurements() {
	return measurements;
}

void PackageMeasurer::findPackage() {
	// TODO
}

void PackageMeasurer::measurePackage() {
	// TODO
}

} /* namespace automatic_package_measuring */
