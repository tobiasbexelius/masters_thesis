#include "../include/PackageMeasurer.h"
#include "../include/CannyEdgeDetector.h"
#include "../include/Morphology.h"
#include "../include/ContourExtractor.h"
#include "../include/PolygonFinder.h"
#include "../include/A4PaperFinder.h"
#include "../include/PackageFinder.h"

namespace automatic_package_measuring {

const int PackageMeasurer::CANNY_LOW_THRESHOLD = 50;
const int PackageMeasurer::MORPH_ELE_RADIUS = 1;
const int PackageMeasurer::MORPH_ITERATIONS = 1;
const double PackageMeasurer::POLY_ERROR_MARGIN = 0.02;
const double PackageMeasurer::CANNY_RATIO = 3.0;
const double PackageMeasurer::PACKAGE_CONTOUR_MIN_LENGTH = 200.0;
const double PackageMeasurer::PACKAGE_CONTOUR_PERIPHERAL_CONSTRAINT = 0.25;

PackageMeasurer::PackageMeasurer() {
}

PackageMeasurer::~PackageMeasurer() {
}

void PackageMeasurer::analyzeImage(cv::Mat image) {
	src = image;
	cv::cvtColor(src, src_gray, CV_BGR2GRAY);

	CannyEdgeDetector canny = CannyEdgeDetector(CANNY_LOW_THRESHOLD,
			CANNY_RATIO * CANNY_LOW_THRESHOLD);
	edges = canny.detectEdges(src_gray);

	Morphology morphology = Morphology(CANNY_RATIO, MORPH_ITERATIONS);
	edges = morphology.close(edges);
	ContourExtractor contour_extractor;
	contours = contour_extractor.extractContours(edges, false);

	findReferenceObject();

	contours = contour_extractor.extractContours(edges, true);
	contour_extractor.pruneShortContours(contours, PACKAGE_CONTOUR_MIN_LENGTH);
	contour_extractor.prunePeripheralContours(contours, src, PACKAGE_CONTOUR_PERIPHERAL_CONSTRAINT);

	findPackage();
}

std::vector<cv::Point2i>& PackageMeasurer::getReferenceObject() {
	return reference_object;
}

std::vector<cv::Point2i>& PackageMeasurer::getPackage() {
	return package;
}

void PackageMeasurer::findReferenceObject() {

	PolygonFinder polygon_finder = PolygonFinder(POLY_ERROR_MARGIN);
	std::vector<std::vector<cv::Point2i>> polygons =
			polygon_finder.findPolygons(contours);

	A4PaperFinder paper_finder;
	reference_object = paper_finder.findObject(src, polygons);

}

cv::Vec3d PackageMeasurer::getMeasurements() {
	return measurements;
}

void PackageMeasurer::findPackage() {

	PackageFinder package_finder;
	package = package_finder.findObject(src, contours);

}

void PackageMeasurer::measurePackage() {
	// TODO
}

} /* namespace automatic_package_measuring */
