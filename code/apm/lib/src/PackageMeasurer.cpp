#include "../include/PackageMeasurer.h"
#include "../include/CannyEdgeDetector.h"
#include "../include/Morphology.h"
#include "../include/ContourExtractor.h"
#include "../include/PolygonFinder.h"
#include "../include/A4PaperFinder.h"

using cv::Mat;
using cv::Point;
using std::vector;

namespace automatic_package_measuring {

Mat src;
Mat src_gray;
vector<Point> reference_object;

int show_image = 1;
int canny_low_threshold = 40;
int morph_ele_radius = 3;
int morph_iterations = 3;
double poly_approx_error_margin = 0.02;
double canny_ratio = 3.0;

PackageMeasurer::PackageMeasurer() {

}

PackageMeasurer::~PackageMeasurer() {
}

void PackageMeasurer::measurePackage(cv::Mat image) {
	// do stuff
	src = image;
	cv::cvtColor(src, src_gray, CV_BGR2GRAY);

	findReferenceObject();
}

vector<Point>& PackageMeasurer::getPaperCoordinates() {
	return reference_object;
}

void PackageMeasurer::findReferenceObject() {

		CannyEdgeDetector canny = CannyEdgeDetector(canny_low_threshold,
				canny_ratio * canny_low_threshold);
		Morphology morphology = Morphology(morph_ele_radius, morph_iterations);
		Mat edges = canny.detectEdges(src_gray);
		edges = morphology.close(edges);

		ContourExtractor contour_extractor;
		vector<vector<Point>> contours = contour_extractor.extractContours(edges);

		PolygonFinder polygon_finder = PolygonFinder(poly_approx_error_margin); //error margin = 0
		vector<vector<Point>> polygons = polygon_finder.findPolygons(contours);

		A4PaperFinder paper_finder;
		reference_object = paper_finder.findObject(src, polygons);
}

} /* namespace automatic_package_measuring */
