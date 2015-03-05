#include <opencv2/opencv.hpp>
#include "../lib/include/CannyEdgeDetector.h"
#include "../lib/include/Morphology.h"
#include "../lib/include/ContourExtractor.h"
#include "../lib/include/PolygonFinder.h"
#include "../lib/include/A4PaperFinder.h"
#include "../lib/include/HoughLineTransform.h"
#include <iostream>
#include <string.h>
#include <cstdlib>

using namespace automatic_package_measuring;
using cv::Mat;
using std::cout;
using std::endl;

typedef std::vector<cv::Point> Curve;

const std::string WINDOW_NAME = "Automatic Package Measuring";
const int CANNY_MAX_LOW_THRESHOLD = 100;
bool simple_mode = false;
int hough_enabled = 0;
int show_image = 1;
int canny_low_threshold = 30;
int canny_ratio = 60;
int morph_ele_radius = 1;
int morph_iterations = 1;
int poly_approx_error_margin = 20;
int hough_rho = HoughLineTransform::DEFAULT_RHO;
int hough_theta = 25;
int hough_threshold = HoughLineTransform::DEFAULT_THRESHOLD;
int hough_min_length = HoughLineTransform::DEFAULT_MIN_LENGTH;
int hough_max_gap = HoughLineTransform::DEFAULT_MAX_GAP;

Mat src, src_gray, edges, result;
std::vector<Curve> contours, polygons;
CannyEdgeDetector canny;
Morphology morphology;
ContourExtractor contour_extractor(false);

int edges_enabled = 0;
int contours_enabled = 0;
int polygons_enabled = 0;
int paper_enabled = 1;

void drawCurves(cv::Mat canvas, std::vector<std::vector<cv::Point>> contours,
		cv::Scalar bgr_colour = cv::Scalar(0, 0, 255)) {
	srand(time(NULL));
	for (int i = 0; i < contours.size(); ++i) {
		std::vector<std::vector<cv::Point>> contour;
		int r = rand() % 256;
		int g = rand() % 256;
		int b = rand() % 256;
		contour.push_back(contours[i]);
		cv::drawContours(canvas, contour, -1, cv::Scalar(b, g, r), 2);
	}

}
double getErrorMargin() {
	return ((double) poly_approx_error_margin) / 1000.0;
}

double getCannyRatio() {
	return 1.0 + (double) (canny_ratio / 20.0);
}

double getHoughTheta() {
	return 0.005 + (double) (hough_theta / 2000.0);
}

int getHoughRho() {
	return 1 + hough_rho;
}

int getHoughThresh() {
	return 1 + hough_threshold;
}

void printSettings() {
	cout << "Settings:" << endl;
	cout << "Canny:" << endl;
	cout << "\t" << "Low threshold: " << canny_low_threshold << endl;
	cout << "\t" << "Ratio: " << getCannyRatio() << endl;
	cout << "Morphology" << endl;
	cout << "\t" << "Element radius: " << morph_ele_radius << endl;
	cout << "\t" << "Iterations: " << morph_iterations << endl;
	cout << "Polygon finder" << endl;
	cout << "\t" << "Error margin: " << getErrorMargin() << endl;
	cout << "Hough:" << endl;
	cout << "\t" << "Theta: " << getHoughTheta() << endl;
	cout << "\t" << "Rho: " << getHoughRho() << endl;
	cout << "\t" << "Thresh: " << getHoughThresh() << endl;
	cout << "\t" << "Min length: " << hough_min_length << endl;
	cout << "\t" << "Max gap: " << hough_max_gap << endl;
}

void setLabel(cv::Mat& im, const std::string label, Curve& contour,
		int x_offset = 0) {
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness,
			&baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(x_offset + r.x + ((r.width - text.width) / 2),
			r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline),
			pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255),
			CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

void doCanny() {
	canny = CannyEdgeDetector(canny_low_threshold,
			getCannyRatio() * canny_low_threshold);
	edges = canny.detectEdges(src_gray);
}

void doMorphology() {
	morphology = Morphology(morph_ele_radius, morph_iterations);
	edges = morphology.close(edges);

	if (edges_enabled) {
		Mat edges_colour = Mat(src.size(), CV_8UC3, cv::Scalar(0, 255, 0));
		edges_colour.copyTo(result, edges);
	}
}

void doHough() {
	if (!hough_enabled)
		return;
#if 0
	HoughLineTransform hough(getHoughRho(), getHoughTheta(), getHoughThresh(),
			hough_min_length, hough_max_gap);
	std::vector<cv::Vec4i> lines = hough.detectLines(edges);
	for (size_t i = 0; i < lines.size(); i++) {
		cv::Vec4i l = lines[i];
		cv::line(result, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
				cv::Scalar(0, 255, 255), 3, CV_AA);
	}
#else
	HoughLineTransform hough(getHoughRho(), getHoughTheta(), getHoughThresh(),
			hough_min_length, hough_max_gap);
	std::vector<cv::Vec2f> lines = hough.detectLinesNonP(edges);
	cout << "HOUGH LINES = " << lines.size() << endl;
	for (size_t i = 0; i < lines.size(); i++) {
		float rho = lines[i][0], theta = lines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		cv::line(result, pt1, pt2, cv::Scalar(0, 255, 255), 3, CV_AA);
	}
#endif
}

void extractCountours() {
	Mat contours_img = Mat(src.size(), src.type());
	contours_img = cv::Scalar::all(0);
	contours = contour_extractor.extractContours(edges);

	if (contours_enabled) {
		drawCurves(contours_img, contours, cv::Scalar(255, 0, 0));
		contours_img.copyTo(result, contours_img);
	}
}

void findPolygons() {
	PolygonFinder polygon_finder = PolygonFinder(getErrorMargin());
	polygons = polygon_finder.findPolygons(contours);

	if (polygons_enabled)
		drawCurves(result, polygons, cv::Scalar(0, 0, 255));
}

void findA4() {
	A4PaperFinder paper_finder;
	std::vector<cv::Point> paper = paper_finder.findObject(src, polygons);

	if (paper_enabled) {
		std::vector<std::vector<cv::Point>> paper_vec;
		paper_vec.push_back(paper);
		drawCurves(result, paper_vec, cv::Scalar(255, 0, 255));
	}
}

void processImage(int, void*) {
	result = cv::Scalar::all(0);
	if (show_image)
		src.copyTo(result);

	printSettings();

	doCanny();
	doMorphology();
	doHough();
	extractCountours();
	findPolygons();
	findA4();

	cv::imshow(WINDOW_NAME, result);
}

void createWindow() {
	cv::namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
	if (!simple_mode) {
		cv::createTrackbar("Edges:", WINDOW_NAME, &edges_enabled, 1,
				processImage);
		//cv::createTrackbar("Show image:", WINDOW_NAME, &show_image, 1,
		//		processImage);
		cv::createTrackbar("Contours:", WINDOW_NAME, &contours_enabled, 1,
				processImage);
		cv::createTrackbar("Polygons:", WINDOW_NAME, &polygons_enabled, 1,
				processImage);
		cv::createTrackbar("Paper:", WINDOW_NAME, &paper_enabled, 1,
				processImage);
	}
	cv::createTrackbar("Canny low threshold:", WINDOW_NAME,
			&canny_low_threshold, CANNY_MAX_LOW_THRESHOLD, processImage);
	cv::createTrackbar("Canny ratio:", WINDOW_NAME, &canny_ratio, 100,
			processImage);
	cv::createTrackbar("Morph ele radius:", WINDOW_NAME, &morph_ele_radius, 10,
			processImage);
	cv::createTrackbar("Morph iters:", WINDOW_NAME, &morph_iterations, 20,
			processImage);
	cv::createTrackbar("Poly Error margin:", WINDOW_NAME,
			&poly_approx_error_margin, 500, processImage);
#if 0
	cv::createTrackbar("Hough:", WINDOW_NAME, &hough_enabled, 1, processImage);
	cv::createTrackbar("Hough rho:", WINDOW_NAME, &hough_rho, 10, processImage);
	cv::createTrackbar("Hough theta:", WINDOW_NAME, &hough_theta, 100,
			processImage);
	cv::createTrackbar("Hough thresh", WINDOW_NAME, &hough_threshold, 1000,
			processImage);
	cv::createTrackbar("Hough min len:", WINDOW_NAME, &hough_min_length, 100,
			processImage);
	cv::createTrackbar("Hough max gap:", WINDOW_NAME, &hough_max_gap, 30,
			processImage);
#endif

}

std::vector<int> getPaperCoordinates() {
	std::vector<int> coordinates;

	return coordinates;
}

int main(int argc, char** argv) {

	if (argc < 2) {
		cout << "Missing parameter: image path." << endl;
		return -1;
	}

	src = cv::imread(argv[1]);

	if (!src.data) {
		cout << "Invalid image." << endl;
		return -1;
	}

	if (argc > 2 && (strcmp(argv[2], "-p") == 0 || strcmp(argv[2], "-P") == 0))
		simple_mode = true;

	result.create(src.size(), CV_8UC3);
	cv::cvtColor(src, src_gray, CV_BGR2GRAY);
	//cv::equalizeHist(src_gray, src_gray);
	createWindow();

	processImage(0, 0);

	int key = 0;
	while (key != 27) // escape
		key = cv::waitKey(0);
}
