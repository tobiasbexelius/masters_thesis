#include "../lib/include/image_processing.h"
#include "../lib/include/image_processing_internal.h"
#include "../lib/include/paper_detection.h"
#include "../lib/include/paper_detection_internal.h"
#include "../lib/include/package_detection.h"
#include "../lib/include/package_detection_internal.h"
#include "../lib/include/package_measuring.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string.h>
#include <cstdlib>

using namespace automatic_package_measuring;
using namespace automatic_package_measuring::internal;
using cv::Mat;
using std::cout;
using std::endl;

const std::string RESULT_WINDOW = "APM Test GUI";

int canny_ratio = (CANNY_RATIO - 1) * 10.0;
int poly_error_tolerance = POLY_ERROR_TOLERANCE * 1000;
int hough_theta = (HOUGH_THETA * 180.0 / CV_PI - 0.1) * 10.0;
int hough_rho = HOUGH_RHO - 1;
int hough_min_length = HOUGH_MIN_LENGTH - 1;
int hough_threshold = HOUGH_THRESHOLD - 1;
int min_contour_length = 20;
int center_threshold = CENTER_THRESHOLD * 200;
int min_paper_contour_length = MIN_PAPER_CONTOUR_LENGTH;
int min_package_contour_length = MIN_PACKAGE_CONTOUR_LENGTH;

cv::Mat image, edges, result;
std::vector<cv::Point2f> paper, package;

int show_image = 1;
int show_edges = 0;
int show_contours = 0;
int show_polygons = 0;
int show_lines = 1;
int show_paper = 1;
int show_package = 1;
int external_contours_only = 1;

void PrintConstants();
void DrawOverlay();
void DrawEdges();
void DrawContours();
void DrawLines();
void DrawPolygons();
void DrawPaper();
void DrawPackage();

void DrawContours(cv::Mat canvas, std::vector<std::vector<cv::Point>> contours, cv::Scalar color =
		cv::Scalar());
void DrawContour(cv::Mat canvas, std::vector<cv::Point> contour, cv::Scalar color = cv::Scalar());

void UpdateConstants() {
	CANNY_RATIO = 1.0 + canny_ratio / 10.0;
	POLY_ERROR_TOLERANCE = poly_error_tolerance / 1000.0;
	HOUGH_THETA = (0.1 + hough_theta / 10.0) * CV_PI / 180.0;
	HOUGH_RHO = hough_rho + 1;
	HOUGH_MIN_LENGTH = hough_min_length + 1;
	HOUGH_THRESHOLD = hough_threshold + 1;
	CENTER_THRESHOLD = center_threshold / 200.0;
	MIN_PACKAGE_CONTOUR_LENGTH = min_package_contour_length;
	MIN_PAPER_CONTOUR_LENGTH = min_paper_contour_length;
}

void ProcessImage(int, void*) {
	edges = cv::Scalar::all(0);
	result = cv::Scalar::all(0);
	paper.clear();
	package.clear();

	if (show_image)
		image.copyTo(result);

	UpdateConstants();
	PrintConstants();

	cv::Mat preprocessed_image;
	PreprocessImage(image, preprocessed_image);
	FindEdges(preprocessed_image, edges);
	CloseEdges(edges);
	paper = FindPaper(image, edges);
	package = FindPackage(image, edges, paper);
	cv::Vec3d measurements = MeasurePackage(image.size(), paper, cv::Vec2d(297, 210), package);
	DrawOverlay();

	cv::imshow(RESULT_WINDOW, result);

}

void DrawOverlay() {
	if (show_edges)
		DrawEdges();

	if (show_contours)
		DrawContours();

	if (show_lines)
		DrawLines();

	if (show_polygons)
		DrawPolygons();

	if (show_paper)
		DrawPaper();

	if (show_package)
		DrawPackage();
}

void DrawEdges() {
	Mat edges_colour = Mat(image.size(), CV_8UC3, cv::Scalar(0, 255, 0));
	edges_colour.copyTo(result, edges);
}

void DrawContours() {
	std::vector<std::vector<cv::Point>> contours;
	FindContours(edges, external_contours_only, contours);
	PruneShortContours(contours, min_package_contour_length);
	PrunePeripheralContours(contours, image.size());
	DrawContours(result, contours);
}

void DrawLines() {
	cv::Mat contours_mat;
	if(external_contours_only){
		std::vector<std::vector<cv::Point>> contours;
			FindContours(edges, false, contours);
			PruneShortContours(contours, min_package_contour_length);
			PrunePeripheralContours(contours, image.size());

			contours_mat = cv::Mat(image.size(), CV_8UC1, cv::Scalar(0));
			cv::drawContours(contours_mat, contours, -1, cv::Scalar(255));
	} else {
		contours_mat = edges;
	}



	std::vector<cv::Vec4i> lines;
	DetectLines(contours_mat, lines);

	for (size_t i = 0; i < lines.size(); i++) {
		cv::Vec4i l = lines[i];
		cv::line(result, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 255), 1, CV_AA);
		cv::circle(result, cv::Point(l[0], l[1]), 2, cv::Scalar(255, 0, 0), 2);
		cv::circle(result, cv::Point(l[2], l[3]), 2, cv::Scalar(255, 0, 0), 2);
	}
}

void DrawPolygons() {
	std::vector<std::vector<cv::Point>> contours;
	FindContours(edges, external_contours_only, contours);
	PruneShortContours(contours, min_contour_length);
	PrunePeripheralContours(contours, image.size());
	std::vector<std::vector<cv::Point>> polygons;
	FindConvexPolygons(contours, polygons);
	DrawContours(result, polygons, cv::Scalar(255, 0, 0));
}

void DrawPaper() {
	std::vector<cv::Point> int_paper;
	for(auto point:paper) {
		int_paper.push_back(cv::Point(point.x,point.y));
	}
	DrawContour(result, int_paper, cv::Scalar(255, 0, 255));
}

void DrawPackage() {
	for (int i = 0; i < package.size(); ++i) {
		cv::circle(result, package[i], 2, cv::Scalar(0, 0, 255), 3);
	}
}

void CreateWindow() {
	std::string CONTOURS_WINDOW = "Contours";
	std::string HOUGH_WINDOW = "Hough";
	std::string DETECTION_WINDOW = "Detection";

	cv::namedWindow(RESULT_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(CONTOURS_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(HOUGH_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(DETECTION_WINDOW, CV_WINDOW_AUTOSIZE);

	cv::Mat width_enforcer = cv::Mat(cv::Size(800, 1), CV_8UC1);
	width_enforcer = cv::Scalar::all(0);
	cv::imshow(CONTOURS_WINDOW, width_enforcer);
	cv::imshow(HOUGH_WINDOW, width_enforcer);
	cv::imshow(DETECTION_WINDOW, width_enforcer);

	cv::createTrackbar("Show Edges:", CONTOURS_WINDOW, &show_edges, 1, ProcessImage);
	cv::createTrackbar("Show Contours:", CONTOURS_WINDOW, &show_contours, 1, ProcessImage);
	cv::createTrackbar("External contours:", CONTOURS_WINDOW, &external_contours_only, 1, ProcessImage);
	cv::createTrackbar("Canny low:", CONTOURS_WINDOW, &CANNY_LOW_THRESHOLD, 100, ProcessImage);
	cv::createTrackbar("Canny ratio:", CONTOURS_WINDOW, &canny_ratio, 50, ProcessImage);
	cv::createTrackbar("Morph radius:", CONTOURS_WINDOW, &MORPH_RADIUS, 10, ProcessImage);
	cv::createTrackbar("Morph iters:", CONTOURS_WINDOW, &MORPH_ITERATIONS, 20, ProcessImage);
	cv::createTrackbar("Min contour len:", CONTOURS_WINDOW, &min_contour_length, 1000, ProcessImage);
	cv::createTrackbar("Contour peripheral:", CONTOURS_WINDOW, &center_threshold, 100, ProcessImage);

	cv::createTrackbar("Show Hough lines:", HOUGH_WINDOW, &show_lines, 1, ProcessImage);
	cv::createTrackbar("Hough rho:", HOUGH_WINDOW, &hough_rho, 10, ProcessImage);
	cv::createTrackbar("Hough theta:", HOUGH_WINDOW, &hough_theta, 100, ProcessImage);
	cv::createTrackbar("Hough thresh", HOUGH_WINDOW, &hough_threshold, 1000, ProcessImage);
	cv::createTrackbar("Hough min len:", HOUGH_WINDOW, &hough_min_length, 100, ProcessImage);
	cv::createTrackbar("Hough max gap:", HOUGH_WINDOW, &HOUGH_MAX_GAP, 100, ProcessImage);

	cv::createTrackbar("Show Polygons:", DETECTION_WINDOW, &show_polygons, 1, ProcessImage);
	cv::createTrackbar("Show Paper:", DETECTION_WINDOW, &show_paper, 1, ProcessImage);
	cv::createTrackbar("Poly error:", DETECTION_WINDOW, &poly_error_tolerance, 500, ProcessImage);
	cv::createTrackbar("Package min contour: ", DETECTION_WINDOW, &min_package_contour_length, 500,
			ProcessImage);
	cv::createTrackbar("Paper min contour: ", DETECTION_WINDOW, &min_paper_contour_length, 100, ProcessImage);
	cv::createTrackbar("Show package", DETECTION_WINDOW, &show_package, 1, ProcessImage);

}

void DrawContours(cv::Mat canvas, std::vector<std::vector<cv::Point>> contours, cv::Scalar color) {
	srand(time(NULL));
	for (int i = 0; i < contours.size(); ++i) {
		DrawContour(canvas, contours[i], color);
	}
}

void DrawContour(cv::Mat canvas, std::vector<cv::Point> contour, cv::Scalar color) {

	if (color == cv::Scalar()) {
		int r = rand() % 256;
		int g = rand() % 256;
		int b = rand() % 256;
		color = cv::Scalar(b, g, r);
	}

	std::vector<std::vector<cv::Point>> contour_vec;
	contour_vec.push_back(contour);

	cv::drawContours(canvas, contour_vec, -1, color, 2);
}

void onMouse(int event, int x, int y, int flags, void* param) {
	char text[100];
	sprintf(text, "x=%d, y=%d", x, y);
	cv::Mat img2 = result.clone();
	cv::rectangle(img2, cv::Point(0, 0), cv::Point(200, 30), cv::Scalar(255, 255, 255), -1);
	cv::putText(img2, text, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 0, 0));
	cv::imshow(RESULT_WINDOW, img2);
}

void PrintConstants() {
	cout << "Settings:" << endl;
	cout << "Canny:" << endl;
	cout << "\t" << "Low threshold: " << CANNY_LOW_THRESHOLD << endl;
	cout << "\t" << "Ratio: " << CANNY_RATIO << endl;
	cout << "Morphology" << endl;
	cout << "\t" << "Element radius: " << MORPH_RADIUS << endl;
	cout << "\t" << "Iterations: " << MORPH_ITERATIONS << endl;
	cout << "Contours" << endl;
	cout << "\t" << "Min length: " << min_contour_length << endl;
	cout << "\t" << "Peripheral: " << CENTER_THRESHOLD << endl;
	cout << "Polygon" << endl;
	cout << "\t" << "Error margin: " << POLY_ERROR_TOLERANCE << endl;
	cout << "Hough:" << endl;
	cout << "\t" << "Theta: " << HOUGH_THETA << endl;
	cout << "\t" << "Rho: " << HOUGH_RHO << endl;
	cout << "\t" << "Thresh: " << HOUGH_THRESHOLD << endl;
	cout << "\t" << "Min length: " << HOUGH_MIN_LENGTH << endl;
	cout << "\t" << "Max gap: " << HOUGH_MAX_GAP << endl;
	cout << "Paper:" << endl;
	cout << "\t" << "Min contour length: " << MIN_PAPER_CONTOUR_LENGTH << endl;
	cout << "Package:" << endl;
	cout << "\t" << "Min contour length: " << MIN_PACKAGE_CONTOUR_LENGTH << endl;
}

int main(int argc, char** argv) {
	if (argc < 2) {
		cout << "Missing parameter: image path." << endl;
		return -1;
	}

	image = cv::imread(argv[1]);

	if (!image.data) {
		cout << "Invalid image." << endl;
		return -1;
	}

	//if (argc > 2 && (strcmp(argv[2], "-p") == 0 || strcmp(argv[2], "-P") == 0))
	//	simple_mode = true;

	result.create(image.size(), CV_8UC3);

	CreateWindow();

	ProcessImage(0, 0);
	cv::setMouseCallback(RESULT_WINDOW, onMouse, 0);

	int key = 0;
	while (key != 27) // escape
		key = cv::waitKey(0);
}
