#include "../include/apm_test_case.h"

namespace automatic_package_measuring {

APMTestCase::APMTestCase() {
}

APMTestCase::APMTestCase(std::string file_name, int package_id, std::string reference_object_type,
		cv::Vec3f dimensions, double distance) :
		file_name(file_name),
		package_id(package_id),
		reference_object_type(reference_object_type),
		distance(distance),
		dimensions(dimensions) {
}

APMTestCase::APMTestCase(cv::Mat image, std::vector<cv::Point2f> reference_object,
		std::vector<cv::Point2f> package, cv::Vec3f dimensions) :
		image(image),
		reference_object(reference_object),
		package(package),
		dimensions(dimensions) {
}

APMTestCase::~APMTestCase() {
}

const std::vector<cv::Point2f>& APMTestCase::GetReferenceObject() const {
	return reference_object;
}

const std::vector<cv::Point2f>& APMTestCase::GetPackage() const {
	return package;
}

cv::Mat APMTestCase::GetImage() const {
	return image;
}

const cv::Vec3f APMTestCase::GetDimensions() const {
	return dimensions;
}

void APMTestCase::AppendReferenceObject(int x, int y) {
	reference_object.push_back(cv::Point(x, y));
}

void APMTestCase::AppnedPackage(int x, int y) {
	package.push_back(cv::Point(x, y));
}

Json::Value APMTestCase::AsJson() {
	Json::Value root;

	root["fileName"] = file_name;
	root["packageId"] = package_id;
	root["referenceObjectType"] = reference_object_type;
	root["dimensions"]["width"] = dimensions[0];
	root["dimensions"]["height"] = dimensions[1];
	root["dimensions"]["depth"] = dimensions[2];
	root["distance"] = distance;

	for (int i = 0; i < reference_object.size(); ++i) {
		root["referenceObject"][i]["x"] = reference_object[i].x;
		root["referenceObject"][i]["y"] = reference_object[i].y;
	}

	for (int i = 0; i < package.size(); ++i) {
		root["package"][i]["x"] = package[i].x;
		root["package"][i]["y"] = package[i].y;
	}

	return root;
}

} /* namespace automatic_package_measuring */

