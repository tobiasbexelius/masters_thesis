#include "json/json.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <stdexcept>
#include <cstdlib>
#include <iomanip>
#include <numeric>
#include "../../include/apm_test.h"
#include "../../include/apm_test_case.h"
#include "../../include/file_util.h"
#include "../../../lib/include/package_measurer.h"

using namespace automatic_package_measuring;

using std::cout;
using std::cerr;
using std::endl;

struct TestData {
	std::string name;

	int n;
	double acc_n;

	std::vector<int> b;
	std::vector<int> d;
	std::vector<int> h;
	std::vector<int> a;
	std::vector<std::vector<std::vector<int>>>v;

	std::vector<double> acc_b;
	std::vector<double> acc_d;
	std::vector<double> acc_h;
	std::vector<double> acc_a;
	std::vector<std::vector<std::vector<double>>> acc_v;

};
std::vector<std::string> GetJsonFiles();
void ParseAllTests();
void ParseTestCases();
APMTestCase CreateTestCase(Json::Value root);
void PrintTests(std::vector<Json::Value> test_cases);
std::vector<std::string> split(const std::string &s, char delim);
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
void ParseTestConfiguration(std::string file_name, int &box, int &dist, int &height, int &angle);
void UpdateTestData(int box, int dist, int height, int angle, TestData &data, double error);
void InitTestData(TestData* test);
void PrintTestData(int num_tests, TestData data);
double CalcVAcc(const std::vector<std::vector<double>>& acc_v, const std::vector<std::vector<int>>& v, int i,
		int j);
double CalcAcc(double err, int n);
std::vector<double> CalcSuccessRates(std::vector<int> counts, int categories, int total);
std::vector<std::string> dirs = { "/Users/tobias/masters_thesis/code/apm/test/data/the_test/json/flytt",
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/json/wood",
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/json/kaffe",
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/json/byrå_trä",
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/json/byrå_metall" };

std::string directory;
std::vector<std::string> files;
std::vector<Json::Value> test_cases;
APMTestCase CreateTestCase(Json::Value root);

int main(int argc, char** argv) {

	ParseAllTests();
	PrintTests(test_cases);

}

void PrintTests(std::vector<Json::Value> test_cases) {

	TestData r, p, m, cm, km, ckm, rp, o_r, o_p, o_rp, o_m, o_cm, o_km, o_ckm;

	int num_tests = test_cases.size();
	for (TestData* test : { &r, &p, &rp, &m, &cm, &km, &ckm, &o_r, &o_p, &o_rp, &o_m, &o_cm, &o_km, &o_ckm }) {
		InitTestData(test);
	}

	r.name = "Reference object";
	p.name = "Package";
	rp.name = "Reference Object and Package";
	m.name = "Measurement";
	cm.name = "Calib measurement";
	km.name = "Key measurement";
	ckm.name = "Calib key measurement";
	o_r.name = "Optimal reference object";
	o_p.name = "Optimal package";
	o_rp.name = "Optimal reference object and package";
	o_m.name = "Optimal mesasurement";
	o_cm.name = "Optimal calib measurement";
	o_km.name = "Optimal key measurement";
	o_ckm.name = "Optimal calib key measurement";

	for (auto it = test_cases.begin(); it != test_cases.end(); ++it) {
		APMTestCase test_case = CreateTestCase(*it);
		APMTest test(test_case, 0.1, PackageMeasurer());
		test.run();
		int box, dist, height, angle;
		ParseTestConfiguration(test_case.GetFileName(), box, dist, height, angle);
		bool optimal_pose = (angle == 2 || angle == 1 || angle == 3) /*&& (dist == 0 || dist == 1) && (height == 1 || height == 2)*/ ? true : false;

		if (test.isReferenceObjectCorrect()) {
			UpdateTestData(box, dist, height, angle, r, test.getReferenceObjectError());
			if (optimal_pose)
				UpdateTestData(box, dist, height, angle, o_r, test.getReferenceObjectError());
		}

		if (test.isPackageCorrect()) {
			UpdateTestData(box, dist, height, angle, p, test.getPackageError());
			if (optimal_pose)
				UpdateTestData(box, dist, height, angle, o_p, test.getPackageError());
		}

		if (test.isReferenceObjectCorrect() && test.isPackageCorrect()) {
			UpdateTestData(box, dist, height, angle, rp, 0);
			if (optimal_pose)
				UpdateTestData(box, dist, height, angle, o_rp, 0);
		}

		if (test.isMeasurementCorrect()) {
			UpdateTestData(box, dist, height, angle, m, test.getMeasurementError());
			if (optimal_pose)
				UpdateTestData(box, dist, height, angle, o_m, test.getMeasurementError());
		}

		if (test.isCalibMeasurementCorrect()) {
			UpdateTestData(box, dist, height, angle, cm, test.getCalibMeasurementError());
			if (optimal_pose)
				UpdateTestData(box, dist, height, angle, o_cm, test.getMeasurementError());
		}

		if (test.isKeyMeasurementCorrect()) {
			UpdateTestData(box, dist, height, angle, km, test.getKeyMeasurementError());
			if (optimal_pose)
				UpdateTestData(box, dist, height, angle, o_km, test.getMeasurementError());
		}

		if (test.isCalibKeyMeasurementCorrect()) {
			UpdateTestData(box, dist, height, angle, ckm, test.getCalibKeyMeasurementError());
			if (optimal_pose)
				UpdateTestData(box, dist, height, angle, o_ckm, test.getMeasurementError());
		}

	}

	std::cout << "\t\t\tTEST COMPLETE" << std::endl;
	for (auto test : { r, p, rp, m, cm, km, ckm })
		PrintTestData(num_tests, test);

	std::cout << "\t\t\tOptimal pose results" << std::endl;
	for (auto test : { o_r, o_p, o_rp, o_m, o_cm, o_km, o_ckm })
		PrintTestData(60, test);

}

void PrintTestData(int num_tests, TestData data) {
	std::cout << "\t\t\tResults for " << data.name << std::endl;
	std::cout << "Success rates: " << std::endl;

	std::vector<double> box_rates = CalcSuccessRates(data.b, 5, num_tests);
	std::vector<double> position_rates = CalcSuccessRates(data.a, 5, num_tests);
	std::vector<double> height_rates = CalcSuccessRates(data.h, 3, num_tests);
	std::vector<double> distance_rates = CalcSuccessRates(data.d, 3, num_tests);
	printf("Overall:\t\t%.2f\n", data.n / (double) num_tests);
	printf("Box:\t\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", box_rates[0], box_rates[1], box_rates[2], box_rates[3],
			box_rates[4]);
	printf("Position:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", position_rates[0], position_rates[1],
			position_rates[2], position_rates[3], position_rates[4]);
	printf("Distance:\t%.2f\t%.2f\t%.2f\n", distance_rates[0], distance_rates[1], distance_rates[2]);
	printf("Height:\t\t%.2f\t%.2f\t%.2f\n", height_rates[0], height_rates[1], height_rates[2]);
//	printf("Angle:\n");
//	for (auto &e : data.v) {
//		std::vector<double> r1 = CalcSuccessRates(e[0], 9, num_tests / 5);
//		std::vector<double> r2 = CalcSuccessRates(e[1], 9, num_tests / 5);
//		std::vector<double> r3 = CalcSuccessRates(e[2], 9, num_tests / 5);
//		printf("\t\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", r1[0], r1[1], r1[2], r2[0],
//				r2[1], r2[2], r3[0], r3[1], r3[2]);
//	}
	std::cout << std::endl;
	std::cout << "Error: " << std::endl;
	printf("Overall:\t\t%.2f\n", data.acc_n / data.n);
	printf("Box:\t\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", CalcAcc(data.acc_b[0], data.b[0]),
			CalcAcc(data.acc_b[1], data.b[1]), CalcAcc(data.acc_b[1], data.b[1]),
			CalcAcc(data.acc_b[3], data.b[3]), CalcAcc(data.acc_b[4], data.b[4]));
	printf("Position:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", CalcAcc(data.acc_a[0], data.a[0]),
			CalcAcc(data.acc_a[1], data.a[1]), CalcAcc(data.acc_a[2], data.a[2]),
			CalcAcc(data.acc_a[3], data.a[3]), CalcAcc(data.acc_a[4], data.a[4]));
	printf("Distance:\t%.2f\t%.2f\t%.2f\n", CalcAcc(data.acc_d[0], data.d[0]),
			CalcAcc(data.acc_d[1], data.d[1]), CalcAcc(data.acc_d[2], data.d[2]));
	printf("Height:\t\t%.2f\t%.2f\t%.2f\n", CalcAcc(data.acc_h[0], data.h[0]),
			CalcAcc(data.acc_h[1], data.h[1]), CalcAcc(data.acc_h[2], data.h[2]));
//	printf("Angle:\n");
//	for (int i = 0; i < data.acc_v.size(); ++i)
//		printf("\t\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",
//				CalcVAcc(data.acc_v[i], data.v[i], 0, 0), CalcVAcc(data.acc_v[i], data.v[i], 0, 1),
//				CalcVAcc(data.acc_v[i], data.v[i], 0, 2), CalcVAcc(data.acc_v[i], data.v[i], 1, 0),
//				CalcVAcc(data.acc_v[i], data.v[i], 1, 1), CalcVAcc(data.acc_v[i], data.v[i], 1, 2),
//				CalcVAcc(data.acc_v[i], data.v[i], 2, 0), CalcVAcc(data.acc_v[i], data.v[i], 2, 1),
//				CalcVAcc(data.acc_v[i], data.v[i], 2, 2));

	std::cout << std::endl;
}

std::vector<double> CalcSuccessRates(std::vector<int> counts, int categories, int total) {
	std::vector<double> success_rates;
	for (auto const &count : counts) {
		success_rates.push_back(((double) count) / (((double) total) / categories));
	}

	return success_rates;
}

double CalcAcc(double err, int n) {
	return n != 0 ? err / n : 0;
}

double CalcVAcc(const std::vector<std::vector<double>>& acc_v, const std::vector<std::vector<int>>& v, int i,
		int j) {
	return v[i][j] != 0 ? acc_v[i][j] / v[i][j] : 0;
}

void UpdateTestData(int box, int dist, int height, int angle, TestData &data, double error) {

	++data.n;
	data.acc_n += error;

	++data.b[box];
	++data.d[dist];
	++data.h[height];
	++data.a[angle];
	++data.v[box][dist][height];
	data.acc_b[box] += error;
	data.acc_d[dist] += error;
	data.acc_h[height] += error;
	data.acc_a[angle] += error;
	data.acc_v[box][dist][height] += error;
}

void InitTestData(TestData* test) {
	test->n = 0;
	test->acc_n = 0;
	test->b = std::vector<int>(5);
	test->d = std::vector<int>(3);
	test->h = std::vector<int>(3);
	test->a = std::vector<int>(5);
	test->v = std::vector<std::vector<std::vector<int>>>(5);
	for (auto &e : test->v) {
		e = std::vector<std::vector<int>>(3);
		for (auto &e2 : e) {
			e2 = std::vector<int>(3);
		}
	}
	test->acc_b = std::vector<double>(5);
	test->acc_d = std::vector<double>(3);
	test->acc_h = std::vector<double>(3);
	test->acc_a = std::vector<double>(5);
	test->acc_v = std::vector<std::vector<std::vector<double>>>(5);
	for (auto &e : test->acc_v) {
		e = std::vector<std::vector<double>>(3);
		for (auto &e2 : e) {
			e2 = std::vector<double>(3);
		}
	}
}

void ParseTestConfiguration(std::string file_name, int &box, int &dist, int &height, int &angle) {
	std::vector<std::string> tokenized_file_name = split(file_name, '/');

	std::string box_name = tokenized_file_name[tokenized_file_name.size() - 2];

	std::string flytt = u8"flytt", wood = u8"wood", kaffe = u8"kaffe", byra_tra = u8"byrå_trä", byra_metall = u8"byrå_metall";
	if (std::strcmp(box_name.c_str(), flytt.c_str()) == 0)
		box = 0;
	else if (std::strcmp(box_name.c_str(), wood.c_str()) == 0)
		box = 1;
	else if (std::strcmp(box_name.c_str(), kaffe.c_str()) == 0)
		box = 2;
	else if (std::strcmp(box_name.c_str(), byra_tra.c_str()) == 0)
		box = 3;
	else if (std::strcmp(box_name.c_str(), byra_metall.c_str()) == 0)
		box = 4;
	else
		std::cout << "no match: " << box_name << std::endl;

	std::string only_file_name = tokenized_file_name[tokenized_file_name.size() - 1];
	tokenized_file_name = split(only_file_name, '.');
	std::string configuration = tokenized_file_name[0];
	tokenized_file_name = split(configuration, '_');
	dist = std::atoi(tokenized_file_name[0].c_str()) - 1;
	height = std::atoi(tokenized_file_name[1].c_str()) - 1;
	angle = std::atoi(tokenized_file_name[2].c_str());
	if (angle == 1 || angle == 9)
		angle = 0;
	else if (angle == 2 || angle == 8 || angle == 10 || angle == 16)
		angle = 1;
	else if (angle == 3 || angle == 7 || angle == 11 || angle == 15)
		angle = 2;
	else if (angle == 4 || angle == 6 || angle == 12 || angle == 14)
		angle = 3;
	else if (angle == 5 || angle == 13)
		angle = 4;
	else
		std::cout << "no match " << angle << std::endl;
}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}

std::vector<std::string> GetJsonFiles() {
	std::vector<std::string> files;
	GetFilesInDirectory(files, directory);

// remove files that do not end with ".json"
	files.erase(
			std::remove_if(files.begin(), files.end(),
					[](std::string file) {return !EndsWith(file, ".json");}), files.end());
	return files;
}

void ParseAllTests() {
	for (auto dir : dirs) {
		directory = dir;
		if (!EndsWith(directory, "/"))
			directory += "/";
		files = GetJsonFiles();
		ParseTestCases();
	}
}

void ParseTestCases() {
	for (auto it = files.begin(); it != files.end(); ++it) {
		Json::Value root;
		Json::Reader reader;
		std::ifstream input_stream(*it, std::ifstream::binary);
		bool parsing_successful = reader.parse(input_stream, root, false);

		if (!parsing_successful) {
			cerr << "An error occurred while parsing " << *it << ":" << endl;
			cerr << reader.getFormatedErrorMessages() << "\n";
			cerr << "Exiting...";
			exit(EXIT_FAILURE);
		}
		test_cases.push_back(root);
	}
}

APMTestCase CreateTestCase(Json::Value root) {
	std::string file_name = root["fileName"].asString();

	cv::Mat image = cv::imread(file_name);

	if (!image.data) {
		throw std::invalid_argument("Invalid image: " + file_name);
	}

	std::vector<cv::Point2f> reference_object;
	int num_points = root["referenceObject"].size();
	for (int i = 0; i < num_points; ++i) {
		int x = (int) root["referenceObject"][i]["x"].asInt();
		int y = (int) root["referenceObject"][i]["y"].asInt();
		reference_object.push_back(cv::Point2f(x, y));
	}

	std::vector<cv::Point2f> package;
	num_points = root["package"].size();

	for (int i = 0; i < num_points; ++i) {
		int x = (int) root["package"][i]["x"].asInt();
		int y = (int) root["package"][i]["y"].asInt();
		package.push_back(cv::Point2f(x, y));
	}

	double width = root["dimensions"]["width"].asDouble();
	double height = root["dimensions"]["height"].asDouble();
	double depth = root["dimensions"]["depth"].asDouble();
	cv::Vec3d dimensions = cv::Vec3f(width, height, depth);

	cv::Vec2f reference_object_size = cv::Vec2f(root["referenceObjectSize"][0].asFloat(),
			root["referenceObjectSize"][1].asFloat());
	int rotation = root["rotation"].asInt();

	APMTestCase test_case = APMTestCase(image, reference_object, package, dimensions, reference_object_size,
			rotation);
	test_case.SetFileName(file_name);

	return test_case;
}
