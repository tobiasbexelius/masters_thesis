#include "json/json.h"
#include "../../include/file_util.h"
#include "../../../lib/include/package_measurer.h"
#include "../../../lib/include/image_processing.h"
#include "../../../lib/include/image_processing_internal.h"
#include "../../../lib/include/paper_detection_internal.h"
#include "../../../lib/include/package_detection_internal.h"
#include "../../../lib/include/package_measuring_internal.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdexcept>
#include <cstdlib>
#include <iomanip>
#include "../../include/apm_test.h"
#include "../../include/apm_test_case.h"

namespace apm = automatic_package_measuring;
using namespace apm::internal;
using namespace apm;

using std::cout;
using std::cerr;
using std::endl;

template<class T> void optimizeConstantMeasureOnly(std::vector<Json::Value> test_cases,
		std::string constant_name, T * constant, T min, T max, T increment);
void runTestsCompact(std::vector<Json::Value> test_cases);
void TestCouldBeStraight(std::vector<Json::Value> test_cases);
std::vector<std::string> GetJsonFiles();
void ParseTestCases();
void runTests(std::vector<Json::Value> test_cases);
apm::APMTestCase CreateTestCase(Json::Value root);
bool EndsWith(std::string const &fullString, std::string const &ending);
void help();
std::string formatDouble(double value, int decimals);
void printResult(std::string name, bool correct, double error);
template<class T> void optimizeConstant(std::vector<Json::Value> test_cases, std::string constant_name,
		T * constant, T min, T max, T increment);
void printFinalResult(int num_tests, int num_passed, int num_ref_object_correct, int num_packages_correct,
		int num_measurements_correct, int num_ref_and_package_correct);
void showAllImages(std::vector<Json::Value> test_cases);
void ParseAllTests();
void separateReports();
void TheGrandTest(std::vector<Json::Value> test_cases);
void DetectFailMeasurements(std::vector<Json::Value> test_cases);
std::vector<Json::Value> randomCases(std::vector<Json::Value> all_tests, int n);
void runTestsWithKey(std::vector<Json::Value> test_cases);

std::vector<std::string> some_dirs = { "/Users/tobias/masters_thesis/code/apm/test/data/structured/box1",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured/box2",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured/box3",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured/unstructured",
		"/Users/tobias/masters_thesis/code/apm/test/data",
		"/Users/tobias/masters_thesis/code/apm/test/data/s6wooden" };

std::vector<std::string> structured = { "/Users/tobias/masters_thesis/code/apm/test/data/structured2/json",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured3/json",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured4/json",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured5/json",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured6/json" };

std::vector<std::string> everything = { "/Users/tobias/masters_thesis/code/apm/test/data/structured2/json",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured3/json",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured4/json",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured5/json",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured6/json",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured/box1",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured/box2",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured/box3",
		"/Users/tobias/masters_thesis/code/apm/test/data/structured/unstructured",
		"/Users/tobias/masters_thesis/code/apm/test/data",
		"/Users/tobias/masters_thesis/code/apm/test/data/s6wooden" };

std::vector<std::string> s2 = { "/Users/tobias/masters_thesis/code/apm/test/data/structured2/json" };
std::vector<std::string> s3 = { "/Users/tobias/masters_thesis/code/apm/test/data/structured3/json" };
std::vector<std::string> s4 = { "/Users/tobias/masters_thesis/code/apm/test/data/structured4/json" };
std::vector<std::string> s5 = { "/Users/tobias/masters_thesis/code/apm/test/data/structured5/json" };
std::vector<std::string> s6 = { "/Users/tobias/masters_thesis/code/apm/test/data/structured6/json" };

std::vector<std::string> the_test = { "/Users/tobias/masters_thesis/code/apm/test/data/the_test/flytt",
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/wood",
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/kaffe",
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/byrå_trä",
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/byrå_metall" };

std::vector<std::string> the_test_full = {
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/json/flytt",
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/json/wood",
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/json/kaffe",
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/json/byrå_trä",
		"/Users/tobias/masters_thesis/code/apm/test/data/the_test/json/byrå_metall" };

std::vector<std::string> all_dirs = the_test_full;

std::string directory;
std::vector<std::string> files;
std::vector<Json::Value> test_cases;

int myrandom(int i) {
	return std::rand() % i;
}

int main(int argc, char** argv) {

	std::srand(std::time(NULL));

	if (argc < 2) {
		std::cout << "No path specified, all known dirs." << std::endl;
		ParseAllTests();
	} else {
		directory = argv[1];

		if (EndsWith(directory, ".json")) {
			files.push_back(directory);
		} else {
			if (!EndsWith(directory, "/"))
				directory += "/";
			files = GetJsonFiles();
			if (files.empty()) {
				cout << "No .json files were found in directory \"" << directory << "\"." << endl;
				help();
				cout << "Exiting..." << endl;
				exit(EXIT_FAILURE);
			}
		}
		ParseTestCases();
	}
	std::cout << "Total number of tests:" << test_cases.size() << std::endl;

//	DetectFailMeasurements(test_cases);
//	TheGrandTest(test_cases);

//	double MIN_ACCEPTED_SCORE = 50.0;
//	double MAX_PARALLEL_LINE_ANGLE = 30.0;
//	int MAX_LINE_PAIRS = 100;
//	double MIN_ACCEPTED_SUBSCORE = 20.0;

//	apm::internal::CANNY_LOW_THRESHOLD = 30;
//	apm::internal::CANNY_HIGH_THRESHOLD = 30*3;
//	apm::internal::MORPH_ITERATIONS = 1;
//	apm::internal::MORPH_RADIUS = 1;
//	apm::internal::HOUGH_THRESHOLD = 40; // TODO Try bilateral
//apm::internal::BLUR_KERNEL_SIZE = 3;
//apm::internal::CANNY_LOW_THRESHOLD = 56;

//optimizeConstantMeasureOnly(test_cases, "Constant", &apm::internal::CANNY_RATIO, 2.5, 5.0, 0.25);

//optimizeConstantMeasureOnly(test_cases, "Constant", &apm::internal::CENTER_THRESHOLD, 0.15, 0.2, 0.01);

//	optimizeConstant(test_cases, "Constant", &apm::internal::HOUGH_THRESHOLD, 25, 35, 1);
//	runTests(test_cases);
//	runTestsWithKey(test_cases);
//	TestCouldBeStraight(test_cases);
	runTestsCompact(test_cases);
//	separateReports();
	//showAllImages(test_cases);
}

void separateReports() {
	test_cases.clear();
	for (auto dir : all_dirs) {
		directory = dir;
		if (!EndsWith(directory, "/"))
			directory += "/";
		files = GetJsonFiles();
		ParseTestCases();
		runTestsCompact(test_cases);
		test_cases.clear();
	}

}

void ParseAllTests() {
	for (auto dir : all_dirs) {
		directory = dir;
		if (!EndsWith(directory, "/"))
			directory += "/";
		files = GetJsonFiles();
		ParseTestCases();
	}
}

std::vector<Json::Value> randomCases(std::vector<Json::Value> all_tests, int n) {
	std::random_shuffle(all_tests.begin(), all_tests.end(), myrandom);
	std::vector<Json::Value> res;
	for (int i = 0; i < n && i < all_tests.size(); ++i) {
		res.push_back(all_tests[i]);
	}
	return res;
}

void TestCouldBeStraight(std::vector<Json::Value> test_cases) {
	auto it = test_cases.begin();
	int num_tests = 0;
	for (int i = 0; it != test_cases.end(); ++it, ++i) {
		++num_tests;
		apm::APMTestCase test_case = CreateTestCase(*it);
		apm::APMTest test(test_case, 0.1, apm::PackageMeasurer());
		test.run();

		std::vector<cv::Point2f> package = test.getActualPackage();
		int top_left, top_right, mid_left, mid_right, bottom_left, bottom_right;
		if (package.size() == 6) {
			IdentifyPackageCornersHeadOn(package, top_left, top_right, mid_left, mid_right, bottom_left,
					bottom_right);
			bool res = CouldBeStraight(package, top_left, top_right, mid_left, mid_right, bottom_left,
					bottom_right);
			std::string res_text = res ? "TRUE" : "FALSE";
			std::string file = test_case.GetFileName();

			std::string correct = test.isMeasurementCorrect() ? "TRUE" : "FALSE";

			std::cout << test_case.GetFileName() << " res: " << res_text << " measurement: " << correct
					<< std::endl;
		}

	}

}

void DetectFailMeasurements(std::vector<Json::Value> test_cases) {

	auto it = test_cases.begin();
	for (int i = 0; it != test_cases.end(); ++it, ++i) {
		apm::APMTestCase test_case = CreateTestCase(*it);
		apm::APMTest test(test_case, 0.1, apm::PackageMeasurer());
		test.run();

		cv::Vec3f measurements = test.getActualMeasurement();

		if (test.isPackageCorrect() && test.isReferenceObjectCorrect() && !test.isMeasurementCorrect())
			printf("Measure fail: %s. Error: %.2f. [%.2f, %.2f, %.2f]\n", test_case.GetFileName().c_str(),
					test.getMeasurementError(), measurements[0], measurements[1], measurements[2]);
	}
}

void TheGrandTest(std::vector<Json::Value> test_cases) {

	int num_tests = 0;
	int num_passed = 0;
	int num_ref_objects_correct = 0;
	int num_packages_correct = 0;
	int num_ref_and_package_correct = 0;
	auto it = test_cases.begin();
	for (int i = 0; it != test_cases.end(); ++it, ++i) {
		++num_tests;
		apm::APMTestCase test_case = CreateTestCase(*it);
		apm::APMTest test(test_case, 0.1, apm::PackageMeasurer());
		test.run();

		std::string ref_text = test.isReferenceObjectCorrect() ? "Pass" : "Fail";
		std::string package_text = test.isPackageCorrect() ? "Pass" : "Fail";
		std::string m_text = test.isMeasurementCorrect() ? "Pass" : "Fail";

		double ref_err = test.getReferenceObjectError() * 100.0;
		double p_err = test.getPackageError() * 100.0;
		double m_err = test.getMeasurementError() * 100.0;

		std::string calib_text = test.isCalibMeasurementCorrect() ? "Pass" : "Fail";
		std::string key_text = test.isKeyMeasurementCorrect() ? "Pass" : "Fail";
		std::string calib_key_text = test.isCalibKeyMeasurementCorrect() ? "Pass" : "Fail";

		double c_err = test.getCalibMeasurementError() * 100.0;
		double k_err = test.getKeyMeasurementError() * 100.0;
		double ck_err = test.getCalibKeyMeasurementError() * 100.0;

		printf(
				"Test #%d (%s). Ref: %s (%.2f). Package: %s (%.2f). UC: %s (%.2f). C: %s (%.2f), K: %s (%.2f). KC: %s (%.2f).\n",
				num_tests, test_case.GetFileName().c_str(), ref_text.c_str(), ref_err, package_text.c_str(),
				p_err, m_text.c_str(), m_err, calib_text.c_str(), c_err, key_text.c_str(), k_err,
				calib_key_text.c_str(), ck_err);

		if (test.isReferenceObjectCorrect()) {
			++num_ref_objects_correct;
		}

		if (test.isPackageCorrect()) {
			++num_packages_correct;
		}

		if (test.isPackageCorrect() && test.isReferenceObjectCorrect())
			++num_ref_and_package_correct;

		if (test.isMeasurementCorrect()) {
			++num_passed;
		}

	}

	int num_measurements_correct = 0;

	printFinalResult(num_tests, num_passed, num_ref_objects_correct, num_packages_correct,
			num_measurements_correct, num_ref_and_package_correct);
}

void showAllImages(std::vector<Json::Value> test_cases) {
	int i = 0;
	for (auto it = test_cases.begin(); it != test_cases.end(); ++it, ++i) {
		apm::APMTestCase test_case = CreateTestCase(*it);
		cv::Mat processed_image;
		apm::PreprocessImage(test_case.GetImage(), processed_image);
		cv::imshow(std::to_string(i), processed_image);

	}
	cv::waitKey(0);
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

template<class T> void optimizeConstant(std::vector<Json::Value> test_cases, std::string constant_name,
		T * constant, T min, T max, T increment) {

	for (T i = min; i <= max; i += increment) {
		*constant = i;
		apm::internal::CANNY_HIGH_THRESHOLD = apm::internal::CANNY_LOW_THRESHOLD * apm::internal::CANNY_RATIO;
		int num_passed = 0;
		int num_ref_correct = 0;
		int num_package_correct = 0;

		for (auto it = test_cases.begin(); it != test_cases.end(); ++it) {
			apm::APMTestCase test_case = CreateTestCase(*it);
			apm::APMTest test(test_case, 0.1, apm::PackageMeasurer());
			test.run();

			if (test.isReferenceObjectCorrect())
				++num_ref_correct;
			if (test.isPackageCorrect())
				++num_package_correct;

			if (test.isReferenceObjectCorrect() && test.isPackageCorrect())
				++num_passed;
		}

		std::string pass_rate = formatDouble((double) num_passed / test_cases.size(), 2);
		std::string ref_rate = formatDouble((double) num_ref_correct / test_cases.size(), 2);
		std::string package_rate = formatDouble((double) num_package_correct / test_cases.size(), 2);
		std::cout << constant_name << " = " << i << " PASS RATE: " << pass_rate << " (ref:" << ref_rate
				<< ", package:" << package_rate << ")" << std::endl;
	}

}

template<class T> void optimizeConstantMeasureOnly(std::vector<Json::Value> test_cases,
		std::string constant_name, T * constant, T min, T max, T increment) {

	for (T i = min; i <= max; i += increment) {
		*constant = i;
		apm::internal::CANNY_HIGH_THRESHOLD = apm::internal::CANNY_LOW_THRESHOLD * apm::internal::CANNY_RATIO;
		int num_passed = 0;
		int num_ref_correct = 0;
		int num_package_correct = 0;

		for (auto it = test_cases.begin(); it != test_cases.end(); ++it) {
			apm::APMTestCase test_case = CreateTestCase(*it);
			apm::APMTest test(test_case, 0.1, apm::PackageMeasurer());
			test.run();

			if (test.isMeasurementCorrect())
				++num_passed;
		}

		std::string pass_rate = formatDouble((double) num_passed / test_cases.size(), 2);
		std::cout << constant_name << " = " << i << " PASS RATE: " << pass_rate << std::endl;
	}

}

void runTestsCompact(std::vector<Json::Value> test_cases) {

	int num_tests = 0;
	int num_passed = 0;
	int num_ref_objects_correct = 0;
	int num_packages_correct = 0;
	int num_ref_and_package_correct = 0;
	auto it = test_cases.begin();
	for (int i = 0; it != test_cases.end(); ++it, ++i) {
		++num_tests;
		apm::APMTestCase test_case = CreateTestCase(*it);
		apm::APMTest test(test_case, 0.10, apm::PackageMeasurer());
		test.run();

		cv::Vec3f measurements = test.getActualMeasurement();

		std::string mode_str;
		if (test.getActualPackage().size() == 0)
			mode_str = "N/A";
		else
			mode_str = angle ? "3" : "2";

		printf("Test #%d (%s). Sides: %s. Measurements: [%.2f, %.2f, %.2f] Result: %s (%.2f %%)\n", num_tests,
				test_case.GetFileName().c_str(), mode_str.c_str(), measurements[0], measurements[1],
				measurements[2], test.isMeasurementCorrect() ? "PASSED" : "FAILED",
				test.getMeasurementError() * 100.0);

		if (test.isMeasurementCorrect()) {
			++num_passed;
		}
	}

	double success_rate = ((double) num_passed / num_tests) * 100;

	cout << std::endl;
	cout << "########## All tests finished. ##########" << endl;
	cout << "Success rate: " << formatDouble(success_rate, 2) << "% (" << num_passed << ")" << std::endl;
	cout << "Total: " << num_tests << endl;

}

void runTestsWithKey(std::vector<Json::Value> test_cases) {

	int num_tests = 0;
	int num_passed = 0;
	int num_ref_objects_correct = 0;
	int num_packages_correct = 0;
	int num_ref_and_package_correct = 0;
	auto it = test_cases.begin();
	for (int i = 0; it != test_cases.end(); ++it, ++i) {
		++num_tests;
		apm::APMTestCase test_case = CreateTestCase(*it);
		apm::APMTest test(test_case, 0.10, apm::PackageMeasurer());
		test.run();

		std::string mode_str;

		mode_str = angle ? "3" : "2";

		printf("Test #%d (%s). Sides: %s. Result: %s (%.2f %%)\n", num_tests,
				test_case.GetFileName().c_str(), mode_str.c_str(), test.isKeyMeasurementCorrect() ? "PASSED" : "FAILED",
				test.getKeyMeasurementError() * 100.0);

		if (test.isKeyMeasurementCorrect()) {
			++num_passed;
		}
	}

	double success_rate = ((double) num_passed / num_tests) * 100;

	cout << std::endl;
	cout << "########## All tests finished. ##########" << endl;
	cout << "Success rate: " << formatDouble(success_rate, 2) << "% (" << num_passed << ")" << std::endl;
	cout << "Total: " << num_tests << endl;

}

void runTests(std::vector<Json::Value> test_cases) {

	int num_tests = 0;
	int num_passed = 0;
	int num_ref_objects_correct = 0;
	int num_packages_correct = 0;
	int num_ref_and_package_correct = 0;
	auto it = test_cases.begin();
	for (int i = 0; it != test_cases.end(); ++it, ++i) {
		++num_tests;
		apm::APMTestCase test_case = CreateTestCase(*it);
		apm::APMTest test(test_case, 0.1, apm::PackageMeasurer());
		test.run();

		std::string ref_text = test.isReferenceObjectCorrect() ? "Pass" : "Fail";
		std::string package_text = test.isPackageCorrect() ? "Pass" : "Fail";
		std::string measurement_text = test.isMeasurementCorrect() ? "Pass" : "Fail";

		printf("Test #%d (%s). Ref: %s (%.2f). Package: %s (%.2f). Measurement: %s (%.2f)\n", num_tests,
				test_case.GetFileName().c_str(), ref_text.c_str(), test.getReferenceObjectError(),
				package_text.c_str(), test.getPackageError(), measurement_text.c_str(),
				test.getMeasurementError());
//		std::cout << "Test #" << num_tests << " (" << test_case.GetFileName() << " ). Ref: "<< std::endl;

//		printResult("Reference object:\t", test.isReferenceObjectCorrect(), test.getReferenceObjectError());
//		printResult("Package:\t\t", test.isPackageCorrect(), test.getPackageError());
//		printResult("Measurement:\t\t", test.isMeasurementCorrect(), test.getMeasurementError());

		if (test.isReferenceObjectCorrect()) {
			++num_ref_objects_correct;
		}

		if (test.isPackageCorrect()) {
			++num_packages_correct;
		}

		if (test.isPackageCorrect() && test.isReferenceObjectCorrect())
			++num_ref_and_package_correct;

		if (test.isMeasurementCorrect()) {
			++num_passed;
//			cout << "Test #" << num_tests << " PASSED." << endl;
		} else {
//			cout << "Test #" << num_tests << " FAILED." << endl;
		}
	}

	int num_measurements_correct = 0;

	printFinalResult(num_tests, num_passed, num_ref_objects_correct, num_packages_correct,
			num_measurements_correct, num_ref_and_package_correct);
}

void printFinalResult(int num_tests, int num_passed, int num_ref_object_correct, int num_packages_correct,
		int num_measurements_correct, int num_ref_and_package_correct) {
	double success_rate = ((double) num_passed / num_tests) * 100;
	double ref_object_rate = ((double) num_ref_object_correct / num_tests) * 100;
	double package_rate = ((double) num_packages_correct / num_tests) * 100;
	double both_rate = ((double) num_ref_and_package_correct / num_tests) * 100;
	cout << std::endl;
	cout << "########## All tests finished. ##########" << endl;
	cout << "Results:" << endl;
	cout << "Reference object rate: " << formatDouble(ref_object_rate, 2) << "% (" << num_ref_object_correct
			<< ")" << std::endl;
	cout << "Package rate: " << formatDouble(package_rate, 2) << "% (" << num_packages_correct << ")"
			<< std::endl;
	cout << "Ref & pack: " << formatDouble(both_rate, 2) << "% (" << num_ref_and_package_correct << ")"
			<< std::endl;
	cout << "Success rate: " << formatDouble(success_rate, 2) << "% (" << num_passed << ")" << std::endl;
	cout << "Total: " << num_tests << endl;

	std::cout << std::endl;
}

void printResult(std::string name, bool correct, double error) {
	std::string formatted_error = formatDouble(error * 100.0, 2) + "%";
	std::string formatted_result = (correct ? "PASS" : "FAIL");
	std::cout << name << formatted_result << "\t" << " (error: " << formatted_error << ")" << std::endl;
}

std::string formatDouble(double value, int decimals) {
	std::ostringstream ss;
	ss << std::fixed << std::setprecision(decimals) << value;
	std::string s = ss.str();
	if (decimals > 0 && s[s.find_last_not_of('0')] == '.') {
		s.erase(s.size() - decimals + 1);
	}
	return s;
}

apm::APMTestCase CreateTestCase(Json::Value root) {
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

	apm::APMTestCase test_case = apm::APMTestCase(image, reference_object, package, dimensions,
			reference_object_size, rotation);
	test_case.SetFileName(file_name);

	return test_case;
}

void help() {
	cout << "Usage: ./apm_test [dir]" << endl;
	cout << "where dir is the path to a directory with .json files which describe the test data." << endl;
	cout << "If no path is specified the current working directory is used." << endl;
}
