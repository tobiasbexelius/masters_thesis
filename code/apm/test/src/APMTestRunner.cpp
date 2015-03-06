#include "json/json.h"
#include "../include/APMTestCase.h"
#include "../include/APMTest.h"
#include "../include/FileUtil.h"
#include "../../lib/include/PackageMeasurer.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdexcept>
#include <cstdlib>
#include <iomanip>

namespace apm = automatic_package_measuring;

using std::cout;
using std::cerr;
using std::endl;

std::vector<std::string> getJsonFiles();
void parseTestCases();
void runTests(std::vector<Json::Value> test_cases);
apm::APMTestCase createTestCase(Json::Value root);
bool endsWith(std::string const &fullString, std::string const &ending);
void help();
std::string toStrMaxDecimals(double value, int decimals);

std::string directory;
std::vector<std::string> files;
std::vector<Json::Value> test_cases;

int main(int argc, char** argv) {

	if (argc < 2) {
		std::cout << "No path specified, using current working directory."
				<< std::endl;
		directory = getCWD();
	} else {
		directory = argv[1];
	}

	if (!endsWith(directory, "/"))
		directory += "/";

	files = getJsonFiles();

	if (files.empty()) {
		cout << "No .json files were found in directory \"" << directory
				<< "\"." << endl;
		help();
		cout << "Exiting..." << endl;
		exit(EXIT_FAILURE);
	}

	parseTestCases();
	runTests(test_cases);
}

std::vector<std::string> getJsonFiles() {
	std::vector<std::string> files;
	getFilesInDirectory(files, directory);

	// remove files that do not end with ".json"
	files.erase(
			std::remove_if(files.begin(), files.end(),
					[](std::string file) {return !endsWith(file, ".json");}),
			files.end());
	return files;
}

void parseTestCases() {
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

void runTests(std::vector<Json::Value> test_cases) {

	std::cout << std::endl;
	int num_tests = 0;
	int num_passed = 0;
	int num_failed = 0;
	auto it = test_cases.begin();
	for (int i = 0; it != test_cases.end(); ++it, ++i) {
		++num_tests;
		apm::APMTestCase test_case = createTestCase(*it);
		apm::APMTest test(test_case);
		test.run();

		std::cout << "########## Test #" << i << " (" << files[i]
				<< " ) ##########" << std::endl;

		std::string ref_object_error = toStrMaxDecimals(test.getReferenceObjectError() * 100.0, 2) + "%";
		std::string package_error = toStrMaxDecimals(test.getPackageError() * 100.0, 2) + "%";
		std::string measurement_error = toStrMaxDecimals(test.getMeasurementError()* 100.0, 2) + "%";

		std::cout << "Reference object: " << (test.isReferenceObjectCorrect() ? "PASS" : "FAIL") << " (error: " << ref_object_error << ")" << std::endl;
		std::cout << "Package: " << (test.isPackageCorrect() ? "PASS" : "FAIL") << " (error: " << package_error << ")" << std::endl;
		std::cout << "Measurement: " << (test.isMeasurementCorrect() ? "PASS" : "FAIL") << " (error: " << measurement_error << ")" << std::endl;


		if (test.success()) {
			++num_passed;
			cout << "Test " << num_tests << " PASSED." << endl;
		} else {
			++num_failed;
			cout << "Test " << num_tests << " FAILED." << endl;
		}

		double success_rate = ((double) num_passed / num_tests);

		cout << "########## All tests finished. ##########" << endl;
		cout << "Statistics:" << endl;
		cout << "Success rate: " << success_rate << " (" << num_passed << ")"
				<< std::endl;
		cout << "Fail rate: " << 1.0 - success_rate << " (" << num_failed << ")"
				<< endl;
		cout << "Total: " << num_tests << endl;

		std::cout << std::endl;
	}
}

std::string toStrMaxDecimals(double value, int decimals)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(decimals) << value;
    std::string s = ss.str();
    if(decimals > 0 && s[s.find_last_not_of('0')] == '.') {
        s.erase(s.size() - decimals + 1);
    }
    return s;
}

apm::APMTestCase createTestCase(Json::Value root) {
	std::string file_name = root["fileName"].asString();

	cv::Mat image = cv::imread(directory + file_name);

	if (!image.data) {
		throw std::invalid_argument("Invalid image: " + directory + file_name);
	}

	std::vector<cv::Point2i> reference_object;
	int num_points = root["referenceObject"].size();

	for (int i = 0; i < num_points; ++i) {
		int x = (int) root["referenceObject"][i]["x"].asInt();
		int y = (int) root["referenceObject"][i]["y"].asInt();
		reference_object.push_back(cv::Point2i(x, y));
	}

	std::vector<cv::Point2i> package;
	num_points = root["package"].size();

	for (int i = 0; i < num_points; ++i) {
		int x = (int) root["package"][i]["x"].asInt();
		int y = (int) root["package"][i]["y"].asInt();
		package.push_back(cv::Point2i(x, y));
	}

	double width = root["dimensions"]["width"].asDouble();
	double height = root["dimensions"]["height"].asDouble();
	double depth = root["dimensions"]["depth"].asDouble();
	cv::Vec3d dimensions = cv::Vec3f(width, height, depth);
	return apm::APMTestCase(image, reference_object, package, dimensions);
}

void help() {
	cout << "Usage: ./apm_test [dir]" << endl;
	cout
			<< "where dir is the path to a directory with .json files which describe the test data."
			<< endl;
	cout << "If no path is specified the current working directory is used."
			<< endl;
}
