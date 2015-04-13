#include "../../include/file_util.h"
#include "../../include/apm_test_case.h"
#include <opencv2/opencv.hpp>
#include "json/json.h"
#include <stdexcept>
#include <cstdlib>

using std::cout;
using std::endl;
namespace apm = automatic_package_measuring;

const std::string WINDOW_NAME = "Test Creator";
cv::Mat image;
int mouse_x;
int mouse_y;
int num_clicks = 0;

std::string reference_object_type = "A4";
int package_id = -1;
double package_width = -1;
double package_height = -1;
double package_depth = -1;
double package_distance = -1;

apm::APMTestCase test_case;

void ShowOverlay();
std::string GetOutputFileName(std::string input_dir, std::string input_file);
std::vector<std::string> GetImages(std::string directory);
void CreateTest(std::string input_dir, std::string output_dir, std::string image);
void onMouse(int event, int x, int y, int flags, void* param);

int main(int argc, char** argv) {

	if (argc < 3) {
		std::cout << "Usage: ./test_creator input_dir output_dir [ref_type] [package_id] [width] [height] [depth] [dist]" << std::endl;
		exit(EXIT_FAILURE);
	}

	std::string input_dir = argv[1];
	std::string output_dir = argv[2];

	if (argc > 3)
		reference_object_type = argv[3];
	if (argc > 4)
		package_id = std::atoi(argv[4]);
	if (argc > 7) {
		package_width = std::atof(argv[5]);
		package_height = std::atof(argv[6]);
		package_depth = std::atof(argv[7]);
	}
	if (argc > 8) {
		package_distance = std::atof(argv[8]);
	}

	std::vector<std::string> images = GetImages(input_dir);

	if (images.empty()) {
		cout << "No image files were found in directory \"" << input_dir << "\". Exiting..." << endl;
		exit(EXIT_FAILURE);
	}

	for (std::string image : images) {
		CreateTest(input_dir, output_dir, image);
	}
}

void CreateTest(std::string input_dir, std::string output_dir, std::string image_file) {

	if (!EndsWith(output_dir, "/"))
		output_dir += "/";

	num_clicks = 0;

	std::string output_file = GetOutputFileName(input_dir, image_file);

	if(FileExists(output_dir + output_file)) {
		std::cout << "Test file for image: " << image_file << " already exists. (" << output_dir << output_file <<  ")" << std::endl;
		return;
	}

	image = cv::imread(image_file);
	cv::imshow(WINDOW_NAME, image);
	cv::setMouseCallback(WINDOW_NAME, onMouse, 0);
	cv::Vec3d dimensions(package_width, package_height, package_depth);
	test_case = apm::APMTestCase(AbsolutePath(image_file), package_id, reference_object_type, dimensions, package_distance);

	cv::waitKey(0);

	Json::Value json_test_case = test_case.AsJson();

	std::ofstream file;
	file.open(output_dir + output_file);
	file << json_test_case;
	file.close();
	std::cout << "Created file " << output_dir << output_file << "."<< std::endl;

}

std::string GetOutputFileName(std::string input_dir, std::string input_file) {
	input_file.erase(0, input_dir.size());
	int file_extension_pos = input_file.find(".jpg");
	input_file.erase(file_extension_pos, 4);
	return input_file + ".json";
}

void onMouse(int event, int x, int y, int flags, void* param) {

	if (event == cv::EVENT_LBUTTONDOWN) {
		if (num_clicks < 4)
			test_case.AppendReferenceObject(x, y);
		else if (num_clicks < 10)
			test_case.AppnedPackage(x, y);

		++num_clicks;
	}

	mouse_x = x;
	mouse_y = y;
	ShowOverlay();
}

void ShowOverlay() {
	char text[100];
	sprintf(text, "x=%d, y=%d", mouse_x, mouse_y);
	char instruction[100];
	sprintf(instruction, "1. Click on reference object corners. 2. Click on package corners. 3. Press a key.");
	cv::Mat overlay = image.clone();
	cv::rectangle(overlay, cv::Point(0, 0), cv::Point(1000, 50), cv::Scalar(255, 255, 255), -1);
	cv::putText(overlay, text, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 0, 0));
	cv::putText(overlay, instruction, cv::Point(5, 35), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 0, 0));
	cv::imshow(WINDOW_NAME, overlay);
}

std::vector<std::string> GetImages(std::string directory) {
	std::vector<std::string> files;

	if (!EndsWith(directory, "/"))
		directory += "/";

	GetFilesInDirectory(files, directory);

	// remove files that do not end with ".jpg"
	files.erase(
			std::remove_if(files.begin(), files.end(),
					[](std::string file) {return !EndsWith(file, ".jpg");}), files.end());
	return files;
}