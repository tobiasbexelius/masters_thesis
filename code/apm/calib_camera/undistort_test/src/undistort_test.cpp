#include <opencv2/opencv.hpp>
#include <iostream>

#include <dirent.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <vector>

const double f_x_calib = 2.7017808972707262e+03; //8.4792405910771959e+02;
const double f_y_calib = 2.7017808972707262e+03; //8.4792405910771959e+02;
const double c_x_calib = 1.6315000000000000e+03; //3.8350000000000000e+02;
const double c_y_calib = 1.2235000000000000e+03; //5.1150000000000000e+02;
int target_width = 768;
int target_height = 1024;
int calib_width = 2448;
int calib_height = 3264;
const cv::Mat distortion_params =
		(cv::Mat_<double>(5, 1) << 3.5609569440893447e-02, 5.2417477793983713e-01, 0, 0, -1.6951251211390113e+00);
//(cv::Mat_<double>(5, 1) << 3.3562504804895275e-02, 5.3683800076921451e-01, 0, 0, -1.7276201071408981e+00);

inline void GetFiles(std::vector<std::string> &out, const std::string &directory);
inline bool EndsWith(std::string const &fullString, std::string const &ending);
int main(int argc, char** argv) {

	if (argc != 2) {
		std::cout << "Usage: ./undistort_test image_dir" << std::endl;
		return -1;
	}

	std::string image_dir = argv[1];

	double f_x = f_x_calib * target_width / calib_width;
	double c_x = c_x_calib * target_width / calib_width;
	double f_y = f_y_calib * target_height / calib_height;
	double c_y = c_y_calib * target_height / calib_height;

	cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1);

	std::vector<std::string> files;
	GetFiles(files, image_dir);

	for (int i = 0; i < files.size(); ++i) {
		if (!EndsWith(files[i], ".jpg"))
			continue;
		cv::Mat image = cv::imread(files[i]);
		cv::Mat undistorted_image;
		cv::undistort(image, undistorted_image, camera_matrix, distortion_params);

		cv::imshow("Undistort image", undistorted_image);
		cv::waitKey(0);
	}
}

inline bool EndsWith(std::string const &fullString, std::string const &ending) {
	if (fullString.length() >= ending.length()) {
		return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
	} else {
		return false;
	}
}

/* Returns a list of files in a directory (except the ones that begin with a dot) */
inline void GetFiles(std::vector<std::string> &out, const std::string &directory) {
	DIR *dir;
	class dirent *ent;
	class stat st;

	dir = opendir(directory.c_str());
	while ((ent = readdir(dir)) != NULL) {
		const std::string file_name = ent->d_name;

		std::string separator = "";
		if (!EndsWith(directory, "/"))
			separator += "/";

		const std::string full_file_name = directory + separator + file_name;
		if (file_name[0] == '.')
			continue;

		if (stat(full_file_name.c_str(), &st) == -1)
			continue;

		const bool is_directory = (st.st_mode & S_IFDIR) != 0;

		if (is_directory)
			continue;

		out.push_back(full_file_name);
	}
	closedir(dir);
} // GetFilesInDirectory
