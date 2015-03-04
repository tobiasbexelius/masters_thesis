#include "../include/jsoncpp/json/json.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <dirent.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>

#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

using std::string;
using std::cout;
using std::endl;
using std::vector;

string getCWD();
void getFilesInDirectory(std::vector<string> &out, const string &directory);
bool endsWith (std::string const &fullString, std::string const &ending);
void help();

int main(int argc, char** argv) {

	string dir;
	if (argc < 2) {
		std::cout << "No path specified, using current working directory."
				<< std::endl;
		dir = getCWD();
	} else {
		dir = argv[1];
	}

	vector<string> files;
	getFilesInDirectory(files, dir);

	// remove files that do not end with ".json"
	files.erase(std::remove_if(files.begin(), files.end(),
	                       [](string file) { return !endsWith(file, ".json"); }), files.end());

	if(files.empty()) {
		cout << "No .json files were found in directory \"" << dir << "\"." << endl;
		help();
		cout << "Exiting..." << endl;
		return -1;
	}


	return 1;

	Json::Value root;   // will contains the root value after parsing.
	Json::Reader reader;
	std::ifstream test("../test/data/box01.json", std::ifstream::binary);
	bool parsingSuccessful = reader.parse(test, root, false);
	if (!parsingSuccessful) {
		// report to the user the failure and their locations in the document.
		cout << reader.getFormatedErrorMessages() << "\n";
	}

	string encoding = root.get("fileName", "UTF-8").asString();
	cout << encoding << endl;

}

std::string getCWD() {
	char currentPath[FILENAME_MAX];
	if (!GetCurrentDir(currentPath, sizeof(currentPath))) {
		std::cerr
				<< "An error occurred while getting current working directory."
				<< endl;
		throw errno;
	}
	currentPath[sizeof(currentPath) - 1] = '\0';
	return currentPath;

}

bool endsWith (string const &fullString, string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

/* Returns a list of files in a directory (except the ones that begin with a dot) */
void getFilesInDirectory(vector<string> &out, const string &directory) {
#ifdef WINDOWS
	HANDLE dir;
	WIN32_FIND_DATA file_data;

	if ((dir = FindFirstFile((directory + "/*").c_str(), &file_data)) == INVALID_HANDLE_VALUE)
	return; /* No files found */

	do {
		const string file_name = file_data.cFileName;
		const string full_file_name = directory + "/" + file_name;
		const bool is_directory = (file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;

		if (file_name[0] == '.')
		continue;

		if (is_directory)
		continue;

		out.push_back(full_file_name);
	}while (FindNextFile(dir, &file_data));

	FindClose(dir);
#else
	DIR *dir;
	class dirent *ent;
	class stat st;

	dir = opendir(directory.c_str());
	while ((ent = readdir(dir)) != NULL) {
		const string file_name = ent->d_name;
		const string full_file_name = directory + "/" + file_name;

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
#endif
} // GetFilesInDirectory

void help() {
	cout << "Usage: ./apm_test [dir]" << endl;
	cout << "where dir is the path to a directory with .json files which describe the test data." << endl;
	cout << "If no path is specified the current working directory is used." << endl;
}
