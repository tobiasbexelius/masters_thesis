#ifndef TEST_INCLUDE_FILEUTIL_H_
#define TEST_INCLUDE_FILEUTIL_H_

#include <dirent.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <stdio.h>

#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

bool endsWith(std::string const &fullString, std::string const &ending) {
	if (fullString.length() >= ending.length()) {
		return (0
				== fullString.compare(fullString.length() - ending.length(),
						ending.length(), ending));
	} else {
		return false;
	}
}

/* Returns a list of files in a directory (except the ones that begin with a dot) */
void getFilesInDirectory(std::vector<std::string> &out,
		const std::string &directory) {
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
		const std::string file_name = ent->d_name;

		std::string separator = "";
		if (!endsWith(directory, "/"))
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
#endif
} // GetFilesInDirectory

std::string getCWD() {
	char currentPath[FILENAME_MAX];
	if (!GetCurrentDir(currentPath, sizeof(currentPath))) {
		std::cerr
				<< "An error occurred while getting current working directory."
				<< std::endl;
		throw errno;
	}
	currentPath[sizeof(currentPath) - 1] = '\0';
	return currentPath;

}

#endif /* TEST_INCLUDE_FILEUTIL_H_ */
