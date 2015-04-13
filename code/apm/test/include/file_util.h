#ifndef TEST_INCLUDE_FILEUTIL_H_
#define TEST_INCLUDE_FILEUTIL_H_

#include <dirent.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <vector>

inline std::string AbsolutePath(std::string relative_path) {
    char resolved_path[200];
	realpath(relative_path.c_str(), resolved_path);
	return std::string(resolved_path);
}

inline bool FileExists (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

inline bool EndsWith(std::string const &fullString, std::string const &ending) {
	if (fullString.length() >= ending.length()) {
		return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
	} else {
		return false;
	}
}

/* Returns a list of files in a directory (except the ones that begin with a dot) */
inline void GetFilesInDirectory(std::vector<std::string> &out, const std::string &directory) {
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

inline std::string GetCWD() {
	char currentPath[FILENAME_MAX];
	if (!getcwd(currentPath, sizeof(currentPath))) {
		std::cerr << "An error occurred while getting current working directory." << std::endl;
		throw errno;
	}
	currentPath[sizeof(currentPath) - 1] = '\0';
	return currentPath;

}

#endif /* TEST_INCLUDE_FILEUTIL_H_ */
