#include "files.h"

#include <cstdio>


namespace IceHalo {

bool exists(const char* filename) {
  boost::filesystem::path p(filename);
  return exists(p);
}


void listDataFiles(const char* dir, std::vector<File>& files) {
  namespace f = boost::filesystem;

  f::path p(dir);
  std::vector<f::path> paths;
  copy(f::directory_iterator(p), f::directory_iterator(), back_inserter(paths));
  for (auto& x : paths) {
    if (x.extension() == ".bin") {
      files.emplace_back(x.c_str());
    }
  }
}


std::string pathJoin(const std::string& p1, const std::string& p2) {
  boost::filesystem::path p(p1);
  p /= (p2);
  return p.string();
}


File::File(const char* filename)
    : file(nullptr), fileOpened(false),
      p(filename) {}


File::File(const char* path, const char* filename)
    : file(nullptr), fileOpened(false),
      p(path) {
  p /= filename;
}


File::~File() {
  if (fileOpened) {
    fclose(file);
    fileOpened = false;
  }
}


bool File::open(uint8_t mode) {
  if (!exists(p.parent_path())) {
    create_directories(p.parent_path());
  }

  char modeBuffer[32];
  const char* m1;
  if (mode & OpenMode::kRead) {
    m1 = "r";
  } else if (mode & OpenMode::kWrite) {
    m1 = "w";
  } else if (mode & OpenMode::kAppend) {
    m1 = "a";
  } else {
    m1 = "r";
  }
  const char* m2 = (mode & OpenMode::kBinary) ? "b" : "";
  std::sprintf(modeBuffer, "%s%s", m1, m2);

  file = std::fopen(p.c_str(), modeBuffer);
  fileOpened = file != nullptr;
  return fileOpened;
}


bool File::close() {
  if (fileOpened) {
    std::fclose(file);
    file = nullptr;
    fileOpened = false;
  }
  return true;
}


size_t File::getSize() {
  auto size = file_size(p);
  if (size == static_cast<uintmax_t>(-1)) {
    return 0;
  } else {
    return static_cast<size_t>(size);
  }
}

}  // namespace IceHalo
