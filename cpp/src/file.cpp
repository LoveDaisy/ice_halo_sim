#include "file.h"

#include <algorithm>
#include <cstdio>


namespace icehalo {

bool FileExists(const char* filename) {
  boost::filesystem::path p(filename);
  return exists(p);
}


std::vector<File> ListDataFiles(const char* dir) {
  namespace f = boost::filesystem;

  std::vector<File> files;
  f::path p(dir);
  std::vector<f::path> paths;
  copy(f::directory_iterator(p), f::directory_iterator(), back_inserter(paths));
  for (auto& x : paths) {
    if (x.extension() == ".bin") {
      files.emplace_back(x.c_str());
    }
  }

  return files;
}


std::string PathJoin(const std::string& p1, const std::string& p2) {
  boost::filesystem::path p(p1);
  p /= (p2);
  return p.string();
}


File::File(const char* filename) : file_(nullptr), state_(FileState::kClosed), path_(filename) {}


File::File(const char* path, const char* filename) : file_(nullptr), state_(FileState::kClosed), path_(path) {
  path_ /= filename;
}


File::~File() {
  if (state_ != FileState::kClosed) {
    fclose(file_);
    state_ = FileState::kClosed;
  }
}


bool File::Open(FileOpenMode mode) {
  if (!boost::filesystem::exists(path_.parent_path())) {
    boost::filesystem::create_directories(path_.parent_path());
  }

  const char* m;
  switch (mode) {
    case FileOpenMode::kWrite:
      m = "wb";
      state_ = FileState::kWriting;
      break;
    case FileOpenMode::kAppend:
      m = "ab";
      state_ = FileState::kWriting;
      break;
    case FileOpenMode::kRead:
    default:
      m = "rb";
      state_ = FileState::kReading;
      break;
  }

  file_ = std::fopen(path_.c_str(), m);
  if (!file_) {
    state_ = FileState::kClosed;
  }
  return state_ != FileState::kClosed;
}


bool File::Close() {
  if (state_ != FileState::kClosed) {
    std::fclose(file_);
    file_ = nullptr;
    state_ = FileState::kClosed;
  }
  return true;
}


size_t File::GetBytes() {
  auto size = file_size(path_);
  if (size == static_cast<uintmax_t>(-1)) {
    return 0;
  } else {
    return static_cast<size_t>(size);
  }
}

}  // namespace icehalo
