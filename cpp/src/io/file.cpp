#include "io/file.hpp"

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


File::File(const char* filename)
    : file_(nullptr), state_(FileState::kClosed), buffer_{ new char[kBufferSize] }, buffer_offset_(0), path_(filename) {
}


File::File(const char* path, const char* filename)
    : file_(nullptr), state_(FileState::kClosed), buffer_{ new char[kBufferSize] }, buffer_offset_(0), path_(path) {
  path_ /= filename;
}


File::File(icehalo::File&& other) noexcept
    : file_(other.file_), state_(other.state_), buffer_(std::move(other.buffer_)), buffer_offset_(other.buffer_offset_),
      path_(std::move(other.path_)) {}


File& File::operator=(File&& other) {
  if (&other != this) {
    file_ = other.file_;
    state_ = other.state_;
    buffer_ = std::move(other.buffer_);
    buffer_offset_ = other.buffer_offset_;
    path_ = std::move(other.path_);
  }
  return *this;
}


File::~File() {
  Close();
}


bool File::Open(FileOpenMode mode) {
  if (!boost::filesystem::exists(path_.parent_path())) {
    boost::filesystem::create_directories(path_.parent_path());
  }

  const char* m = nullptr;
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


void File::Flush() {
  if (state_ != FileState::kWriting) {
    return;
  }
  std::fwrite(buffer_.get(), 1, buffer_offset_, file_);
  buffer_offset_ = 0;
}


bool File::Close() {
  if (state_ != FileState::kClosed) {
    Flush();
    std::fclose(file_);
    file_ = nullptr;
    state_ = FileState::kClosed;
    buffer_offset_ = 0;
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
