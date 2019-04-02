#ifndef SRC_FILES_H_
#define SRC_FILES_H_

#include <string>
#include <vector>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>


namespace IceHalo {

namespace OpenMode {
constexpr uint8_t kRead = 0b0001;
constexpr uint8_t kWrite = 0b0010;
constexpr uint8_t kAppend = 0b0100;
constexpr uint8_t kBinary = 0b1000;
}  // namespace OpenMode


class File {
 public:
  explicit File(const char* filename);
  File(const char* path, const char* filename);
  ~File();

  bool Open(uint8_t mode = OpenMode::kRead);
  bool Close();

  size_t GetSize();

  template <class T>
  size_t Read(T* buffer, size_t n = 1);

  template <class T>
  size_t Write(T data);

  template <class T>
  size_t Write(const T* data, size_t n);

 private:
  std::FILE* file_;
  bool file_opened_;

  boost::filesystem::path path_;
};


template <class T>
size_t File::Read(T* buffer, size_t n) {
  if (!file_opened_) {
    return 0;
  }

  auto count = std::fread(buffer, sizeof(T), n, file_);
  return count;
}


template <class T>
size_t File::Write(T data) {
  if (!file_opened_) {
    return 0;
  }

  auto count = std::fwrite(&data, sizeof(T), 1, file_);
  return count;
}


template <class T>
size_t File::Write(const T* data, size_t n) {
  if (!file_opened_) {
    return 0;
  }

  auto count = std::fwrite(data, sizeof(T), n, file_);
  return count;
}

bool exists(const char* filename);

std::vector<File> ListDataFiles(const char* dir);

std::string PathJoin(const std::string& p1, const std::string& p2);

}  // namespace IceHalo

#endif  // SRC_FILES_H_
