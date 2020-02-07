#ifndef SRC_FILES_H_
#define SRC_FILES_H_

#include <string>
#include <vector>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>


namespace icehalo {

namespace openmode {
constexpr uint8_t kRead = 0b0001;
constexpr uint8_t kWrite = 0b0010;
constexpr uint8_t kAppend = 0b0100;
constexpr uint8_t kBinary = 0b1000;
}  // namespace openmode


namespace endian {

#if defined(OS_MAC) || defined(OS_LINUX)
#include "arpa/inet.h"
#elif defined(OS_WIN)
#include "winsock.h"
#endif

using Endianness = uint8_t;
constexpr Endianness kUndetermined = 0;
constexpr Endianness kLittleEndian = 1;
constexpr Endianness kBigEndian = 2;

#if defined(__BYTE_ORDER) && __BYTE_ORDER == __BIG_ENDIAN || \
    defined(__BIG_ENDIAN__) || \
    defined(__ARMEB__) || \
    defined(__THUMBEB__) || \
    defined(__AARCH64EB__) || \
    defined(_MIBSEB) || defined(__MIBSEB) || defined(__MIBSEB__)
constexpr Endianness kCompileEndian = kBigEndian;
#elif defined(__BYTE_ORDER) && __BYTE_ORDER == __LITTLE_ENDIAN || \
    defined(__LITTLE_ENDIAN__) || \
    defined(__ARMEL__) || \
    defined(__THUMBEL__) || \
    defined(__AARCH64EL__) || \
    defined(_MIPSEL) || defined(__MIPSEL) || defined(__MIPSEL__)
constexpr Endianness kCompileEndian = kLittleEndian;
#else
#error "I don't know what architecture this is!"
#endif

inline Endianness CheckRuntimeEndianness() noexcept {
  uint32_t test = 0x01020304;
  auto* p = reinterpret_cast<uint8_t*>(&test);
  if (p[0] == 1) {
    return kBigEndian;
  } else {
    return kLittleEndian;
  }
}


template <Endianness from, size_t = 0>
struct ConvertFrom {};

template <Endianness from>
struct ConvertFrom<from, 1> {
  template <typename T>
  static void ToNative(T*) noexcept {}
};

template <Endianness from>
struct ConvertFrom<from, 2> {
  template <typename T>
  static void ToNative(T* x) noexcept {
    if (CheckRuntimeEndianness() != from) {
      *x = ntohs(*x);
    }
  }
};

template <Endianness from>
struct ConvertFrom<from, 4> {
  template <typename T>
  static void ToNative(T* x) noexcept {
    if (CheckRuntimeEndianness() != from) {
      *x = ntohl(*x);
    }
  }
};

template <Endianness from>
struct ConvertFrom<from, 8> {
  template <typename T>
  static void ToNative(T* x) noexcept {
    if (CheckRuntimeEndianness() != from) {
      auto* p0 = reinterpret_cast<uint32_t*>(x);
      auto* p1 = p0 + 1;
      auto tmp = ntohl(*p0);
      *p0 = ntohl(*p1);
      *p1 = tmp;
    }
  }
};

}  // namespace endian


class File {
 public:
  explicit File(const char* filename);
  File(const char* path, const char* filename);
  ~File();

  bool Open(uint8_t mode = openmode::kRead);
  bool Close();
  bool IsOpen() const;

  size_t GetBytes();

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

bool FileExists(const char* filename);

std::vector<File> ListDataFiles(const char* dir);

std::string PathJoin(const std::string& p1, const std::string& p2);

}  // namespace icehalo

#endif  // SRC_FILES_H_
