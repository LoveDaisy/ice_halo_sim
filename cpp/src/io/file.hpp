#ifndef SRC_IO_FILE_H_
#define SRC_IO_FILE_H_

#include <string>
#include <vector>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>


namespace icehalo {

enum class FileOpenMode {
  kRead,
  kWrite,
  kAppend,
};


namespace endian {

#if defined(OS_MAC) || defined(OS_LINUX)
#include "arpa/inet.h"
#elif defined(OS_WIN)
#include "winsock.h"
#endif

using Endianness = uint8_t;
constexpr Endianness kUnknownEndian = 0;
constexpr Endianness kLittleEndian = 1;
constexpr Endianness kBigEndian = 2;

/**
 * Determine endianness on compile time.
 * See [this page](https://stackoverflow.com/questions/4239993/determining-endianness-at-compile-time)
 */
#if defined(__BYTE_ORDER) && __BYTE_ORDER == __BIG_ENDIAN || defined(__BIG_ENDIAN__) || defined(__ARMEB__) || \
    defined(__THUMBEB__) || defined(__AARCH64EB__) || defined(_MIBSEB) || defined(__MIBSEB) || defined(__MIBSEB__)
constexpr Endianness kCompileEndian = kBigEndian;
#elif defined(__BYTE_ORDER) && __BYTE_ORDER == __LITTLE_ENDIAN || defined(__LITTLE_ENDIAN__) || defined(__ARMEL__) || \
    defined(__THUMBEL__) || defined(__AARCH64EL__) || defined(_MIPSEL) || defined(__MIPSEL) || defined(__MIPSEL__)
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


template <size_t = 0>
struct ByteSwapImp {};

template <>
struct ByteSwapImp<1> {
  template <typename T>
  void operator()(T* /* x */) noexcept {}
};

template <>
struct ByteSwapImp<2> {
  template <typename T>
  void operator()(T* x) noexcept {
    *x = ntohs(*x);
  }
};

template <>
struct ByteSwapImp<4> {
  template <typename T>
  void operator()(T* x) noexcept {
    *x = ntohl(*x);
  }
};

template <>
struct ByteSwapImp<8> {
  template <typename T>
  void operator()(T* x) noexcept {
    auto* p0 = reinterpret_cast<uint32_t*>(x);
    auto* p1 = p0 + 1;
    auto tmp = ntohl(*p0);
    *p0 = ntohl(*p1);
    *p1 = tmp;
  }
};

struct ByteSwap {
  template <typename T>
  static void Swap(T* x) noexcept {
    static ByteSwapImp<sizeof(T)> imp;
    imp(x);
  }

  template <typename T>
  static void Swap(T* x, size_t num) noexcept {
    static ByteSwapImp<sizeof(T)> imp;
    for (size_t i = 0; i < num; i++) {
      imp(x + i);
    }
  }
};

}  // namespace endian


enum class FileState {
  kClosed,
  kWriting,
  kReading,
};


class File {
 public:
  explicit File(const char* filename);
  File(const char* path, const char* filename);
  File(const File& other) = delete;
  File(File&& other) noexcept;
  ~File();

  File& operator=(const File& other) = delete;
  File& operator=(File&& other);

  bool Open(FileOpenMode mode = FileOpenMode::kRead);
  bool Close();

  size_t GetBytes();

  template <class T>
  size_t Read(T* buffer, size_t n = 1);

  template <class T>
  size_t Write(T data);

  template <class T>
  size_t Write(const T* data, size_t n);

  void Flush();

 private:
  std::FILE* file_;
  FileState state_;
  std::unique_ptr<char[]> buffer_;
  size_t buffer_offset_;
  boost::filesystem::path path_;

  static constexpr size_t kBufferSize = 2 * 1024 * 1024;
};


template <class T>
size_t File::Read(T* buffer, size_t n) {
  if (state_ != FileState::kReading) {
    throw std::logic_error("File state is not for reading!");
  }

  if (n == 0) {
    return 0;
  }

  if (buffer_offset_ == 0) {
    std::fread(buffer_.get(), 1, kBufferSize, file_);
  }

  constexpr size_t kTypeSize = sizeof(T);
  size_t count = 0;
  char* p = reinterpret_cast<char*>(buffer);
  for (size_t i = 0; i < n; i++) {
    if (buffer_offset_ + kTypeSize >= kBufferSize) {
      size_t remained_bytes = kBufferSize - buffer_offset_;
      std::memcpy(buffer_.get(), buffer_.get() + buffer_offset_, remained_bytes);
      std::fread(buffer_.get() + remained_bytes, 1, kBufferSize - remained_bytes, file_);
      buffer_offset_ = 0;
    }
    std::memcpy(p + i * kTypeSize, buffer_.get() + buffer_offset_, kTypeSize);
    buffer_offset_ += kTypeSize;
    count += kTypeSize;
  }

  return count;
}


template <class T>
size_t File::Write(T data) {
  if (state_ != FileState::kWriting) {
    throw std::logic_error("File state is not for writing!");
  }

  constexpr size_t kTypeSize = sizeof(T);
  size_t count = 0;
  if (buffer_offset_ + kTypeSize >= kBufferSize) {
    std::fwrite(buffer_.get(), 1, buffer_offset_, file_);
    buffer_offset_ = 0;
  }
  std::memcpy(buffer_.get() + buffer_offset_, &data, kTypeSize);
  buffer_offset_ += kTypeSize;
  count += kTypeSize;
  return count;
}


template <class T>
size_t File::Write(const T* data, size_t n) {
  if (state_ != FileState::kWriting) {
    throw std::logic_error("File state is not for writing!");
  }

  constexpr size_t kTypeSize = sizeof(T);
  size_t count = 0;
  for (size_t i = 0; i < n; i++) {
    if (buffer_offset_ + kTypeSize >= kBufferSize) {
      std::fwrite(buffer_.get(), 1, buffer_offset_, file_);
      buffer_offset_ = 0;
    }
    std::memcpy(buffer_.get() + buffer_offset_, data + i, kTypeSize);
    buffer_offset_ += kTypeSize;
    count += kTypeSize;
  }

  return count;
}

bool FileExists(const char* filename);

std::vector<File> ListDataFiles(const char* dir);

std::string PathJoin(const std::string& p1, const std::string& p2);

}  // namespace icehalo

#endif  // SRC_IO_FILE_H_
