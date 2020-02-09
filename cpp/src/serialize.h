#ifndef SRC_SERIALIZE_H_
#define SRC_SERIALIZE_H_

#include <cstdint>

#include "file.h"
#include "rapidjson/document.h"

namespace icehalo {

struct ISerializable {
  virtual ~ISerializable() = default;

  /**
   * @brief Serialize data into a file.
   * @param file The destination file.
   * @param with_boi Whether write BOI(Byte Order Indicator).
   */
  virtual void Serialize(File& file, bool with_boi) = 0;

  /**
   * @brief Deserialize data from a file.
   * @param file The source file.
   * @param endianness The endianness of file. It can be endian::kUnknownEndian. When it is set to
   *        endian::kUnknownEndian, this method will determine the endian itself, typically by reading
   *        the BOI. That is, this method might read some extra bytes to determine the file endianness.
   */
  virtual void Deserialize(File& file, endian::Endianness endianness) = 0;

  static endian::Endianness CheckEndianness(File& file, endian::Endianness endianness) {
    if (endianness == endian::kUnknownEndian) {
      uint32_t test_boi;
      file.Read(&test_boi);
      if (test_boi != ISerializable::kDefaultBoi) {
        endianness = endian::kBigEndian;
      } else {
        endianness = endian::kCompileEndian;
      }
    }
    return endianness;
  }

  /**
   * @brief The default BOI(Byte Order Indicator).
   */
  static constexpr uint32_t kDefaultBoi = 0x34CD28AF;
};


constexpr uintptr_t CombineU32AsPointer(uint32_t high, uint32_t low) {
  return static_cast<uintptr_t>(high) << 32 | static_cast<uintptr_t>(low);
}


class IJsonizable {
 public:
  virtual ~IJsonizable() = default;

  /**
   * @brief Save self to a JSON object.
   *
   * @param root
   * @param allocator
   */
  virtual void SaveToJson(rapidjson::Value& root, rapidjson::Value::AllocatorType& allocator) = 0;

  /**
   * @brief Load data from a JSON object.
   * @param root
   */
  virtual void LoadFromJson(rapidjson::Value& root) = 0;
};

}  // namespace icehalo

#endif  // SRC_SERIALIZE_H_
