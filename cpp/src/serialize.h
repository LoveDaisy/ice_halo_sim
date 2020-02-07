#ifndef SRC_SERIALIZE_H_
#define SRC_SERIALIZE_H_

#include <cstdint>

#include "file.h"

namespace icehalo {

struct ISerializable {
  /**
   * @brief Serialize data into a file.
   * @param file The destination file.
   * @param with_boi Whether write BOI(Byte Order Indicator).
   */
  virtual void Serialize(File& file, bool with_boi) = 0;

  /**
   * @brief Deserialize data from a file.
   * @param file The source file.
   * @param endianness Whether write BOI(Byte Order Indicator).
   */
  virtual void Deserialize(File& file, endian::Endianness endianness) = 0;

  /**
   * @brief The default BOI(Byte Order Indicator).
   */
  static constexpr uint32_t kDefaultBoi = 0x34CD28AF;
};

}  // namespace icehalo

#endif  // SRC_SERIALIZE_H_
