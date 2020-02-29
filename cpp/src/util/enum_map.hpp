#ifndef SRC_UTIL_ENUM_MAP_H_
#define SRC_UTIL_ENUM_MAP_H_

#include <unordered_map>

/* A workaround for disgusting C++11 standard that enum class cannot be a key */
struct EnumClassHash {
  template <typename T>
  std::size_t operator()(T t) const {
    return static_cast<std::size_t>(t);
  }
};

template <typename Key>
using HashType = typename std::conditional<std::is_enum<Key>::value, EnumClassHash, std::hash<Key>>::type;

template <typename Key, typename T>
using EnumMap = std::unordered_map<Key, T, HashType<Key>>;
/* Workaround end */


#endif  // SRC_UTIL_ENUM_MAP_H_
