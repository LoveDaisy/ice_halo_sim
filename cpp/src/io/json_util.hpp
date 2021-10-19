#ifndef SRC_IO_JSON_UTIL_H_
#define SRC_IO_JSON_UTIL_H_

#include "json.hpp"
#include "util/log.hpp"

namespace icehalo {

template <typename T>
struct function_traits;

template <typename R, typename... Args>
struct function_traits<std::function<R(Args...)>> {
  static const size_t nargs = sizeof...(Args);

  using result_type = R;

  template <size_t i>
  struct arg {
    using type = typename std::tuple_element<i, std::tuple<Args...>>::type;
  };
};

}  // namespace icehalo

#define JSON_CHECK_AND_UPDATE_SIMPLE_VALUE(obj, key, dst)   \
  if (obj.contains(key)) {                                  \
    obj.at(key).get_to(dst);                                \
  } else {                                                  \
    LOG_VERBOSE("missing key %s. use default value.", key); \
  }

#define JSON_CHECK_AND_APPLY_SIMPLE_VALUE(obj, key, type, f) \
  if (obj.contains(key)) {                                   \
    auto tmp = obj.at(key).get<type>();                      \
    f(tmp);                                                  \
  } else {                                                   \
    LOG_VERBOSE("missing key %s. do nothing.", key);         \
  }

#define JSON_CHECK_AND_UPDATE_ARRAY_VALUE(obj, key, dst, n) \
  if (!obj.contains(key)) {                                 \
    LOG_VERBOSE("missing key %s", key);                     \
  } else if (!obj.at(key).is_array()) {                     \
    LOG_VERBOSE("%s must be an array.", key);               \
  } else {                                                  \
    for (decltype(n) i = 0; i < (n); i++) {                 \
      obj.at(key)[i].get_to((dst)[i]);                      \
    }                                                       \
  }

#endif  // SRC_IO_JSON_UTIL_H_
