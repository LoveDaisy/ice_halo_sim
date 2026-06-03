#include "util/cpu_info.hpp"

#include <algorithm>
#include <thread>

#if defined(OS_LINUX)
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#elif defined(OS_MAC)
#include <sys/sysctl.h>
#include <sys/types.h>
#elif defined(OS_WIN)
// clang-format off
#include <windows.h>
// clang-format on
#include <vector>
#endif

namespace lumice {

namespace {

int Fallback() {
  return std::max(1, static_cast<int>(std::thread::hardware_concurrency()) / 2);
}

#if defined(OS_LINUX)
int PhysicalCoreCountLinux() {
  std::ifstream f("/proc/cpuinfo");
  if (!f.is_open()) {
    return 0;
  }
  std::set<std::pair<int, int>> pairs;
  std::set<int> core_ids_only;
  int cur_physical_id = -1;
  int cur_core_id = -1;
  bool saw_any_physical_id = false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) {
      if (cur_core_id >= 0) {
        if (cur_physical_id >= 0) {
          pairs.emplace(cur_physical_id, cur_core_id);
        }
        core_ids_only.insert(cur_core_id);
      }
      cur_physical_id = -1;
      cur_core_id = -1;
      continue;
    }
    auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    auto key = line.substr(0, colon);
    auto val = line.substr(colon + 1);
    // Trim trailing whitespace from key.
    while (!key.empty() && (key.back() == ' ' || key.back() == '\t')) {
      key.pop_back();
    }
    // Trim leading whitespace from val.
    size_t i = 0;
    while (i < val.size() && (val[i] == ' ' || val[i] == '\t')) {
      ++i;
    }
    val = val.substr(i);
    if (key == "physical id") {
      saw_any_physical_id = true;
      try {
        cur_physical_id = std::stoi(val);
      } catch (...) {
        cur_physical_id = -1;
      }
    } else if (key == "core id") {
      try {
        cur_core_id = std::stoi(val);
      } catch (...) {
        cur_core_id = -1;
      }
    }
  }
  // Flush last record if file did not end with a blank line.
  if (cur_core_id >= 0) {
    if (cur_physical_id >= 0) {
      pairs.emplace(cur_physical_id, cur_core_id);
    }
    core_ids_only.insert(cur_core_id);
  }
  if (saw_any_physical_id && !pairs.empty()) {
    return static_cast<int>(pairs.size());
  }
  if (!core_ids_only.empty()) {
    return static_cast<int>(core_ids_only.size());
  }
  return 0;
}
#endif

#if defined(OS_MAC)
int PhysicalCoreCountMac() {
  int n = 0;
  size_t sz = sizeof(n);
  if (sysctlbyname("hw.physicalcpu", &n, &sz, nullptr, 0) != 0) {
    return 0;
  }
  return n > 0 ? n : 0;
}
#endif

#if defined(OS_WIN)
int PhysicalCoreCountWin() {
  DWORD len = 0;
  GetLogicalProcessorInformation(nullptr, &len);
  if (len == 0) {
    return 0;
  }
  std::vector<SYSTEM_LOGICAL_PROCESSOR_INFORMATION> buf(len / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION));
  if (!GetLogicalProcessorInformation(buf.data(), &len)) {
    return 0;
  }
  int count = 0;
  for (const auto& info : buf) {
    if (info.Relationship == RelationProcessorCore) {
      ++count;
    }
  }
  return count;
}
#endif

}  // namespace

int PhysicalCoreCount() {
  int n = 0;
#if defined(OS_LINUX)
  n = PhysicalCoreCountLinux();
#elif defined(OS_MAC)
  n = PhysicalCoreCountMac();
#elif defined(OS_WIN)
  n = PhysicalCoreCountWin();
#endif
  if (n <= 0) {
    return Fallback();
  }
  return n;
}

}  // namespace lumice
