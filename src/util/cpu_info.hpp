#ifndef UTIL_CPU_INFO_H_
#define UTIL_CPU_INFO_H_

namespace lumice {

// Returns the number of physical CPU cores on the host machine.
// On SMT systems this is typically half of std::thread::hardware_concurrency().
// On detection failure, falls back to max(1, hardware_concurrency() / 2).
// Always returns a value >= 1.
int PhysicalCoreCount();

}  // namespace lumice

#endif  // UTIL_CPU_INFO_H_
