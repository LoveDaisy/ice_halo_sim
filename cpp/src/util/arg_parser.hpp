#ifndef SRC_UTIL_ARG_PARSER_H_
#define SRC_UTIL_ARG_PARSER_H_

#include <map>
#include <string>
#include <vector>

namespace icehalo {

enum class ArgMode {
  kCompact,
  kNormal,
};


using ArgParseResult = std::map<std::string, std::vector<std::string>>;

/**
 * @brief A parser to parse command line arguments.
 *
 * A normal command line call may be like:
 * ~~~bash
 * cmd -a -m message -tvSs --input file1 file2 ... --with-test
 * ~~~
 * Here `-a` and `-m` are single letter option, `-tvSs` is a compact form of 4 single letter options.
 * These options can be flags without any value, like `-a` and `-tvSs`, and can be key-value options,
 * like `-m` with value `message`.
 *
 * `--input` and `--with-test` are full options, followed by 0 or more values, until another option.
 *
 * If arg-mode is set to ArgMode::kCompact (default), then single letter options can be group together, like
 * `-tvSs`; otherwise (mode is set to ArgMode::kNormal) it is recognized as a single word option name.
 * If minus appears in the middle, the whole term is recognized as a single option name,
 * rather than several single letter groups.
 *
 * The leading minus `-` is necessary.
 * In any cases, the option starts with double minus `--` is recognized as a full option name, rather than
 * a group of compacted single letter options.
 *
 * Any word / term appears **AFTER** the last recognized term (option or value), is recognized as no-named
 * arguments, and corresponds to empty string as key. And those appear **BEFORE** the last recognized option,
 * are simply ignored.
 */
class ArgParser {
 public:
  ArgParser();

  void AddArgument(const std::string& name, int value_num);
  void SetArgMode(ArgMode mode);
  ArgParseResult Parse(int argc, char** argv) const;

 private:
  void ShowHelp(const char* cmd) const;

  ArgMode mode_;
  std::map<std::string, int> option_map_;
};

}  // namespace icehalo

#endif  // SRC_UTIL_ARG_PARSER_H_
