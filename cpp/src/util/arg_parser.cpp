#include "util/arg_parser.hpp"

#include <stdexcept>

#include "util/log.hpp"

namespace icehalo {

ArgParser::ArgParser() : mode_(ArgMode::kCompact) {}


void ArgParser::AddArgument(const std::string& name, int value_num) {
  if (name.size() > 1 && name[0] != '-') {
    LOG_WARNING("Add argument %s not starts with minus! Prepend a minus to it!", name.c_str());
    option_map_.emplace("-" + name, value_num);
  } else {
    option_map_.emplace(name, value_num);
  }
}


void ArgParser::SetArgMode(ArgMode mode) {
  mode_ = mode;
}


void ArgParser::ShowHelp(const char* cmd) const {
  constexpr size_t kBufSize = 1024;
  char buf[kBufSize];
  size_t offset = 0;

  auto n = std::snprintf(buf + offset, kBufSize - offset, "%s", cmd);
  offset += n;
  if (offset > kBufSize) {
    return;
  }

  for (const auto& kv : option_map_) {
    n = std::snprintf(buf + offset, kBufSize - offset, " %s", kv.first.c_str());
    offset += n;
    if (offset > kBufSize) {
      return;
    }
    if (kv.second < 0) {
      n = std::snprintf(buf + offset, kBufSize - offset, " [val]...");
      offset += n;
    } else if (kv.second > 0) {
      for (int i = 0; i < kv.second; i++) {
        n = std::snprintf(buf + offset, kBufSize - offset, " val%d", i);
        offset += n;
      }
    }
  }

  LOG_INFO("USAGE:");
  LOG_INFO("%s", buf);
}


enum ParseState {
  kStart,
  kReceivedKey,
  kWaitingValue,
  kFail,
};


using ParseStateAction =
    std::function<ParseState(const std::string& curr_arg,                   // current arg term
                             const std::string& key,                        // the key by now
                             const std::map<std::string, int>& option_map,  // the pre-defined option map
                             ArgMode mode,                                  // option compact mode
                             ArgParseResult& result)>;                      // result


ParseState StartAction(const std::string& curr_arg,                   // current arg term
                       const std::string& /* key */,                  // the key by now
                       const std::map<std::string, int>& option_map,  // pre-defined option map
                       ArgMode mode,                                  // option compact mode
                       ArgParseResult& result) {                      // result
  bool is_single_minus = curr_arg.size() > 1 && curr_arg[0] == '-' && curr_arg[1] != '-';
  if (option_map.count(curr_arg)) {  // curr_arg is a key
    if (result.count(curr_arg)) {
      result.at(curr_arg).clear();
    } else {
      result.emplace(curr_arg, std::vector<std::string>{});
    }
    return kReceivedKey;
  } else if (is_single_minus && mode == ArgMode::kCompact) {  // curr_arg is single minus and it's compact mode
    char tmp_arg[3]{};
    tmp_arg[0] = '-';
    for (size_t i = 1; i < curr_arg.size(); i++) {
      tmp_arg[1] = curr_arg[i];
      if (result.count(tmp_arg)) {
        result.at(tmp_arg).clear();
      } else {
        result.emplace(tmp_arg, std::vector<std::string>{});
      }
    }
    return kStart;
  } else {  // do not recognize
    result.at("").emplace_back(curr_arg);
    return kStart;
  }
}


ParseState ReceivedKeyAction(const std::string& curr_arg,                   // current arg term
                             const std::string& key,                        // the key by now
                             const std::map<std::string, int>& option_map,  // pre-defined option map
                             ArgMode /* mode */,                            // option compact mode
                             ArgParseResult& result) {                      // result
  if (!option_map.count(key)) {
    return ParseState::kFail;
  }

  result.at("").clear();
  int value_num = option_map.at(key);
  if (option_map.count(curr_arg)) {  // curr_arg is a key
    if (value_num <= 0) {            // prev_arg has 0 or undetermined values
      if (result.count(curr_arg)) {
        result.at(curr_arg).clear();
      } else {
        result.emplace(curr_arg, std::vector<std::string>{});
      }
      return ParseState::kReceivedKey;
    } else {
      return ParseState::kFail;
    }
  } else {                // curr_arg is not a key
    if (value_num < 0 ||  // undetermined number of values
        (value_num > 0 &&
         result.at(key).size() < static_cast<size_t>(value_num))) {  // positive number of values and still not full
      result.at(key).emplace_back(curr_arg);
      if (result.at(key).size() == static_cast<size_t>(value_num)) {
        return ParseState::kStart;
      } else {  // curr_arg is not a key && value number is not full
        return ParseState::kWaitingValue;
      }
    } else {
      return ParseState::kFail;
    }
  }
}


ParseState WaitingValueAction(const std::string& curr_arg,                   // current arg term
                              const std::string& key,                        // the key by now
                              const std::map<std::string, int>& option_map,  // pre-defined option map
                              ArgMode /* mode */,                            // option compact mode
                              ArgParseResult& result) {                      // result
  if (option_map.count(curr_arg)) {
    return ParseState::kFail;
  } else {  // curr_arg is not a key
    int value_num = option_map.at(key);
    if (value_num < 0 ||  // undetermined number of values
        (value_num > 0 &&
         result.at(key).size() < static_cast<size_t>(value_num))) {  // positive number of values and still not full
      result.at(key).emplace_back(curr_arg);
      if (result.at(key).size() == static_cast<size_t>(value_num)) {
        return ParseState::kStart;
      } else {  // curr_arg is not a key && value number is not full
        return ParseState::kWaitingValue;
      }
    } else {  // value number is full
      result.at("").emplace_back(curr_arg);
      return ParseState::kStart;
    }
  }
}


std::map<ParseState, ParseStateAction>& GetParseStateActions() {
  static std::map<ParseState, ParseStateAction> actions{
    { kStart, &StartAction },
    { kReceivedKey, &ReceivedKeyAction },
    { kWaitingValue, &WaitingValueAction },
  };
  return actions;
}


ArgParseResult ArgParser::Parse(int argc, char** argv) const {
  ParseState state = ParseState::kStart;
  auto& state_actions = GetParseStateActions();
  std::string key;
  std::string curr_arg;
  ArgParseResult result{ { "", {} } };
  for (int i = 0; i < argc; i++) {
    curr_arg = argv[i];
    state = state_actions[state](curr_arg, key, option_map_, mode_, result);
    if (state == ParseState::kReceivedKey) {
      key = curr_arg;
    } else if (state == ParseState::kStart) {
      key = "";
    } else if (state == ParseState::kFail) {
      ShowHelp(argv[0]);
      throw std::invalid_argument("arguments cannot recognize!");
    }
  }

  for (const auto& kv : option_map_) {
    if (kv.second > 0 && !result.count(kv.first)) {
      ShowHelp(argv[0]);
      throw std::invalid_argument("missing some required options!");
    }
  }
  return result;
}

}  // namespace icehalo
