#include "gui/gui_logger.hpp"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <utility>
#include <vector>

#include "gui/app.hpp"
#include "gui/log_sink.hpp"
#include "gui/user_defaults.hpp"

// Why the two stages exist and why they cannot be merged is documented at their declarations in
// gui_logger.hpp — that is the file a caller reads, so the ordering rule lives there.

namespace lumice::gui {

void InstallEarlyGuiSinks() {
  // ImGui ring buffer sink (shared between the GUI logger and the Core log callback, which
  // main() registers later next to LUMICE_CreateServer — the callback only carries meaning once
  // a server exists to emit Core logs).
  g_imgui_log_sink = std::make_shared<ImGuiLogSink>();

  auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  SetGuiLoggerSinks({ stdout_sink, g_imgui_log_sink });
  GetGuiLogger().set_formatter(lumice::CreateLumiceFormatter(kGuiLogPattern));

  // Flush strategy: warning+ immediately, all levels every 1s.
  // spdlog::err = our warning level (see spdlog_levels.hpp).
  GetGuiLogger().flush_on(spdlog::level::err);
  spdlog::flush_every(std::chrono::seconds(1));
}

void AttachGuiFileSink() {
  // The log file lives beside the other per-user Lumice artifacts (see GetUserConfigDir), not at
  // $HOME — a read-only or multi-user install must not scatter files in the home directory root.
  std::filesystem::path log_path;
  if (auto config_dir = GetUserConfigDir()) {
    log_path = *config_dir / "lumice.log";
  } else {
    log_path = "lumice.log";
  }
  log_path = std::filesystem::absolute(log_path);

  // Only the log panel reads these two, and it runs in the frame loop, so publishing them at
  // this point in startup is not observable.
  g_log_file_path = log_path.u8string();
  g_file_log_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path.string(), true);
  g_file_log_sink->set_level(spdlog::level::off);

  auto sinks = GetGuiLogger().sinks();
  sinks.push_back(g_file_log_sink);
  SetGuiLoggerSinks(std::move(sinks));
  // Re-apply: set_formatter clones into each sink, so the new one needs it too.
  GetGuiLogger().set_formatter(lumice::CreateLumiceFormatter(kGuiLogPattern));
}

}  // namespace lumice::gui
