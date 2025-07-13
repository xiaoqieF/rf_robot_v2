#pragma once

#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <array>
#include <ctime>
#include <string>

namespace elog {
namespace os {

static const char FOLDER_SEP[] = "/";

inline size_t getThreadId() {
    static const thread_local size_t tid = static_cast<size_t>(::syscall(SYS_gettid));
    return tid;
}

inline tm localtime(const time_t& t) {
    tm tm;
    ::localtime_r(&t, &tm);
    return tm;
}

inline std::string dirName(const std::string& path) {
    auto pos = path.find_last_of(FOLDER_SEP);
    return pos != std::string::npos ? path.substr(0, pos) : std::string{};
}

inline bool existsPath(const std::string& path) {
    struct stat buffer;
    return (::stat(path.c_str(), &buffer) == 0);
}

inline void createDir(const std::string& dir_path) {
    if (dir_path.empty() || existsPath(dir_path)) {
        return;
    }
    size_t search_offset = 0;
    while (search_offset < dir_path.size()) {
        auto token_pos = dir_path.find_first_of(FOLDER_SEP, search_offset);
        if (token_pos == std::string::npos) {
            token_pos = dir_path.size();
        }
        auto subdir = dir_path.substr(0, token_pos);
        if (subdir.empty() || existsPath(subdir)) {
            search_offset = token_pos + 1;
            continue;
        }
        ::mkdir(subdir.c_str(), 0755);
        search_offset = token_pos + 1;
    }
}

inline bool isColorTerminal() {
    static const bool result = []() {
        const char* env_colorterm_p = std::getenv("COLORTERM");
        if (env_colorterm_p != nullptr) {
            return true;
        }

        static constexpr std::array<const char*, 16> terms = {
            {"ansi", "color", "console", "cygwin", "gnome", "konsole", "kterm", "linux", "msys",
             "putty", "rxvt", "screen", "vt100", "xterm", "alacritty", "vt102"}};

        const char* env_term_p = std::getenv("TERM");
        if (env_term_p == nullptr) {
            return false;
        }

        return std::any_of(terms.begin(), terms.end(), [&](const char* term) {
            return std::strstr(env_term_p, term) != nullptr;
        });
    }();

    return result;
}

} // namespace os
} // namespace elog
