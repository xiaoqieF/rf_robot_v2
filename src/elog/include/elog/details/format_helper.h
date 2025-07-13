#pragma once

#include "elog/common.h"

namespace elog {
namespace details {
namespace format_helper {

inline void appendStringView(string_view_t s, memory_buf_t& dest) {
    auto* buf_ptr = s.data();
    dest.append(buf_ptr, buf_ptr + s.size());
}

// T must be integer(int, long, unsigned...)
template <typename T>
inline void appendInt(T n, memory_buf_t& dest) {
    fmt::format_int i(n);
    dest.append(i.data(), i.data() + i.size());
}

inline void pad2(int n, memory_buf_t& dest) {
    if (n >= 0 && n < 100) {
        dest.push_back(static_cast<char>('0' + n / 10));
        dest.push_back(static_cast<char>('0' + n % 10));
    } else { // should not go here
        fmt::format_to(std::back_inserter(dest), FMT_STRING("{:02}"), n);
    }
}

inline void pad3(int n, memory_buf_t& dest) {
    if (n >= 0 && n < 1000) {
        dest.push_back(static_cast<char>(n / 100 + '0'));
        n = n % 100;
        dest.push_back(static_cast<char>((n / 10) + '0'));
        dest.push_back(static_cast<char>((n % 10) + '0'));
    } else {
        appendInt(n, dest);
    }
}

inline void pad_uint(uint32_t n, unsigned int width, memory_buf_t& dest) {
    for (unsigned int digits = fmt::detail::count_digits(n); digits < width; ++digits) {
        dest.push_back('0');
    }
    appendInt(n, dest);
}

template <typename ToDuration>
inline ToDuration timeFraction(LogClock::time_point tp) {
    using std::chrono::duration_cast;
    using std::chrono::seconds;
    auto duration = tp.time_since_epoch();
    auto secs = duration_cast<seconds>(duration);
    return duration_cast<ToDuration>(duration) - duration_cast<ToDuration>(secs);
}

} // namespace format_helper
} // namespace details
} // namespace elog
