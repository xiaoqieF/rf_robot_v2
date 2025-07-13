#pragma once

#include <memory>
#include <string>
#include <vector>

#include "elog/details/flag_formatter.h"
#include "elog/details/format_helper.h"
#include "elog/details/log_msg.h"

namespace elog {
class Formatter {
public:
    virtual ~Formatter() = default;
    virtual void format(const details::LogMsg& msg, memory_buf_t& dest) = 0;
    virtual std::unique_ptr<Formatter> clone() const = 0;
};

class PatternFormatter final : public Formatter {
public:
    PatternFormatter();
    explicit PatternFormatter(std::string pattern);

    PatternFormatter(const PatternFormatter&) = delete;
    PatternFormatter& operator=(const PatternFormatter&) = delete;

    std::unique_ptr<Formatter> clone() const override;
    void format(const details::LogMsg& msg, memory_buf_t& dest) override;
    void setPattern(std::string pattern);

private:
    void compilePattern(const std::string& pattern);
    void handleFlag(char ch);
    std::string pattern_;
    std::vector<std::unique_ptr<details::FlagFormatter>> formatters_;
};

inline PatternFormatter::PatternFormatter()
    : pattern_("%+") {
    formatters_.push_back(make_unique<details::FullFormatter>());
}

inline PatternFormatter::PatternFormatter(std::string pattern)
    : pattern_(std::move(pattern)) {
    compilePattern(pattern_);
}

inline void PatternFormatter::setPattern(std::string pattern) {
    pattern_ = std::move(pattern);
    compilePattern(pattern_);
}

inline void PatternFormatter::format(const details::LogMsg& msg, memory_buf_t& dest) {
    time_t t = LogClock::to_time_t(msg.time);
    tm tm = os::localtime(t);
    for (auto& f : formatters_) {
        f->format(msg, tm, dest);
    }
    details::format_helper::appendStringView("\n", dest);
}

inline std::unique_ptr<Formatter> PatternFormatter::clone() const {
    return make_unique<PatternFormatter>(pattern_);
}

inline void PatternFormatter::handleFlag(char ch) {
    switch (ch) {
        case '+':
            formatters_.push_back(make_unique<details::FullFormatter>());
            break;
        case 'n':
            formatters_.push_back(make_unique<details::LogNameFormatter>());
            break;
        case 'l':
            formatters_.push_back(make_unique<details::LogLevelFormatter>());
            break;
        case 'L':
            formatters_.push_back(make_unique<details::SimpleLogLevelFormatter>());
            break;
        case 't':
            formatters_.push_back(make_unique<details::TidFormatter>());
            break;
        case 'Y':
            formatters_.push_back(make_unique<details::YearFormatter>());
            break;
        case 'M':
            formatters_.push_back(make_unique<details::MonthFormatter>());
            break;
        case 'D':
            formatters_.push_back(make_unique<details::DayFormatter>());
            break;
        case 'h':
            formatters_.push_back(make_unique<details::HourFormatter>());
            break;
        case 'm':
            formatters_.push_back(make_unique<details::MinFormatter>());
            break;
        case 's':
            formatters_.push_back(make_unique<details::SecondFormatter>());
            break;
        case 'e':
            formatters_.push_back(make_unique<details::MilliFormatter>());
            break;
        case 'f':
            formatters_.push_back(make_unique<details::MicroFormatter>());
            break;
        case 'v':
            formatters_.push_back(make_unique<details::PayloadFormatter>());
            break;
        case 'u':
            formatters_.push_back(make_unique<details::SourceLocFormatter>());
            break;
        case 'w':
            formatters_.push_back(make_unique<details::FuncNameFormatter>());
            break;
        default:
            auto unknown_flag = make_unique<details::AggregateFormatter>();
            unknown_flag->addChar('%');
            unknown_flag->addChar(ch);
            formatters_.push_back(std::move(unknown_flag));
            break;
    }
}

inline void PatternFormatter::compilePattern(const std::string& pattern) {
    std::unique_ptr<details::AggregateFormatter> user_chars;
    formatters_.clear();
    for (auto it = pattern.begin(); it != pattern.end(); ++it) {
        if (*it == '%') {
            if (user_chars) {
                formatters_.push_back(std::move(user_chars));
            }

            if (++it != pattern.end()) {
                handleFlag(*it);
            } else {
                break;
            }
        } else {
            if (!user_chars) {
                user_chars = make_unique<details::AggregateFormatter>();
            }
            user_chars->addChar(*it);
        }
    }
    if (user_chars) {
        formatters_.push_back(std::move(user_chars));
    }
}

} // namespace elog