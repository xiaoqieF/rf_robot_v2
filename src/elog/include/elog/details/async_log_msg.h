#pragma once

#include "elog/details/log_msg.h"

namespace elog {
class AsyncLoggerBase;
namespace details {
struct AsyncMsg {
    AsyncMsg() = default;
    AsyncMsg(const LogMsg& m, AsyncLoggerBase* l)
        : msg(m),
          own_logger(l) {
        // copy logger_name and payload to loval_buf
        // and update string_view_t
        local_buf.append(msg.logger_name.begin(), msg.logger_name.end());
        local_buf.append(msg.payload.begin(), msg.payload.end());
        updateStringView();
    }

    AsyncMsg(AsyncMsg&& other)
        : msg(std::move(other.msg)),
          local_buf(std::move(other.local_buf)),
          own_logger(std::move(other.own_logger)) {
        updateStringView();
    }

    AsyncMsg& operator=(AsyncMsg&& other) {
        msg = std::move(other.msg);
        local_buf = std::move(other.local_buf);
        own_logger = std::move(other.own_logger);
        updateStringView();
        return *this;
    }

    void updateStringView() {
        msg.logger_name = string_view_t(local_buf.data(), msg.logger_name.size());
        msg.payload = string_view_t(local_buf.data() + msg.logger_name.size(), msg.payload.size());
    }

    LogMsg msg;
    memory_buf_t local_buf;
    AsyncLoggerBase* own_logger;
};
} // namespace details

} // namespace elog
