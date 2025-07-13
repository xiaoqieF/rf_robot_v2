#pragma once

namespace elog {
namespace details {
struct NullMutex {
    void lock() const {}
    void unlock() const {}
};
} // namespace details

} // namespace elog
