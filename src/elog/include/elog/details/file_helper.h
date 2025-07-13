#pragma once

#include <iostream>

#include "elog/common.h"
#include "elog/details/os.h"

namespace elog {
namespace details {
class FileHelper {
public:
    FileHelper() = default;
    FileHelper(const FileHelper&) = delete;
    FileHelper& operator=(const FileHelper&) = delete;
    ~FileHelper();

    void open(const std::string& filename);
    void reopen(const std::string& filename) { open(filename); }
    void close();
    void write(const memory_buf_t& buf);
    void flush();
    const std::string& filename() const { return filename_; }

    size_t written_bytes() const { return written_bytes_; };

private:
    FILE* fd_{nullptr};
    std::string filename_;
    size_t written_bytes_ = 0;
};

inline FileHelper::~FileHelper() { close(); }

inline void FileHelper::open(const std::string& filename) {
    close();
    filename_ = filename;
    os::createDir(os::dirName(filename_));
    fd_ = ::fopen(filename_.c_str(), "ab");
    if (fd_ == nullptr) {
        std::cerr << "Error in opening file: " << filename_ << std::endl;
        return;
    }
}

inline void FileHelper::close() {
    if (fd_) {
        ::fclose(fd_);
    }
    written_bytes_ = 0;
}

inline void FileHelper::write(const memory_buf_t& buf) {
    if (::fwrite(buf.data(), 1, buf.size(), fd_) != buf.size()) {
        std::cerr << "Failed writing to file: " << filename_ << std::endl;
        return;
    }
    written_bytes_ += buf.size();
}

inline void FileHelper::flush() {
    if (::fflush(fd_) != 0) {
        std::cerr << "Failed flush to file: " << filename_ << std::endl;
    }
}

} // namespace details
} // namespace elog
