#include "CppUtils/io/FileHandle.h"

#include <stdexcept>

FileHandle::FileHandle()
    : BasicHandle(), file_(nullptr)
{}

FileHandle::FileHandle(const std::string& filename, OpenMode mode)
    : FileHandle()
{
    open(filename, mode);
}

FileHandle::~FileHandle() {
    close();
}

void FileHandle::open(const std::string& filename, OpenMode mode) {
    switch (mode) {
        case OpenMode::Append:
            file_ = fopen(filename.c_str(), "a");
            break;
        case OpenMode::Truncate:
            file_ = fopen(filename.c_str(), "w");
            break;
        case OpenMode::Read:
            file_ = fopen(filename.c_str(), "r");
            break;
        case OpenMode::ReadWrite:
            file_ = fopen(filename.c_str(), "r+");
            break;
    }
}

bool FileHandle::good() const { 
    return file_ != nullptr;
}

void FileHandle::close() {
    if (good()) {
        fclose(file_);
        file_ = nullptr;
    }
}

void FileHandle::_write(const uint8_t* buffer, size_t N) {
    size_t n_write = fwrite(buffer, sizeof(uint8_t), N, file_);
    if (n_write != N)
        throw std::runtime_error("Write failure");
}

void FileHandle::_read(uint8_t* buffer, size_t N) {
    size_t n_read = _var_read(buffer, N);
    if (n_read != N)
        throw std::runtime_error("Read failure");
}

size_t FileHandle::_var_read(uint8_t* buffer, size_t N) {
    return fread(buffer, sizeof(uint8_t), N, file_);
}
