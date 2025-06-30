#include "PipeHandle.h"
#include "IOUtils.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

PipeHandle::PipeHandle()
    : BasicHandle(), fd_(-1)
{}

PipeHandle::~PipeHandle()
{
    close();
}

bool PipeHandle::good() const {
    return fd_ >= 0;
}

void PipeHandle::close() {
    if (good()) {
        ::close(fd_);
        fd_ = -1;
    }
}

void PipeHandle::_write(const uint8_t* buffer, size_t N) {
    detail::staggered_io(
            [fd = fd_] (const uint8_t* xs, size_t n) {
                return ::write(fd, xs, n);
            },
            buffer, N);
}


void PipeHandle::_read(uint8_t* buffer, size_t N) {
    detail::staggered_io(
            [this] (uint8_t* xs, size_t n) {
                return this->_var_read(xs, n);
            },
            buffer, N);
}

size_t PipeHandle::_var_read(uint8_t* buffer, size_t N) {
    int result = ::read(fd_, buffer, N);
    if (result < 0)
        throw std::runtime_error("Failed to read pipe: " + std::to_string(result));
    return static_cast<size_t>(result);
}

void PipeHandle::make_pipe(const std::string& name) {
    if(mkfifo(name.c_str(), 0666) == -1) {
        int errsv = errno;
        if (errsv == EEXIST) {
            // do nothing
        } else {
            throw std::runtime_error("Making named FIFO pipe failed with errno = " + std::to_string(errsv));
        }
    }
}



InputPipeHandle::InputPipeHandle()
    : PipeHandle()
{}

InputPipeHandle::InputPipeHandle(const std::string& name) {
    open(name);
}

void InputPipeHandle::open(const std::string& filename) {
    make_pipe(filename);
    fd_ = ::open(filename.c_str(), O_RDONLY | O_NONBLOCK);
}


OutputPipeHandle::OutputPipeHandle()
    : PipeHandle()
{}

OutputPipeHandle::OutputPipeHandle(const std::string& name) {
    open(name);
}

void OutputPipeHandle::open(const std::string& filename) {
    make_pipe(filename);
    fd_ = ::open(filename.c_str(), O_WRONLY | O_NONBLOCK);
}
