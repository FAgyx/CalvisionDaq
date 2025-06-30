#include "Socket.h"

#include "CppUtils/c_util/CUtil.h"
#include "CppUtils/io/IOUtils.h"

#include <unistd.h>
#include <cstring>
#include <sys/types.h>
#include <netdb.h>
#include <stdexcept>


SocketHandle::SocketHandle()
    : socket_fd_(-1)
{
    initialize_zero(address_);
}

SocketHandle::~SocketHandle() {
    close();
}

bool SocketHandle::good() const {
    return socket_fd_ > 0;
}

void SocketHandle::close() {
    if (good()) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

void SocketHandle::listen(int port) {
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (!good())
        throw std::runtime_error("Error opening server socket");

    address_.sin_family = AF_INET;
    address_.sin_port = htons(port);
    address_.sin_addr.s_addr = INADDR_ANY;

    if (bind(socket_fd_, (struct sockaddr*) &address_, sizeof(address_)) < 0)
        throw std::runtime_error("Error on binding");

    ::listen(socket_fd_, 5);
}

void SocketHandle::connect(const std::string& hostname, int port) {
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (!good())
        throw std::runtime_error("Error opening client socket");

    struct hostent* hostentry = gethostbyname(hostname.c_str());
    if (!hostentry)
        throw std::runtime_error("Error finding hostname " + hostname);

    address_.sin_family = AF_INET;
    address_.sin_port = htons(port);
    copy_raw_buffer(&address_.sin_addr.s_addr, hostentry->h_addr, hostentry->h_length);

    if (::connect(socket_fd_, (struct sockaddr*) &address_, sizeof(address_)) < 0)
        throw std::runtime_error("Error on connecting");
}

void SocketHandle::accept(const SocketHandle& server) {
    // Not currently using client address...
    socket_fd_ = ::accept(server.socket_fd_, nullptr, nullptr);
    if (!good())
        throw std::runtime_error("Error opening client socket");
}

void SocketHandle::_write(const uint8_t* buffer, size_t N) {

    detail::staggered_io(
            [this] (const uint8_t* xs, size_t n) {
                return ::send(socket_fd_, xs, n, 0);
            },
            buffer, N);
}

void SocketHandle::_read(uint8_t* buffer, size_t N) {

    detail::staggered_io(
            [this] (uint8_t* xs, size_t n) {
                return this->_var_read(xs, n);
            },
            buffer, N);
}

size_t SocketHandle::_var_read(uint8_t* buffer, size_t N) {
    int result = ::recv(socket_fd_, buffer, N, 0);
    if (result < 0)
        throw std::runtime_error("Error while reading data from socket");
    else
        return static_cast<size_t>(result);
}
