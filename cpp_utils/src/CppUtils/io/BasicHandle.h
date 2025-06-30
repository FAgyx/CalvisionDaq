#pragma once

#include "BinaryIO.h"

enum class OpenMode {
    Read,
    Truncate,
    Append,
    ReadWrite
};

class BasicHandle {
public:
    BasicHandle()
    {}

    BasicHandle(const BasicHandle&) = delete;
    BasicHandle& operator=(const BasicHandle&) = delete;

    BasicHandle(BasicHandle&&) = default;
    BasicHandle& operator=(BasicHandle&&) = default;

    virtual ~BasicHandle() {}

    virtual bool good() const = 0;
    virtual void close() = 0;
};
