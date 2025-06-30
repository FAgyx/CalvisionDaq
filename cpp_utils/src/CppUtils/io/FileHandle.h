#pragma once

#include "BasicHandle.h"
#include <stdio.h>

class FileHandle : public BasicHandle {
public:
    FileHandle();
    FileHandle(const std::string& filename, OpenMode mode);

    virtual ~FileHandle();

    virtual bool good() const override;
    virtual void close() override;
    
protected:
    FILE* file_;

    void open(const std::string& filename, OpenMode mode);
    void _write(const uint8_t* buffer, size_t N);
    void _read(uint8_t* buffer, size_t N);
    size_t _var_read(uint8_t* buffer, size_t N);
};

using FileReader = BinaryReaderTemplate<FileHandle>;
using FileWriter = BinaryWriterTemplate<FileHandle>;
using FileReaderWriter = BinaryReaderWriterTemplate<FileHandle>;
