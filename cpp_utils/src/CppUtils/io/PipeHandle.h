#pragma once

#include "BasicHandle.h"

class PipeHandle : public BasicHandle {
public:
    PipeHandle();

    virtual ~PipeHandle();

    virtual void open(const std::string& filename) = 0;
    virtual bool good() const override;
    virtual void close() override;

protected:
    int fd_;

    void make_pipe(const std::string& name);

    void _write(const uint8_t* buffer, size_t N);
    void _read(uint8_t* buffer, size_t N);
    size_t _var_read(uint8_t* buffer, size_t N);
};

class InputPipeHandle : public PipeHandle {
public:
    InputPipeHandle();
    InputPipeHandle(const std::string& filename);

    void open(const std::string& filename) override;
};

using PipeReader = BinaryReaderTemplate<InputPipeHandle>;




class OutputPipeHandle : public PipeHandle {
public:
    OutputPipeHandle();
    OutputPipeHandle(const std::string& filename);

    void open(const std::string& filename) override;
};

using PipeWriter = BinaryWriterTemplate<OutputPipeHandle>;
