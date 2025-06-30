#pragma once

#include <cstddef>
#include <array>

/*
 * Non-owning view of a finite size array.
 */
template <typename T, size_t N>
class ArrayView {
public:
    using Type = T;
    using Self = ArrayView<T,N>;

    ArrayView(T* arr)
        : data_(arr)
    {}

    ArrayView(std::array<T,N>& arr)
        : data_(arr.data())
    {}

    ArrayView(const Self& arr)
        : data_(arr.data_)
    {}

    ArrayView(Self&& arr)
        : data_(arr.data_)
    {}

    Self& operator=(const Self& arr) {
        data_ = arr.data_;
        return *this;
    }

    Self& operator=(Self&& arr) {
        data_ = arr.data_;
        return *this;
    }

    ~ArrayView()
    {}

    T& operator[](size_t i) {
        return data_[i];
    }

    constexpr const T& operator[](size_t i) const {
        return data_[i];
    }

    constexpr static size_t size() {
        return N;
    }

    T* data() {
        return data_;
    }

    constexpr const T* data() const {
        return data_;
    }

private:
    T* data_;
};
