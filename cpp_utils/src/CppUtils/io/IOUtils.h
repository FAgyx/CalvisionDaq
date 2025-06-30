#pragma once

#include <functional>
#include <stdexcept>

namespace detail {

/*
 * Loop IO calls until N elements have been transferred.
 *
 * For various reasons, IO calls may not read / write all of the data in a single
 * call. This loop repeated attempts to transfer data until exactly N elements
 * have been transferred, or until an error occurs. The transfer function
 * signature should look like
 *
 * f(T* buffer, size_t N) -> int
 *
 * where the return value is the number of elements written. If it is 0, then
 * it's assumed to be the end of the stream. If it is negative, then an error
 * has occured.
 */
template <typename T, typename FuncType>
void staggered_io(FuncType&& f, T* buffer, size_t N) {
    size_t total = 0;
    while (total < N) {
        int result = f(buffer + total, N - total);
        if (result < 0)
            throw std::runtime_error("Error while transferring data");
        if (result == 0)
            throw std::runtime_error("End of stream while transferring data (incomplete)");

        total += static_cast<size_t>(result);
    }
}

/*
 * Predicate functor to match suffix against a given array.
 */
template <typename T, size_t N>
struct match_suffix {
    match_suffix(const T* target)
        : target_(target)
    {}

    bool operator()(T* buffer, size_t count) const {
        if (count < N) return false;

        for (size_t i = 0; i < N; i++) {
            if (buffer[count - N + i] != target_[i])
                return false;
        }
        return true;
    }

private:
    const T* target_;
};

/*
 * Like with staggered_io, but reads until it matches some condition.
 *
 * If EOF is encountered, it returns the read data even if it doesn't match
 * the predicate.
 */
template <typename T, typename FuncType, typename PredType>
size_t staggered_read(FuncType&& read, T* buffer, size_t N, PredType&& stop) {
    size_t total = 0;
    while (total < N && !stop(buffer, total)) {
        int result = read(buffer + total, N - total);
        if (result < 0)
            throw std::runtime_error("Failure in staggered_read");
        if (result == 0)
            break;

        total += static_cast<size_t>(result);
    }
    return total;
}

}
