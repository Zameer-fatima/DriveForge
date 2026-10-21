#pragma once

#include <cstddef>
#include <vector>

namespace driveforge {

// Fixed-capacity buffer that overwrites the oldest element when full. at(0) is the oldest.
template <typename T>
class RingBuffer {
public:
    explicit RingBuffer(std::size_t capacity) : data_(capacity > 0 ? capacity : 1) {}

    void push(const T& value) {
        data_[(start_ + size_) % data_.size()] = value;
        if (size_ < data_.size()) ++size_;
        else start_ = (start_ + 1) % data_.size();
    }
    void clear() { start_ = 0; size_ = 0; }

    std::size_t size() const { return size_; }
    std::size_t capacity() const { return data_.size(); }
    bool empty() const { return size_ == 0; }
    const T& at(std::size_t i) const { return data_[(start_ + i) % data_.size()]; }

private:
    std::vector<T> data_;
    std::size_t start_ = 0;
    std::size_t size_ = 0;
};

}  // namespace driveforge
