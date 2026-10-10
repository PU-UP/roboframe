#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <vector>
#include <utility>

namespace roboframe {
// Copy the block table, share bytes. Set never exposes a writable alias to a published block.
// A writer owns its GridCells instance; independent copies can be read concurrently.
class GridCells {
public:
    static constexpr std::size_t block_capacity = 4096;
    GridCells() = default;
    GridCells(const GridCells&) = default;
    GridCells(GridCells&& other) noexcept : size_(other.size_), blocks_(std::move(other.blocks_)) {
        other.size_ = 0;
        other.blocks_.clear();
    }
    GridCells& operator=(GridCells other) noexcept {
        std::swap(size_, other.size_);
        blocks_.swap(other.blocks_);
        return *this;
    }
    explicit GridCells(std::size_t size, std::uint8_t value = 205) : size_(size) {
        for (std::size_t offset = 0; offset < size; offset += block_capacity) {
            auto block = std::make_shared<Block>();
            block->fill(value);
            blocks_.push_back(std::move(block));
        }
    }
    GridCells(std::initializer_list<std::uint8_t> values) : GridCells(values.size()) {
        std::size_t i = 0;
        for (auto value : values) Set(i++, value);
    }
    std::size_t size() const { return size_; }
    std::size_t block_count() const { return blocks_.size(); }
    std::size_t table_bytes() const { return blocks_.size() * sizeof(blocks_[0]); }
    const std::uint8_t* BlockData(std::size_t block) const { return blocks_.at(block)->data(); }
    std::size_t BlockSize(std::size_t block) const {
        if (block >= blocks_.size()) throw std::out_of_range("grid block");
        return std::min(std::size_t(block_capacity), size_ - block * block_capacity);
    }
    std::uint8_t operator[](std::size_t index) const {
        if (index >= size_) throw std::out_of_range("grid cell");
        return (*blocks_[index / block_capacity])[index % block_capacity];
    }
    // Return copied bytes for resource accounting, not changed-cell count.
    std::size_t Set(std::size_t index, std::uint8_t value) {
        if (index >= size_) throw std::out_of_range("grid cell");
        auto& block = blocks_[index / block_capacity];
        const auto offset = index % block_capacity;
        if ((*block)[offset] == value) return 0;
        std::size_t copied = 0;
        if (!block.unique()) {
            block = std::make_shared<Block>(*block);
            copied = block_capacity;
        }
        (*block)[offset] = value;
        return copied;
    }
    bool operator==(const GridCells& other) const {
        if (size_ != other.size_) return false;
        for (std::size_t i = 0; i < block_count(); ++i)
            if (blocks_[i] != other.blocks_[i] &&
                !std::equal(BlockData(i), BlockData(i) + BlockSize(i), other.BlockData(i))) return false;
        return true;
    }
private:
    using Block = std::array<std::uint8_t, block_capacity>;
    std::size_t size_ = 0;
    std::vector<std::shared_ptr<Block>> blocks_;
};
} // namespace roboframe
