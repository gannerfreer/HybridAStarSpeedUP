#ifndef astar_NODE2D_H
#define astar_NODE2D_H

#include <cmath>
#include <cstdint>

namespace astar {

class Node2D {
  public:
    Node2D() : Node2D(0, 0, 0.0f, 0.0f) {}

    Node2D(int16_t x, int16_t y, float g, float h) : x_(x), y_(y), g_(g), h_(h), idx_(SetIdx()) {}

    uint32_t SetIdx() const {
        uint32_t hash = 0U;
        hash |= static_cast<uint32_t>(static_cast<uint16_t>(x_)) << 16U;
        hash |= static_cast<uint32_t>(static_cast<uint16_t>(y_));
        return hash;
    }

    void Open() {
        o_ = true;
        c_ = false;
    }

    void Close() {
        c_ = true;
        o_ = false;
    }

    void Discover() {
        d_ = true;
    }

    int16_t GetX() const {
        return x_;
    }

    int16_t GetY() const {
        return y_;
    }

    float GetG() const {
        return g_;
    }

    float GetH() const {
        return h_;
    }

    float GetF() const {
        return g_ + h_;
    }

    uint32_t GetIdx() const {
        return idx_;
    }

    bool IsOpen() const {
        return o_;
    }

    bool IsClosed() const {
        return c_;
    }

    bool IsDiscovered() const {
        return d_;
    }

    void UpdateH(const Node2D& goal) {
        const float dx = static_cast<float>(x_ - goal.x_);
        const float dy = static_cast<float>(y_ - goal.y_);
        h_             = std::sqrt(dx * dx + dy * dy);
    }

    bool operator==(const Node2D& rhs) const {
        return x_ == rhs.x_ && y_ == rhs.y_;
    }

    Node2D CreateSuccessor(int16_t index) const;

    static const int16_t dir;
    static const int16_t dx[8];
    static const int16_t dy[8];
    static const float   dg[8];

  private:
    int16_t  x_;
    int16_t  y_;
    float    g_;
    float    h_;
    uint32_t idx_;
    bool     o_ = false;
    bool     c_ = false;
    bool     d_ = false;
};

} // namespace astar

#endif // astar_NODE2D_H
