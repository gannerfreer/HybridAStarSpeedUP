#include "astar/node2d.h"

namespace astar {

const int16_t Node2D::dir = 8;
const int16_t Node2D::dx[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
const int16_t Node2D::dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
const float Node2D::dg[8] = {1.0f, 1.414f, 1.0f, 1.414f, 1.0f, 1.414f, 1.0f, 1.414f};

Node2D Node2D::CreateSuccessor(int16_t index) const {
    const int16_t x_succ = static_cast<int16_t>(x_ + Node2D::dx[index]);
    const int16_t y_succ = static_cast<int16_t>(y_ + Node2D::dy[index]);
    return Node2D(x_succ, y_succ, g_ + Node2D::dg[index], 0.0f);
}

} // namespace astar
