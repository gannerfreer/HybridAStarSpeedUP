// Probe-side minimal SFML stub (ad-hoc, NOT upstream code).
// Purpose: compile the upstream core (tree.cpp/rrt.cpp) without pulling SFML
// into the minimal isolation build. Visualization functions are never called.
// This mirrors the stub approach used in prior X-S feasibility probes.
#pragma once

#include <cstddef>

namespace sf {

struct Vector2f {
  float x{0.f}, y{0.f};
  Vector2f() = default;
  Vector2f(float x_, float y_) : x(x_), y(y_) {
  }
};

struct Color {
  unsigned char r{0}, g{0}, b{0}, a{255};
  static const Color Black;
  static const Color Red;
  static const Color White;
};

inline const Color Color::Black{0, 0, 0, 255};
inline const Color Color::Red{255, 0, 0, 255};
inline const Color Color::White{255, 255, 255, 255};

struct Event {
  enum EventType { Closed, Resized, Count };
  EventType type{Count};
};

struct View {
  struct Size {
    float x{0.f}, y{0.f};
  };
  Size size{};
  Size getSize() const {
    return size;
  }
};

struct CircleShape {
  float radius{0.f};
  CircleShape() = default;
  explicit CircleShape(float r) : radius(r) {
  }
  float getRadius() const {
    return radius;
  }
  void setFillColor(const Color&) {
  }
  void setOrigin(float, float) {
  }
  void setPosition(const Vector2f&) {
  }
};

struct Vertex {
  Vector2f position;
  Color color;
  Vertex() = default;
  explicit Vertex(const Vector2f& p) : position(p) {
  }
};

enum PrimitiveType { Lines, Points };

class RenderWindow {
 public:
  bool isOpen() const {
    return open_;
  }
  void close() {
    open_ = false;
  }
  bool pollEvent(Event&) {
    return false;
  }
  View getView() const {
    return View{};
  }
  void draw(const CircleShape&) {
  }
  void draw(const Vertex*, std::size_t, PrimitiveType) {
  }

 private:
  bool open_{true};
};

}  // namespace sf
