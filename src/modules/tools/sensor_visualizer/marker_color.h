#include "visualization_msgs/Marker.h"
#include <unordered_map>
#include <vector>

namespace apollo {
namespace tools {

enum class Color { WHITE, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, SKY_BLUE };

struct RGB {
  // 0 ~ 255
  int r = 0;
  int g = 0;
  int b = 0;
};

// https://www.rapidtables.com/web/color/RGB_Color.html
std::unordered_map<Color, std::vector<int>> color2rgb{
    {Color::WHITE, {255, 255, 255}}, {Color::RED, {255, 0, 0}},
    {Color::GREEN, {0, 255, 0}},     {Color::YELLOW, {255, 255, 0}},
    {Color::BLUE, {0, 0, 255}},      {Color::MAGENTA, {255, 0, 255}},
    {Color::CYAN, {0, 255, 255}},    {Color::SKY_BLUE, {135, 206, 235}}};

inline void SetRGBA(visualization_msgs::Marker &marker, Color color,
                    float scale) {
  marker.color.r = static_cast<float>(color2rgb[color][0]) / 255;
  marker.color.g = static_cast<float>(color2rgb[color][1]) / 255;
  marker.color.b = static_cast<float>(color2rgb[color][2]) / 255;
  marker.color.a = scale;
}

} // namespace tools
} // namespace apollo