#include "pure_nav/map_io.hpp"
#include <cctype>
#include <filesystem>
#include <fstream>
#include <limits>
#include <yaml-cpp/yaml.h>
namespace pure_nav {
namespace {
std::string token(std::istream &in) {
    std::string value;
    while (in >> value) {
        if (value.front() == '#') {
            in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }
        return value;
    }
    throw std::runtime_error("truncated PGM header/data");
}
int integer(const std::string &text) {
    std::size_t used{};
    int value = std::stoi(text, &used);
    if (used != text.size())
        throw std::runtime_error("invalid PGM integer");
    return value;
}
} // namespace
Grid loadMap(const std::string &path) {
    const auto config = YAML::LoadFile(path);
    if (config["mode"] && config["mode"].as<std::string>() != "trinary")
        throw std::runtime_error("only trinary maps supported");
    const auto origin = config["origin"].as<std::vector<double>>();
    if (origin.size() != 3 || !std::isfinite(origin[0]) || !std::isfinite(origin[1]) || origin[2] != 0)
        throw std::runtime_error("map origin must be finite [x,y,0]");
    const double occupied = config["occupied_thresh"].as<double>(), free = config["free_thresh"].as<double>();
    if (!std::isfinite(occupied) || !std::isfinite(free) || free < 0 || occupied > 1 || free >= occupied)
        throw std::runtime_error("invalid occupancy thresholds");
    const int negate = config["negate"].as<int>();
    if (negate != 0 && negate != 1)
        throw std::runtime_error("negate must be 0 or 1");
    const auto image = std::filesystem::path(path).parent_path() / config["image"].as<std::string>();
    std::ifstream in(image, std::ios::binary);
    if (!in)
        throw std::runtime_error("cannot open map image: " + image.string());
    const auto magic = token(in);
    if (magic != "P5" && magic != "P2")
        throw std::runtime_error("expected PGM P5 or P2");
    const int width = integer(token(in)), height = integer(token(in)), maximum = integer(token(in));
    if (maximum <= 0 || maximum > 255)
        throw std::runtime_error("only 8-bit PGM supported");
    Grid grid(width, height, config["resolution"].as<double>());
    grid.origin_x = origin[0];
    grid.origin_y = origin[1];
    if (magic == "P5") {
        char sep{};
        in.get(sep);
        if (sep == '\r' && in.peek() == '\n')
            in.get();
        if (!std::isspace(static_cast<unsigned char>(sep)))
            throw std::runtime_error("invalid PGM separator");
    }
    for (int row = 0; row < height; ++row)
        for (int x = 0; x < width; ++x) {
            const int value = magic == "P5" ? in.get() : integer(token(in));
            if (value < 0 || value > maximum)
                throw std::runtime_error("invalid or truncated PGM pixels");
            const double probability =
                negate ? static_cast<double>(value) / maximum : 1 - static_cast<double>(value) / maximum;
            grid.mark(x, height - 1 - row, probability > occupied ? 254 : (probability < free ? 0 : 255));
        }
    return grid;
}
} // namespace pure_nav
