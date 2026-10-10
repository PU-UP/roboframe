#include <map_server/file_map_component.hpp>
#include <cerrno>
#include <cctype>
#include <cstdio>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <sys/stat.h>
#include <unistd.h>

namespace map_server {
static_assert(roboframe::Compatible(roboframe::sdk_version, {1, 1, 0}), "MapServer requires SDK >=1.1, same major");
roboframe::ComponentInfo GetComponentInfo() {
    return {"map-server", ROBOFRAME_COMPONENT_VERSION, roboframe::sdk_version, contract_version};
}
namespace {
std::uint64_t Hash(const std::string& metadata, const std::vector<std::uint8_t>& bytes) {
    std::uint64_t result = 14695981039346656037ULL;
    for (unsigned char ch : metadata) { result ^= ch; result *= 1099511628211ULL; }
    for (auto ch : bytes) { result ^= ch; result *= 1099511628211ULL; }
    return result; // Integrity check, not authentication.
}
std::uint64_t Unsigned(std::istream& input, std::uint64_t maximum) {
    std::string token;
    if (!(input >> token) || token.empty() || token.size() > 20 ||
        token.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("invalid unsigned map metadata");
    const auto value = std::stoull(token);
    if (value > maximum) throw std::runtime_error("map metadata exceeds limit");
    return value;
}
void Validate(const roboframe::Grid& map) {
    roboframe::ValidateGridSpec(map.spec);
    if (!map.revision || map.stamp_us < 0 ||
        map.cells.size() != std::size_t(map.spec.width) * map.spec.height)
        throw std::invalid_argument("incomplete map");
    for (auto cell : map.cells)
        if (cell != 0 && cell != 254 && cell != 205) throw std::invalid_argument("invalid occupancy cell");
}
void Write(int fd, const void* data, std::size_t length) {
    const auto* bytes = static_cast<const char*>(data);
    while (length) {
        const auto count = ::write(fd, bytes, length);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) throw std::system_error(errno, std::generic_category(), "map write");
        bytes += count;
        length -= static_cast<std::size_t>(count);
    }
}
}
FileMapComponent::FileMapComponent(std::string directory) : directory_(std::move(directory)) {}
void FileMapComponent::Activate() {
    struct stat status{};
    if (::stat(directory_.c_str(), &status) || !S_ISDIR(status.st_mode))
        throw std::invalid_argument("map directory must exist");
    lifecycle_ = roboframe::Lifecycle::active;
}
std::string FileMapComponent::Path(const std::string& name) const {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("map store inactive");
    if (name.empty() || name.size() > 80) throw std::invalid_argument("invalid map name");
    for (unsigned char ch : name)
        if (!std::isalnum(ch) && ch != '-' && ch != '_') throw std::invalid_argument("map name must be a basename");
    return directory_ + "/" + name + ".rfgrid";
}
void FileMapComponent::Save(const std::string& name, const roboframe::GridSnapshot& map) const {
    const auto destination = Path(name);
    if (!map) throw std::invalid_argument("null map");
    Validate(*map);
    std::ostringstream metadata;
    metadata << std::setprecision(17) << map->spec.width << ' ' << map->spec.height << ' '
             << map->spec.resolution << ' ' << map->spec.origin_x << ' ' << map->spec.origin_y << ' '
             << map->revision << ' ' << map->stamp_us;
    const std::string header = "RFGRID1\n" + metadata.str() + "\n" +
        std::to_string(Hash(metadata.str(), map->cells)) + "\n";
    std::string temporary = directory_ + "/.rfgrid-XXXXXX";
    std::vector<char> path(temporary.begin(), temporary.end());
    path.push_back('\0');
    int fd = ::mkstemp(path.data());
    if (fd < 0) throw std::system_error(errno, std::generic_category(), "create temporary map");
    try {
        Write(fd, header.data(), header.size());
        Write(fd, map->cells.data(), map->cells.size());
        if (::fsync(fd)) throw std::system_error(errno, std::generic_category(), "flush map");
        const int close_result = ::close(fd);
        fd = -1;
        if (close_result) throw std::system_error(errno, std::generic_category(), "close map");
        // Hard-link publication is atomic and fails if destination exists, including symlinks.
        if (::link(path.data(), destination.c_str()))
            throw std::system_error(errno, std::generic_category(), "publish map without overwrite");
        ::unlink(path.data());
        // shortcut: add directory fsync when requiring power-loss durable saves; current success is process-visible.
    } catch (...) {
        if (fd >= 0) ::close(fd);
        ::unlink(path.data());
        throw;
    }
}
roboframe::GridSnapshot FileMapComponent::Load(const std::string& name) const {
    std::ifstream input(Path(name), std::ios::binary);
    if (!input) throw std::runtime_error("cannot open map");
    auto line = [&] {
        std::string value;
        char ch;
        while (input.get(ch)) {
            if (ch == '\n') return value;
            if (value.size() >= 256) throw std::runtime_error("oversized map header");
            value.push_back(ch);
        }
        throw std::runtime_error("incomplete map header");
    };
    if (line() != "RFGRID1") throw std::runtime_error("unsupported map format");
    const auto metadata = line();
    const auto checksum = line();
    auto map = std::make_shared<roboframe::Grid>();
    std::istringstream fields(metadata);
    std::string extra;
    map->spec.width = static_cast<std::uint32_t>(Unsigned(fields, 4096));
    map->spec.height = static_cast<std::uint32_t>(Unsigned(fields, 4096));
    if (!(fields >> map->spec.resolution >> map->spec.origin_x >> map->spec.origin_y))
        throw std::runtime_error("invalid map geometry");
    map->revision = Unsigned(fields, std::numeric_limits<std::uint64_t>::max());
    map->stamp_us = static_cast<std::int64_t>(Unsigned(fields, std::numeric_limits<std::int64_t>::max()));
    if (fields >> extra) throw std::runtime_error("unexpected map metadata");
    roboframe::ValidateGridSpec(map->spec);
    map->cells.resize(std::size_t(map->spec.width) * map->spec.height);
    if (!input.read(reinterpret_cast<char*>(map->cells.data()), map->cells.size()) ||
        input.peek() != std::char_traits<char>::eof()) throw std::runtime_error("map payload length mismatch");
    Validate(*map);
    if (checksum != std::to_string(Hash(metadata, map->cells))) throw std::runtime_error("map checksum mismatch");
    return map;
}
} // namespace map_server
