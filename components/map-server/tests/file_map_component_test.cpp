#include <map_server/file_map_component.hpp>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <unistd.h>
void Check(bool ok) { if (!ok) throw std::runtime_error("store check failed"); }
template<class F> void Reject(F f) {
    bool caught = false; try { f(); } catch (const std::exception&) { caught = true; } Check(caught);
}
int main() {
    char directory[] = "/tmp/roboframe-store-XXXXXX";
    Check(::mkdtemp(directory) != nullptr);
    map_server::FileMapComponent store(directory);
    auto map = std::make_shared<roboframe::Grid>();
    map->spec = {2, 2, 0.1, -1, -2};
    map->revision = 3; map->stamp_us = 100;
    map->cells = {0, 254, 205, 0};
    Reject([&] { store.Save("test", map); });
    store.Activate();
    Reject([&] { store.Save("../outside", map); });
    store.Save("test", map);
    Reject([&] { store.Save("test", map); });
    auto copy = store.Load("test");
    Check(copy->cells == map->cells && copy->spec.origin_x == -1 && copy->revision == 3);
    Reject([&] { store.Load("missing"); });
    const std::string file = std::string(directory) + "/test.rfgrid";
    { std::fstream output(file, std::ios::in | std::ios::out | std::ios::binary);
      output.seekp(-1, std::ios::end); output.put(char(254)); }
    Reject([&] { store.Load("test"); });
    { std::ofstream output(file); output << "RFGRID1\n999999999 100 0.1 0 0 1 1\n0\n"; }
    Reject([&] { store.Load("test"); });
    { std::ofstream output(file); output << "RFGRID1\n-4294967295 1 1 0 0 1 1\n0\n"; }
    Reject([&] { store.Load("test"); });
    store.Deactivate();
    Reject([&] { store.Load("test"); });
    std::remove(file.c_str());
    ::rmdir(directory);
}
