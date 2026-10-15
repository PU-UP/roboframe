#!/usr/bin/env python3
"""Validate the actual components and product against installed/relocated packages."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]


def run(*args):
    result = subprocess.run(list(map(str, args)), capture_output=True, text=True, timeout=180)
    if result.returncode:
        raise RuntimeError(str(args) + "\n" + result.stdout + result.stderr)
    return result.stdout


def configure(source, build, *args):
    run("cmake", "-S", source, "-B", build, "-DCMAKE_BUILD_TYPE=Debug",
        "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF",
        "-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF", *args)


def verify(build):
    run("cmake", "--build", build, "-j4")
    run("ctest", "--test-dir", build, "--output-on-failure")


def copy(source, target):
    shutil.copytree(source, target, ignore=shutil.ignore_patterns("build*", "__pycache__"))


with tempfile.TemporaryDirectory(prefix="odom-mission-delivery-") as temp:
    work = Path(temp)
    prefix = work / "prefix"
    copy(ROOT / "sdk", work / "sdk")
    configure(work / "sdk", work / "sdk-build", "-DBUILD_TESTING=OFF")
    run("cmake", "--install", work / "sdk-build", "--prefix", prefix)
    for name in ["odom", "path_distance", "distance_goal"]:
        copy(ROOT / "components" / name, work / name)
        build = work / (name + "-build")
        configure(work / name, build, "-DCMAKE_PREFIX_PATH=" + str(prefix), "-DODOM_BUILD_RECEIVER=OFF")
        verify(build)
        run("cmake", "--install", build, "--prefix", prefix)
        print("PASS: independent component " + name, flush=True)
    relocated = work / "relocated"
    prefix.rename(relocated)
    consumer = work / "consumer"
    consumer.mkdir()
    (consumer / "CMakeLists.txt").write_text("""cmake_minimum_required(VERSION 3.16)
project(Consumer LANGUAGES CXX)
find_package(PathDistance CONFIG REQUIRED)
find_package(DistanceGoal CONFIG REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE PathDistance::component DistanceGoal::component)
""")
    (consumer / "main.cpp").write_text("""#include <path_distance/path_distance_component.hpp>
#include <distance_goal/distance_goal_component.hpp>
int main() {
    path_distance::PathDistanceComponent path;
    distance_goal::DistanceGoalComponent goal({});
    path.Activate(); goal.Activate();
    path.Push({1, {0, 0, 0}}); goal.UpdateDistance(0);
    path.Deactivate(); goal.Deactivate();
}
""")
    configure(consumer, work / "consumer-build", "-DCMAKE_PREFIX_PATH=" + str(relocated))
    run("cmake", "--build", work / "consumer-build", "-j4")
    run(work / "consumer-build/consumer")
    print("PASS: relocated package consumer", flush=True)

    app = work / "app"
    app.mkdir()
    for name in ["CMakeLists.txt", "VERSION"]:
        shutil.copy2(ROOT / name, app / name)
    for name in ["cmake", "interface", "products/odom_session"]:
        copy(ROOT / name, app / name)
    (app / "data/intel-lab").mkdir(parents=True)
    shutil.copy2(ROOT / "data/intel-lab/intel.log.gz", app / "data/intel-lab/intel.log.gz")
    for mode, sdk_source, component_source in [
        ("packages-only", "PACKAGE", "PACKAGE"),
        ("source-sdk", "SOURCE", "PACKAGE"),
        ("source-components", "PACKAGE", "SOURCE"),
    ]:
        if mode == "source-sdk":
            copy(ROOT / "sdk", app / "sdk")
        if mode == "source-components":
            shutil.rmtree(app / "sdk")
            for name in ["odom", "path_distance", "distance_goal"]:
                copy(ROOT / "components" / name, app / "components" / name)
        build = work / mode
        configure(app, build, "-DROBOFRAME_PRODUCT=odom_session",
                  "-DROBOFRAME_SDK_SOURCE=" + sdk_source,
                  "-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON",
                  "-DROBOFRAME_ODOM_SOURCE=" + component_source,
                  "-DROBOFRAME_PATH_DISTANCE_SOURCE=" + component_source,
                  "-DROBOFRAME_DISTANCE_GOAL_SOURCE=" + component_source,
                  "-DCMAKE_PREFIX_PATH=" + str(relocated))
        verify(build)
        print("PASS: actual product " + mode, flush=True)
