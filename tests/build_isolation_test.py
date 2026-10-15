#!/usr/bin/env python3
"""Verify SDK delivery and product/component selection without shipping a demo product."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

def run(*args, success=True):
    result = subprocess.run(list(map(str, args)), capture_output=True, text=True, timeout=180)
    if (result.returncode == 0) != success:
        raise RuntimeError(str(args) + '\n' + result.stdout + result.stderr)
    return result.stdout + result.stderr

def configure(source, build, *options, success=True):
    return run('cmake', '-S', source, '-B', build, '-DCMAKE_BUILD_TYPE=Debug',
               '-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF',
               '-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF', *options, success=success)

def verify(build):
    run('cmake', '--build', build, '-j4')
    run('ctest', '--test-dir', build, '--output-on-failure')

def copy(source, target):
    shutil.copytree(source, target, ignore=shutil.ignore_patterns('build*', '__pycache__'))

def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)

def passed(text):
    print('PASS: ' + text, flush=True)

with tempfile.TemporaryDirectory(prefix='roboframe-skeleton-') as directory:
    work = Path(directory)
    sdk = work / 'sdk'
    copy(ROOT / 'sdk', sdk)
    configure(sdk, work / 'sdk-build')
    verify(work / 'sdk-build')
    prefix = work / 'packages'
    run('cmake', '--install', work / 'sdk-build', '--prefix', prefix)
    passed('SDK builds/tests/installs independently of the repository')

    # This tiny build fixture is never created under repository components/products.
    component = work / 'counter'
    write(component / 'include/counter/counter.hpp',
          '#pragma once\n#include <counter/version.hpp>\nnamespace counter { int Value(); }\n')
    write(component / 'src/counter.cpp',
          '#include <counter/counter.hpp>\nnamespace counter { int Value() { return 42; } }\n')
    write(component / 'CMakeLists.txt', r"""
cmake_minimum_required(VERSION 3.16)
project(Counter LANGUAGES CXX)
include(GNUInstallDirs)
if(NOT TARGET Roboframe::sdk)
    find_package(RoboframeSDK CONFIG REQUIRED)
endif()
add_library(counter STATIC src/counter.cpp)
add_library(Counter::component ALIAS counter)
set_target_properties(counter PROPERTIES EXPORT_NAME component)
target_link_libraries(counter PUBLIC Roboframe::sdk)
target_include_directories(counter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
roboframe_component_metadata(counter counter)
install(TARGETS counter EXPORT CounterTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT CounterTargets NAMESPACE Counter:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Counter)
install(FILES CounterConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Counter)
""")
    write(component / 'CounterConfig.cmake', r"""
include(CMakeFindDependencyMacro)
find_dependency(RoboframeSDK CONFIG)
include("${CMAKE_CURRENT_LIST_DIR}/CounterTargets.cmake")
""")
    configure(component, work / 'component-build', '-DCMAKE_PREFIX_PATH=' + str(prefix))
    run('cmake', '--build', work / 'component-build', '-j4')
    run('cmake', '--install', work / 'component-build', '--prefix', prefix)
    assert (work / 'component-build/generated/counter/version.hpp').is_file()
    assert (prefix / 'include/counter/version.hpp').is_file()
    relocated = work / 'relocated'
    prefix.rename(relocated)
    passed('independent component generates and installs version.hpp')

    configure(relocated / 'share/roboframe/sdk/templates/integration',
              work / 'templates', '-DCMAKE_PREFIX_PATH=' + str(relocated))
    verify(work / 'templates')
    passed('both component-owned execution templates work with a relocated SDK package')

    headers = sorted((relocated / 'include').rglob('*.hpp'))
    for header in headers:
        probe = work / 'header.cpp'
        probe.write_text('#include <' + header.relative_to(relocated / 'include').as_posix() + '>\n')
        run('c++', '-std=c++14', '-pedantic-errors', '-pthread', '-fsyntax-only',
            '-I' + str(relocated / 'include'), probe)
    run('python3', sdk / 'tests/header_layers_test.py', relocated / 'include/roboframe')
    passed(str(len(headers)) + ' installed headers compile alone; SDK layering stays intact')

    app = work / 'app'
    app.mkdir()
    for name in ['CMakeLists.txt', 'VERSION']:
        shutil.copy2(ROOT / name, app / name)
    copy(ROOT / 'cmake', app / 'cmake')
    copy(ROOT / 'interface', app / 'interface')
    copy(sdk, app / 'sdk')
    configure(app, work / 'empty')
    verify(work / 'empty')
    assert not (work / 'empty/robot_runtime').exists()
    passed('empty host builds/tests without any product or component directory')

    version = re.search(r'set\(ROBOFRAME_SDK_VERSION "([^"]+)"\)', (sdk / 'VERSION').read_text()).group(1)
    product = app / 'products/probe'
    write(product / 'Dependencies.cmake',
          'set(ROBOFRAME_REQUIRED_SDK_VERSION "' + version + '")\n'
          'function(roboframe_load_components)\n'
          '    roboframe_component(counter Counter)\nendfunction()\n')
    write(product / 'CMakeLists.txt',
          'add_executable(probe main.cpp)\ntarget_link_libraries(probe PRIVATE Counter::component)\n')
    write(product / 'main.cpp',
          '#include <counter/counter.hpp>\nint main() { return counter::Value()!=42 || !counter::source_id[0]; }\n')
    copy(component, app / 'components/counter')
    for sdk_source, component_source in [('SOURCE','SOURCE'), ('PACKAGE','SOURCE'), ('SOURCE','PACKAGE')]:
        build = work / (sdk_source + '-' + component_source)
        configure(app, build, '-DROBOFRAME_PRODUCT=probe', '-DBUILD_TESTING=OFF',
                  '-DROBOFRAME_SDK_SOURCE=' + sdk_source, '-DROBOFRAME_COUNTER_SOURCE=' + component_source,
                  '-DCMAKE_PREFIX_PATH=' + str(relocated))
        run('cmake', '--build', build, '-j4')
        run(build / 'products/probe/probe')
    # Package-only host cannot accidentally find SDK/component source.
    shutil.rmtree(app / 'sdk')
    shutil.rmtree(app / 'components')
    configure(app, work / 'package', '-DROBOFRAME_PRODUCT=probe', '-DBUILD_TESTING=OFF',
              '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON', '-DCMAKE_PREFIX_PATH=' + str(relocated))
    run('cmake', '--build', work / 'package', '-j4')
    run(work / 'package/products/probe/probe')
    passed('new product builds/runs with source, both mixed modes, and packages only')

    error = configure(app, work / 'bad-name', '-DROBOFRAME_PRODUCT=../probe', success=False)
    assert 'Invalid ROBOFRAME_PRODUCT name' in error
    error = configure(app, work / 'missing-product', '-DROBOFRAME_PRODUCT=missing', success=False)
    assert 'Selected product needs' in error
    error = configure(app, work / 'missing-component', '-DROBOFRAME_PRODUCT=probe',
                      '-DROBOFRAME_SDK_SOURCE=PACKAGE', '-DCMAKE_PREFIX_PATH=' + str(relocated),
                      '-DROBOFRAME_COUNTER_SOURCE=SOURCE', success=False)
    assert 'SOURCE selected but component source is missing' in error
    passed('invalid product names and missing selected source fail clearly')

    configure(app, work / 'empty-package', '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON',
              '-DBUILD_TESTING=OFF', '-DCMAKE_PREFIX_PATH=' + str(relocated))
    run('cmake', '--build', work / 'empty-package', '-j4')
    passed('empty host also builds using an installed SDK without requiring Python tests')
