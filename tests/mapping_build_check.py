#!/usr/bin/env python3
"""Build components without the framework, then exercise package-only and mixed-source products."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(args, success=True):
    result = subprocess.run(list(map(str, args)), capture_output=True, text=True, timeout=90)
    if (result.returncode == 0) != success:
        raise RuntimeError(str(args) + '\n' + result.stdout + result.stderr)
    return result.stdout + result.stderr


def copy(source, target):
    shutil.copytree(source, target, ignore=shutil.ignore_patterns('build*', '.git', '.cache', '__pycache__'))


def configure(source, build, prefix=None, *options):
    args = ['cmake', '-S', source, '-B', build,
            '-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF', '-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF']
    if prefix:
        args.append('-DCMAKE_PREFIX_PATH=' + str(prefix))
    run(args + list(options))


def verify(build):
    run(['cmake', '--build', build, '-j4'])
    run(['ctest', '--test-dir', build, '--output-on-failure'])


checks = []
with tempfile.TemporaryDirectory(prefix='roboframe-mapping-build-') as directory:
    work = Path(directory)
    sdk = work / 'sdk'
    prefix = work / 'packages'
    copy(ROOT / 'sdk', sdk)
    configure(sdk, work / 'sdk-build')
    verify(work / 'sdk-build')
    run(['cmake', '--install', work / 'sdk-build', '--prefix', prefix])
    for name in ['sample-accumulator', 'odom-buffer', 'occupancy-grid', 'map-store']:
        source = work / name
        copy(ROOT / 'components' / name, source)
        configure(source, work / (name + '-build'), prefix)
        verify(work / (name + '-build'))
        run(['cmake', '--install', work / (name + '-build'), '--prefix', prefix])
    checks.append('PASS: all 4 components independently build/test/install with SDK only')

    app = work / 'app'
    app.mkdir()
    for name in ['CMakeLists.txt', 'VERSION', 'main.cpp', 'mapping_main.cpp']:
        shutil.copy2(ROOT / name, app / name)
    for name in ['cmake', 'interface', 'integration', 'tests', 'tools']:
        copy(ROOT / name, app / name)
    relocated = work / 'relocated'
    prefix.rename(relocated)
    configure(app, work / 'package-build', relocated,
              '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON', '-DROBOFRAME_BUILD_MAPPING=ON')
    verify(work / 'package-build')
    checks.append('PASS: no SDK/component source in host; relocated packages pass both IPC products')

    # Only one component source is present; odom/map-store/sample source is not copied into the host.
    (app / 'components').mkdir()
    copy(ROOT / 'components/occupancy-grid', app / 'components/occupancy-grid')
    configure(app, work / 'mixed-build', relocated,
              '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON', '-DROBOFRAME_BUILD_MAPPING=ON',
              '-DROBOFRAME_OCCUPANCY_GRID_SOURCE=SOURCE')
    verify(work / 'mixed-build')
    checks.append('PASS: occupancy-grid SOURCE + other components PACKAGE + SDK PACKAGE')

    copy(sdk, app / 'sdk')
    configure(app, work / 'source-sdk-build', relocated,
              '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON', '-DROBOFRAME_BUILD_MAPPING=ON',
              '-DROBOFRAME_SDK_SOURCE=SOURCE', '-DROBOFRAME_OCCUPANCY_GRID_SOURCE=SOURCE')
    verify(work / 'source-sdk-build')
    checks.append('PASS: one source SDK shared by source and packaged components')

    # VERSION is a real CMake dependency, not a forgotten external text file.
    version = list(map(int, (app / 'sdk/VERSION').read_text().strip().split('.')))
    version[2] += 1
    (app / 'sdk/VERSION').write_text('.'.join(map(str, version)) + '\n')
    run(['cmake', '--build', work / 'source-sdk-build', '-j4'])
    generated = (work / 'source-sdk-build/sdk/generated/roboframe/sdk_version.hpp').read_text()
    assert '#define ROBOFRAME_SDK_VERSION_PATCH ' + str(version[2]) in generated
    checks.append('PASS: VERSION-only edit triggers configure and version header regeneration')

    headers = sorted((relocated / 'include').rglob('*.hpp'))
    for header in headers:
        probe = work / 'header.cpp'
        probe.write_text('#include <' + str(header.relative_to(relocated / 'include')) + '>\nint main() {}\n')
        run(['c++', '-std=c++14', '-pedantic-errors', '-fsyntax-only',
             '-I' + str(relocated / 'include'), probe])
    checks.append('PASS: ' + str(len(headers)) + ' installed public headers compile independently')

    error = run(['cmake', '-S', app, '-B', work / 'bad-source', '-DCMAKE_PREFIX_PATH=' + str(relocated),
                 '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON', '-DROBOFRAME_BUILD_MAPPING=ON',
                 '-DROBOFRAME_ODOM_BUFFER_SOURCE=SOURCE'], success=False)
    assert 'SOURCE selected but component source is missing' in error
    checks.append('PASS: missing chosen source fails configuration; no silent package fallback')

(ROOT / 'build-mapping').mkdir(exist_ok=True)
(ROOT / 'build-mapping/build-check.txt').write_text('\n'.join(checks) + '\n')
print('\n'.join(checks))
