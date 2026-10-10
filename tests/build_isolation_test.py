#!/usr/bin/env python3
"""Build components without the framework, then exercise package-only and mixed-source products."""
from pathlib import Path
import re
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


def field(manifest, key):
    return re.search(r'set\(' + key + r' "([^"]+)"\)', manifest.read_text()).group(1)


def change_field(manifest, key, value):
    text = manifest.read_text()
    text, count = re.subn(r'(set\(' + key + r' ")[^"]+("\))',
                         lambda match: match[1] + value + match[2], text)
    assert count == 1
    manifest.write_text(text)


checks = []
with tempfile.TemporaryDirectory(prefix='roboframe-mapping-build-') as directory:
    work = Path(directory)
    sdk = work / 'sdk'
    prefix = work / 'packages'
    copy(ROOT / 'sdk', sdk)
    configure(sdk, work / 'sdk-build')
    verify(work / 'sdk-build')
    run(['cmake', '--install', work / 'sdk-build', '--prefix', prefix])
    for name in ['odom', 'map-updater', 'map-server', 'planner']:
        source = work / name
        copy(ROOT / 'components' / name, source)
        configure(source, work / (name + '-build'), prefix)
        verify(work / (name + '-build'))
        run(['cmake', '--install', work / (name + '-build'), '--prefix', prefix])
    checks.append('PASS: all 4 components independently build/test/install with SDK only')

    app = work / 'app'
    app.mkdir()
    for name in ['CMakeLists.txt', 'VERSION', 'main.cpp']:
        shutil.copy2(ROOT / name, app / name)
    for name in ['cmake', 'interface', 'integration', 'tests', 'tools']:
        copy(ROOT / name, app / name)
    relocated = work / 'relocated'
    prefix.rename(relocated)
    configure(app, work / 'package-build', relocated,
              '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON')
    verify(work / 'package-build')
    checks.append('PASS: no SDK/component source in host; relocated packages pass the IPC product')

    # Only one component source is present; odom/map-server source is not copied into the host.
    (app / 'components').mkdir()
    copy(ROOT / 'components/map-updater', app / 'components/map-updater')
    configure(app, work / 'mixed-build', relocated,
              '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON',
              '-DROBOFRAME_MAP_UPDATER_SOURCE=SOURCE')
    verify(work / 'mixed-build')
    checks.append('PASS: map-updater SOURCE + other components PACKAGE + SDK PACKAGE')

    copy(sdk, app / 'sdk')
    configure(app, work / 'source-sdk-build', relocated,
              '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON',
              '-DROBOFRAME_SDK_SOURCE=SOURCE', '-DROBOFRAME_MAP_UPDATER_SOURCE=SOURCE')
    verify(work / 'source-sdk-build')
    checks.append('PASS: one source SDK shared by source and packaged components')

    # VERSION is a real CMake dependency, not a forgotten external text file.
    version = list(map(int, field(app / 'sdk/VERSION', 'ROBOFRAME_SDK_VERSION').split('.')))
    version[2] += 1
    change_field(app / 'sdk/VERSION', 'ROBOFRAME_SDK_VERSION', '.'.join(map(str, version)))
    error = run(['cmake', '--build', work / 'source-sdk-build'], success=False)
    assert 'Product selects SDK' in error
    change_field(app / 'cmake/Dependencies.cmake', 'ROBOFRAME_REQUIRED_SDK_VERSION', '.'.join(map(str, version)))
    run(['cmake', '--build', work / 'source-sdk-build', '-j4'])
    generated = (work / 'source-sdk-build/sdk/generated/roboframe/sdk_version.hpp').read_text()
    assert '#define ROBOFRAME_SDK_VERSION_PATCH ' + str(version[2]) in generated
    checks.append('PASS: VERSION-only edit triggers configure and version header regeneration')

    # Installed manifests describe the package build, not the consumer's current SDK.
    sdk_version = field(sdk / 'VERSION', 'ROBOFRAME_SDK_VERSION')
    for name in ['odom', 'map-updater', 'map-server', 'planner']:
        manifest = relocated / 'share/roboframe' / name / 'VERSION'
        assert field(manifest, 'component_name') == name
        assert field(manifest, 'component_built_with_sdk') == sdk_version
    assert field(work / 'source-sdk-build/components/map-updater/VERSION',
                 'component_built_with_sdk') == '.'.join(map(str, version))
    # Source edits regenerate identity without any developer-maintained VERSION.
    build_manifest = work / 'source-sdk-build/components/map-updater/VERSION'
    original_id = field(build_manifest, 'component_source_id')
    source = app / 'components/map-updater/src/occupancy_map_component.cpp'
    source.write_text(source.read_text() + '\n// Local uncommitted source change.\n')
    run(['cmake', '--build', work / 'source-sdk-build', '-j4'])
    changed_id = field(build_manifest, 'component_source_id')
    assert changed_id != original_id
    assert field(relocated / 'share/roboframe/map-updater/VERSION', 'component_source_id') == original_id
    probe = work / 'metadata.cpp'
    probe.write_text('#include <map_updater/occupancy_map_component.hpp>\n'
        '#include <string>\nint main() { auto info = map_updater::GetComponentInfo();\n'
        'return std::string(info.source_id) != "' + changed_id + '" || '
        'info.built_with_sdk.patch != ' + str(version[2]) + '; }\n')
    run(['c++', '-std=c++14', '-I' + str(app / 'sdk/include'),
         '-I' + str(work / 'source-sdk-build/sdk/generated'),
         '-I' + str(app / 'components/map-updater/include'),
         '-I' + str(work / 'source-sdk-build/components/map-updater/generated'), probe,
         work / 'source-sdk-build/components/map-updater/libmap_updater.a', '-o', work / 'metadata'])
    run([work / 'metadata'])
    added = app / 'components/map-updater/include/map_updater/new_header.hpp'
    added.write_text('#pragma once\n')
    run(['cmake', '--build', work / 'source-sdk-build', '-j4'])
    assert field(build_manifest, 'component_source_id') != changed_id
    added.unlink()
    run(['cmake', '--build', work / 'source-sdk-build', '-j4'])
    assert field(build_manifest, 'component_source_id') == changed_id
    checks.append('PASS: source edits/additions/removals update identity; installed package records stay unchanged')

    # A previously published SDK package cannot masquerade as the selected product SDK.
    error = run(['cmake', '-S', app, '-B', work / 'wrong-sdk', '-DCMAKE_PREFIX_PATH=' + str(relocated),
                 '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON'], success=False)
    assert 'RoboframeSDK' in error
    # Old component packages lack the build metadata needed to safely call the new SDK structure.
    legacy = work / 'legacy-odom'
    legacy.mkdir()
    (legacy / 'OdomConfig.cmake').write_text('add_library(Odom::component INTERFACE IMPORTED)\n')
    error = run(['cmake', '-S', app, '-B', work / 'legacy-build', '-DCMAKE_PREFIX_PATH=' + str(relocated),
                 '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON', '-DROBOFRAME_SDK_SOURCE=SOURCE',
                 '-DOdom_DIR=' + str(legacy)], success=False)
    assert 'legacy package without build metadata' in error
    # Reject an SDK-major mismatch even when a source SDK target already exists.
    targets = next((relocated / 'lib/cmake/MapUpdater').glob('*Targets.cmake'))
    original_targets = targets.read_text()
    targets.write_text(original_targets.replace('ROBOFRAME_BUILT_WITH_SDK "' + sdk_version + '"',
                                               'ROBOFRAME_BUILT_WITH_SDK "9.0.0"'))
    error = run(['cmake', '-S', app, '-B', work / 'mismatch-build', '-DCMAKE_PREFIX_PATH=' + str(relocated),
                 '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON', '-DROBOFRAME_SDK_SOURCE=SOURCE'], success=False)
    assert 'is incompatible' in error
    targets.write_text(original_targets)
    checks.append('PASS: product SDK selection, legacy packages and incompatible component SDKs are rejected')

    # No component headers or integration private headers exist in this compile environment.
    public = work / 'public-only'
    (public / 'integration').mkdir(parents=True)
    (public / 'interface').mkdir()
    copy(relocated / 'include/roboframe', public / 'include/roboframe')
    for name in ['integration/robot_integration.hpp', 'interface/stdio_interface.hpp',
                 'interface/stdio_interface.cpp', 'main.cpp']:
        shutil.copy2(ROOT / name, public / name)
    run(['c++', '-std=c++14', '-pedantic-errors', '-fsyntax-only', '-I' + str(public),
         '-I' + str(public / 'include'), public / 'interface/stdio_interface.cpp', public / 'main.cpp'])
    checks.append('PASS: interface and main compile with SDK + public entry only, no component/private headers')

    headers = sorted((relocated / 'include').rglob('*.hpp'))
    for header in headers:
        probe = work / 'header.cpp'
        probe.write_text('#include <' + str(header.relative_to(relocated / 'include')) + '>\nint main() {}\n')
        run(['c++', '-std=c++14', '-pedantic-errors', '-fsyntax-only',
             '-I' + str(relocated / 'include'), probe])
    checks.append('PASS: ' + str(len(headers)) + ' installed public headers compile independently')

    error = run(['cmake', '-S', app, '-B', work / 'bad-source', '-DCMAKE_PREFIX_PATH=' + str(relocated),
                 '-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON',
                 '-DROBOFRAME_ODOM_SOURCE=SOURCE', '-DROBOFRAME_SDK_SOURCE=SOURCE'], success=False)
    assert 'SOURCE selected but component source is missing' in error
    checks.append('PASS: missing chosen source fails configuration; no silent package fallback')

(ROOT / 'build-product').mkdir(exist_ok=True)
(ROOT / 'build-product/build-check.txt').write_text('\n'.join(checks) + '\n')
print('\n'.join(checks))
