#!/usr/bin/env python3
"""Verify source isolation, installed package relocation and runnable SDK templates."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / 'components/sample-accumulator'


def run(args, success=True):
    result = subprocess.run(list(map(str, args)), text=True, capture_output=True, timeout=60)
    if (result.returncode == 0) != success:
        raise RuntimeError('Command: ' + str(args) + '\n' + result.stdout + result.stderr)
    return result.stdout + result.stderr


def copy_items(source, target, names):
    target.mkdir()
    for name in names:
        item = source / name
        if item.is_dir():
            shutil.copytree(item, target / name)
        else:
            shutil.copy2(item, target / name)


def configure(source, build, prefix=None, options=()):
    args = ['cmake', '-S', source, '-B', build,
            '-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF',
            '-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF']
    if prefix is not None:
        args.append('-DCMAKE_PREFIX_PATH=' + str(prefix))
    run(args + list(options))


def build_and_test(build):
    run(['cmake', '--build', build, '-j2'])
    run(['ctest', '--test-dir', build, '--output-on-failure'])


checks = []
with tempfile.TemporaryDirectory(prefix='roboframe-isolation-') as directory:
    temp = Path(directory)
    sdk_source = temp / 'sdk-source'
    copy_items(ROOT / 'sdk', sdk_source,
               ['CMakeLists.txt', 'VERSION', 'cmake', 'include', 'tests', 'templates'])
    prefix = temp / 'installed'
    configure(sdk_source, temp / 'sdk-build')
    build_and_test(temp / 'sdk-build')
    run(['cmake', '--install', temp / 'sdk-build', '--prefix', prefix])
    checks.append('PASS: SDK builds/tests/installs without component or application')

    source = temp / 'component-source'
    copy_items(COMPONENT, source,
               ['CMakeLists.txt', 'VERSION', 'cmake', 'include', 'src', 'tests', 'examples'])
    missing = run(['cmake', '-S', source, '-B', temp / 'missing-sdk',
                   '-DCMAKE_DISABLE_FIND_PACKAGE_RoboframeSDK=TRUE'], success=False)
    assert 'RoboframeSDK' in missing
    configure(source, temp / 'component-build', prefix)
    build_and_test(temp / 'component-build')
    run(['cmake', '--install', temp / 'component-build', '--prefix', prefix])
    checks.append('PASS: standalone component and direct main need only installed SDK')

    # Package selection rejects a major mismatch before compilation.
    incompatible = temp / 'incompatible'
    incompatible.mkdir()
    (incompatible / 'CMakeLists.txt').write_text(
        'cmake_minimum_required(VERSION 3.16)\nproject(VersionCheck LANGUAGES CXX)\n'
        'find_package(RoboframeSDK 2 CONFIG REQUIRED)\n')
    diagnostic = run(['cmake', '-S', incompatible, '-B', temp / 'version-build',
                      '-DCMAKE_PREFIX_PATH=' + str(prefix)], success=False)
    assert 'RoboframeSDK' in diagnostic and 'version' in diagnostic
    checks.append('PASS: CMake rejects incompatible SDK major version')

    # Source mode bypasses find_package: the component still checks its SDK minimum.
    wrong_headers = temp / 'wrong-headers/roboframe'
    wrong_headers.mkdir(parents=True)
    (wrong_headers / 'sdk_version.hpp').write_text(
        '#define ROBOFRAME_SDK_VERSION_MAJOR 2\n'
        '#define ROBOFRAME_SDK_VERSION_MINOR 0\n'
        '#define ROBOFRAME_SDK_VERSION_PATCH 0\n'
        '#define ROBOFRAME_SDK_VERSION_STRING "2.0.0"\n')
    diagnostic = run(['c++', '-std=c++14', '-fsyntax-only',
                      '-DSAMPLE_ACCUMULATOR_VERSION="1.0.0"',
                      '-I' + str(wrong_headers.parent), '-I' + str(prefix / 'include'),
                      '-I' + str(source / 'include'), source / 'src/component.cpp'], success=False)
    assert 'requires RoboframeSDK 1.x' in diagnostic
    checks.append('PASS: source build rejects incompatible SDK at compile time')

    relocated = temp / 'relocated-packages'
    prefix.rename(relocated)
    app = temp / 'app-source'
    copy_items(ROOT, app, ['CMakeLists.txt', 'VERSION', 'main.cpp',
                           'interface', 'integration', 'tests', 'cmake'])
    configure(app, temp / 'app-build', relocated,
              ['-DROBOFRAME_USE_INSTALLED_COMPONENTS=ON', '-DROBOFRAME_BUILD_MAPPING=OFF'])
    build_and_test(temp / 'app-build')
    checks.append('PASS: relocated packages build source-isolated application; IPC unchanged')

    # Actual library metadata must be checked, not fabricated from the host headers.
    runtime_probe = temp / 'runtime-probe.cpp'
    runtime_probe.write_text(
        '#include <sample_accumulator/component.hpp>\n#include <stdexcept>\n'
        'int main() {\n'
        ' const auto info = sample_accumulator::GetComponentInfo();\n'
        ' try { roboframe::CheckCompatibility(info, {2,0,0}); }\n'
        ' catch (const std::runtime_error&) { return 0; }\n return 1;\n}\n')
    archive = next(relocated.rglob('libsample_accumulator.a'))
    run(['c++', '-std=c++14', '-pthread', '-I' + str(relocated / 'include'),
         runtime_probe, archive, '-o', temp / 'runtime-probe'])
    run([temp / 'runtime-probe'])
    # Recompile only the host against an incompatible SDK header; keep the library unchanged.
    runtime_probe.write_text(runtime_probe.read_text().replace('{2,0,0}', '{1,0,0}'))
    run(['c++', '-std=c++14', '-pthread', '-I' + str(wrong_headers.parent),
         '-I' + str(relocated / 'include'), runtime_probe, archive,
         '-o', temp / 'runtime-sdk-probe'])
    run([temp / 'runtime-sdk-probe'])
    checks.append('PASS: runtime rejects SDK/contract mismatch using unchanged compiled library metadata')

    # Install templates are genuine projects, not illustrative snippets.
    template = temp / 'starter'
    shutil.copytree(relocated / 'share/roboframe/templates', template)
    configure(template, temp / 'starter-build', relocated)
    build_and_test(temp / 'starter-build')
    component_template = temp / 'component-template-only'
    shutil.copytree(template / 'component', component_template)
    configure(component_template, temp / 'template-component-build', relocated)
    build_and_test(temp / 'template-component-build')
    checks.append('PASS: installed component/integration templates and component-only main run')

    headers = sorted(str(path.relative_to(relocated / 'include'))
                     for path in (relocated / 'include').rglob('*.hpp'))
    headers += ['integration/components.hpp', 'integration/demo_integration.hpp',
                'interface/stdio_interface.hpp']
    for header in headers:
        source_file = temp / 'header.cpp'
        source_file.write_text('#include "' + header + '"\nint main() {}\n')
        run(['c++', '-std=c++14', '-pedantic-errors', '-fsyntax-only',
             '-I' + str(relocated / 'include'), '-I' + str(app), source_file])
    checks.append('PASS: ' + str(len(headers)) + ' public headers compile independently as C++14')

(ROOT / 'build').mkdir(exist_ok=True)
(ROOT / 'build/isolation-check.txt').write_text('\n'.join(checks) + '\n')
print('\n'.join(checks))
