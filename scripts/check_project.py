#!/usr/bin/env python3
# Copyright (c) 2026 00carmen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Check package structure without importing ROS or generating build output."""

import argparse
import ast
import os
from pathlib import Path
import re
import runpy
import sys
import types
import xml.etree.ElementTree as ET


def read_setup(package_dir):
    """Capture setup metadata using a temporary setuptools shim."""
    metadata = {}
    shim = types.ModuleType('setuptools')
    shim.setup = lambda **kwargs: metadata.update(kwargs)
    shim.find_packages = lambda **kwargs: ['canyon_control']
    previous_module = sys.modules.get('setuptools')
    previous_dir = Path.cwd()
    try:
        sys.modules['setuptools'] = shim
        os.chdir(package_dir)
        runpy.run_path(str(package_dir / 'setup.py'), run_name='__project_check__')
    finally:
        os.chdir(previous_dir)
        if previous_module is None:
            sys.modules.pop('setuptools', None)
        else:
            sys.modules['setuptools'] = previous_module
    return metadata


def check_project(repo, release=False):
    package = repo / 'src' / 'canyon_control'
    errors = []
    warnings = []
    trees = {}

    for path in sorted([*package.rglob('*.py'), *repo.joinpath('scripts').glob('*.py')]):
        try:
            trees[path] = ast.parse(path.read_text(encoding='utf-8-sig'),
                                    filename=str(path), feature_version=(3, 10))
        except (SyntaxError, UnicodeError) as error:
            errors.append(f'{path.relative_to(repo)}: {error}')

    xml_paths = [package / 'package.xml', *sorted(package.joinpath('worlds').glob('*'))]
    for path in xml_paths:
        if path.suffix in ('.xml', '.world', '.sdf'):
            try:
                ET.parse(path)
            except (ET.ParseError, OSError) as error:
                errors.append(f'{path.relative_to(repo)}: {error}')

    try:
        manifest = ET.parse(package / 'package.xml').getroot()
        metadata = read_setup(package)
    except Exception as error:
        return [*errors, f'Cannot read package metadata: {error}'], warnings, {}

    for key in ('name', 'version', 'description', 'license', 'maintainer'):
        if manifest.findtext(key) != metadata.get(key):
            errors.append(f'package.xml and setup.py disagree on {key}')
    maintainer = manifest.find('maintainer')
    email = maintainer.get('email', '') if maintainer is not None else ''
    if email != metadata.get('maintainer_email'):
        errors.append('package.xml and setup.py disagree on maintainer_email')
    if manifest.findtext('export/build_type') != 'ament_python':
        errors.append('Missing ament_python build type')
    if not re.fullmatch(r'\d+\.\d+\.\d+', metadata.get('version', '')):
        errors.append('Package version must have major.minor.patch format')

    entries = {}
    for entry in metadata.get('entry_points', {}).get('console_scripts', []):
        try:
            name, target = (part.strip() for part in entry.split('=', 1))
            module, function = target.split(':', 1)
            module_file = package.joinpath(*module.split('.')).with_suffix('.py')
            if name in entries:
                errors.append(f'Duplicate executable: {name}')
            entries[name] = target
            module_tree = trees.get(module_file)
            if module_tree is None or not any(
                isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                and node.name == function for node in module_tree.body
            ):
                errors.append(f'Executable target not found: {entry}')
        except ValueError:
            errors.append(f'Invalid executable declaration: {entry}')

    installed = set()
    for destination, sources in metadata.get('data_files', []):
        for source in sources:
            if not (package / source).is_file():
                errors.append(f'Installed resource is missing: {source}')
            installed.add((Path(destination) / Path(source).name).as_posix())
    expected = {
        'share/ament_index/resource_index/packages/canyon_control',
        'share/canyon_control/package.xml',
        'share/canyon_control/README.md',
        'share/canyon_control/LICENSE',
    }
    for folder in ('launch', 'worlds', 'rviz'):
        for resource in (package / folder).iterdir():
            if resource.is_file():
                expected.add(f'share/canyon_control/{folder}/{resource.name}')
    for missing in sorted(expected - installed):
        errors.append(f'Resource omitted from setup.py: {missing}')

    for launch_file in sorted((package / 'launch').glob('*.launch.py')):
        tree = trees.get(launch_file)
        if tree is None:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'Node':
                keywords = {kw.arg: kw.value.value for kw in node.keywords
                            if isinstance(kw.value, ast.Constant)}
                if keywords.get('package') == metadata['name'] and keywords.get('executable') not in entries:
                    errors.append(f'{launch_file.name}: unregistered executable {keywords.get("executable")}')
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                for extension, folder in (('.world', 'worlds'), ('.rviz', 'rviz')):
                    if node.value.endswith(extension) and not (package / folder / node.value).is_file():
                        errors.append(f'{launch_file.name}: missing resource {node.value}')

    if not (repo / 'LICENSES' / 'Apache-2.0.txt').is_file():
        errors.append('Missing Apache-2.0 text for retained third-party templates')
    package_license = package / 'LICENSE'
    if not package_license.is_file():
        errors.append('Missing package LICENSE')
    elif (repo / 'LICENSE').is_file() and package_license.read_bytes() != (repo / 'LICENSE').read_bytes():
        errors.append('Root and package LICENSE files differ')
    pending_license = 'TODO' in metadata.get('license', '') or not (repo / 'LICENSE').is_file()
    if pending_license:
        warnings.append('Project license is not finalized; this is a local preparation copy.')
    if not email or 'todo' in email.lower() or email.endswith('@example.com'):
        warnings.append('Public maintainer identity/email is not finalized.')

    details = {'python_files': len(trees), 'xml_files': len(xml_paths),
               'executables': len(entries), 'installed_resources': len(installed),
               'version': metadata.get('version')}
    if release:
        errors.extend(warnings)
    return errors, warnings, details


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', action='store_true',
                        help='fail on unfinished release metadata as well')
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    errors, warnings, details = check_project(repo, args.release)
    for key, value in details.items():
        print(f'{key}: {value}')
    for warning in warnings:
        print(f'WARNING: {warning}')
    for error in errors:
        print(f'ERROR: {error}')
    print('FAIL' if errors else 'PASS (static checks only; no ROS runtime validation)')
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
