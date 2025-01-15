#!/usr/bin/env python3

# Copyright 2025 PAL Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import tempfile
import json

from ament_skilllint.validate import validate_manifest

import ament_index_python.packages
# get path to skill manifest schema
pkg_path = ament_index_python.packages.get_package_share_directory(
    "ament_skilllint")
schema_path = Path(pkg_path) / "schema" / "skill.schema.json"
with open(schema_path) as f:
    schema = json.load(f)


def test_invalid_file():

    report = validate_manifest(Path("invalid_file.xml"), schema)
    assert report == [
        ('invalid_file.xml', [{'category': 'unknown', 'message': 'File not found'}])]

    report = validate_manifest(Path("invalid_file.json"), schema)
    assert report == [
        ('invalid_file.json', [{'category': 'unknown', 'message': 'File not found'}])]

    report = validate_manifest(Path("invalid_file.txt"), schema)
    assert report == [('invalid_file.txt', [
                       {'category': 'unknown', 'message': 'File not found'}])]

    with tempfile.NamedTemporaryFile(mode='w', suffix=".txt", delete=False) as f:
        f.write('hello world')
        f.close()
        report = validate_manifest(Path(f.name), schema)
        assert report == [(f.name, [
            {'category': 'unknown', 'message': 'Unsupported file type'}])]

    # write a malformed xml file and test it
    with tempfile.NamedTemporaryFile(mode='w', suffix=".xml", delete=False) as f:
        f.write('<invalid_xml>')
        f.close()
        report = validate_manifest(Path(f.name), schema)
        assert report == [
            (f.name, [{'category': 'unknown',
                       'message': 'Error parsing XML file: '
                                  'no element found: line 1, column 13'}])]

    # write a well-formed XML file with no <skill> tags and test it
    with tempfile.NamedTemporaryFile(mode='w', suffix=".xml", delete=False) as f:
        f.write('<package format="2">\n</package>')
        f.close()
        report = validate_manifest(Path(f.name), schema)
        assert report == [(f.name, [])]

    # write a well-formed XML file with a <skill> tag, but invalid JSON content
    with tempfile.NamedTemporaryFile(mode='w', suffix=".xml", delete=False) as f:
        f.write(
            '<package format="2">\n  '
            '<export>\n    '
            '<skill>invalid json</skill>\n  '
            '</export>\n</package>')
        f.close()
        report = validate_manifest(Path(f.name), schema)
        assert report == [(f.name, [
                           {'skill': 'unknown',
                            'message': 'Expecting value: line 1 column 1 (char 0)'}])]


def test_invalid_manifest():

    # missing id
    with tempfile.NamedTemporaryFile(mode='w', suffix=".xml", delete=False) as f:
        f.write(
            '<package format="2">\n  '
            '<export>\n    '
            '<skill>{"ide": "skill1"}</skill>\n  '
            '</export>\n</package>')
        f.close()
        report = validate_manifest(Path(f.name), schema)
        assert len(report[0][1]) != 0

    # missing description
    with tempfile.NamedTemporaryFile(mode='w', suffix=".xml", delete=False) as f:
        f.write(
            '<package format="2">\n  '
            '<export>\n    '
            '<skill>{"id": "skill1", '
            '"datatype": "data1", '
            '"interface": "action"}</skill>\n  '
            '</export>\n</package>')
        f.close()
        report = validate_manifest(Path(f.name), schema)
        assert len(report[0][1]) != 0

    # invalid interface
    with tempfile.NamedTemporaryFile(mode='w', suffix=".xml", delete=False) as f:
        f.write(
            '<package format="2">\n  '
            '<export>\n    '
            '<skill>{"id": "skill1", '
            '"description": "nice desc", '
            '"datatype": "data1", '
            '"interface": "telepathy"}</skill>\n  '
            '</export>\n</package>')
        f.close()
        report = validate_manifest(Path(f.name), schema)
        assert len(report[0][1]) != 0


def test_valid_file():

    # write a well-formed XML file with a <skill> tag and valid JSON content
    with tempfile.NamedTemporaryFile(mode='w', suffix=".xml", delete=False) as f:
        f.write(
            '<package format="2">\n  '
            '<export>\n    '
            '<skill>{"id": "skill1", '
            '"description": "nice desc", '
            '"datatype": "data1", '
            '"interface": "action"}</skill>\n  '
            '</export>\n</package>')
        f.close()
        report = validate_manifest(Path(f.name), schema)
        assert report == [(f.name, [])]


if __name__ == '__main__':
    test_invalid_file()
    test_valid_file()
    print('All tests passed')
