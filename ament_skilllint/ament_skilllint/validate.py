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

import os
import json
from jsonschema import validate
import xml.etree.ElementTree as ET
import argparse
from pathlib import Path

try:
    import ament_index_python.packages
except ImportError:
    print("ament_index_python not found. ROS 2 packages will not be validated. You can still validate JSON files.")


def validate_manifest(path: Path, schema: dict):

    report = []

    print(f"Validating {path}:")

    if not path.exists():
        print('File not found')
        report.append(
            (f'{path}', [{'category': 'unknown', 'message': 'File not found'}]))
        return report

    if path.suffix == '.xml':

        # Parse the XML
        tree = ET.parse(path)
        root = tree.getroot()

        # Find all <skill> tags inside <export> tags
        skill_tags = root.findall(".//export/skill")
        skills = [s.text.strip() for s in skill_tags]

    elif path.suffix == '.json':
        with open(path) as f:
            skills = [f.read()]
    else:
        print('Unsupported file type')
        report.append(
            (f'{path}', [{'category': 'unknown', 'message': 'Unsupported file type'}]))
        return report

    # Extract the JSON content from each <skill> tag
    for skill in skills:
        try:
            json_skill = {}
            json_skill = json.loads(skill)
            validate(json_skill, schema)
            print(f'\t- skill <{json_skill["id"]}>: OK')
            report.append((f'{path}', []))
        except Exception as e:
            print(f'Error validating {path}')
            report.append(
                (f'{path}', [{'skill': json_skill["id"] if "id" in json_skill else "unknown", 'message': str(e)}]))
            print(e)

    return report


if __name__ == "__main__":

    # use argparse to get the name of the schema file and the name(s) of the xml files to validate
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--schema", default="skill.schema.json",
                    help="Path to the schema file")
    ap.add_argument("pkgs", nargs='+',
                    help="ROS 2 package names, or direct JSON manifest to validate")

    args = ap.parse_args()

    with open(args.schema) as f:
        schema = json.load(f)

    for package in args.pkgs:
        if package.endswith('.json'):
            validate_manifest(Path(package), schema)
            print()
        if package.endswith('.xml'):
            validate_manifest(Path(package), schema)
            print()
        else:
            package_path = ament_index_python.packages.get_package_share_directory(
                package)
            pkg = Path(package_path) / "package.xml"
            validate_manifest(package, schema)
            print()
