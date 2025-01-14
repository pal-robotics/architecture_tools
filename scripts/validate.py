#!/usr/bin/env python3

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

    print(f"\nValidating {path}...", end="")

    if not path.exists():
        print('File not found')
        return

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
        return

    # Extract the JSON content from each <skill> tag
    for skill in skills:
        try:
            json_skill = json.loads(skill)
            validate(json_skill, schema)
            print('OK. ', end="")
        except Exception as e:
            print(f'Error validating {path}')
            print(e)


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
