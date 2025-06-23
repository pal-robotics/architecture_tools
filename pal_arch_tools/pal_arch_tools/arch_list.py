# Copyright (c) 2025 PAL Robotics. All rights reserved.
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

import ament_index_python as aip
import xml.etree.ElementTree as ET
import json
import yaml
from enum import Enum
from collections import UserDict


class ComponentType(Enum):
    SKILL = "skill"
    TASK = "task"
    MISSION = "mission"

# subclaqss UserDict to create a 'dot notation' dictionary


class Component(UserDict):
    """
    A class to represent a component (skill, task, mission) with its manifest.

    Inherits from UserDict to allow dot notation access to the dictionary keys.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def __getattr__(self, item):
        return self.__getitem__(item)


def get_manifests(pkg_name):
    """
    Return the manifest of a package, if it exports a skill, task or mission.

    :param pkg_name: the name of the package
    :returns: a list of tuples (type of component, manifest dictionary)
    """
    manifests = []

    path = aip.get_package_share_path(pkg_name)

    package_xml = path / 'package.xml'
    if not package_xml.exists():
        return None, None

    tree = ET.parse(package_xml)
    root = tree.getroot()
    export = root.find('export')
    if export is not None:
        for type in ["skill", "task", "mission", "app"]:
            components = export.findall(type)
            for res in components:
                content_type = res.attrib.get('content-type', 'json')
                content = res.text
                if not content.strip():
                    print(f"Error while reading the manifest of {type} {pkg_name} "
                          f"({package_xml}): empty manifest! Skipping.")
                    continue
                if content_type == 'json':
                    try:
                        manifest = json.loads(content)
                        manifests.append((type, manifest))
                    except json.decoder.JSONDecodeError as jde:
                        print(f"Error while reading the manifest of {type} {pkg_name} "
                              f"(from {package_xml}): {jde}. Skipping.")
                        continue
                elif content_type == 'yaml':
                    manifest = yaml.safe_load(content)
                    manifests.append((type, manifest))

    return manifests


def get_components(type: ComponentType | None = None):
    """
    Return the list of mission/tasks/skills installed in the system.

    Process:

    1. using ament_index, get the list of all installed packages
    2. parse each of their package.xml
    3. check if they export a <mission>, <skill> or <task>, and find the ones
    matching the required dependencies
    4. return the list of package dependencies, with their manifests.

    :param type: the type of component to look for. If None, return all components.
    """
    components = []
    for pkg, _ in aip.get_resources("packages").items():
        for c_type, manifest in get_manifests(pkg):
            if manifest and (type is None or c_type == type.value):
                if "id" not in manifest:
                    print(
                        f"Error: missing 'id' in manifest of {c_type} in {pkg}. Skipping.")
                    continue

                manifest["from_package"] = pkg
                manifest["component_type"] = c_type
                components.append(Component(**manifest))

    return components


def get_skills():
    """Return the list of skills installed in the system."""
    return get_components(ComponentType.SKILL)


def get_tasks():
    """Return the list of tasks installed in the system."""
    return get_components(ComponentType.TASK)


def get_missions():
    """Return the list of missions installed in the system."""
    return get_components(ComponentType.MISSION)


if __name__ == "__main__":

    print("Components installed in the system:")
    for id, component in get_components().items():
        pkg, c_type, manifest = component
        print(
            f"- {c_type} <{id}> ({manifest['interface']}: "
            f"{manifest['default_interface_path']}), implemented by package {pkg}")
