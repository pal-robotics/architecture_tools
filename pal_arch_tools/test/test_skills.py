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
import os
import json

from pal_arch_tools import get_skills


def create_manifest(temp_dir, content):
    print(f'Temporary directory created at: {temp_dir}')

    # create the directory structure for ament_index
    os.makedirs(Path(temp_dir) /
                'share' / 'ament_index' / 'resource_index' / 'packages', exist_ok=True)
    open(Path(temp_dir) /
         'share' / 'ament_index' / 'resource_index' / 'packages' / 'test_pkg', 'w').close()

    # create the package.xml file
    os.makedirs(Path(temp_dir) /
                'share' / 'test_pkg', exist_ok=True)
    f = open(Path(temp_dir) / 'share' /
             'test_pkg' / 'package.xml', 'w')
    f.write(
        '<package format="2">\n  '
        '<export>\n    ')

    for component in content:
        key, values = list(component.items())[0]
        f.write(f'<{key}>\n')
        f.write(json.dumps(values))
        f.write(f'</{key}>\n')

    f.write(
        '</export>\n</package>')
    f.close()


def test_skills():

    # write a well-formed XML file with a <skill> tag and valid JSON content
    with tempfile.TemporaryDirectory() as temp_dir:

        create_manifest(temp_dir,
                        [{"skill":
                          {"id": "skill1",
                           "description": "nice desc",
                           "datatype": "data1",
                           "interface": "action"}}]
                        )

        # save the previous AMENT_PREFIX_PATH
        previous_ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')

        # add the path of the temporary file to AMENT_PREFIX_PATH
        os.environ['AMENT_PREFIX_PATH'] = str(temp_dir)

        skills = get_skills()

        assert skills
        assert len(skills) == 1

        print(f"Skills found: {skills}")
        skill = skills[0]

        assert skill.from_package == 'test_pkg'
        assert skill.component_type == 'skill'
        assert skill.id == 'skill1'
        assert skill.description == 'nice desc'
        assert skill.datatype == 'data1'
        assert skill.interface == 'action'

        # restore the previous AMENT_PREFIX_PATH
        os.environ['AMENT_PREFIX_PATH'] = previous_ament_prefix_path


if __name__ == '__main__':
    test_skills()
    print('All tests passed')
