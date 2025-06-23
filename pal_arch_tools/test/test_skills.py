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
import pytest

from pal_arch_tools import get_skills, get_tasks, get_missions


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
        f.write(f'\n</{key}>\n')

    f.write(
        '</export>\n</package>')
    f.close()

    # add the path of the temporary file to AMENT_PREFIX_PATH
    os.environ['AMENT_PREFIX_PATH'] = str(temp_dir)


@pytest.fixture(autouse=True)
def protect_ament_prefix():
    # save the previous AMENT_PREFIX_PATH
    previous_ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')

    # the test function will be run at this point
    yield

    # restore the previous AMENT_PREFIX_PATH
    os.environ['AMENT_PREFIX_PATH'] = previous_ament_prefix_path


def test_single_skill():

    # write a well-formed XML file with a <skill> tag and valid JSON content
    with tempfile.TemporaryDirectory() as temp_dir:

        create_manifest(temp_dir,
                        [{"skill":
                          {"id": "skill1",
                           "description": "nice desc",
                           "datatype": "data1",
                           "interface": "action"}}]
                        )

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


def test_multi_skills():

    # write a well-formed XML file with a <skill> tag and valid JSON content
    # with tempfile.TemporaryDirectory() as temp_dir:
    temp_dir = "/tmp/test_skills"
    create_manifest(temp_dir,
                    [{"skill":
                        {"id": "skill1",
                         "description": "nice desc",
                         "datatype": "data1",
                         "interface": "action"}
                      },
                     {"skill":
                        {"id": "skill2",
                         "description": "nice desc",
                         "datatype": "data2",
                         "interface": "topic"}
                      }]
                    )

    skills = get_skills()

    print(f"Skills found: {skills}")
    assert len(skills) == 2

    assert {skills[0].id, skills[1].id} == {'skill1', 'skill2'}
    assert {skills[0].datatype, skills[1].datatype} == {'data1', 'data2'}
    assert {skills[0].interface, skills[1].interface} == {'topic', 'action'}


def test_multi_components():

    # write a well-formed XML file with a <skill> tag and valid JSON content
    # with tempfile.TemporaryDirectory() as temp_dir:
    temp_dir = "/tmp/test_skills"
    create_manifest(temp_dir,
                    [{"skill":
                        {"id": "skill1",
                         "description": "nice desc",
                         "datatype": "data1",
                         "interface": "action"}
                      },
                     {"skill":
                        {"id": "skill2",
                         "description": "nice desc",
                         "datatype": "data2",
                         "interface": "topic"}
                      },
                     {"mission":
                        {"id": "mission1",
                         "description": "nice desc",
                         "datatype": "data2",
                         "interface": "action"}
                      },
                     {"task":
                        {"id": "task1",
                         "description": "nice desc",
                         "datatype": "data2",
                         "interface": "action"}
                      }]
                    )

    skills = get_skills()

    print(f"Skills found: {skills}")
    assert len(skills) == 2

    assert {skills[0].id, skills[1].id} == {'skill1', 'skill2'}
    assert {skills[0].datatype, skills[1].datatype} == {'data1', 'data2'}
    assert {skills[0].interface, skills[1].interface} == {'topic', 'action'}

    tasks = get_tasks()
    print(f"Tasks found: {tasks}")
    assert len(tasks) == 1
    assert tasks[0].id == 'task1'
    assert tasks[0].datatype == 'data2'
    assert tasks[0].interface == 'action'

    missions = get_missions()
    print(f"Missions found: {missions}")
    assert len(missions) == 1
    assert missions[0].id == 'mission1'
    assert missions[0].datatype == 'data2'
    assert missions[0].interface == 'action'


if __name__ == '__main__':
    test_single_skill()
    test_multi_skills()
    test_multi_components()
    print('All tests passed')
