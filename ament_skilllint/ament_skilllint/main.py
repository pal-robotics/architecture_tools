#!/usr/bin/env python3

# Copyright 2025 PAL Robotics
# Copyright 2015 Open Source Robotics Foundation, Inc.
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

import argparse
import os
from pathlib import Path
import sys
import time
from xml.sax.saxutils import escape
from xml.sax.saxutils import quoteattr
import json

import ament_index_python.packages

from ament_skilllint.validate import validate_manifest


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Check ROS skills have conformant skill manifests.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        'path',
        default=[os.curdir],
        help='Path to package.xml file to validate')

    # not using a file handle directly
    # in order to prevent leaving an empty file when something fails early
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file')
    args = parser.parse_args(argv)

    if args.xunit_file:
        start_time = time.time()

    # get path to skill manifest schema
    pkg_path = ament_index_python.packages.get_package_share_directory(
        "ament_skilllint")
    schema_path = Path(pkg_path) / "schema" / "skill.schema.json"
    with open(schema_path) as f:
        schema = json.load(f)

    report = validate_manifest(
        Path(args.path) / "package.xml", schema)
    error_count = sum(len(r[1]) for r in report)

    # print summary
    if not error_count:
        print('No problems found')
        rc = 0
    else:
        print('%d errors' % error_count, file=sys.stderr)
        rc = 1

    # generate xunit file
    if args.xunit_file:
        folder_name = os.path.basename(os.path.dirname(args.xunit_file))
        file_name = os.path.basename(args.xunit_file)
        suffix = '.xml'
        if file_name.endswith(suffix):
            file_name = file_name[0:-len(suffix)]
            suffix = '.xunit'
            if file_name.endswith(suffix):
                file_name = file_name[0:-len(suffix)]
        testname = '%s.%s' % (folder_name, file_name)

        xml = get_xunit_content(report, testname, time.time() - start_time)
        path = os.path.dirname(os.path.abspath(args.xunit_file))
        if not os.path.exists(path):
            os.makedirs(path)
        with open(args.xunit_file, 'w') as f:
            f.write(xml)

    return rc


def get_xunit_content(report, testname, elapsed):
    test_count = sum(max(len(r[1]), 1) for r in report)
    error_count = sum(len(r[1]) for r in report)
    data = {
        'testname': testname,
        'test_count': test_count,
        'error_count': error_count,
        'time': '%.3f' % round(elapsed, 3),
    }
    xml = """<?xml version="1.0" encoding="UTF-8"?>
<testsuite
  name="%(testname)s"
  tests="%(test_count)d"
  errors="0"
  failures="%(error_count)d"
  time="%(time)s"
>
""" % data

    for (filename, errors) in report:
        if errors:
            # report each error as a failing testcase
            for error in errors:
                data = {
                    'quoted_location': quoteattr(
                        '%s (%s)' % (
                            error['skill'], filename)),
                    'testname': testname,
                    'quoted_message': quoteattr(error['message']),
                }
                xml += """  <testcase
    name=%(quoted_location)s
    classname="%(testname)s"
  >
      <failure message=%(quoted_message)s/>
  </testcase>
""" % data

        else:
            # if there are no lint_cmake errors report a single successful test
            data = {
                'quoted_location': quoteattr(filename),
                'testname': testname,
            }
            xml += """  <testcase
    name=%(quoted_location)s
    classname="%(testname)s"/>
""" % data

    # output list of checked files
    data = {
        'escaped_files': escape(''.join(['\n* %s' % r[0] for r in report])),
    }
    xml += """  <system-out>Checked files:%(escaped_files)s</system-out>
""" % data

    xml += '</testsuite>\n'
    return xml


if __name__ == '__main__':
    sys.exit(main())
