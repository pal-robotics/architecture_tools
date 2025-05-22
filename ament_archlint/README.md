ament_archlint
===============

The package provides a linter for ROS 2 packages that expose skills.
It checks that the package export conformant skill manifests in their
`package.xml` files.

Example of conformant skill manifest:

```xml
<?xml version="1.0"?>
<package format="3">
    <name>pkg_name</name>
    <version>0.1.0</version>
  <!-- ...regular package.xml content -->

  <!-- add this dependency to check the skill manifest -->
  <test_depend>ament_archlint</test_depend>

  <export>
      <build_type>ament_python</build_type>

      <skill content-type="json">
          {
            "id": "pkg_name",
            "description": "This is an example skill.",
            "interface": "action",
            "default_interface_path": "/pkg_name/my_skill",
            "datatype": "pkg_name_skill_msgs/action/MySkill",
            "functional_domains": [
                "communication"
            ]
        }
      </skill>
  </export>
</package>
```

The schema for the skill manifest is defined in the
[architecture_schemas](https://gitlab.pal-robotics.com/interaction/architecture_schemas)
repository.

Usage
-----

### Python

To use the linter in a Python project, add
`<test_depend>ament_archlint</test_depend>` to your `package.xml`.

Then, add the following to your `setup.py` (if not already present):

```python
setup(
    #...
    tests_require=['pytest'],
    #...
    )
```

And finally create a file `test/test_archlint.py` with the following content:

```python
from ament_archlint.main import main
import pytest

@pytest.mark.linter
@pytest.mark.archlint
def test_archlint():
    rc = main(argv=['.'])
    assert rc == 0, 'Found error in skill manifest'
```

### CMake

To use the linter, you simply need to depend on `ament_cmake_archlint` in your
`package.xml`:

```xml
<test_depend>ament_cmake_archlint</test_depend>
```

Then, make sure the linters are called in your `CMakeLists.txt`.

Here an example of a complete `CMakeLists.txt` using `ament_cmake_auto`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(your_project)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MyAction.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
    ament_auto_find_test_dependencies()
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
```
