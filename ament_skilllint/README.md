ament_skilllint
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
  <test_depend>ament_skilllint</test_depend>

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

The schema for the skill manifest is defined in `schema/skill.schema.json`.

Usage
-----

### Python

To use the linter in a Python project, add
`<test_depend>ament_skilllint</test_depend>` to your `package.xml`.

Then, add the following to your `setup.py` (if not already present):

```python
setup(
    #...
    tests_require=['pytest'],
    #...
    )
```

And finally create a file `test/test_skilllint.py` with the following content:

```python
from ament_skilllint.main import main
import pytest

@pytest.mark.linter
@pytest.mark.skilllint
def test_skilllint():
    rc = main(argv=['.'])
    assert rc == 0, 'Found error in skill manifest'
```

### CMake

To use the linter, you simply need to depend on `ament_cmake_skilllint` in your
`package.xml`:

```xml
<test_depend>ament_cmake_skilllint</test_depend>
```

If you you `ament_lint_auto` in your `CMakeLists.txt`, the linter will be run automatically.

