PAL Software Architecture tools
===============================

The repository contains several tools to help develop and validate software components for the PAL architecture.

The repository is closely related to:

- the
  [architecture_schemas](https://gitlab.pal-robotics.com/interaction/architecture_schemas)
  repo, that contains the formal schemas of the various PAL software components
- the [rpk](https://gitlab.pal-robotics.com/interaction/rpk) repo, that
  contains PAL's template-based component generator

Component manifest validation
-------------------------

You can automatically check your skill manifest using `ament_archlint`.

See [ament_archlint documentation](ament_archlint/README.md) for more
information (including integration in Python and CMake-based ROS 2 packages).
