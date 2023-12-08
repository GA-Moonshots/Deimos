# Teamcode V1.3.5.22

This holds all of the team-written code. The version naming scheme is based on the Vulkan spec,
and is as follows:

{tweak}.{major}.{minor}.{revision}

The tweak is responsible for keeping ***multiple versions of the same file*** in check, for example, 
the Mecanum Drive that inherits from the abstract Drivetrain class is considered tweak 0, and the
Mecanum Drive that doesn't inherit from the abstract Drivetrain is considered tweak 1. 

The major version holds ***breaking changes***. These are changes that required rewriting more than
the target file, for example, the mecanum drive's autonomous commands changing from the specific
commands (Major 0) to the more general universal mode it is now (Major 1)

The minor version holds ***New features***. This would include the 3-axis arm (Minor 2) from the old 
cascade arm (Minor 1). The functional change did not break any classes, but did require a rewrite 
of the main class Deimos.

the revision hold ***bug fixes***. This includes the daily changes made, and so this version 
increments once per day since the last major or minor version change.