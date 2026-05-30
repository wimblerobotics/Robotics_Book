---
title: "Workspace Troubleshooting Guide"
chapter_number: TBD
dependencies: ["understanding_the_workspace_chain.md"]
related_chapters: ["managing_multiple_workspaces.md", "launch_files.md"]
related_skills: []
related_repos: []
last_verified: "2026-05-20"
ros_version: "jazzy"
status: "draft"
---

# Workspace Troubleshooting Guide

<details open>
<summary>Diagnosing and fixing common workspace and package search problems</summary>
</details>

You've built your workspace, sourced it correctly (you think), but things still aren't working. ROS can't find your package, or it's finding the wrong version, or your changes aren't taking effect. This chapter walks you through systematic troubleshooting for workspace and package search problems.

Each section starts with **symptoms** (what you're experiencing), followed by **diagnosis** (how to confirm the problem), and **solutions** (how to fix it).

## Symptom: "Package 'my_package' not found"

You know you built the package, but ROS 2 claims it doesn't exist:

```bash
$ ros2 launch my_robot_bringup robot.launch.py
Package 'my_robot_bringup' not found
```

### Diagnosis: Is the Package Actually Built?

Check if the package exists in your install directory:

```bash
$ ls ~/ros2_ws/install/
```

Do you see a directory for `my_robot_bringup`? If not, the package didn't build successfully.

Check the build output:

```bash
$ cd ~/ros2_ws
$ colcon build --packages-select my_robot_bringup
```

Look for errors. Common issues:
- Missing dependencies in `package.xml`
- Syntax errors in `CMakeLists.txt` or `setup.py`
- Python import errors
- Missing files referenced in install instructions

If the build says "0 packages finished" or shows errors, fix those first before continuing.

### Diagnosis: Is the Workspace Sourced?

Even if the package built successfully, ROS won't find it unless you sourced the workspace:

```bash
$ echo $AMENT_PREFIX_PATH
```

Does this include your workspace path (e.g., `/home/user/ros2_ws/install`)? If not, you haven't sourced it in this terminal.

**Solution**: Source your workspace:

```bash
$ source ~/ros2_ws/install/setup.bash
$ echo $AMENT_PREFIX_PATH
/home/user/ros2_ws/install:/opt/ros/jazzy
```

### Diagnosis: Are You in the Right Terminal?

Did you source the workspace in Terminal 1 but try to run commands in Terminal 2?

Each terminal session has its own environment. Sourcing in one terminal doesn't affect others.

**Solution**: Either:
- Source the workspace in the terminal you're actually using, or
- Add `source ~/ros2_ws/install/setup.bash` to your `~/.bashrc` so it happens automatically (see gotchas below)

### Diagnosis: Did You Misspell the Package Name?

List all packages ROS can find:

```bash
$ ros2 pkg list | grep my_robot
```

Is your package there? If not, either:
- It's not built
- It's not sourced
- The package name in `package.xml` doesn't match what you're trying to use

Check `package.xml`:

```xml
<package format="3">
  <name>my_robot_bringup</name>  <!-- ← This must match exactly -->
</package>
```

**Solution**: Use the exact name from `package.xml`, or fix `package.xml` and rebuild.

### The Nuclear Option: Clean Rebuild

If none of the above helps:

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

This guarantees a fresh build with no stale artifacts.

<details>
<summary><strong>Deep Dive: What Can Go Wrong During Build</strong></summary>

`colcon build` processes packages in dependency order. If package A depends on package B, colcon builds B first.

But `colcon build` doesn't stop on errors by default—it continues building what it can. This means you might see:

```
Starting >>> package_a
Starting >>> package_b
Finished <<< package_a [2.5s]
Failed <<< package_b
Finished <<< package_c
```

Package B failed, but package A and C succeeded. If package C depended on B, it might have built with stale artifacts or missing dependencies.

**Best practice**: Check the summary at the end:

```
Summary: 5 packages finished [1min 30s]
  1 package failed: package_b
```

If any package failed, fix it and rebuild. Don't assume everything is fine because some packages built.

Use `--packages-up-to my_package` to build only what's needed for a specific package, stopping on errors:

```bash
colcon build --packages-up-to my_robot_bringup
```

</details>

## Symptom: "My code changes aren't taking effect"

You modified a Python file, rebuilt (or didn't, if using `--symlink-install`), but the old code still runs.

### Diagnosis: Did You Build with --symlink-install?

Check how you built:

```bash
$ cd ~/ros2_ws
$ cat build/COLCON_IGNORE  # If this exists, build artifacts are here
$ ls -la install/my_package/lib/my_package/my_node.py
```

If the file in `install/` is a regular file (not a symlink), you didn't use `--symlink-install`, so changes require rebuild.

**Solution**: Rebuild with symlinks:

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

### Diagnosis: Did You Change Something That Needs Rebuild?

Even with `--symlink-install`, some changes require rebuild:

**Don't need rebuild** (✓):
- Python source code in existing modules
- Launch files
- Configuration YAML files
- URDF files

**Do need rebuild** (❌):
- C++ code (always needs rebuild)
- Changes to `package.xml`
- Changes to `CMakeLists.txt` or `setup.py`
- Adding new Python modules or entry points
- Adding new packages

If you changed something in the "needs rebuild" list:

**Solution**:

```bash
cd ~/ros2_ws
colcon build --packages-select my_changed_package --symlink-install
source install/setup.bash  # In a fresh terminal
```

### Diagnosis: Are You Running the Wrong Version?

Your workspace might be built and sourced, but ROS is finding a different version of the package (maybe from `/opt/ros/jazzy`).

Check which version is being used:

```bash
$ ros2 pkg prefix my_robot_bringup
/opt/ros/jazzy
```

If this shows `/opt/ros/jazzy` but you expected `~/ros2_ws`, the search order is wrong.

Check the search path:

```bash
$ echo $AMENT_PREFIX_PATH
/opt/ros/jazzy:/home/user/ros2_ws/install  # ← WRONG ORDER!
```

Your workspace should be first! If `/opt/ros/jazzy` is first, you likely sourced it after sourcing your workspace.

**Solution**: Open a fresh terminal and source only your workspace:

```bash
# New terminal
source ~/ros2_ws/install/setup.bash
echo $AMENT_PREFIX_PATH
/home/user/ros2_ws/install:/opt/ros/jazzy  # ← Correct order
```

### Diagnosis: Python Bytecode Cache

Python caches compiled `.pyc` files in `__pycache__` directories. Sometimes these get stale.

**Solution**: Delete Python cache:

```bash
find ~/ros2_ws -type d -name __pycache__ -exec rm -rf {} +
find ~/ros2_ws -name "*.pyc" -delete
```

Then try running again (no rebuild needed).

### Diagnosis: Are You Running a Daemon or Long-Running Process?

If you have a ROS 2 node running in the background (maybe from a previous test), killing and restarting it will load the new code.

**Solution**: Kill the old process:

```bash
# Find ROS 2 processes
ps aux | grep ros2

# Kill specific node
killall -9 my_node_name

# Or kill all ROS 2 processes (use with caution)
killall -9 ros2
```

Then restart your launch file.

## Symptom: "Wrong package version is loading"

You have a custom version of `nav2_bringup` in your workspace, but the system version keeps loading.

### Diagnosis: Check Search Order

```bash
$ ros2 pkg prefix nav2_bringup
/opt/ros/jazzy/share/nav2_bringup  # ← System version

$ echo $AMENT_PREFIX_PATH
/opt/ros/jazzy:/home/user/ros2_ws/install  # ← Wrong order
```

The problem: `/opt/ros/jazzy` is first in the search path.

**Solution**: Fix the search order (see previous section).

### Diagnosis: Package Name Mismatch

Maybe your custom package has a different name than you think.

```bash
$ ls ~/ros2_ws/install/
nav2_custom_bringup  # ← You named it differently!
```

If your `package.xml` says `<name>nav2_custom_bringup</name>`, then you need to use that name, not `nav2_bringup`.

**Solution**: Either:
- Use the actual package name: `ros2 launch nav2_custom_bringup ...`
- Or rename the package (edit `package.xml`, rebuild)

### Diagnosis: Launch File References

Your launch file might explicitly reference the system package. Check your launch file:

```python
from ament_index_python.packages import get_package_share_directory

# This will search for 'nav2_bringup' in the workspace chain
pkg_dir = get_package_share_directory('nav2_bringup')
```

If you renamed your custom package to `nav2_custom_bringup`, but the launch file still says `nav2_bringup`, it will load the system version.

**Solution**: Update the launch file to reference your package name:

```python
pkg_dir = get_package_share_directory('nav2_custom_bringup')
```

Or, if you want true overlay behavior, keep the name exactly as `nav2_bringup` in your custom package.

<details>
<summary><strong>Deep Dive: True Overlay vs. Different Package Name</strong></summary>

There are two strategies for customizing existing packages:

**Strategy 1: True Overlay (same name)**
- Your package has the exact same name: `nav2_bringup`
- Located in your workspace
- Searched before the system version
- Pros: Launch files and other references work unchanged
- Cons: Can be confusing (which nav2_bringup is running?)

**Strategy 2: Different Name**
- Your package has a different name: `nav2_custom_bringup`
- Clearly distinct from system package
- Pros: No ambiguity about which version
- Cons: Must update all references to use new name

For experimenting, Strategy 1 (true overlay) is often easier. For production, Strategy 2 (different name) is clearer.

</details>

## Symptom: "Package found, but files inside are wrong"

ROS finds your package, but loads wrong launch files, config files, or other resources.

### Diagnosis: Multiple Versions in Search Path

```bash
$ echo $AMENT_PREFIX_PATH
/home/user/ros2_ws/install:/opt/ros/jazzy:/home/user/ros2_ws/install  # ← Duplicate!
```

The workspace appears twice. This happens if you sourced things multiple times.

**Solution**: Fresh terminal, source once:

```bash
# New terminal
source ~/ros2_ws/install/setup.bash  # Only once
```

### Diagnosis: Symbolic Link Issues

If you built with `--symlink-install` but moved or renamed source files, the symlinks are broken.

Check if symlinks are valid:

```bash
$ ls -la ~/ros2_ws/install/my_package/share/my_package/launch/
lrwxrwxrwx 1 user user 64 May 20 10:30 my_launch.py -> /old/path/that/no/longer/exists  # ← Broken!
```

**Solution**: Rebuild after moving/renaming files:

```bash
cd ~/ros2_ws
colcon build --packages-select my_package --symlink-install
```

### Diagnosis: Launch File Uses Explicit Paths

Your launch file might have hard-coded paths:

```python
# Bad: Hard-coded path
config_file = '/opt/ros/jazzy/share/nav2_bringup/config/nav2_params.yaml'

# Good: Uses package search
pkg_dir = get_package_share_directory('nav2_bringup')
config_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
```

**Solution**: Use `get_package_share_directory()` instead of hard-coded paths.

## Symptom: "Package found, but imports fail"

Your Python node starts but crashes with import errors:

```python
ImportError: cannot import name 'SomeClass' from 'my_package.my_module'
```

### Diagnosis: Did You Add a New Python Module?

If you added a new `.py` file to your package, Python needs to know about it.

For Python packages with `setup.py`, check that your package discovery is correct:

```python
# setup.py
from setuptools import setup

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    packages=[package_name],  # ← Must list all package directories
    install_requires=['setuptools'],
    # ...
)
```

If you have submodules, list them:

```python
packages=[package_name, f'{package_name}.submodule'],
```

Or use automatic discovery:

```python
from setuptools import find_packages

setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    # ...
)
```

**Solution**: Update `setup.py` and rebuild:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_bringup
```

### Diagnosis: Python Path Issues

Check if your package is in the Python path:

```bash
$ python3 -c "import sys; print('\n'.join(sys.path))"
```

You should see something like `/home/user/ros2_ws/install/my_robot_bringup/lib/python3.10/site-packages`.

If it's not there, your workspace isn't sourced properly in the current context.

**Solution**: Source workspace and try again.

### Diagnosis: Circular Imports

If your Python modules import each other in a circle, you'll get import errors.

Check your import structure:
- `module_a.py` imports from `module_b.py`
- `module_b.py` imports from `module_a.py`

**Solution**: Restructure your code to break the circular dependency, or use lazy imports (import inside functions instead of at module level).

## Symptom: "ros2 pkg list doesn't show my package"

Your package builds successfully, but doesn't appear in `ros2 pkg list`.

### Diagnosis: Is package.xml Correct?

The package must have a valid `package.xml` for ROS to recognize it:

```bash
$ cat ~/ros2_ws/src/my_package/package.xml
```

Check:
- File exists
- XML is valid (no syntax errors)
- Has required fields: `<name>`, `<version>`, `<description>`, `<maintainer>`, `<license>`

**Solution**: Fix `package.xml`, rebuild:

```bash
cd ~/ros2_ws
colcon build --packages-select my_package
```

### Diagnosis: Is It Installed?

The package source exists, but did it install?

```bash
$ ls ~/ros2_ws/install/my_package
```

If this directory doesn't exist, the package didn't install during build.

Check build output for errors. For Python packages, check `setup.py` has proper `install_requires` and `data_files` configuration.

### Diagnosis: Wrong Build Type

If `package.xml` says `ament_python` but you have a `CMakeLists.txt`, or vice versa, the build might skip the package.

**Solution**: Ensure `package.xml` and build files match:

For Python packages:
- `<build_type>ament_python</build_type>` in `package.xml`
- Have `setup.py` and `setup.cfg`
- NO `CMakeLists.txt` (or only a minimal one for colcon)

For C++ packages:
- `<build_type>ament_cmake</build_type>` in `package.xml`
- Have `CMakeLists.txt`
- NO `setup.py`

## Symptom: "Deleted package still found by ROS"

You deleted a package from `src/` but ROS still finds it.

### Diagnosis: Package Still in install/

```bash
$ rm -rf ~/ros2_ws/src/old_package  # Delete source
$ colcon build  # Rebuild
$ ros2 pkg list | grep old_package
old_package  # ← Still there!
```

`colcon build` only builds what's in `src/`, it doesn't delete anything from `install/`.

**Solution**: Manually remove from install:

```bash
rm -rf ~/ros2_ws/install/old_package
rm -rf ~/ros2_ws/build/old_package
```

Or nuclear option:

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

## Symptom: "ROS finds package in unexpected location"

You have the package in multiple places and don't know which one is being used.

### Diagnosis: Find All Instances

```bash
$ ros2 pkg prefix --all nav2_bringup
/opt/ros/jazzy
/home/user/ros2_ws/install
/home/user/experimental_ws/install
```

The `--all` flag shows every instance found in `AMENT_PREFIX_PATH`.

Without `--all`, it shows the first match (the one that will be used):

```bash
$ ros2 pkg prefix nav2_bringup
/opt/ros/jazzy  # ← This is what's being used
```

**Solution**: Fix search order so the version you want is first.

### Diagnosis: Check Actual Search Path

```bash
$ echo $AMENT_PREFIX_PATH | tr ':' '\n'
/opt/ros/jazzy
/home/user/ros2_ws/install
/home/user/experimental_ws/install
```

Each path is searched in order. The first match wins.

**Solution**: Rebuild with correct underlay to get desired search order.

## Symptom: "Source command changes nothing"

You run `source ~/ros2_ws/install/setup.bash` but `$AMENT_PREFIX_PATH` doesn't change.

### Diagnosis: Wrong File

Did you source the right file?

```bash
$ source ~/ros2_ws/setup.bash  # ← WRONG! There's no setup.bash in workspace root
bash: ~/ros2_ws/setup.bash: No such file or directory
```

The setup files are in `install/`:

```bash
$ source ~/ros2_ws/install/setup.bash  # ← Correct
```

### Diagnosis: Build Failed

If colcon build failed, `install/setup.bash` might not have been generated.

Check if it exists:

```bash
$ ls ~/ros2_ws/install/setup.bash
ls: cannot access: No such file or directory
```

**Solution**: Fix build errors, rebuild:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### Diagnosis: Script vs. Source

Did you run it instead of sourcing it?

```bash
$ ~/ros2_ws/install/setup.bash  # ← WRONG! This runs in a subshell
$ ./setup.bash  # ← Also wrong

$ source ~/ros2_ws/install/setup.bash  # ← Correct
$ . ~/ros2_ws/install/setup.bash  # ← Also correct (. is short for source)
```

Running the script in a subshell changes that subshell's environment, which exits immediately. Your terminal's environment is unchanged.

**Solution**: Always use `source` or `.` before setup files.

## Symptom: "bashrc auto-source causes problems"

You added `source ~/ros2_ws/install/setup.bash` to `~/.bashrc` for convenience, but now everything is broken.

### Diagnosis: Sourcing Before Build Complete

If `~/.bashrc` sources the workspace, but the workspace isn't built yet, every new terminal will show errors:

```bash
# .bashrc
source ~/ros2_ws/install/setup.bash

# New terminal
bash: ~/ros2_ws/install/setup.bash: No such file or directory
```

**Solution**: Add a check:

```bash
# .bashrc
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi
```

### Diagnosis: Conflicts with Multiple Workspaces

If you work with multiple workspaces, auto-sourcing one in `.bashrc` means you can't easily use others.

```bash
# .bashrc
source ~/robot_ws/install/setup.bash  # ← Always sources this

# New terminal - want to use experimental_ws instead
source ~/experimental_ws/install/setup.bash
echo $AMENT_PREFIX_PATH
/home/user/experimental_ws/install:/home/user/robot_ws/install:...  # ← Both are sourced!
```

**Solution**: Don't auto-source in `.bashrc`. Instead, use shell aliases:

```bash
# .bashrc  
alias setup-robot='source ~/robot_ws/install/setup.bash'
alias setup-exp='source ~/experimental_ws/install/setup.bash'
```

Then in a new terminal:

```bash
$ setup-exp  # Choose which workspace for this terminal
```

### Diagnosis: .bashrc Sources Wrong Order

If `.bashrc` sources `/opt/ros/jazzy` after your workspace, search order breaks:

```bash
# .bashrc - WRONG ORDER
source ~/ros2_ws/install/setup.bash
source /opt/ros/jazzy/setup.bash  # ← This puts jazzy first!
```

**Solution**: Only source your workspace (it automatically sources its underlay):

```bash
# .bashrc - CORRECT
source ~/ros2_ws/install/setup.bash
# Don't source /opt/ros/jazzy again - the workspace setup does it
```

Or just source jazzy and manually source workspace when needed:

```bash
# .bashrc - SAFE DEFAULT
source /opt/ros/jazzy/setup.bash
# Then manually source workspace in specific terminals
```

<details>
<summary><strong>Deep Dive: .bashrc Best Practices for ROS</strong></summary>

Different developers have different `.bashrc` strategies:

**Strategy 1: Auto-source ROS only**
```bash
# .bashrc
source /opt/ros/jazzy/setup.bash
```
- Pros: Every terminal has ROS available
- Pros: Can manually source any workspace
- Cons: Must remember to source workspace each time

**Strategy 2: Auto-source primary workspace**
```bash
# .bashrc
if [ -f ~/robot_ws/install/setup.bash ]; then
    source ~/robot_ws/install/setup.bash
fi
```
- Pros: Primary workspace always ready
- Cons: Hard to use other workspaces
- Cons: Can't build primary workspace in these terminals (circular sourcing)

**Strategy 3: Use aliases**
```bash
# .bashrc
source /opt/ros/jazzy/setup.bash
alias sws='source ~/robot_ws/install/setup.bash'
alias swe='source ~/experimental_ws/install/setup.bash'
```
- Pros: Easy to choose which workspace
- Cons: Must type alias every time

**Strategy 4: Different profiles**
```bash
# .bashrc
source /opt/ros/jazzy/setup.bash

# .bashrc.robot (separate file)
source ~/robot_ws/install/setup.bash

# .bashrc.exp
source ~/experimental_ws/install/setup.bash
```

Then in terminal:
```bash
$ bash --rcfile ~/.bashrc.robot  # Launch terminal with robot workspace
```

**Recommendation**: Start with Strategy 1 (ROS only), add aliases as needed. Keeps things explicit and debuggable.

</details>

## Diagnostic Checklist: When Nothing Makes Sense

If you're completely lost, work through this checklist systematically:

### 1. Verify Build Success

```bash
cd ~/ros2_ws
colcon build --packages-select my_package
# Look for "Finished <<<" not "Failed <<<"
```

### 2. Verify Install Artifacts

```bash
ls ~/ros2_ws/install/my_package
# Should exist and have share/, lib/, etc.
```

### 3. Verify Workspace Sourced

```bash
echo $AMENT_PREFIX_PATH
# Should include your workspace path
```

### 4. Verify Search Order

```bash
echo $AMENT_PREFIX_PATH | tr ':' '\n' | nl
# Your workspace should be line 1
```

### 5. Verify Package Found

```bash
ros2 pkg list | grep my_package
# Should appear
```

### 6. Verify Package Location

```bash
ros2 pkg prefix my_package
# Should point to your workspace, not /opt/ros
```

### 7. Verify Launch File Location

```bash
ros2 pkg prefix my_package
# Then ls <that_path>/share/my_package/launch/
# Verify launch file is there
```

### 8. Check for Duplicate Packages

```bash
ros2 pkg prefix --all my_package
# Should only show expected locations
```

### 9. Fresh Terminal Test

```bash
# Open brand new terminal
source ~/ros2_ws/install/setup.bash
ros2 launch my_package my_launch.py
# Does it work now?
```

### 10. Nuclear Rebuild

```bash
cd ~/ros2_ws
rm -rf build install log
source /opt/ros/jazzy/setup.bash  # Fresh underlay
colcon build --symlink-install
# Fresh terminal
source ~/ros2_ws/install/setup.bash
ros2 launch my_package my_launch.py
```

If it still doesn't work after this checklist, the problem is likely:
- Code bug (not workspace issue)
- Missing system dependency
- Permission problem
- Filesystem corruption (rare)

## Common Gotchas Reference

Quick reference of things that trip people up:

| Gotcha | Why It Happens | Fix |
|--------|----------------|-----|
| "Package not found" right after build | Forgot to source | `source install/setup.bash` |
| Changes don't take effect | Running wrong version | Check `ros2 pkg prefix`, fix search order |
| Package found in wrong location | Sourced things in wrong order | Fresh terminal, source correctly |
| Deleted package still found | Not removed from install/ | `rm -rf install/old_package` |
| Source command does nothing | Ran script instead of sourcing | Use `source` not `./` |
| Build succeeds but package missing | Build error ignored | Check build summary for failures |
| Workspace in PATH multiple times | Sourced multiple times | Fresh terminal, source once |
| .bashrc breaks new terminals | Setup file doesn't exist | Add `if [ -f ... ]` check |
| Python imports fail | Forgot to rebuild after adding module | `colcon build --packages-select` |
| Symlinks broken | Moved source files | Rebuild with `--symlink-install` |

## Where to Go From Here

**Need to understand the fundamentals?** → [Understanding the Workspace Chain](understanding_the_workspace_chain.md)  
*If troubleshooting revealed gaps in your understanding of how workspace search works, go back to the fundamentals chapter for a solid mental model.*

**Working with multiple workspaces?** → [Managing Multiple Workspaces](managing_multiple_workspaces.md)  
*If you're juggling multiple custom workspaces and need advanced techniques for managing them, this chapter covers complex scenarios and best practices.*

**Launch files giving you trouble?** → [Launch Files](launch_files.md)  
*Many "package not found" errors actually come from launch files searching incorrectly for packages or configuration files. Learn how launch files interact with the workspace chain.*

**Package structure questions?** → [Creating Your First Workspace and Package](creating_your_first_workspace_and_package.md)  
*If your package isn't building correctly, review the basics of package structure, `package.xml`, and build configuration.*

---

*Troubleshooting is part of development. Build up your diagnostic skills with these patterns, and workspace problems will become quick fixes instead of hour-long debugging sessions.*
