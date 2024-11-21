# URDFS or Uniform Resource Definition Files

## Setup
If you haven't done so yet, you need to create a ROS 2 workspace to hold the book code.

```bash
# Do the following if you haven't already created a workspace
mkdir -p ~/wr_book_ws/src && cd ~/wr_book_ws/src # Change this if you need a different name or location
git clone git@github.com:wimblerobotics/Robotics_Book.git
cd .. # Should be back at ~/wr_book_ws
colcon build --symlink-install
source install/setup.bash
```