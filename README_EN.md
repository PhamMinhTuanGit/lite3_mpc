[简体中文](./README.md)

[![Discord](https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white)](https://discord.gg/gdM9mQutC8)
Please go through the whole process on a Ubuntu system.
## Tutorial Videos
We've released the following tutorials for training and deploying a reinforcement learning policy. Please check it out on [Bilibili](https://b23.tv/UoIqsFn) or [YouTube](https://youtube.com/playlist?list=PLy9YHJvMnjO0X4tx_NTWugTUMJXUrOgFH&si=pjUGF5PbFf3tGLFz)!

## Sim-to-Sim

```bash
# segmentation debug tools install
sudo apt-get install libdw-dev
wget https://raw.githubusercontent.com/bombela/backward-cpp/master/backward.hpp
sudo mv backward.hpp /usr/include

# Dependency install (python3.10)
pip install pybullet "numpy < 2.0" mujoco
git clone --recurse-submodule https://github.com/DeepRoboticsLab/Lite3_cmpc_deploy.git

# compile
mkdir build && cd build
cmake .. -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DSEND_REMOTE=OFF

# Explanation
# -DBUILD_PLATFORM：device platform，Ubuntu is x86，quadruped is arm
# -DBUILD_SIM：whether or not to use simulatior, if deployed on real robots, set to OFF
make -j
```

```bash
# run (open 2 terminals)
# Terminal 1 (pybullet)
cd interface/robot/simulation
python pybullet_simulation.py

# Terminal 1 (mujoco)
cd interface/robot/simulation
python mujoco_simulation.py

# Terminal 2
cd build
./cmpc_deploy
```

## Usage(Terminal 2)

tips：right click simulator window and select "always on top"

- z： default position
- x： cmpc control default position
- wasd：forward/leftward/backward/rightward
- qe：clockwise/counter clockwise
