# unitair
Unified single-board GNC-stack for UAVs. Check out [the wiki](https://github.com/Unitair-Autopilot/unitair/wiki) for more details.

### Dependencies
- **Firmware**
  - `sudo apt install libi2c-dev`
  - `pip install --user inputs` (optional)
  - [sbus_serial](https://github.com/jenswilly/sbus_serial) (optional)
  - [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver) (optional)
- **Middleware**
  - [ROS](http://www.ros.org/) version Noetic or Melodic
  - On Ubuntu >= 20 you'll need `sudo apt install python-is-python3`
- **Software**
  - `sudo apt install libeigen3-dev libyaml-cpp-dev`
  - `pip install --user numpy scipy matplotlib pyyaml`
- **Documentation**
  - `sudo apt install doxygen` (optional)
- **Simulator**
  - [Stokesim](https://github.com/Unitair-Autopilot/stokesim) (optional)

### Build
Once the dependencies have been successfully installed, clone Unitair into your initialized catkin-workspace `src` directory, run `catkin build` (or `catkin_make` if you prefer), and then `source` your `devel/setup.bash` script (which should also happen in your `~/.bashrc` script). Note that on some processors you may have to set `-DFAST_MATH=OFF` for sufficient accuracy in some calculations. Once built, you can test numerics with `rosrun unitair tests.sh CFG_PATH` where `CFG_PATH` is the path to the parameter files you want to use (for example `unitair/cfg/fixedwing/`).

### Testing in Simulation
Using the example `cfg` named "multirotor":
```
roslaunch unitair simulation.launch cfg:=multirotor
```
```
rosservice call /mission/setpath "
speed: 3.0
timestep: 0.0
times: [0]
path: [{x: 0, y: 0, z: 1}, {x: 0, y: 4, z: 3}, {x: -4, y: 0, z: 3}, {x: 0, y: -5, z: 2}, {x: 4, y: 0, z: 3}, {x: 0, y: 0, z: 1}]
loop_index: -1"
```
Using the example `cfg` named "fixedwing":
```
roslaunch unitair simulation.launch cfg:=fixedwing
```
```
rosservice call /mission/setpath "
speed: 18.0
timestep: 0.0
times: [0]
path: [{x: 120.0, y: 0.0, z: 20.0}, {x: 120.0, y: 120.0, z: 20.0}, {x: 120.0, y: 0.0, z: 20.0}, {x: 0.0, y: 0.0, z: 20.0}]
loop_index: 0"
```

### Disclosure Notice

DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
This material is based upon work supported by the Department of the Air Force under Air Force Contract No. FA8702-15-D-0001 or FA8702-25-D-B002. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Department of the Air Force.
© 2024 Massachusetts Institute of Technology.

Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)
The software/firmware is provided to you on an As-Is basis
Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
