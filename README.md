

## install

clone PX4 v1.15.4 or newest release branch

```
mkdir PX4-Autopilot/Tools/simulation/mujoco
cd PX4-Autopilot/Tools/simulation/mujoco
git clone mujoco_bridge
```


needs QGroundControl running, presumably.

Install Mujoco, but may need to disable some LTO bullshit; can modify the 
mujoco/cmake/MujocoLinkOptions.cmake to be like the one in the `copy_files`
directory (ie remove all the LTO stuff). But otherwise install mujoco to the system
so the PX4 build process can find it.


when you run `make px4_sitl mujoco_skydio_x2` under PX4-Autopilot, it will
build the mujoco bridge code but not run it; it only launches the px4 software.

then in the mujoco_bridge directory, you can 

```
mkdir build
cd build
cmake ..
make -j4
bin/simulate ../models/skydio_x2/scene.xml INADDR_ANY 4560
```

to build the mujoco bridge that launches a simulator and sends data over mavlink
to the px4 software.


## copy files

cp 1100_mujoco_x500 PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
cp 1101_mujoco_skydio_x2 PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
also need to modify PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
to include those two mujoco airframes

cp sitl_run.sh PX4-Autopilot/Tools/simulation/mujoco

cp sitl_targets_mujoco.cmake PX4-Autopilot/src/modules/simulation/simulator_mavlink/
also need to modify PX4-Autopilot/src/modules/simulation/simulator_mavlink/CMakeLists.txt
to now include the sitl_targets_mujoco.cmake file.

## run

make px4_sitl mujoco

most likely the settings for px4_sitl or px4_sitl_default need to change or be
better understood to know what information Mujoco needs to be sending over.
For instance, is magentometer / barometer required? How many? IMU update rate?
Source of positional data (ie GPS? ground truth? optical flow?)? What is the 
correct frame or reference to be used?