A simple rigid body dynamics simulator based on microblx, for the UR5 robot.

This repository may serve as a starting point to develop a more generic
simulation/control system for articulated robots, based on the microblx
framework.

# Setup

## Dependencies

### Core

- [microblx](https://github.com/kmarkus/microblx), the framework with which
  the application in this repository is built.

- [Eigen](http://eigen.tuxfamily.org/) and the
  [iit-rbd](https://bitbucket.org/robcogenteam/cpp-iitrbd/src/master/)
  library. These are required to build the C++ dynamics engine of the robot,
  which is included in this repository.

- [odeint](https://headmyshoulder.github.io/odeint-v2/index.html), which is
  used for the integration of the forward dynamics of the robot.

### Visualization

- [meshcat](https://github.com/rdeits/meshcat-python), the application providing
  the 3D visualization of the robot meshes.

- [msgpack C++](https://github.com/msgpack/msgpack-c/tree/cpp_master), used to
  serialize the commands for Meshcat

- The [robot-model-tools](https://github.com/mfrigerio17/robot-model-tools),
  required to load the robot model into Meshcat.  
  Please **switch** to the branch `meshcat-viewer`.


## Installation
1. After cloning this repository, use `cmake` to build and install the
   `libmeshcatcpp` included with the robot-model-tools
   (in `etc/libmeshcat`).  
   This library enables communication (via ZMQ) between a C++ client and the
   Meshcat application.

2. Use `cmake` to build and install the UR5 dynamics library, whose sources are
   under `rcgen/cpp/`. This is C++ code that was generated using
   [RobCoGen](https://robcogenteam.bitbucket.io/)

3. Install microblx according to its [documentation](https://microblx.readthedocs.io/en/latest/installing.html#building-from-source)

4. Install the microblx blocks provided in this repository:

    ```
    $ cd microblx/
	$ ./bootstrap
	$ mkdir build && cd build
	$ ../configure
	$ make -j8
	$ sudo make install
    ```

5. Download [here](https://kuleuven.box.com/s/hdw2gpa4s0q4kct1ok0rsy8vjyj3ovu1)
   the meshes for the UR5 robot, and copy them in the `meshes/` subdirectory.

## Running the program
1. From the root, please execute `./meshcat.sh`. The robot-model-tools and
   Meshcat itself must be in the current Python path (you may want to use a
   virtual environment).  
   Open the browser at the specified URL. You should be able to see the robot.

2. Launch the actual microblx application

     ```
     cd microblx/
     ubx-launch -c ur5-sim.usc,ur5-sim__mix-ptrig.usc
     ```

    The robot should now start to move.



