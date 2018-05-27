# C++ version

[中文版](README_zh.md)

Besides matlab codes, I start a C++ project for higher performance. Currently the C++ version is just 
pieces of toy codes and can only run from command, no GUI.

With integration of [Halide](http://halide-lang.org/) I can accelarate these codes by parallelism.

## Build and run

This C++ project is built with [CMake](https://cmake.org/). A simple way to build form start is as follows:

1. `cd cpp`
2. `mkdir build && cd build`
3. `cmake .. && make -j4`, or you can set `CMAKE_BUILD_TYPE` to `release` to get highest performance.

Then the executable binary will be at `build/bin`. And you can start by 
`./bin/IceHaloSim <path-to-your-config-file>`. The file [`cpp/config.json`](./config.json) is an example configuration file.

## Configuration file

It is file containing all configurations. It uses JSON format. It must containts `sun`, `ray_number`,
`max_recursion`, `crystal` fields.

### Basic infomation for simulation

* `sun`:
It only contains one attribute, `altitude`, defining the altitude of the sun.

* `ray_number`:
The total ray number for simulation. Note that even with a single ray input, it may result in multiple
rays output, due to reflections and refractions in crystal. This `ray_number` defines the input ray number,
but not output ray number.

* `max_recursion`:
It defines the max number that a ray hits a surface during a simulation. If a ray hits more than this number
and still doesn't leave the crystal, it will be dropped.

### Crystal settings

* `type` and `parameter`:
Currently I create 5 shapes, `HexCylinder`, `HexPyramid`, `HexPyramidStackHalf`, `TriPyramid`, `CubicPyramid`.
Each shape has its own shape parameters.

  * `HexCylinder`:
  Only 1 parameter, defines `h / a` where `h` is the cylinder height, `a` is the diameter along
  a-axis.  
  <img src="figs/hex_cylinder_01.png" width="400">
  
  * `HexPyramid`:
  May have 3, 5, or 7 parameters.
  * `HexPyramidStackHalf`:
  7 parameters.
  * `TriPyramid`:
  5 parameters.
  * `CubicPyramid`:
  2 parameters.

* `axis` and `roll`:
These two fields defines the orientation of crystals. `axis` defines the c-axis orientation, and `roll`
defines the rotation around c-axis.

  These fields all has three attributes, `mean`, `std`, `type`. `type` defines the random distribution
type, either `Gauss`, for Gaussian distribution, or `Uniform`, for uniform distribution. `mean` defines
the mean of the random distribution. For `axis`, it means the zenith angle. 
`std` defines the deviation of the distribution. For Gaussian distribution,
it is the standard deviation, and for uniform distribution, it defines the value range.

  All angles are in degrees.

* `population`:
It defines how many crystals used in a simulation. Note that it is not the actual number, just for a
ratio. So if one crystal set to 2.0 and the other set to 3.0, it is equivalent to set one to 20 and
the other to 30.

## TODO list

* Use OpenCL / OpenGL / CUDA to accelerate. Since I've seen good enough performance with integration of
  Halide, I doubt the margin to more acceleration.
* Write a GUI for these code.
