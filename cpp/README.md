# C++ codes

[中文版说明](README_zh.md)

It is a simulation program for ice halo. It is fast and efficient.

## Quick start

After cloning, you can run the build script to build and install it.

~~~bash
cd ice_halo_sim/cpp
./build.sh -rj release
~~~

If everything goes well, the executable will be installed into `cpp/build/cmake_install`. And
you can run it like:

~~~bash
./build/cmake_install/IceHaloV3 -f v3_config_example.json
~~~

It will output some information, as well as several rendered picture files.

If you are interested in more details, just go ahead to following sections.


## Getting start

### Build project

This project is built with [CMake](https://cmake.org/). And it denpends on [OpenCV](https://opencv.org/) and [boost](https://www.boost.org/). Make sure they are installed before you start to build. (In fact they are not critical for core functions. I'm planning remove the dependencies).

I put a build script to make things simpler.
With `-h` you will see help message:

~~~bash
./build.sh -h
Usage:
  ./build.sh [-tjkrh1] <debug|release|minsizerel>
    Executables will be installed at build/cmake_install
OPTIONS:
  -t:          Build test cases and run test on them.
  -j:          Build in parallel, i.e. use make -j option.
  -k:          Clean temporary building files.
  -b:          Run a benchmarking. It tells how fast the program runs on your computer.
  -v:          Enable verbose log.
  -r:          Use random seed for random number generator. Without this option,
               the program will use a constant value. Thus generate a repeatable result
               (usually together with -1).
  -1:          Use single thread.
  -h:          Show this message.
~~~

Note that debug version executables will not be installed, so they will be found in `build/cmake_build`.

I use the [GoogleTest](https://github.com/google/googletest) framework for my unit tests.
If `-t` option is set, the test cases will be built and test on.
It is usefull in a CI/CD pipeline.


## Configuration file

Configuration file contains all configurations. It written in JSON format.
I use [nlohmann's json](https://github.com/nlohmann/json) to parse JSON file.

I put a example configuration file `v3_config_example.json`.

### Light source

Here is an example for one element:

~~~json
"id": 2,
"type": "sun",
"altitude": 20.0,
"azimuth": 0,
"diameter": 0.5,
"wavelength": [ 420, 460, 500, 540, 580, 620 ],
"wl_weight": [ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ]
~~~

A `light_source` section describes properties of light source. It may contain multiple elements corrensponding to multiple light sources. They are referenced by `id`.
ID should be a unique number greater than 0. It is not necessary to keep IDs increasing one by one.

Fields `azimuth`, `altitude` describe position of the sun. They are in degrees, and so is `diameter`.

`wavelength` and `wl_weight` descibe spectrum of the light source.
They are arrays containing all wavelengths you want to use in a simulation. Wavelength determines refractive index, whose data is from
[Refractive Index of Crystals](https://refractiveindex.info/?shelf=3d&book=crystals&page=ice).


### Crystal

Here is an example for one element:

~~~json
"id": 3,
"type": "prism",
"shape": {
  "height": 1.3,
  "face_distance": [1, 1, 1, 1, 1, 1]
},
"axis": {
  "zenith": {
    "type": "gauss",
    "mean": 90,
    "std": 0.3
  },
  "roll": {
    "type": "uniform",
    "mean": 0,
    "std": 360
  },
  "azimuth": {
    "type": "uniform",
    "mean": 0,
    "std": 360
  }
}
~~~

A `crystal` section stores all crystals used in simulation. It may contain multiple elements (different crystals). They are referenced by `id`.

`zenith`, `roll` and `azimuth` (optional):
These fields defines the pose of crystals. `zenith` defines the c-axis orientation, and `roll`
defines the rotation around c-axis.
They are of *distribution type*, which can be a scalar, indicating a deterministic distribution, or can be a tuple of (`type`, `mean`, `std`) discribing a uniform or Gaussian distribution. All angles are in degrees.

`type` and `shape`: they describe the shape of a crystal.
Currently there are 2 kind of crystals, `HexPrism`, `HexPyramid`.
Each type has its own shape parameters.

  * `HexPrism`:
  parameter `height`, defines `h / a` where `h` is the prism height, `a` is the diameter along
  a-axis (also x-axis in my program). It is of *distribution type*.
  And `face_distance` describes an irregular hexangonal face (will describe later).  
  <img src="doc/figs/hex_prism_01.png" width="400">.

  * `HexPyramid`:
  `{upper|lower|prism}_h` describe heights of each segment, see picture below. `{upper|lower}_h` represent `h1 / H1` and `h3 / H3` respectly, where
  `H1` means the max possible height for upper pyramid segment, and `H3` the same but for lower pyramid segment. `prism_h` has the same meaning as for `HexPrism`.  
      <img src="doc/figs/hex_pyramid_01.png" width="400">.  
  `{upper|lower}_indices` are 
  [Miller index](https://en.wikipedia.org/wiki/Miller_index) describing the
  pyramidal face orientation.
  For example, `[a, b]`, means a face with Miller index of `(a, 0, -a, b)`. For a
  typical ice crystal face (face number 13), its Miller index is `(1, 0, -1, 1)`.  
  It may also have a `face_distance` parameter.

  * `face_distance`:
  The distance here means the ratio of actual face distance
  to a regular haxegon distance. A regular haxegon has distance of `[1, 1, 1, 1, 1, 1]`.
  The following figure shows an irregular hexegon with distance of `[1.1, 0.9, 1.5, 0.9, 1.7, 1.2]`  
  <img src="doc/figs/irr_hex_01.png" width="400">.

### Filter

Here are two common examples:

~~~json
[
  {
    "id": 3,
    "type": "raypath",
    "raypath": [3, 5],
    "symmetry": "P"
  },
  {
    "id": 4,
    "type": "entry_exit",
    "entry": 3,
    "exit": 5,
    "action": "filter_in"
  }
]
~~~

`type`: can be one of these types: `raypath`, `entry_exit`, `direction`, `crystal`.

### Scene

Here is an example:

~~~json
"id": 3,
"light_source": 2,
"ray_num": 1000000,
"max_hits": 7,
"scattering": [
  {
    "crystal": [1, 2, 3],
    "prob": 0.2
  },
  {
    "crystal": [2, 3],
    "proportion": [20, 100],
    "filter": [2, 1]
  }
]
~~~

### Render

Here is an example:

~~~json
"id": 3,
"lens": {
  "type": "linear",
  "fov": 40
},
"resolution": [1920, 1080],
"view": {
  "azimuth": -50,
  "elevation": 30,
  "roll": 0,
  "distance": 8
},
"visible": "upper",
"background": [0, 0, 0],
"ray": [1, 1, 1],
"opacity": 0.8,
"grid": {
  "central": [
    {
      "value": 22,
      "color": [1, 1, 1],
      "opacity": 0.4,
      "width": 1.2
    }
  ],
  "elevation": [],
  "outline": true
}
~~~

`view`: describes camera pose.

`lens`: lens type, can be one of these values: `fisheye`, `linear`, `dual_fisheye_equidistant`, `dual_fisheye_equiarea`, `rectangular`.

### Project

Nothing complicated. It just keep references to scene and render.


## TODO list

* Write a (web) GUI for these code.
