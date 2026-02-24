# C++ codes

[中文版说明](README_zh.md)

A simulation program for ice halo phenomena. It traces light rays interacting with ice crystals to reproduce various halo patterns. Fast and efficient, supporting natural color rendering, multiple scattering, and custom crystal models (.obj format).

## Quick start

After cloning, you can run the build script to build and install it.

~~~bash
cd cpp
./build.sh -j release
~~~

If everything goes well, the executable will be installed into `build/cmake_install`. And
you can run it like:

~~~bash
./build/cmake_install/Lumice -f config_example.json
~~~

It will output some information, as well as several rendered picture files.

If you are interested in more details, just go ahead to following sections.


## Getting started

### Prerequisites

- **CMake** >= 3.14
- **Ninja** (recommended, used as default build generator; install via `brew install ninja` on macOS or `apt install ninja-build` on Ubuntu)
- **C++17** compatible compiler (GCC, Clang, or MSVC)

All other dependencies are automatically downloaded and managed via [CPM.cmake](https://github.com/cpm-cmake/CPM.cmake):
- [nlohmann/json](https://github.com/nlohmann/json) v3.10.5 — JSON parsing (header-only)
- [spdlog](https://github.com/gabime/spdlog) v1.15.0 — Logging (header-only)
- [tl-expected](https://github.com/TartanLlama/expected) v1.1.0 — `expected<T,E>` for C++17 (header-only)
- [GoogleTest](https://github.com/google/googletest) v1.15.2 — Unit testing (downloaded when `-t` is enabled)

> **Note on Ninja**: If Ninja is not installed, you can remove `-G Ninja` from `build.sh` to fall back to the system default generator (usually Unix Makefiles).

### Build project

A build script is provided to simplify the process.
With `-h` you will see help message:

~~~bash
./build.sh -h
Usage:
  ./build.sh [-tjksh] <debug|release|minsizerel>
    Executables will be installed at build/cmake_install
OPTIONS:
  -t:          Build test cases and run test on them.
  -j:          Build in parallel, i.e. use make -j
  -k:          Clean temporary building files.
  -s:          Build shared library (default: static).
  -h:          Show this message.
~~~

Note that debug version executables will not be installed, so they will be found in `build/cmake_build`.

[GoogleTest](https://github.com/google/googletest) is used for unit tests.
If `-t` option is set, the test cases will be built and run.


## Configuration file

Configuration file contains all settings for a simulation. It is written in JSON format,
parsed with [nlohmann/json](https://github.com/nlohmann/json).

An example configuration file is provided: `config_example.json`.

For the complete configuration reference, see [Configuration Guide](doc/configuration.md).
Below is a brief overview of each section.

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

A `light_source` section describes properties of light source. It may contain multiple elements corresponding to multiple light sources. They are referenced by `id`.
ID should be a unique number greater than 0. It is not necessary to keep IDs increasing one by one.

Fields `azimuth`, `altitude` describe position of the sun. They are in degrees, and so is `diameter`.

`wavelength` and `wl_weight` describe spectrum of the light source.
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
These fields define the pose of crystals. `zenith` defines the c-axis orientation, and `roll`
defines the rotation around c-axis.
They are of *distribution type*, which can be a scalar, indicating a deterministic distribution, or can be a tuple of (`type`, `mean`, `std`) describing a uniform or Gaussian distribution. All angles are in degrees.

`type` and `shape`: they describe the shape of a crystal.
Currently there are 2 kinds of crystals, `prism` and `pyramid`.
Each type has its own shape parameters.

  * `prism` (hexagonal prism):
  Parameter `height` defines `h / a` where `h` is the prism height, `a` is the diameter along
  a-axis (also x-axis in the program). It is of *distribution type*. Default: `1.0`.
  `face_distance` describes an irregular hexagonal face (see below). Default: `[1, 1, 1, 1, 1, 1]`.
  <img src="doc/figs/hex_prism_01.png" width="400">.

  * `pyramid` (hexagonal pyramid):
  `{upper|lower|prism}_h` describe heights of each segment, see picture below. `{upper|lower}_h` represent `h1 / H1` and `h3 / H3` respectively, where
  `H1` means the max possible height for upper pyramid segment, and `H3` the same but for lower pyramid segment. `prism_h` has the same meaning as for `prism`.
      <img src="doc/figs/hex_pyramid_01.png" width="400">.
  `{upper|lower}_indices` are
  [Miller index](https://en.wikipedia.org/wiki/Miller_index) describing the
  pyramidal face orientation. Default: `[1, 0, 1]`.

  * `face_distance`:
  The distance here means the ratio of actual face distance
  to a regular hexagon distance. A regular hexagon has distance of `[1, 1, 1, 1, 1, 1]`.
  The following figure shows an irregular hexagon with distance of `[1.1, 0.9, 1.5, 0.9, 1.7, 1.2]`
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

`type`: can be one of these types: `raypath`, `entry_exit`, `direction`, `crystal`, `complex`, `none`.

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

`lens`: lens type, can be one of these values: `linear`, `fisheye_equal_area`, `fisheye_equidistant`, `fisheye_stereographic`, `dual_fisheye_equal_area`, `dual_fisheye_equidistant`, `dual_fisheye_stereographic`, `rectangular`.

You can use `fov` (field of view in degrees) or `f` (focal length in mm) to specify the lens. If `f` is used, the program automatically calculates the corresponding `fov`.

### Project

Nothing complicated. It just keeps references to scene and render.


## Documentation

For detailed documentation, please refer to:
- [Documentation Index](doc/README.md) - Navigation and index of all documents
- [Configuration Guide](doc/configuration.md) - Complete configuration reference
- [Architecture Document](doc/architecture.md) - System architecture
- [Developer Guide](doc/developer-guide.md) - Developer guide
- [C API Documentation](doc/c_api.md) - C interface usage
- [API Documentation](doc/api/html/) - Auto-generated API docs (generate locally with `doxygen .doxygen-config`)
