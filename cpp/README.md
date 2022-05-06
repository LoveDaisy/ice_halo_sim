# C++ codes

[中文版说明](README_zh.md)

It is a simulation program for ice halo. It is fast and efficient.

## Quick start

After cloning, you can just run the build script to build and install it.

~~~bash
cd ice_halo_sim/cpp
./build.sh -rj release
~~~

If everything goes well, the executable will be installed into `cpp/build/cmake_install`. And
just run it like:

~~~bash
./build/cmake_install/IceHaloV3 -f v3_config_example.json
~~~

It will output some information, as well as several rendered picture files.

If you are interested in more details, just go ahead for following sections.


## Getting start

### Build project

This project is built with [CMake](https://cmake.org/).
I put a build script to make things simpler.

With `-h` you will see help message:

~~~bash
./build.sh -h
Usage:
  ./build.sh [-tjkrh1] <debug|release|minsizerel>
    Executables will be installed at build/cmake_install
OPTIONS:
  -t:          Build test cases and run test on them.
  -j:          Build in parallel, i.e. use make -j
  -k:          Clean temporary building files.
  -b:          Run a benchmarking. It tells how fast the program runs on your computer.
  -v:          Enable verbose log.
  -r:          Use random seed for random number generator. Without this option,
               the program will use a constant value. Thus generate a repeatable result
               (usually together with -1).
  -1:          Use single thread.
  -h:          Show this message.
~~~

Note that debug version will not be installed, so it is found in `build/cmake_build`.

I use the [GoogleTest](https://github.com/google/googletest) framework for my unit tests.
If `-t` option is set, the test cases will be built and test on.
It is usefull in a CI/CD pipeline.


## Configuration file

This file containing all configurations. It uses JSON format.
I use [nlohmann's json](https://github.com/nlohmann/json) to parse JSON file.

### Basic infomation for simulation

Here is an example for basic information:

~~~javascript
"sun": {
    "altitude": 25,
    "diameter": 0.5
},
"ray": {
    "number": 500000,
    "wavelength": [420, 460, 500, 540, 580, 620],
    "weight": [1, 1, 1, 1, 1, 1]
},
"max_recursion": 8,
"data_folder": "<path-to-your-data-folder>"
~~~

* `sun`:
It has two attributes,
  * `altitude`, defining the altitude of the sun. In degree.  
  * `diameter`, defining the actual diameter used in the simulation, in degree. Please set to 0.5 for ture sun.

* `ray`:
It defines some properties of rays used in simulation,
  * `number`, the total ray number for simulation.
    Note that even with a single incident ray, it may result in multiple
    rays output, due to reflections and refractions in crystal. This `number` defines the input ray number,
    but not output ray number. **Note**, this is the ray number for a single wavelength. If you want a real color
    simulation (thus there will be multiple wavelengths), the total number will be multiplier of this `number`
    by the number of following `wavelength`.
  * `wavelength`, the wavelengths used during simulation.
    It is an array contains all wavelengths you want to use. The refractive index data is from
    [Refractive Index of Crystals](https://refractiveindex.info/?shelf=3d&book=crystals&page=ice).
  * `weight`, the weights for wavelengths. It must have the same length with `wavelength`.

* `max_recursion`:
It defines the max number that a ray hits a surface during a simulation. If a ray hits more than this number
and still doesn't leave the crystal, it will be dropped.

* `data_folder`:
It defines where output data files should be located. The simulation program will put data into this
folder and the rendering program will read data from this folder. Also the rendered image will be put
in this folder. In endless mode, there is no intermediate data file, only final image will be put in
this folder.

### Simulation settings

Here is an example of simulation settings:

~~~javascript
"multi_scatter": [
    {
        "crystal": [2, 1, 5, 11, 12],
        "population": [150, 100, 15, 30, 10],
        "probability": 1.0,
        "ray_path_filter": [0, 0, 0, 0, 0]
    },
    {
        "crystal": [2, 1, 5, 11, 12],
        "population": [150, 100, 15, 30, 10],
        "probability": 1.0,
        "ray_path_filter": [0, 0, 0, 0, 0]
    }
]
~~~

* `multi_scatter`:
It is an array defining how to perform multi/single scattering. The example defines a 2-scatter simulation.
Each element in this array defines a single scattering process, and contains 4 properties:
  * `crystal`: what crystal(s) are used in this scattering process. It is an array, filled with crystal IDs
    (See the section Crystal Settings).
  * `population`: the crystal population. It is an array and its length must be the same with of `crystal`.
  * `ray_path_filter`: what filter(s) are used to filter out rays. It is an array and its length must
    be the same with of `crystal`. The rays filtered out are not used in the next scattering process.
    See the section Filter Settings for detail.
  * `probability`: how many output rays are used for next scattering process. If it is set to 0.5, then
    50% output rays are used for next scattering process.

  Multi-scattering is a highlight feature of this project.

### Rendering settings

Here is an example of rendering settings:

~~~javascript
"camera": {
    "azimuth": 0,
    "elevation": 89.99,
    "rotation": 0,
    "fov": 95,
    "lens": "fisheye_equidistant"
},
"render": {
    "width": 4096,
    "height": 4096,
    "visible_semi_sphere": "upper",
    "ray_color": "real",
    "background_color": [0, 0, 0],
    "intensity_factor": 20,
    "offset": [0, 0]
}
~~~

* `camera`:
It defines properties related to camera, including:  
  * `azimuth`, `elevation`, `rotation`: the direction where camera pointing at. In degree.
  * `fov`: (half) field of view, the angle from center to edge. In degree.
  * `lens`: lens type, can be one of these values: `fisheye`, `linear`, `dual_fisheye_equidistant`, `dual_fisheye_equiarea`.

* `render`:
It defines some useful attributes used when rendering:
  * `width`, `height`: the size of output image. In pixel.
  * `visible_semi_sphere`, which semi-sphere should be rendered. The example value is `uppper`,
    indicating the upper semi sphere should be rendered, and the lower one will be black,
    which is the common scene. If it is set to `lower`, then
    halos that occure under horizontal, say, [subparhilia](https://www.atoptics.co.uk/halo/subpars.htm),
    will be rendered. The values could be one of these: `upper`, `lower`, `camera`, `full`.
  * `intensity_factor`, controls the intensity. The value locates between 0.01 and 100.0.
  * `offset`, defines the rendering offset. In pixel.
  * `ray_color`, defines the color used to plot the ray scatter points. It
    can be a 3-element array defining the RGB color, or can be a string `'real'` indicating
    to use real colors. NOTE: RGB value must between 0.0 and 1.0.  
    Real-color is also a highlighted feature of this project.
  * `background_color`, defines the RGB color used for background. Each element must be between 0.0 and 1.0.

### Crystal settings

Here is an example of it:

~~~javascript
"crystal": [
    {
        "id": 1,
        "type": "HexPrism",
        "parameter": 2.4,
        "zenith": {
            "mean": 90,
            "std": 0.3,
            "type": "gauss"
        },
        "roll": {
            "mean": 0,
            "std": 0,
            "type": "uniform"
        }
    },
    {
        "id": 10,
        "type": "HexPyramid",
        "parameter": [0.0, 0.0, 0.2],
        "zenith": {
            "mean": 0,
            "std": 0.3,
            "type": "gauss"
        },
        "roll": {
            "mean": 0,
            "std": 0,
            "type": "uniform"
        }
    },
    {
        "id": 3,
        "type": "IrregularHexPyramid",
        "parameter": [1, 2, 1, 2, 1, 2, 1, 1, 1, 1, 0.3, 0.8, 0.3],
        "zenith": {
            "mean": 90,
            "std": 0.8,
            "type": "gauss"
        },
        "roll": {
            "mean": 0,
            "std": 0,
            "type": "uniform"
        }
    },
    {
        "id": 9,
        "type": "CubicPyramid",
        "parameter": [0.8, 0.8],
        "zenith": {
            "mean": 90,
            "std": 0.3,
            "type": "gauss"
        },
        "roll": {
            "mean": 0,
            "std": 0,
            "type": "uniform"
        }
    }
]
~~~

* `id` is the ID for this crystal. It is the only reference used in other settings, e.g. in multi-scatter settings.
It should be a unique number greater than 0. It is not necessary to keep IDs increasing one by one.

* `zenith`, `roll` and `azimuth` (optional):
These fields defines the orientation of crystals. `zenith` defines the c-axis orientation, and `roll`
defines the rotation around c-axis.

  These fields all has three attributes, `mean`, `std`, `type`.  
  * `type` defines the random distribution
type, either `gauss`, for Gaussian distribution, or `uniform`, for uniform distribution. **NOTE**:
if `type` of `axis` is `uniform`, then `mean` and `std` will be ignored and
the axis will uniformly distributed on sphere. Similarly, if `type` of `roll` is `uniform`
then it will uniformly distributed between 0 and 360 degree.
  * `mean` defines
the mean of random distribution. For example for `zenith` it means the zenith angle.  
  * `std` defines the deviation of the distribution. For Gaussian distribution,
it is the standard deviation, and for uniform distribution, it defines the value range.

  All angles are in degrees.

* `population`:
It defines how many crystals used in a simulation. **Note** that it is not the actual number, just for a
ratio. So if one crystal set to 2.0 and the other set to 3.0, it is equivalent to set one to 20 and
the other to 30.

* `type` and `parameter`:
Currently there are 7 crystal shapes, `HexPrism`, `HexPyramid`, `HexPyramidStackHalf`,
`IrregularHexPrism`, `IrregularHexPyramid`, `CubicPyramid`,
`Custom`.
Each shape has its own shape parameters.

  * `HexPrism`:
  Only 1 parameter, defines `h / a` where `h` is the prism height, `a` is the diameter along
  a-axis (also x-axis in my program).  
  <img src="doc/figs/hex_prism_01.png" width="400">.

  * `HexPyramid`:
  May have 3, 5, or 7 parameters.  
    * For 3 parameters case, they are `h1 / H1`, `h2 / a`, `h3 / H3` respectly,
      where `H1` means the max possible height for upper pyramid segment, and `H3` the same but for lower pyramid segment.  
      <img src="doc/figs/hex_pyramid_01.png" width="400">.  
    * For 5 parameters case, the last 3 parameters are same as the first case,
      and the first 2 parameters indicate the face direction. They must be integers. The
      face direction is described with [Miller index](https://en.wikipedia.org/wiki/Miller_index).
      For example, `a`, `b`, represents a face with Miller index of `(a, 0, -a, b)`. For a
      typical ice crystal face (face number 13), its Miller index is `(1, 0, -1, 1)`.
      So it can be described using parameters 1, 1.  
    * For 7 parameters case, the first 4 parameters are interges and describe the upper and lower pyramid segment
      face directions. For example `a`, `b`, `c`, `d` describe upper pyramid segment with Miller index of
      `(a, 0, -a, b)` and lower pyramid segment of `(c, 0, -c, d)`. NOTE: for faces with different
      Miller index, their maximumn height `H` for pyramid segment are also different.

    With these description, you will have the maximized freedom to design your crystal shape.

  * `HexPyramidStackHalf`:
  7 parameters. Similar to 7 parameters `HexPyramid` case. `h / H` for pyramid segment, and `h / a`
  for prism segment.  
  <img src="doc/figs/hex_pyramid_stack_half_01.png" width="400">.

  * `IrregularHexPrism`:
  Last parameter is height of the crystal. The distance factor here means the ratio of actual distance
  w.r.t regular haxegon distance. Thus, a regular haxegon has distance of `[1, 1, 1, 1, 1, 1]`.
  The following figure shows an irregular hexegon with distance of `[1.1, 0.9, 1.5, 0.9, 1.7, 1.2]`  
  <img src="doc/figs/irr_hex_01.png" width="400">.

  * `IrregularHexPyramid`:
  13 parameters. First 6 parameters define the prism face distance from the origin. Next 4 parameters
  are Miller index describing upper and lower pyramid segment. Last 3 parameters are heights of each segment, from
  upper to lower.  
  <img src="doc/figs/irr_hex_pyramid_01.png" width="400">.

  * `CubicPyramid`:
  2 parameters. Similar to cases above, the 2 parameters defines `h1 / H1` and `h2 / H2`.  
  NOTE: this kind crystal has cubic system.  
  <img src="doc/figs/cubic_pyramid_01.png" width="400">.

  * `Custom`:
  1 parameters that indicates the model file name.  
  Customized crystal type supports
  [Wavefront obj file](https://www.wikiwand.com/en/Wavefront_.obj_file) format. It is an ASCII based
  3D model file format, which means it is human read-frendly and you can open and edit the model file
  with any text editor. Of course it is a better and elegent way to create your crystal in
  a 3D modeling software, such as Maya, 3DMax, Blender, etc.  
  **NOTE**: Though the obj file can contain polygons having more than 3 vertexes, my program
  can only handle triangles. A face is only represented with its first 3 vertexes (if you set more than 3).
  Currently my program cannot handle vertex texture nor vertex normal information. Please make sure your
  obj file does not contain any of them (the face line does not contain any slashes). 


## TODO list

* Use OpenCL / OpenGL / CUDA to accelerate. Since I've seen good enough performance with a simple
  threading pool implemented by myself, I doubt the margin to more improvements.
* Write a (web) GUI for these code.
