# C++ 代码

*提示: 从开发方便的角度, 我会以英文版文档为优先, 中文版内容可能滞后. 个人精力有限, 无法全面照顾, 敬请谅解*

[English version](README.md)

matlab 代码只是一个原型, 并没有充分考虑性能, 因此我专门写了一个 C++ 项目, 以期获得高性能表现.
目前 C++ 版本没有用户界面, 只能从命令行启动.

代码使用了 [Halide](http://halide-lang.org/) 库, 以获得并行加速.

## 构建和运行

本项目使用 [CMake](https://cmake.org/) 构建. 在构建之前, 请参考 [Halide](http://halide-lang.org/) 项目,
并进行安装. 本项目简单的构建过程如下:

1. `cd cpp`
2. `mkdir build && cd build`
3. `cmake .. && make -j4`, 或者设置 `CMAKE_BUILD_TYPE` 为 `release` 以获得高性能可执行程序.
请注意, cmake 文件中有一些硬编码的路径, 用于指定 Halide 的安装路径, 请使用你自己本机的路径.

生成的可执行程序位于 `build/bin`. 在命令行输入 
`./bin/IceHaloSim <path-to-your-config-file>` 即可运行.
这里有一个 [`cpp/config.json`](./config.json) 文件作为输入配置的样本, 可供参考.

## 可视化

在运行程序之后, 你将得到一些 `.bin` 文件, 这些文件包含了所有光线追踪的结果. 此外, 程序还会同时在屏幕上打印冰晶形状的信息.
我写了一些 matlab 代码用于可视化这些信息.

`matlab/src/read_binary_result.m` 用于读取 `.bin` 文件并输出光晕效果图.

`matlab/src/plot_crystal_main.m` 用于绘制冰晶的形状. 你可以把程序运行后在屏幕输出的结果作为数据, 其中,
`V:` 开头的那些代表顶点 (verte) 数据, `F:` 开头的那些代表面 (face) 数据.

## 配置文件

配置文件中包含了用于模拟的所有参数, 配置文件使用 JSON 格式, 这里进行简单介绍. 
配置文件第一层是一个对象, 必须包含 `sun`, `ray_number`,
`max_recursion`, `crystal` 这几项. 本项目中我选择了 [Rapidjson](http://rapidjson.org/index.html)
对 JSON 文件进行解析.

### 模拟的基本设置

* `sun`:
只有一个变量, `altitude`, 用于定义太阳的地平高度.

* `ray_number`:
定义了用于模拟的总光线数量. 请注意, 这里光线数量并不是最终输出的光线数量, 而是输入光线数量. 由于光线在晶体内部进行折射和反射,
在模拟中对所有的折射和反射光线都进行记录, 因此最终的输出光线数量将大于这里定义的值.

* `max_recursion`:
定义了在模拟中光线与晶体表面相交的最多次数. 如果模拟中光线与晶体表面相交次数超过这个值, 而仍然没有离开晶体,
那么对这条光线的模拟将终止, 这条光线的结果将被舍弃.

### 晶体设置

* `type` 和 `parameter`:
用于设置晶体的类型和形状参数. 目前我支持 5 种晶体,
`HexCylinder`, `HexPyramid`, `HexPyramidStackHalf`, `TriPyramid`, `CubicPyramid`.
每种晶体都有自己的形状参数.

  * `HexCylinder`: 六棱柱形冰晶.
  只有 1 个参数, `h`, 定义为 `h / a`, 其中 `h` 是柱体的高, `a` 是底面直径.  
  <img src="figs/hex_cylinder_01.png" width="400">.

  * `HexPyramid`: 六棱锥形冰晶.
  可能有 3, 5, 或者 7 个参数.  
  3 个参数的情况, 分别表示 `h1 / H1`, `h2 / a`, `h3 / H3`, 其中 `H1` 代表第一段锥体最大可能高度, `H3` 类似.  
  <img src="figs/hex_pyramid_01.png" width="400">.  
  5 个参数的情况. 最后 3 个参数含义同上, 开头 2 个参数用于定义晶体表面的方向, 这 2 个参数必须是整数.
  这里使用 [Miller index](https://en.wikipedia.org/wiki/Miller_index) 来表示晶体表面方向. 
  举个例子, 2 个参数为 `a`, `b`, 那么表示 Miller index (`a`, 0, `-a`, `b`). 对于一个正常的冰晶,
  比如编号 13 的那个表面, 对应的 Miller index 是 (1, 0, -1, 1), 那么这里参数就写成 1, 1.  
  7 个参数的情况. 最后 3 个参数含义同上, 开头 4 个参数用于定义晶体表面方向, 前 2 个定义上面一段锥体的表面,
  后 2 个定义下面一段锥体表面, 定义方式同样是基于 Miller index, 与之前的相同.
  请注意, 对不同 Miller index 的表面, 其最大可能高度 `H` 是不一样的.

  * `HexPyramidStackHalf`:
  有 7 个参数. 与前面六棱锥形冰晶参数类似, 开头 4 个参数用于定义锥面的角度, 后面 3 个参数用于定义 3 段的长度,
  对锥体段表示 `h / H`, 对柱体段表示 `h / a`.  
  <img src="figs/hex_pyramid_stack_half_01.png" width="400">. 

  * `TriPyramid`:
  有 5 个参数. 与前面六棱锥形冰晶参数类似, 开头 2 个参数用于定义锥面的角度. 这里上下两个锥体段角度是一样的.
  (其他情况的我稍后有空再加进程序里).  
  <img src="figs/tri_pyramid_01.png" width="400">. 

  * `CubicPyramid`:
  有 2 个参数. 与上面的情形类似, 2 个参数定义了上下两段锥体的长度. 请注意, 这里是立方晶系.  
  <img src="figs/cubic_pyramid_01.png" width="400">

* `axis` and `roll`:
这两个参数定义了晶体的朝向, 其中 `axis` 定义了 c-轴 的指向,`roll` 定义了晶体自身绕 c-轴 的旋转.

  这两个参数都有 3 个属性, `mean`, `std`, `type`. 其中 `type` 定义了随机分布的类型. 要么是 `Gauss`, 
代表高斯分布, 要么是 `Uniform`, 代表平均分布. `mean` 定义了随机分布的均值. 对 `axis` 来说, 
这个值代表天顶角, 从天顶开始度量.
`std` 定义了随机分布的范围. 对高斯分布来说, 这个值就是高斯分布的标准差,
对平均分布来说, 这个值定义了取值范围的大小.

  所有角度单位都是度.

* `population`:
这个参数定义了用于模拟的晶体数量. 请注意, 这里并不是实际数量, 而是一个比例. 比如两种晶体一种是 2.0 一种是 3.0,
那么这等价于一种是 20 另一种是 30.

## 未来的工作

* 使用 OpenCL / OpenGL / CUDA 等对程序进行加速. 不过, 目前使用 Halide 加速的结果已经很好, 进一步加速的必要性并不大.
* 写一个用户界面.
