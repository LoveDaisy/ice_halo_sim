# C接口使用文档

本文档详细说明C接口API使用方法。

## 概述

Lumice 提供了完整的C接口，方便与其他语言集成。C接口封装了C++实现，提供了简洁的API。

### 头文件

```c
#include "lumice.h"
```

### 链接库

链接 `lumice` 静态库。

### 设计原则

- **统一错误码**：大多数 API 返回 `LUMICE_ErrorCode`，实际输出通过指针参数传递
- **数组+哨兵模式**：结果获取 API 使用固定大小数组 + 哨兵终止，无堆分配
- **零拷贝**：渲染结果的 `img_buffer` 直接指向内部缓冲区，无拷贝开销

## API参考

### 常量

```c
#define LUMICE_MAX_RENDER_RESULTS 16  // 渲染结果数组最大容量
#define LUMICE_MAX_STATS_RESULTS 1    // 统计结果数组最大容量
```

### 数据类型

#### LUMICE_Server

服务器句柄，不透明指针类型。

```c
typedef struct LUMICE_Server_ LUMICE_Server;
```

#### LUMICE_ErrorCode

错误码枚举。大多数 API 的返回值。

```c
typedef enum LUMICE_ErrorCode_ {
  LUMICE_OK = 0,             // 成功
  LUMICE_ERR_NULL_ARG,       // NULL 参数
  LUMICE_ERR_INVALID_JSON,   // JSON 格式错误
  LUMICE_ERR_INVALID_CONFIG, // 配置内容错误
  LUMICE_ERR_MISSING_FIELD,  // 缺少必填字段
  LUMICE_ERR_INVALID_VALUE,  // 字段值无效
  LUMICE_ERR_FILE_NOT_FOUND, // 文件不存在/无法打开
  LUMICE_ERR_SERVER,         // 服务器内部错误
} LUMICE_ErrorCode;
```

#### LUMICE_ServerState

服务器状态枚举。

```c
typedef enum LUMICE_ServerState_ {
  LUMICE_SERVER_IDLE,      // 空闲状态
  LUMICE_SERVER_RUNNING,   // 运行中
  LUMICE_SERVER_NOT_READY, // 未就绪
} LUMICE_ServerState;
```

#### LUMICE_LogLevel

日志级别枚举。

```c
typedef enum LUMICE_LogLevel_ {
  LUMICE_LOG_TRACE,    // 追踪
  LUMICE_LOG_DEBUG,    // 调试
  LUMICE_LOG_INFO,     // 信息（默认）
  LUMICE_LOG_WARNING,  // 警告
  LUMICE_LOG_ERROR,    // 错误
  LUMICE_LOG_OFF,      // 关闭日志
} LUMICE_LogLevel;
```

#### LUMICE_RenderResult

渲染结果结构体。

```c
typedef struct LUMICE_RenderResult_ {
  int renderer_id;                   // 渲染器ID
  int img_width;                     // 图像宽度（像素）
  int img_height;                    // 图像高度（像素）
  const unsigned char* img_buffer;   // 图像数据缓冲区（RGB格式，只读）
} LUMICE_RenderResult;
```

**注意**：
- `img_buffer` 指向的图像数据由库内部管理，不需要手动释放
- `img_buffer` 有效期：直到下一次 `LUMICE_GetRenderResults()` 或 `LUMICE_CommitConfig()` 调用
- 如需长期持有，应自行 `memcpy` 拷贝
- 图像数据格式为RGB，每个像素3个字节（R, G, B）
- 图像数据大小 = `img_width * img_height * 3` 字节
- 哨兵标识：`img_buffer == NULL`

#### LUMICE_StatsResult

统计结果结构体。

```c
typedef struct LUMICE_StatsResult_ {
  unsigned long ray_seg_num;   // 光线段数量
  unsigned long sim_ray_num;   // 模拟光线数量
  unsigned long crystal_num;   // 晶体数量
} LUMICE_StatsResult;
```

**注意**：
- 哨兵标识：全零（`sim_ray_num == 0`）

### 服务器生命周期

#### LUMICE_CreateServer

创建服务器实例。

```c
LUMICE_Server* LUMICE_CreateServer(void);
```

**返回值**：
- 成功：返回服务器句柄指针
- 失败：返回 `NULL`

**注意**：
- 返回的句柄必须使用 `LUMICE_DestroyServer()` 释放

#### LUMICE_DestroyServer

销毁服务器实例。

```c
void LUMICE_DestroyServer(LUMICE_Server* server);
```

**参数**：
- `server`: 服务器句柄指针，可以为 `NULL`（安全）

**注意**：
- 销毁服务器会停止所有正在进行的处理
- 销毁后不应再使用该句柄

### 日志

#### LUMICE_InitLogger

初始化日志系统。

```c
void LUMICE_InitLogger(LUMICE_Server* server);
```

#### LUMICE_SetLogLevel

设置日志级别。

```c
void LUMICE_SetLogLevel(LUMICE_Server* server, LUMICE_LogLevel level);
```

### 配置

#### LUMICE_CommitConfig

提交JSON字符串配置。

```c
LUMICE_ErrorCode LUMICE_CommitConfig(LUMICE_Server* server, const char* config_str);
```

**参数**：
- `server`: 服务器句柄指针
- `config_str`: JSON格式的配置字符串

**返回值**：
- `LUMICE_OK`: 成功
- `LUMICE_ERR_NULL_ARG`: `server` 或 `config_str` 为 `NULL`
- `LUMICE_ERR_INVALID_JSON`: JSON 格式错误
- `LUMICE_ERR_INVALID_CONFIG` / `LUMICE_ERR_MISSING_FIELD` / `LUMICE_ERR_INVALID_VALUE`: 配置内容错误

**注意**：
- 提交配置后，服务器会立即开始处理
- 配置格式参见 [配置文档](configuration.md)

#### LUMICE_CommitConfigFromFile

从文件加载并提交配置。

```c
LUMICE_ErrorCode LUMICE_CommitConfigFromFile(LUMICE_Server* server, const char* filename);
```

**参数**：
- `server`: 服务器句柄指针
- `filename`: 配置文件路径

**返回值**：
- `LUMICE_OK`: 成功
- `LUMICE_ERR_NULL_ARG`: `server` 或 `filename` 为 `NULL`
- `LUMICE_ERR_FILE_NOT_FOUND`: 文件不存在或无法打开
- 其他错误码：配置内容错误

### 结果获取

结果获取使用统一的数组+哨兵模式：
- 函数签名：`LUMICE_ErrorCode LUMICE_GetXxxResults(server, out, max_count)`
- `out` 数组大小至少为 `max_count + 1`（含哨兵位）
- 有效结果之后紧跟一个全零的哨兵条目
- 调用方循环到哨兵自然结束

#### LUMICE_GetRenderResults

获取渲染结果。

```c
LUMICE_ErrorCode LUMICE_GetRenderResults(LUMICE_Server* server, LUMICE_RenderResult* out, int max_count);
```

**参数**：
- `server`: 服务器句柄指针
- `out`: 输出数组，大小至少为 `max_count + 1`
- `max_count`: 最大结果数量（推荐使用 `LUMICE_MAX_RENDER_RESULTS`）

**返回值**：
- `LUMICE_OK`: 成功
- `LUMICE_ERR_NULL_ARG`: `server` 或 `out` 为 `NULL`

**哨兵**：末尾条目的 `img_buffer == NULL`

#### LUMICE_GetStatsResults

获取统计结果。

```c
LUMICE_ErrorCode LUMICE_GetStatsResults(LUMICE_Server* server, LUMICE_StatsResult* out, int max_count);
```

**参数**：
- `server`: 服务器句柄指针
- `out`: 输出数组，大小至少为 `max_count + 1`
- `max_count`: 最大结果数量（推荐使用 `LUMICE_MAX_STATS_RESULTS`）

**返回值**：
- `LUMICE_OK`: 成功
- `LUMICE_ERR_NULL_ARG`: `server` 或 `out` 为 `NULL`

**哨兵**：末尾条目的 `sim_ray_num == 0`

### 状态与控制

#### LUMICE_QueryServerState

查询服务器状态。

```c
LUMICE_ErrorCode LUMICE_QueryServerState(LUMICE_Server* server, LUMICE_ServerState* out);
```

**参数**：
- `server`: 服务器句柄指针
- `out`: 输出状态指针

**返回值**：
- `LUMICE_OK`: 成功，状态写入 `*out`
- `LUMICE_ERR_NULL_ARG`: `server` 或 `out` 为 `NULL`

#### LUMICE_StopServer

停止服务器。

```c
void LUMICE_StopServer(LUMICE_Server* server);
```

**参数**：
- `server`: 服务器句柄指针，可以为 `NULL`（安全）

**注意**：
- 停止后可以继续提交新配置
- 停止不会释放服务器资源，需要调用 `LUMICE_DestroyServer()` 释放

## 使用示例

### 基础示例

```c
#include "lumice.h"
#include <stdio.h>

int main() {
    // 1. 创建服务器
    LUMICE_Server* server = LUMICE_CreateServer();
    if (!server) {
        fprintf(stderr, "Failed to create server\n");
        return 1;
    }

    // 2. 初始化日志
    LUMICE_InitLogger(server);

    // 3. 从文件加载配置
    if (LUMICE_CommitConfigFromFile(server, "config.json") != LUMICE_OK) {
        LUMICE_DestroyServer(server);
        return 1;
    }

    // 4. 轮询结果
    while (1) {
        usleep(1000000);  // 1 second

        // 获取渲染结果
        LUMICE_RenderResult renders[LUMICE_MAX_RENDER_RESULTS + 1];
        if (LUMICE_GetRenderResults(server, renders, LUMICE_MAX_RENDER_RESULTS) == LUMICE_OK) {
            for (int i = 0; renders[i].img_buffer != NULL; i++) {
                printf("Render[%02d]: %dx%d\n",
                       renders[i].renderer_id, renders[i].img_width, renders[i].img_height);
            }
        }

        // 获取统计结果
        LUMICE_StatsResult stats[LUMICE_MAX_STATS_RESULTS + 1];
        if (LUMICE_GetStatsResults(server, stats, LUMICE_MAX_STATS_RESULTS) == LUMICE_OK) {
            for (int i = 0; stats[i].sim_ray_num != 0; i++) {
                printf("Stats: rays=%lu, crystals=%lu\n",
                       stats[i].sim_ray_num, stats[i].crystal_num);
            }
        }

        // 检查是否完成
        LUMICE_ServerState state;
        if (LUMICE_QueryServerState(server, &state) == LUMICE_OK && state == LUMICE_SERVER_IDLE) {
            break;
        }
    }

    // 5. 销毁服务器
    LUMICE_DestroyServer(server);
    return 0;
}
```

### 完整示例（包含错误处理）

```c
#include "lumice.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <config.json>\n", argv[0]);
        return 1;
    }

    // 1. 创建服务器
    LUMICE_Server* server = LUMICE_CreateServer();
    if (!server) {
        fprintf(stderr, "Error: Failed to create server\n");
        return 1;
    }

    // 2. 初始化日志
    LUMICE_InitLogger(server);
    LUMICE_SetLogLevel(server, LUMICE_LOG_INFO);

    // 3. 加载配置
    LUMICE_ErrorCode err = LUMICE_CommitConfigFromFile(server, argv[1]);
    if (err != LUMICE_OK) {
        fprintf(stderr, "Error: Failed to load config (error code: %d)\n", err);
        LUMICE_DestroyServer(server);
        return 1;
    }

    // 4. 轮询结果
    while (1) {
        usleep(1000000);

        // 获取渲染结果
        LUMICE_RenderResult renders[LUMICE_MAX_RENDER_RESULTS + 1];
        if (LUMICE_GetRenderResults(server, renders, LUMICE_MAX_RENDER_RESULTS) == LUMICE_OK) {
            for (int i = 0; renders[i].img_buffer != NULL; i++) {
                printf("Render[%02d]: %dx%d, buffer=%p\n",
                       renders[i].renderer_id, renders[i].img_width, renders[i].img_height,
                       (const void*)renders[i].img_buffer);

                // 保存图像（示例）
                char filename[256];
                snprintf(filename, sizeof(filename), "output_%d_%dx%d.raw",
                         renders[i].renderer_id, renders[i].img_width, renders[i].img_height);
                FILE* img_file = fopen(filename, "wb");
                if (img_file) {
                    size_t img_size = (size_t)renders[i].img_width * renders[i].img_height * 3;
                    fwrite(renders[i].img_buffer, 1, img_size, img_file);
                    fclose(img_file);
                }
            }
        }

        // 获取统计结果
        LUMICE_StatsResult stats[LUMICE_MAX_STATS_RESULTS + 1];
        if (LUMICE_GetStatsResults(server, stats, LUMICE_MAX_STATS_RESULTS) == LUMICE_OK) {
            for (int i = 0; stats[i].sim_ray_num != 0; i++) {
                printf("Stats: rays=%lu, segments=%lu, crystals=%lu\n",
                       stats[i].sim_ray_num, stats[i].ray_seg_num, stats[i].crystal_num);
            }
        }

        // 检查是否完成
        LUMICE_ServerState state;
        if (LUMICE_QueryServerState(server, &state) == LUMICE_OK && state == LUMICE_SERVER_IDLE) {
            break;
        }
    }

    // 5. 清理
    LUMICE_DestroyServer(server);
    return 0;
}
```

## 错误处理

### LUMICE_ErrorCode 错误码

所有配置/查询/结果获取 API 返回 `LUMICE_ErrorCode`。最佳实践：

```c
LUMICE_ErrorCode err = LUMICE_CommitConfigFromFile(server, "config.json");
if (err != LUMICE_OK) {
    switch (err) {
        case LUMICE_ERR_NULL_ARG:
            fprintf(stderr, "Null argument\n");
            break;
        case LUMICE_ERR_FILE_NOT_FOUND:
            fprintf(stderr, "Config file not found\n");
            break;
        case LUMICE_ERR_INVALID_JSON:
            fprintf(stderr, "Invalid JSON format\n");
            break;
        default:
            fprintf(stderr, "Error code: %d\n", err);
            break;
    }
}
```

### 错误处理最佳实践

1. **检查错误码**：所有返回 `LUMICE_ErrorCode` 的函数都应检查返回值
2. **检查创建结果**：`LUMICE_CreateServer()` 返回 `NULL` 表示失败
3. **资源清理**：确保在所有退出路径上调用 `LUMICE_DestroyServer()`

## 线程安全性

### 线程安全API

以下API是**线程安全**的：
- `LUMICE_QueryServerState()`: 可以安全地从多个线程调用
- `LUMICE_GetRenderResults()` / `LUMICE_GetStatsResults()`: 可以安全地从多个线程调用

### 非线程安全API

以下API**不是线程安全**的，不应从多个线程同时调用：
- `LUMICE_CreateServer()` / `LUMICE_DestroyServer()`: 服务器生命周期管理
- `LUMICE_CommitConfig()` / `LUMICE_CommitConfigFromFile()`: 配置提交
- `LUMICE_StopServer()`: 服务器停止

### 多线程使用建议

1. **单服务器多线程**：使用互斥锁保护非线程安全的操作
2. **多服务器**：每个线程使用独立的服务器实例

## 与其他语言集成

### Python集成（使用ctypes）

```python
import ctypes
import json

# 加载库
lib = ctypes.CDLL('./liblumice.so')  # Linux
# lib = ctypes.CDLL('./liblumice.dylib')  # macOS

# 定义类型
class LUMICE_RenderResult(ctypes.Structure):
    _fields_ = [
        ('renderer_id', ctypes.c_int),
        ('img_width', ctypes.c_int),
        ('img_height', ctypes.c_int),
        ('img_buffer', ctypes.POINTER(ctypes.c_ubyte)),
    ]

class LUMICE_StatsResult(ctypes.Structure):
    _fields_ = [
        ('ray_seg_num', ctypes.c_ulong),
        ('sim_ray_num', ctypes.c_ulong),
        ('crystal_num', ctypes.c_ulong),
    ]

LUMICE_MAX_RENDER_RESULTS = 16
LUMICE_MAX_STATS_RESULTS = 1

# 定义函数签名
lib.LUMICE_CreateServer.restype = ctypes.c_void_p
lib.LUMICE_DestroyServer.argtypes = [ctypes.c_void_p]
lib.LUMICE_CommitConfigFromFile.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
lib.LUMICE_CommitConfigFromFile.restype = ctypes.c_int
lib.LUMICE_GetRenderResults.argtypes = [ctypes.c_void_p, ctypes.POINTER(LUMICE_RenderResult), ctypes.c_int]
lib.LUMICE_GetRenderResults.restype = ctypes.c_int
lib.LUMICE_GetStatsResults.argtypes = [ctypes.c_void_p, ctypes.POINTER(LUMICE_StatsResult), ctypes.c_int]
lib.LUMICE_GetStatsResults.restype = ctypes.c_int
lib.LUMICE_QueryServerState.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
lib.LUMICE_QueryServerState.restype = ctypes.c_int
lib.LUMICE_StopServer.argtypes = [ctypes.c_void_p]

# 使用示例
def simulate(config_file):
    server = lib.LUMICE_CreateServer()
    if not server:
        raise RuntimeError("Failed to create server")

    try:
        err = lib.LUMICE_CommitConfigFromFile(server, config_file.encode('utf-8'))
        if err != 0:
            raise RuntimeError(f"Failed to load config (error: {err})")

        import time
        while True:
            time.sleep(1)

            # 获取渲染结果
            renders = (LUMICE_RenderResult * (LUMICE_MAX_RENDER_RESULTS + 1))()
            if lib.LUMICE_GetRenderResults(server, renders, LUMICE_MAX_RENDER_RESULTS) == 0:
                for r in renders:
                    if not r.img_buffer:
                        break
                    print(f"Render[{r.renderer_id:02d}]: {r.img_width}x{r.img_height}")

            # 检查状态
            state = ctypes.c_int()
            if lib.LUMICE_QueryServerState(server, ctypes.byref(state)) == 0 and state.value == 0:
                break
    finally:
        lib.LUMICE_DestroyServer(server)
```

### Rust集成

```rust
use std::ffi::CString;
use std::os::raw::{c_char, c_int, c_uchar, c_ulong, c_void};

const LUMICE_MAX_RENDER_RESULTS: usize = 16;
const LUMICE_MAX_STATS_RESULTS: usize = 1;

#[repr(C)]
struct LUMICE_RenderResult {
    renderer_id: c_int,
    img_width: c_int,
    img_height: c_int,
    img_buffer: *const c_uchar,
}

#[repr(C)]
struct LUMICE_StatsResult {
    ray_seg_num: c_ulong,
    sim_ray_num: c_ulong,
    crystal_num: c_ulong,
}

#[link(name = "lumice")]
extern "C" {
    fn LUMICE_CreateServer() -> *mut c_void;
    fn LUMICE_DestroyServer(server: *mut c_void);
    fn LUMICE_CommitConfigFromFile(server: *mut c_void, filename: *const c_char) -> c_int;
    fn LUMICE_GetRenderResults(server: *mut c_void, out: *mut LUMICE_RenderResult, max_count: c_int) -> c_int;
    fn LUMICE_GetStatsResults(server: *mut c_void, out: *mut LUMICE_StatsResult, max_count: c_int) -> c_int;
    fn LUMICE_QueryServerState(server: *mut c_void, out: *mut c_int) -> c_int;
    fn LUMICE_StopServer(server: *mut c_void);
}
```

## 常见问题

### Q1: 如何从文件读取配置？

**A**: 使用 `LUMICE_CommitConfigFromFile()`，直接传入文件路径。也可以将文件内容读取为字符串后传给 `LUMICE_CommitConfig()`。

### Q2: 图像数据格式是什么？

**A**: RGB格式，每个像素3个字节（R, G, B），按行存储。图像大小 = `img_width * img_height * 3` 字节。

### Q3: img_buffer 何时失效？

**A**: `img_buffer` 指针有效至下一次 `LUMICE_GetRenderResults()` 或 `LUMICE_CommitConfig()` 调用。如需长期持有，应自行 `memcpy` 拷贝数据。

### Q4: 服务器可以同时处理多个配置吗？

**A**: 不可以。每次调用 `LUMICE_CommitConfig()` 会停止当前任务并开始新任务。如需并行处理，应创建多个服务器实例。

### Q5: 如何判断结果数组是否为空？

**A**: 检查数组第一个元素是否为哨兵。对于渲染结果：`renders[0].img_buffer == NULL` 表示无结果。对于统计结果：`stats[0].sim_ray_num == 0` 表示无结果。

## 相关文档

- [配置文档](configuration.md) - 配置格式详细说明
- [系统架构文档](architecture.md) - 系统架构设计
- [开发指南](developer-guide.md) - 开发指南
- [文档索引](README.md) - 所有文档的导航
