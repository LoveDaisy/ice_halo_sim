# C接口使用文档

**版本**: V3  
**最后更新**: 2025-12-19

本文档详细说明V3版本的C接口API使用方法。

## 概述

Ice Halo Simulation 提供了完整的C接口，方便与其他语言集成。C接口封装了V3版本的C++实现，提供了简洁的API。

### 头文件

```c
#include "cserver.h"
```

### 链接库

链接 `IceHaloLibV3` 静态库。

## API参考

### 数据类型

#### HS_HaloSimServer

服务器句柄，不透明指针类型。

```c
typedef struct HS_HaloSimServer_ HS_HaloSimServer;
```

#### HS_ServerState

服务器状态枚举。

```c
typedef enum HS_ServerState_ {
  HS_SERVER_IDLE,      // 空闲状态
  HS_SERVER_RUNNING,   // 运行中
  HS_SERVER_NOT_READY, // 未就绪（服务器指针无效）
} HS_ServerState;
```

#### HS_SimResult

结果集句柄，不透明指针类型。

```c
typedef struct HS_SimResult_ HS_SimResult;
```

#### HS_SimResultType

结果类型枚举。

```c
typedef enum HS_SimResultType_ {
  HS_RESULT_NONE,   // 无结果
  HS_RESULT_RENDER, // 渲染结果
  HS_RESULT_STATS,  // 统计结果
} HS_SimResultType;
```

#### HS_RenderResult

渲染结果结构体。

```c
typedef struct HS_RenderResult_ {
  int renderer_id_;        // 渲染器ID
  int img_width_;          // 图像宽度（像素）
  int img_height_;         // 图像高度（像素）
  unsigned char* img_buffer_; // 图像数据缓冲区（RGB格式）
} HS_RenderResult;
```

**注意**：
- `img_buffer_` 指向的图像数据由库管理，不需要手动释放
- 图像数据格式为RGB，每个像素3个字节（R, G, B）
- 图像数据大小 = `img_width_ * img_height_ * 3` 字节

#### HS_StatsResult

统计结果结构体。

```c
typedef struct HS_StatsResult_ {
  unsigned long ray_seg_num_;  // 光线段数量
  unsigned long sim_ray_num_;   // 模拟光线数量
  unsigned long crystal_num_;   // 晶体数量
} HS_StatsResult;
```

### 创建和销毁

#### HS_CreateServer

创建服务器实例。

```c
HS_HaloSimServer* HS_CreateServer();
```

**返回值**：
- 成功：返回服务器句柄指针
- 失败：返回 `NULL`（通常不会失败）

**注意**：
- 返回的句柄必须使用 `HS_DestroyServer()` 释放
- 服务器创建后立即开始运行，无需调用启动函数

#### HS_DestroyServer

销毁服务器实例。

```c
void HS_DestroyServer(HS_HaloSimServer* server);
```

**参数**：
- `server`: 服务器句柄指针，可以为 `NULL`（安全）

**注意**：
- 销毁服务器会停止所有正在进行的处理
- 销毁后不应再使用该句柄
- 如果服务器正在处理，会等待处理完成后再销毁

### 服务器控制

#### HS_QueryServerState

查询服务器状态。

```c
HS_ServerState HS_QueryServerState(HS_HaloSimServer* server);
```

**参数**：
- `server`: 服务器句柄指针

**返回值**：
- `HS_SERVER_IDLE`: 服务器空闲，没有正在处理的任务
- `HS_SERVER_RUNNING`: 服务器正在运行，有任务在处理
- `HS_SERVER_NOT_READY`: 服务器未就绪（`server` 为 `NULL`）

**使用场景**：
- 检查服务器是否空闲
- 等待处理完成

#### HS_CommitConfig

提交配置。

```c
void HS_CommitConfig(HS_HaloSimServer* server, const char* config_str);
```

**参数**：
- `server`: 服务器句柄指针
- `config_str`: JSON格式的配置字符串

**注意**：
- `config_str` 必须是有效的JSON格式
- 配置格式参见 [配置文档](configuration.md)
- 提交配置后，服务器会立即开始处理
- 如果服务器正在处理其他任务，会先停止当前任务

**配置字符串示例**：
```c
const char* config = "{"
  "\"light_source\": [{"
    "\"id\": 1,"
    "\"type\": \"sun\","
    "\"altitude\": 20.0,"
    "\"diameter\": 0.5,"
    "\"wavelength\": [550],"
    "\"wl_weight\": [1.0]"
  "}],"
  "\"crystal\": [{"
    "\"id\": 1,"
    "\"type\": \"prism\","
    "\"shape\": {\"height\": 1.2}"
  "}],"
  "\"scene\": [{"
    "\"id\": 1,"
    "\"light_source\": 1,"
    "\"ray_num\": 1000000,"
    "\"max_hits\": 7,"
    "\"scattering\": [{\"crystal\": [1]}]"
  "}],"
  "\"render\": [{"
    "\"id\": 1,"
    "\"resolution\": [1920, 1080],"
    "\"lens\": {\"type\": \"linear\", \"fov\": 40}"
  "}],"
  "\"project\": [{"
    "\"id\": 1,"
    "\"scene\": 1,"
    "\"render\": [1]"
  "}]"
"}";
```

#### HS_StopServer

停止服务器。

```c
void HS_StopServer(HS_HaloSimServer* server);
```

**参数**：
- `server`: 服务器句柄指针

**注意**：
- 停止服务器会中断正在进行的处理
- 停止后可以继续提交新配置
- 停止不会释放服务器资源，需要调用 `HS_DestroyServer()` 释放

### 结果获取

#### HS_GetAllResults

获取所有结果。

```c
HS_SimResult* HS_GetAllResults(HS_HaloSimServer* server);
```

**参数**：
- `server`: 服务器句柄指针

**返回值**：
- 成功：返回结果集句柄指针
- 失败：返回 `NULL`（`server` 为 `NULL` 时）

**注意**：
- 返回的结果集必须使用 `HS_DeleteAllResults()` 释放
- 结果集可能为空（没有结果时）
- 结果集是快照，不会自动更新
- 非阻塞调用，即使服务器正在处理也会立即返回

#### HS_HasNextResult

检查是否有下一个结果。

```c
int HS_HasNextResult(HS_SimResult* result);
```

**参数**：
- `result`: 结果集句柄指针

**返回值**：
- `1`: 有下一个结果
- `0`: 没有下一个结果或 `result` 为 `NULL`

#### HS_GetNextResult

移动到下一个结果。

```c
HS_SimResult* HS_GetNextResult(HS_SimResult* result);
```

**参数**：
- `result`: 结果集句柄指针

**返回值**：
- 返回 `result` 指针本身（用于链式调用）
- 如果 `result` 为 `NULL`，返回 `NULL`

**注意**：
- 此函数只是移动内部索引，不返回结果数据
- 使用 `HS_QueryResultType()` 和相应的获取函数获取结果数据

#### HS_QueryResultType

查询当前结果的类型。

```c
HS_SimResultType HS_QueryResultType(HS_SimResult* result);
```

**参数**：
- `result`: 结果集句柄指针

**返回值**：
- `HS_RESULT_RENDER`: 渲染结果
- `HS_RESULT_STATS`: 统计结果
- `HS_RESULT_NONE`: 无结果或 `result` 无效

#### HS_GetRenderResult

获取渲染结果。

```c
HS_RenderResult HS_GetRenderResult(HS_SimResult* result);
```

**参数**：
- `result`: 结果集句柄指针

**返回值**：
- 渲染结果结构体
- 如果当前结果不是渲染结果或 `result` 无效，返回空结构体（所有字段为0或NULL）

**注意**：
- `img_buffer_` 指向的图像数据由库管理，不需要手动释放
- 图像数据在 `HS_DeleteAllResults()` 之前有效

#### HS_GetStatsResult

获取统计结果。

```c
HS_StatsResult HS_GetStatsResult(HS_SimResult* result);
```

**参数**：
- `result`: 结果集句柄指针

**返回值**：
- 统计结果结构体
- 如果当前结果不是统计结果或 `result` 无效，返回空结构体（所有字段为0）

#### HS_DeleteAllResults

删除所有结果。

```c
void HS_DeleteAllResults(HS_SimResult* result);
```

**参数**：
- `result`: 结果集句柄指针，可以为 `NULL`（安全）

**注意**：
- 删除结果集会释放所有相关资源
- 删除后不应再使用该句柄
- 删除结果集不会影响服务器状态

## 使用示例

### 基础示例

```c
#include "cserver.h"
#include <stdio.h>
#include <string.h>

int main() {
    // 1. 创建服务器
    HS_HaloSimServer* server = HS_CreateServer();
    if (!server) {
        fprintf(stderr, "Failed to create server\n");
        return 1;
    }

    // 2. 准备配置（简化示例，实际应从文件读取）
    const char* config = "{"
        "\"light_source\": [{\"id\": 1, \"type\": \"sun\", \"altitude\": 20.0, "
        "\"diameter\": 0.5, \"wavelength\": [550], \"wl_weight\": [1.0]}],"
        "\"crystal\": [{\"id\": 1, \"type\": \"prism\", \"shape\": {\"height\": 1.2}}],"
        "\"scene\": [{\"id\": 1, \"light_source\": 1, \"ray_num\": 1000000, "
        "\"max_hits\": 7, \"scattering\": [{\"crystal\": [1]}]}],"
        "\"render\": [{\"id\": 1, \"resolution\": [1920, 1080], "
        "\"lens\": {\"type\": \"linear\", \"fov\": 40}}],"
        "\"project\": [{\"id\": 1, \"scene\": 1, \"render\": [1]}]"
    "}";

    // 3. 提交配置
    HS_CommitConfig(server, config);

    // 4. 等待处理完成
    while (HS_QueryServerState(server) == HS_SERVER_RUNNING) {
        // 可以在这里做其他事情，或使用sleep等待
    }

    // 5. 获取结果
    HS_SimResult* results = HS_GetAllResults(server);
    if (results) {
        // 遍历所有结果
        while (HS_HasNextResult(results)) {
            HS_SimResultType type = HS_QueryResultType(results);
            
            if (type == HS_RESULT_RENDER) {
                HS_RenderResult render = HS_GetRenderResult(results);
                printf("Render result: ID=%d, Size=%dx%d\n",
                       render.renderer_id_, render.img_width_, render.img_height_);
                // 处理图像数据...
            } else if (type == HS_RESULT_STATS) {
                HS_StatsResult stats = HS_GetStatsResult(results);
                printf("Stats: rays=%lu, segments=%lu, crystals=%lu\n",
                       stats.sim_ray_num_, stats.ray_seg_num_, stats.crystal_num_);
            }
            
            HS_GetNextResult(results);
        }
        
        // 6. 清理结果
        HS_DeleteAllResults(results);
    }

    // 7. 销毁服务器
    HS_DestroyServer(server);
    
    return 0;
}
```

### 完整示例（包含错误处理）

```c
#include "cserver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 从文件读取配置
char* read_config_file(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        return NULL;
    }
    
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    char* buffer = malloc(size + 1);
    if (!buffer) {
        fclose(file);
        return NULL;
    }
    
    fread(buffer, 1, size, file);
    buffer[size] = '\0';
    
    fclose(file);
    return buffer;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <config.json>\n", argv[0]);
        return 1;
    }

    // 1. 创建服务器
    HS_HaloSimServer* server = HS_CreateServer();
    if (!server) {
        fprintf(stderr, "Error: Failed to create server\n");
        return 1;
    }

    // 2. 读取配置
    char* config = read_config_file(argv[1]);
    if (!config) {
        fprintf(stderr, "Error: Failed to read config file\n");
        HS_DestroyServer(server);
        return 1;
    }

    // 3. 提交配置
    HS_CommitConfig(server, config);
    free(config);

    // 4. 等待处理完成
    printf("Processing...\n");
    int max_wait = 1000; // 最大等待次数
    int wait_count = 0;
    while (HS_QueryServerState(server) == HS_SERVER_RUNNING) {
        if (++wait_count > max_wait) {
            fprintf(stderr, "Warning: Processing timeout\n");
            break;
        }
        // 简单的等待，实际可以使用sleep或事件机制
    }

    // 5. 获取并处理结果
    HS_SimResult* results = HS_GetAllResults(server);
    if (!results) {
        fprintf(stderr, "Error: Failed to get results\n");
        HS_DestroyServer(server);
        return 1;
    }

    int result_count = 0;
    while (HS_HasNextResult(results)) {
        HS_SimResultType type = HS_QueryResultType(results);
        
        switch (type) {
            case HS_RESULT_RENDER: {
                HS_RenderResult render = HS_GetRenderResult(results);
                printf("Render result #%d: ID=%d, Size=%dx%d\n",
                       result_count++, render.renderer_id_,
                       render.img_width_, render.img_height_);
                
                // 保存图像（示例）
                if (render.img_buffer_) {
                    char filename[256];
                    snprintf(filename, sizeof(filename),
                            "output_%d_%dx%d.raw",
                            render.renderer_id_,
                            render.img_width_, render.img_height_);
                    
                    FILE* img_file = fopen(filename, "wb");
                    if (img_file) {
                        size_t img_size = render.img_width_ * render.img_height_ * 3;
                        fwrite(render.img_buffer_, 1, img_size, img_file);
                        fclose(img_file);
                        printf("  Saved to %s\n", filename);
                    }
                }
                break;
            }
            
            case HS_RESULT_STATS: {
                HS_StatsResult stats = HS_GetStatsResult(results);
                printf("Stats result #%d: rays=%lu, segments=%lu, crystals=%lu\n",
                       result_count++,
                       stats.sim_ray_num_,
                       stats.ray_seg_num_,
                       stats.crystal_num_);
                break;
            }
            
            case HS_RESULT_NONE:
            default:
                printf("Unknown result type\n");
                break;
        }
        
        HS_GetNextResult(results);
    }

    if (result_count == 0) {
        printf("No results available\n");
    }

    // 6. 清理
    HS_DeleteAllResults(results);
    HS_DestroyServer(server);
    
    return 0;
}
```

## 错误处理

### 常见错误

#### 1. 服务器创建失败

**原因**：内存不足（极少见）

**处理**：
```c
HS_HaloSimServer* server = HS_CreateServer();
if (!server) {
    // 处理错误
    return;
}
```

#### 2. 配置格式错误

**原因**：`config_str` 不是有效的JSON格式

**处理**：
- 验证JSON格式（可以使用JSON验证库）
- 检查配置字符串是否正确转义
- 参考 [配置文档](configuration.md) 确保配置格式正确

#### 3. 结果获取失败

**原因**：服务器指针无效或服务器未就绪

**处理**：
```c
HS_SimResult* results = HS_GetAllResults(server);
if (!results) {
    // 检查服务器状态
    HS_ServerState state = HS_QueryServerState(server);
    if (state == HS_SERVER_NOT_READY) {
        // 服务器无效
    }
    return;
}
```

#### 4. 结果类型不匹配

**原因**：尝试获取错误类型的结果

**处理**：
```c
HS_SimResultType type = HS_QueryResultType(results);
if (type == HS_RESULT_RENDER) {
    HS_RenderResult render = HS_GetRenderResult(results);
    // 处理渲染结果
} else if (type == HS_RESULT_STATS) {
    HS_StatsResult stats = HS_GetStatsResult(results);
    // 处理统计结果
}
```

### 错误处理最佳实践

1. **检查返回值**：所有可能返回 `NULL` 的函数都应检查返回值
2. **验证状态**：使用 `HS_QueryServerState()` 检查服务器状态
3. **资源清理**：确保在所有退出路径上释放资源
4. **错误日志**：记录错误信息便于调试

## 线程安全性

### 线程安全API

以下API是**线程安全**的：
- `HS_QueryServerState()`: 可以安全地从多个线程调用
- `HS_GetAllResults()`: 可以安全地从多个线程调用

### 非线程安全API

以下API**不是线程安全**的，不应从多个线程同时调用：
- `HS_CreateServer()` / `HS_DestroyServer()`: 服务器生命周期管理
- `HS_CommitConfig()`: 配置提交
- `HS_StopServer()`: 服务器停止
- `HS_SimResult` 相关API: 结果集操作

### 多线程使用建议

1. **单服务器多线程**：
   - 每个线程使用独立的服务器实例
   - 或使用互斥锁保护服务器操作

2. **结果处理**：
   - 每个线程使用独立的结果集句柄
   - 不要在多个线程间共享 `HS_SimResult` 句柄

3. **推荐模式**：
   ```c
   // 模式1：每个线程独立的服务器
   void* worker_thread(void* arg) {
       HS_HaloSimServer* server = HS_CreateServer();
       // 使用服务器...
       HS_DestroyServer(server);
       return NULL;
   }
   
   // 模式2：使用互斥锁保护
   pthread_mutex_t server_mutex = PTHREAD_MUTEX_INITIALIZER;
   
   void use_server(HS_HaloSimServer* server) {
       pthread_mutex_lock(&server_mutex);
       HS_CommitConfig(server, config);
       // ...
       pthread_mutex_unlock(&server_mutex);
   }
   ```

## 与其他语言集成

### Python集成（使用ctypes）

```python
import ctypes
import json

# 加载库
lib = ctypes.CDLL('./libIceHaloLibV3.so')  # Linux
# lib = ctypes.CDLL('./libIceHaloLibV3.dylib')  # macOS
# lib = ctypes.WinDLL('./IceHaloLibV3.dll')  # Windows

# 定义类型
class HS_RenderResult(ctypes.Structure):
    _fields_ = [
        ('renderer_id_', ctypes.c_int),
        ('img_width_', ctypes.c_int),
        ('img_height_', ctypes.c_int),
        ('img_buffer_', ctypes.POINTER(ctypes.c_ubyte)),
    ]

class HS_StatsResult(ctypes.Structure):
    _fields_ = [
        ('ray_seg_num_', ctypes.c_ulong),
        ('sim_ray_num_', ctypes.c_ulong),
        ('crystal_num_', ctypes.c_ulong),
    ]

# 定义函数签名
HS_CreateServer = lib.HS_CreateServer
HS_CreateServer.restype = ctypes.c_void_p

HS_DestroyServer = lib.HS_DestroyServer
HS_DestroyServer.argtypes = [ctypes.c_void_p]

HS_CommitConfig = lib.HS_CommitConfig
HS_CommitConfig.argtypes = [ctypes.c_void_p, ctypes.c_char_p]

HS_QueryServerState = lib.HS_QueryServerState
HS_QueryServerState.argtypes = [ctypes.c_void_p]
HS_QueryServerState.restype = ctypes.c_int

HS_GetAllResults = lib.HS_GetAllResults
HS_GetAllResults.restype = ctypes.c_void_p

# 使用示例
def simulate(config_file):
    # 读取配置
    with open(config_file, 'r') as f:
        config_json = json.load(f)
        config_str = json.dumps(config_json).encode('utf-8')
    
    # 创建服务器
    server = HS_CreateServer()
    if not server:
        raise RuntimeError("Failed to create server")
    
    try:
        # 提交配置
        HS_CommitConfig(server, config_str)
        
        # 等待完成
        while HS_QueryServerState(server) != 0:  # 0 = IDLE
            pass
        
        # 获取结果
        results = HS_GetAllResults(server)
        if results:
            # 处理结果...
            pass
        
    finally:
        HS_DestroyServer(server)

# 使用
simulate('config.json')
```

### Rust集成

```rust
use std::ffi::{CString, CStr};
use std::os::raw::{c_char, c_void, c_int, c_ulong};

#[repr(C)]
struct HS_RenderResult {
    renderer_id: c_int,
    img_width: c_int,
    img_height: c_int,
    img_buffer: *const u8,
}

#[repr(C)]
struct HS_StatsResult {
    ray_seg_num: c_ulong,
    sim_ray_num: c_ulong,
    crystal_num: c_ulong,
}

#[link(name = "IceHaloLibV3")]
extern "C" {
    fn HS_CreateServer() -> *mut c_void;
    fn HS_DestroyServer(server: *mut c_void);
    fn HS_CommitConfig(server: *mut c_void, config: *const c_char);
    fn HS_QueryServerState(server: *mut c_void) -> c_int;
    fn HS_GetAllResults(server: *mut c_void) -> *mut c_void;
}

fn simulate(config_json: &str) -> Result<(), Box<dyn std::error::Error>> {
    let server = unsafe { HS_CreateServer() };
    if server.is_null() {
        return Err("Failed to create server".into());
    }
    
    let config_cstr = CString::new(config_json)?;
    
    unsafe {
        HS_CommitConfig(server, config_cstr.as_ptr());
        
        while HS_QueryServerState(server) != 0 {
            // 等待完成
        }
        
        let results = HS_GetAllResults(server);
        // 处理结果...
        
        HS_DestroyServer(server);
    }
    
    Ok(())
}
```

## 常见问题

### Q1: 如何从文件读取配置？

**A**: 读取文件内容为字符串，然后传递给 `HS_CommitConfig()`。参见完整示例中的 `read_config_file()` 函数。

### Q2: 图像数据格式是什么？

**A**: RGB格式，每个像素3个字节（R, G, B），按行存储。图像大小 = `width * height * 3` 字节。

### Q3: 如何保存渲染结果？

**A**: 将 `img_buffer_` 中的数据写入文件。可以保存为RAW格式，或使用图像库转换为PNG/JPEG等格式。

### Q4: 服务器可以同时处理多个配置吗？

**A**: 不可以。每次调用 `HS_CommitConfig()` 会停止当前任务并开始新任务。如需并行处理，应创建多个服务器实例。

### Q5: 结果数据何时失效？

**A**: 调用 `HS_DeleteAllResults()` 后，结果数据失效。在此之前，结果数据一直有效。

### Q6: 如何处理配置错误？

**A**: 配置错误通常不会导致API调用失败，但处理结果可能为空。建议：
- 验证JSON格式
- 检查配置项的有效性
- 查看服务器日志（如启用）

## 相关文档

- [配置文档](configuration.md) - 配置格式详细说明
- [系统架构文档](architecture.md) - 系统架构设计
- [开发指南](developer-guide.md) - 开发指南
- [文档索引](README.md) - 所有文档的导航
