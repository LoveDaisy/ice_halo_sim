[English version](README.md)

# 文档索引

欢迎查阅 Lumice 项目文档。本文档提供所有文档的导航和索引。

## 文档结构

```
doc/
├── README.md / README_zh.md       # 文档索引（英文/中文）
├── architecture.md / _zh.md       # 系统架构文档
├── configuration.md / _zh.md      # 配置文档
├── c_api.md / _zh.md              # C接口使用文档
├── developer-guide.md / _zh.md    # 开发指南
├── api/                           # API文档（Doxygen生成）
└── figs/                          # 图片资源
```

## 用户文档

面向使用本项目的用户：

- **[配置文档](configuration_zh.md)**: 配置文件的完整说明
  - 配置项详细说明和默认值
  - 配置验证规则
  - 常见配置错误和解决方案

- **[系统架构文档](architecture_zh.md)**: 系统架构设计
  - 服务器-消费者架构说明
  - 模块职责和依赖关系
  - 数据流和处理流程
  - 程序入口说明

## 开发者文档

面向项目开发者和贡献者：

- **[开发指南](developer-guide_zh.md)**: 开发指南
  - 开发环境设置
  - 代码风格指南
  - 如何添加新功能
  - 测试指南
  - 调试技巧

- **[C接口使用文档](c_api_zh.md)**: C接口API使用说明
  - C API完整参考
  - 使用示例
  - 错误处理指南
  - 线程安全性说明
  - 与其他语言集成示例

- **[API文档](api/html/)**: 自动生成的API文档
  - 使用Doxygen生成
  - 包含所有公共API的详细说明
  - **本地生成**：运行 `doxygen .doxygen-config`（在项目根目录下）
  - 生成的文档位于 `doc/api/html/`

## 快速链接

- [项目 README](../README.md)
- [项目中文 README](../README_zh.md)
- [配置示例文件](../config_example.json)

## 文档语言策略

- **双语文档**：所有文档提供中英文两个版本
- **中文版**：文件名以 `_zh` 后缀标记
- **英文版**：文件名不带后缀（默认）
- **代码注释**：使用英文
- **API文档**：Doxygen生成，使用英文

### 文档生成

**API文档生成**（本地验证）：
```bash
doxygen .doxygen-config
```
生成的文档位于 `doc/api/html/`。

## 反馈和建议

如发现文档问题或有改进建议，欢迎通过Issue或Pull Request反馈。
