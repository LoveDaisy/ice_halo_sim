# 文档索引

**最后更新**: 2025-12-19

欢迎查阅 Ice Halo Simulation 项目文档。本文档提供所有文档的导航和索引。

## 文档结构

```
cpp/doc/
├── README.md              # 本文档索引
├── architecture.md        # 系统架构文档
├── configuration.md       # 配置文档
├── c_api.md              # C接口使用文档（待创建）
├── developer-guide.md    # 开发指南（待创建）
├── api/                  # API文档（Doxygen生成，待创建）
└── figs/                 # 图片资源
```

## 用户文档

面向使用本项目的用户：

- **[配置文档](configuration.md)**: V3版本配置文件的完整说明
  - 配置项详细说明和默认值
  - 配置验证规则
  - 常见配置错误和解决方案
  - V3配置迁移指南

- **[系统架构文档](architecture.md)**: V3版本系统架构设计
  - 服务器-消费者架构说明
  - 模块职责和依赖关系
  - 数据流和处理流程
  - 程序入口说明

## 开发者文档

面向项目开发者和贡献者：

- **[开发指南](developer-guide.md)**: 开发指南（待创建）
  - 开发环境设置
  - 代码风格指南
  - 如何添加新功能
  - 测试指南
  - 调试技巧

- **[C接口使用文档](c_api.md)**: C接口API使用说明（待创建）
  - C API完整参考
  - 使用示例
  - 错误处理指南
  - 线程安全性说明
  - 与其他语言集成示例

- **[API文档](api/)**: 自动生成的API文档（待创建）
  - 使用Doxygen生成
  - 包含所有公共API的详细说明
  - 本地生成：运行 `doxygen .doxygen-config`
  - 将来由CI自动生成并发布

## 快速链接

- [项目主README](../README.md)
- [C++项目README](../README.md)
- [C++项目中文README](../README_zh.md)
- [配置示例文件](../v3_config_example.json)

## 文档维护

### 文档语言策略

- **中文文档优先**：所有用户文档和开发者文档使用中文
- **代码注释**：使用英文，避免Unicode显示问题
- **API文档**：Doxygen生成，使用英文（Doxygen默认输出）

### 文档更新

- 文档应与代码保持同步
- 重大功能变更时及时更新相关文档
- 发现文档错误请及时修正

### 文档生成

**API文档生成**（本地验证）：
```bash
cd cpp
doxygen .doxygen-config
```
生成的文档位于 `cpp/doc/api/`（或配置指定的目录）。

**注意**：生成的API文档可以删除，将来由CI pipeline自动生成并发布到GitHub Pages。

## 文档版本

- **架构文档**: V3版本
- **配置文档**: V3版本
- **开发指南**: 待创建
- **C接口文档**: 待创建
- **API文档**: 待生成

## 反馈和建议

如发现文档问题或有改进建议，欢迎通过Issue或Pull Request反馈。
