# 1.6.26：自动路径兼容与快速 sparse 点云导出

修复自动路径创建报错：`module '...contour_jobs' has no attribute 'fraction'`。

Blender 扩展更新时，旧的自动路径 modal 和 `contour_jobs` 子模块可能被保留在内存中。旧 modal 仍调用 `status/fraction`，而被保留的子模块可能缺少其中之一，导致后台任务尚未完成就被主界面中止。

新版继续使用独立的进度文件读取逻辑，同时在加载时只为缺失的旧接口补充兼容实现。旧 modal、现行 modal 与正在运行的后台任务可以共存；缺失或损坏的进度文件只保留上次显示，不会取消路径生成。真实生成、导入、取消和失败仍保留原有清理逻辑。

本包包含 1.6.25 的相机避障修复和此前的路径进度兼容修复。

## 稀疏点云导出

- 稀疏点云采样现在为一次导出共享一份只读场景 BVH，不再让每条相机射线重新走 Blender 场景查询。
- 后台点云进度按相机真实显示 `Sampling sparse point cloud: N/总数`，不会再把采样阶段固定显示为渲染完成数。
- 精简模式新增“仅生成 sparse 点云”。它保留已有 RGB 图像，只写 `sparse/0/cameras.txt`、`images.txt`、`points3D.txt`、`transforms.json` 与报告，不会重新渲染图片。

## 验证

- 可用的 Blender 5.0 后台环境：模拟保留的旧 `contour_jobs` 模块缺少 `status/fraction`，重新加载后旧接口被恢复，并读取相同进度文件。
- 自动路径后台生成、结果导入、取消、真实失败和进度文件损坏回归通过。
- Blender 5.0：稀疏点云导出测试通过。3 个视角只构建 1 次场景 BVH，写出了 COLMAP 三个文本文件和 `transforms.json`。
