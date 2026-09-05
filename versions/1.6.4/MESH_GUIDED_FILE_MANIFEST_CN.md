# 1.2.8 新增与修改文件清单

## 修改

- `__init__.py`：版本号、Blender 5.1 EEVEE 标识、Mesh 引导模块注册与面板挂接。
- `blender_manifest.toml`：版本升为 `1.2.8`。
- `README.md`：标明当前版本、真实基线和新文档入口。
- `IMPLEMENTATION_REPORT_zh.md`：标记为旧版历史报告，避免把 `1.2.0` 误认为当前版本。

## 新增代码

```text
mesh_guided/
├── __init__.py
├── properties.py
├── panel.py
├── operators.py
├── pipeline.py
├── state.py
├── validation.py
├── cameras/
│   ├── camera_export.py
│   ├── camera_selector.py
│   └── coordinate_conversion.py
├── mesh/
│   └── evaluated_mesh.py
├── materials/
│   └── material_parser.py
├── gaussians/
│   ├── surface_sampler.py
│   ├── gaussian_initializer.py
│   └── gaussian_export.py
├── render/
│   ├── gbuffer_renderer.py
│   └── render_queue.py
├── training/
│   ├── config_writer.py
│   ├── environment_check.py
│   └── process_manager.py
└── utils/
    ├── json_io.py
    ├── logging.py
    ├── memory.py
    └── paths.py
```

各子目录包含必要的 `__init__.py`。

## 新增文档

- `CHANGELOG.md`
- `README_MESH_GUIDED_GS_CN.md`
- `DATA_FORMAT_MESH_GUIDED_CN.md`
- `TRAINER_INTERFACE_CN.md`
- `MESH_GUIDED_IMPLEMENTATION_STATUS_CN.md`
- `TEST_RESULTS_MESH_GUIDED_CN.md`
- `MESH_GUIDED_FILE_MANIFEST_CN.md`

## 工作目录测试

测试源码保存在安装包外的 `blender_gs_colmap_exporter_work/tests/`：

- `test_mesh_guided_core.py`
- `blender_mesh_guided_smoke.py`
- `blender_mesh_guided_dataset_smoke.py`
- `blender_triangle_id_smoke.py`
- `run_baseline_regression.py`

最终安装 ZIP 不包含测试输出、`.blend` 临时场景、缓存或 `__pycache__`。

