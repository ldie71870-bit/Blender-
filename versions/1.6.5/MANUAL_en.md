# Gaussian Splat COLMAP Dataset Generator Manual

## Purpose

This Blender 5.1 add-on automates dataset generation for Gaussian Splatting. It mirrors the core workflow of Unreal to Gaussian Splat - Automated COLMAP Dataset Generator style UE5 tools: place many virtual cameras in a controlled 3D scene, render training images, and export a COLMAP-compatible sparse model with known camera intrinsics and poses.

The add-on does not train Gaussian Splatting models and does not run COLMAP feature matching. It writes COLMAP text files directly from Blender camera transforms, which is ideal for synthetic data, product renders, digital assets, interiors, virtual scans, and controlled training sets.

## UE5 Feature Interpretation and Blender Mapping

Comparable UE5 plugins usually solve four problems:

1. Automated viewpoints  
   UE tools place CineCamera actors around a target, along paths, or inside capture volumes. This add-on provides sphere, hemisphere, cylinder, half-cylinder, path curve, volume grid, and existing-camera modes.

2. Automated image rendering  
   UE tools batch-render images through Movie Render Queue or internal render APIs. This add-on uses Blender's Cycles path tracer by default for training images and writes PNG or JPEG images into `images/`. Incremental rendering skips existing frames.

3. COLMAP dataset export  
   UE tools export `cameras.txt`, `images.txt`, and `points3D.txt` so 3DGS training tools can read known poses. This add-on writes the same `sparse/0/` structure and also exports `transforms.json` for Nerfstudio or Instant-NGP style pipelines.

4. Auxiliary training data  
   UE tools often include depth, masks, CustomDepth-style isolation, reports, and quality checks. This add-on supports optional OpenEXR depth, black-white masks, sparse point sampling from scene ray casts, and `dataset_report.json`.

## Installation

1. Open `Edit > Preferences > Add-ons` in Blender 5.1.
2. Click `Install from Disk...`.
3. Select `blender_gs_colmap_exporter.zip`.
4. Enable `Render: Gaussian Splat COLMAP Dataset Generator`.
5. In the 3D View, press `N` to open the right sidebar, then open the `GS` tab.
6. If the 3D sidebar is not visible, open the `GS Dataset` panel in the Render Properties editor instead.

You can also drag the zip file into Blender and follow the installation prompt. When installing an updated build, disable the old add-on first, then install the new zip, or restart Blender and enable it again.

## Quick Start

1. Put the target model in the scene.
2. Place the 3D cursor at the capture center, or assign an Empty/Object as the look-at target.
3. Choose an empty output directory.
4. Select `Product Hemisphere`, `Product Sphere`, or scientific path planning.
5. Set the camera count, for example 120.
6. Confirm that `Renderer` is set to `Cycles`, then choose Cycles samples and denoising.
7. Click `Create Camera Rig`, select one path, then use `Single Path Camera Inspection`.
8. Click `Render Dataset`.

The resulting dataset has this layout:

```text
your_dataset/
  images/
    frame_0001.png
    frame_0002.png
  sparse/
    0/
      cameras.txt
      images.txt
      points3D.txt
  transforms.json
  dataset_report.json
```

If depth and masks are enabled, these folders are also written:

```text
your_dataset/
  depth/
    frame_0001.exr
  masks/
    frame_0001.png
```

## Camera Rig Modes

Product Sphere: full spherical coverage around a target.

Product Hemisphere: upper hemisphere coverage for tabletop products, vehicles, characters, and sculptures.

Product Cylinder: multi-ring orbit capture for architecture, interiors, or elongated assets.

Half Cylinder: one-sided capture for walls, stages, shop windows, or inaccessible scenes.

Path Curve: samples camera positions evenly along curve length for custom fly-throughs. It supports either a fixed camera count or an automatic count computed from path density. Path mode does not require a look-at target.

Volume Grid: places cameras inside a box, useful for rooms, corridors, and internal scans. An exclusion collection can prevent cameras from being placed inside obstacle bounding boxes.

Existing Cameras: uses cameras already present in the scene or in the `GS_COLMAP_Cameras` collection.

## Lightweight Single Path Camera Inspection

Select a path curve in the 3D View, then click `Isolate Selected Path and Inspect Cameras` in the N-panel or press `Alt + Shift + I`.

The full scene remains visible. The mode hides only the other curves in the configured path collection and the large native wireframes of generated cameras. The selected path is temporarily drawn in front of scene geometry.

Cyan GPU points mark optical centres and short orange lines show local `-Z` lens directions. Marker size, metric ray length, and x-ray display are adjustable. Use `Refresh` after moving cameras. Press the same shortcut or click `Exit and Restore` to restore all previous visibility states.

This overlay creates no Blender objects, meshes, or materials. The former green pyramid feature has been removed, and legacy `GS_CAMERA_MESH_STYLE` data is cleaned when files are loaded.

## Coverage Patch

Coverage Patch adds a local second-pass capture after the original planning and render. It never moves or renumbers the original cameras.

Choose selected mesh objects, an oriented Mesh/Empty bounds object, or automatic under-observed detection. The add-on recomputes observation counts, direction diversity and overlap, generates deterministic safe local candidates, prefilters at 4 x 4 rays, and fully checks only the strongest candidates.

Use `Generate Preview`, optionally delete individual cameras in `PatchCameras_Preview`, then `Apply Patch`. Final cameras use `cam_patch_*` names and `frame_patch_*` image stems. `Render Patch Cameras Only` renders only those views and merges COLMAP, `transforms.json`, `dataset_report.json`, and `patch_manifest.json`. The original resume checkpoint is unchanged.

## Key Settings

Language: switches between Chinese and English UI text. The Chinese text is meaning-based manual translation, not literal machine output.

Output Directory: dataset root folder.

Camera Count: number of viewpoints. Gaussian Splatting datasets often start around 80 to 300 views; complex scenes may need more.

Radius: distance from camera to target center.

Focal Length: camera lens in millimeters. The add-on converts Blender camera settings to COLMAP PINHOLE intrinsics.

Camera Model: supports perspective, panoramic equirectangular, fisheye equidistant, and fisheye equisolid cameras. Perspective exports standard COLMAP PINHOLE data; fisheye exports OPENCV_FISHEYE parameters; equirectangular writes panorama metadata. Standard COLMAP may not import the equirectangular model directly, but `transforms.json` and rendered images can be used by training pipelines that support panoramic input.

Panorama FOV: field of view for fisheye panoramic modes, from 1 to 360 degrees. Equirectangular panorama covers 360 degrees by design.

Live Camera Update: when enabled, changing camera count, radius, height, rings, camera model, path density, curve detail, and related rig parameters immediately rebuilds the `GS_COLMAP_Cameras` collection. Disable it if you prefer to refresh manually with `Create Camera Rig`.

Path Aim: in Path Curve mode, choose `Look at Target`, `Follow Curve`, or `Curve Rotation`. Only `Look at Target` requires a target. `Follow Curve` aims along the curve tangent, while `Curve Rotation` uses the path curve object's world rotation.

Path Count Mode: in Path Curve mode, choose `Fixed Count` or `Density`. Fixed Count uses the camera count slider; Density computes the count from curve length.

Path Density: camera quantity density in cameras per Blender unit. The actual count is approximately `curve length * path density`, capped by Max Path Cameras.

Curve Detail: controls how finely Bezier curves are approximated for length measurement and sampling. Higher values improve placement fidelity but make live rebuilds a little slower.

Renderer: Cycles is the default. Training images should usually be rendered with Cycles because it gives more stable global illumination, material, and shadow results. Switch to EEVEE only for realtime-style output or quick previews.

Cycles Samples: sample count per rendered image. Use 16 to 64 for previews; start around 128 for final training images; use 256 or higher for difficult lighting.

Cycles Denoise: reduces render noise. It is usually recommended for Gaussian Splatting training images. Disable it only if you intentionally want raw path-tracing noise.

Cycles Device: `Auto` keeps Blender's current setting; `GPU` requests GPU rendering; `CPU` requests CPU rendering. Actual GPU use also depends on Blender's Cycles compute device preferences.

Cycles Backend: keep `Auto`, or explicitly choose `HIP` (AMD), `CUDA`/`OptiX` (NVIDIA), `oneAPI` (Intel), or `Metal` (Apple). Choosing a concrete backend forces GPU selection; when it is unavailable the add-on clearly falls back to CPU instead of silently switching to another GPU backend.

HIP Memory-safe Mode: enabled by default. HIP uses short background batches (1 frame per Blender worker by default), disables cross-frame Persistent Data, and releases Render Result after every frame. If a worker runs out of memory, the supervisor halves the batch size and retries. Increase `HIP Frames per Process` only after confirming that the GPU has enough headroom.

HIP OOM CPU Fallback: enabled by default. If a single frame cannot allocate its Cycles buffers on HIP, the add-on releases the buffers and retries that frame on CPU. If CPU also fails, the original error is preserved and no incomplete image is accepted.

Cycles Persistent Data: enabled by default outside HIP memory-safe mode to reuse Cycles BVH, textures, and render data inside one chunk. HIP memory-safe mode overrides it to disabled. Every chunk still exits its worker process and releases all caches.

Background Render: enabled by default. A lightweight supervisor launches a fresh scene-loading Blender worker for each chunk, 500 frames by default for balanced throughput. `Frames per Process` is configurable. Each chunk verifies its outputs before the worker exits and releases RAM, VRAM, Cycles BVH, texture caches, and denoiser state.

Cancel Background Render: terminates the supervisor and current worker process tree while retaining atomically committed frames.

Background Job Files: files live in the system temporary directory under `gs_colmap_jobs/<output-path-hash>/`. `background_render.log` aggregates the job, `chunk_*.log` stores each attempt, and `chunk_history.json` records ranges, PIDs, retries, peak RAM/VRAM, and detected OOM/CUDA/OptiX/device errors.

Synchronous Fallback: disable `Background Render` if you want to use the original synchronous render path. Synchronous rendering uses Blender's main thread and will make the foreground UI stall during rendering.

Incremental: cross-checks checkpoint state against valid RGB/Depth/ID/object files, skips complete frames even when the checkpoint was lost, and resumes directly at the first missing or corrupt frame. A failed chunk is retried once.

Point Samples/View: ray-casts sample rays from each camera into the scene and writes hit points into `points3D.txt`. Set to 0 to disable.

Point Dedup Size: larger values create a sparser cloud; smaller values preserve more points.

Depth EXR: writes OpenEXR depth maps.

Masks: renders a selected collection as white and all other meshes as black.

## Training Compatibility

COLMAP text files are written to `sparse/0/`; rendered images are written to `images/`. Many Gaussian Splatting tools can read this layout directly.

If your training tool requires binary COLMAP files, install COLMAP and run:

```bash
colmap model_converter --input_path sparse/0 --output_path sparse/0 --output_type BIN
```

If your training tool requires real SfM feature tracks, use this add-on to render images, then run COLMAP matching separately. If the tool accepts known poses, the exported text model can be used directly.

## Notes

- The add-on uses Cycles by default for training images; quality depends on lighting, materials, Cycles samples, denoising, and device settings.
- Dataset rendering uses a background Blender process by default, while the foreground panel reads a progress file.
- The add-on exports the PINHOLE camera model and does not write distortion parameters.
- If you use equirectangular panorama cameras, make sure your Gaussian Splatting trainer supports panorama images or can read the panorama metadata in `transforms.json`.
- All cameras share one intrinsic model. If you use existing cameras, keep lens and resolution consistent.
- Mask rendering temporarily swaps mesh materials and restores them afterwards.
- Depth rendering temporarily adds compositor nodes and removes them afterwards.
- For large datasets, test 10 to 20 low-resolution views first, then render the full set.
