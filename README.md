# ORBSLAM3 Lib Building Repo

This is a more-detailed documentation of the code and the technical implementation of Lynx ORB-SLAM3. We recommand to read this documentation at first to start : https://portal.lynx-r.com/documentation/view/orb-slam-3

This project focuses on building the `libandroidorbslam3.so` library for Android.

To build and use it, simply clone the repo, open the project in Android Studio, and build the library.

# Contacts

For any questions, feel free to reach the Lynx team at system-team+6dof@lynx-r.com

# Summary

- Porting heavy computations of ORB-SLAM3 (https://github.com/UZ-SLAMLab/ORB_SLAM3) to the DSP and HWA for lower latency and higher FPS in SLAM.
- Ported on DSP by Gaston Rouquette (Lynx Mixed Reality).

# Clone & Installation
- Please clone this repo using `git clone --recurse-submodules <GIT URL>` to automatically download the dependencies.
- Open the project in Android Studio to build the project (CPU-Side)
- For the DSP side, open the DSP folder in an IDE and use the different scripts.

## How ORBSLAM3 Works
- Although there are many files in the project, the main steps involving heavy computations on the main thread are:
  - `ORBExtractor.cc` → Extracts features and computes ORB descriptors (Reimplemented on DSP; runs in the "DSP" thread).
  - `ORBMatcher.cc` → Useful reprojection methods (Still on CPU).
  - BFMatcher in `Frame.cc` → Brute-force matching of ORB descriptors (Previously using OpenCV; now on DSP but in the "CPU" thread).
  - Pose estimation in `Optimizer.cc` → Estimates SLAM pose based on observations (Uses Eigen + g2o; on CPU thread; iterations reduced for performance).

## Current Work
- Focus so far: Pipeline from receiving a camera frame to processing on the DSP.
- Uses `AHardwareBuffer` directly from the image to avoid unnecessary copies.
- Key implementation: ORB Extractor on Hexagon DSP and HWA (detailed below).
- BFMatcher also implemented on DSP.
- ORBVocabulary loaded as binary data for faster startup.
- Two threads for parallel processing: One handles extraction on DSP; the other does pose estimation on CPU (requires callback for position feedback).
- Pose optimization modified for fewer iterations.
- Current performance: SLAM runs at 90 FPS, but can be slower if hand-tracking or the Qualcomm Default Slam System is enabled at the same time.
- Implementation of the HWA Pipeline for efficient in-parallel feature extraction.

## How ORB Extractor Works
- Refer to OpenCV docs for basics: [FAST](https://docs.opencv.org/3.4/df/d0c/tutorial_py_fast.html) and [ORB](https://docs.opencv.org/3.4/d1/d89/tutorial_py_orb.html).
- Custom implementation based on this paper (suited for DSP's SIMD architecture): [ACCV 2014 Paper](https://doi.org/10.1007/978-3-319-16628-5_48).

## How DSP Programming Works

### Prepare Qualcomm Tools
- Download the Hexagon NPU SDK: [Qualcomm Developer Site](https://www.qualcomm.com/developer/software/hexagon-npu-sdk).
- Use version 5.5.5.0 (6+ is incompatible with Lynx R1 DSP architecture).
- Lynx R1 DSP is v66. Always verify version-specific docs/tools.
- Key docs (in `C:\Qualcomm\Hexagon_SDK\5.5.5.0\docs\pdf` by default):
  - `80-N2040-44_B_Qualcomm_Hexagon_V66_HVX_Reference_Manual_Programmers.pdf` (Essential).
  - `80-N2040-42_A_Qualcomm_Hexagon_V66_Programmer_Reference_Manual.pdf`.
  - `80-N2040-1786_AF_Hexagon_Simulator_User_Guide.pdf`.

### Simple Explanations of DSP Project Setup
- See the `calculator_c++` Android example in the Hexagon SDK (or existing code in this project).
- Structure: "Skel" (DSP-side code in `dsp` folder) and "stub" (auto-generated for RPC).
- Build skel:
  - Set up environment (e.g., VSCode, not Android Studio).
  - `cd ${Qualcomm Hexagon SDK}/5.5.5.0/`
  - Run `.\setup_sdk_env_power.ps1` (Windows).
  - `cd ...../liborbslam3/dsp`
  - Build: `build_cmake hexagon DSP_ARCH=v66 BUILD=Release` (Add new files to `CMakeLists.txt`). (Please note that hexagon.min is the legacy build system)
  - Sign: `%HEXAGON_SDK_ROOT%\utils\scripts\signer.py -i [input_skel.so]`
  - Please note that Debug allows logging but is much slower than Release version.


- Note: Use `buildAndSignDspLib.bat` to build, sign and copy the dsp lib directly to the service. (Please adapt the script `..\..\jniLibs\arm64-v8a\` part if you want to modify the output location)
- Note: You must sign the lib and the device to run the program since it needs privileged access for hardware accelerators.
- IDL: `orbslam3.idl` defines RPC data transfer; generates stub in Android Studio project.
- Android side: Init memory/handle, call DSP methods. Debug mode may add latency in data transfer.

### Simple DSP Programming Tips
- Code resembles standard C; libc functions are available.
- Use HVX for 1024-bit SIMD operations where possible.
- For images: Align stride to multiples of 128 (stride ≠ width; not convolution stride).
- Allocate vectors with `memalign` for alignment.
- Load vectors from aligned addresses; use `valign` for reconstruction.
- VLIW: Instructions like `A; B;` can run in parallel if independent and resources available.
- Gather "1" indices: Use prefixsum + scatter (allocate VTCM). Retrieve pixels with gather (may need image copy to VTCM).
- Avoid `+`, `+=`, etc., on vectors—use intrinsics for type clarity (logical ops like `&`, `|=` are fine).
- VTCM: Use sparingly; XR2 Gen 1 allows 256Ko alloc in one process only (XR2 Gen 2 may allow 8MB and multiple processes).

# Extraction Hexagon Implementation Explanations

## Main Steps
- Two threads: Parallel extraction on left/right images.

Per thread:
- Image pyramid construction (bilinear reduction).
- HWA Features extraction.
- Calculate angles.
- Compute ORB descriptors.

## Pyramid Structure and Cache Management

### Image Pyramid
- 7 levels for multi-scale tracking.
- Reduction factors ~1.2x–1.35x for accuracy, but optimized for SIMD: Levels 0–6 widths/heights multiples of 128/80.
- Factors: `5 → 4 → 3 → sqrt(3*2) → 2 → 2^(2/3) → 2^(1/3)` (multiplied by 128/80; approximate non-integers).

### Pyramid Image Block Divisions
- Divide due to VTCM limits and per-area thresholds.
- Each block: 128x80.
- Level 0: 5x5=25 blocks; Level 1: 4x4=16; etc.

### VTCM Cache
- Per-block cache: 256x111 pixels (includes neighbors; aligned).
- Layout (non-border; borders pad with black):
  - 64 bytes left | 128 bytes center | 64 bytes right.
- Used for points/positions too.

## HVX Implementations

### Image Pyramid Construction
- Input: Side-by-side images; extract left/right with `calculate_pyramid_image_copy_hvx`.
  - Per line: `l2fetch` next; copy vectors; skip other image.
- Recursive bilinear reduction (factor 1–2):
  - Vertical: Iterate output lines; interpolate floor/ceil input lines.
  - Horizontal: Precompute indices/coefs; gather/multiply/add/store.

### FAST (Pre-)Detection Algorithm (Legacy)
- Per pyramid level/block: Call `calculate_fast_features_hvx` with offsets; manage `l2fetch`.
- Load initial 7 lines; iterate: Shift vectors, load new.
- Per line: Abs diff for positions 1/5/9/13 vs. center; mask/threshold; combine for interest check (cross-checks approximate contiguity).
- If potential: Prefixsum indices; scatter to VTCM.

### Score Algorithm (Legacy)
- Exact score to filter false positives.
- For each pixel: Min/max diffs over 9 pixels; score = max(global_max, global_min).
- SIMD-parallelized with half-word vectors.

### Clean Features (Legacy)
- Filter scores below threshold; scatter valid to temp VTCM; update count.

### Non-Maximal Suppression (Legacy)
- Approximate: Horizontal (check ordered positions/scores); vertical (swap coords, bitonic sort, repeat).

### Sort and Clean Features (Legacy)
- Filter like clean; sort by score; keep top N.

### Gaussian Blur (Legacy)
- 5x5 convolution (Qualcomm-based; custom Gaussian filter).

### Angles Calculation
- Use intensity moments for rotation invariance.
- Per block/vector: Accumulate moments (VectorPair for 32-bit); log/shift for encoded angle; table lookup for sin/cos.

### ORB Descriptors
- Comparisons of rotated surrounding pixels; store as masks.
- Per block/vector: Rotate patterns; gather/compare; store in special vector format (64 features x 16 vectors).

### ORB Descriptors Optimized for Up to 16 Points per Block
- 4x speedup: Pack 4 patterns per vector; preprocess patterns; periodic angles/indices.
- Shift/OR for compatibility.

### Post-Processing
- Scalar reorder before CPU return.

### BFMatcher
- Compute distances for matching (XOR + popcount + sum).
- Outputs: Min dist, second min, indices for triangulation/3D points.

## HWA Implementation
- Uses Qualcomm proprietary Computer Vision Accelerator (prebuilt in repo) for tasks like feature extraction.
- Per level: DSP for pyramid; HWA for features; DSP for descriptors.
- Parallel: HWA on level N while DSP pyramids N+1.
- Dynamic threshold: Targets average features ± deviation for balance.

# Future Works
- Optimize VLIW instructions and vector registers for efficiency.
- Improve algorithm accuracy/fix bugs (e.g., non-max suppression).
- Optimize PnP algorithm.
- Resolve geometry/calibration offsets with Qualcomm SLAM (coordinate systems differ).
- Stabilize pose with fisheye params/algorithms.
- Integrate IMU data.
- Tune frequencies: Increase DSP, decrease GPU.
- Reserve CPU 0 for DSP thread.
- Optimize `l2fetch`.
- Fix segfaults/crashes from threading/corrupted data.

## Known Errors & Limitations 

- Current camera resolution is limited to the Lynx-R1 device for now (some indication on resolution here: https://portal.lynx-r.com/documentation/view/video-capture )
- Windows support only for now (but should be easily adaptable on Linux-based systems)
- Use of proprietary hardware acceleration requires to have rooted device
- NO IMU : Leads to some latency and lose of tracking if moving the head very fast.
- Some crashes could sometimes randomly occur in the library of ORB-SLAM3, specially because of some race conditions that can occur more often after porting the code from x86_64 to ARM64 architecture.