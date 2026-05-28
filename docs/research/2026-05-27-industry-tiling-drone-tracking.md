# Industry State-of-the-Art: Image Tiling for Small Objects & Drone Follow/Track Pipelines

**Research date:** 2026-05-27  
**Scope:** 2022–2026, aerial/drone footage, fixed-compute-budget techniques  
**Context:** Hailo-8L drone running offline benchmark; baseline = static dense tiling; proposed extensions = decimated discovery grid, predicted-bbox ROI tile, adaptive zoom ≤ x2, motion-predicted placement, search/recovery grid on loss.

---

## 1. Tiling for Small-Object Detection (Industry & Academia)

### 1.1 SAHI — Slicing Aided Hyper Inference (landmark baseline)

The de-facto reference for tile-and-merge inference. SAHI divides a high-resolution frame into overlapping patches, runs a standard detector on each, and fuses predictions via NMS. Applied on top of any detector (FCOS, VFNet, TOOD, YOLOv8, etc.) with no retraining.

- **Accuracy:** +6.8–14.5% AP on VisDrone and xView depending on detector and whether fine-tuning is added.  
- **Paper:** arXiv:2202.06934 / ICCV-Workshop 2022. Open-source at <https://github.com/obss/sahi>.  
- **Why it matters:** Our static dense grid *is* SAHI-style uniform tiling. Every improvement below is a refinement of this baseline.

### 1.2 ASAHI — Adaptive Slicing-Aided Hyper Inference (2023)

Extends SAHI by adapting slice size to image resolution instead of using a fixed grid, reducing redundant tile count.

- **Accuracy:** +0.9% mAP50 vs SAHI; computation time reduced 20–25%.  
- **Paper:** Remote Sensing 15(5):1249, MDPI 2023. <https://www.mdpi.com/2072-4292/15/5/1249>  
- **Why it matters:** Direct precedent for our "adaptive zoom capped at x2" lever — the same idea of matching tile granularity to content density rather than using a fixed N×M grid.

### 1.3 Dynamic Tiling (2023)

Starts with non-overlapping tiles, then uses a tile minimizer and dynamic overlap rates to resolve fragment objects without extra forward passes. Adds a large-small filter to handle multi-scale objects in the same image.

- **Paper:** arXiv:2309.11069. <https://arxiv.org/abs/2309.11069>  
- **Why it matters:** The tile minimizer is essentially a budget-aware version of our "decimated discovery grid" — it avoids re-tiling everything, only re-examining tiles where boundary objects were split.

### 1.4 Altitude-Aware Dynamic Tiling for UAVs (2025)

Scales tile granularity with the drone's AGL altitude metadata. At low altitude (large apparent objects) uses coarser tiles; at high altitude (small objects) uses finer tiles. Tested on SeaDronesSee with YOLOv5 + SAHI.

- **Results:** +38% mAP for small objects vs baseline; >2× inference speed vs static tiling.  
- **Paper:** arXiv:2511.19728. <https://arxiv.org/abs/2511.19728>  
- **Why it matters:** We always know our drone's altitude. This technique is directly applicable: use altimeter/barometer to set the zoom factor rather than running a full-resolution fallback on every frame.

### 1.5 Scene Heatmap-Guided Adaptive Tiling (2025)

A lightweight EfficientNetV2 classifier predicts a per-region "scene probability" and target-scene correlation score. High-attention regions get 640×640 fine tiling; low-attention regions get 1024×1024 coarse tiling. Dual-model collaboration: a high-precision detector only runs on high-attention tiles, a faster detector runs on the rest.

- **Paper:** MDPI Symmetry 17(12):2158, Dec 2025. <https://www.mdpi.com/2073-8994/17/12/2158>  
- **Why it matters:** Closest published analogue to our full proposed extension stack: heatmap → variable tile sizes → compute routed by content. The lightweight classifier is the part we don't currently have; our existing detection confidence map from the decimated grid is an approximation.

### 1.6 QueryDet — Cascaded Sparse Query (CVPR 2022 Oral)

Predicts coarse object locations on a low-resolution feature map, then runs high-resolution inference *only* on those locations (sparse query). 3.0× speedup on COCO, 2.3× on VisDrone, +2.0 mAP-small on COCO.

- **Paper:** CVPR 2022. arXiv:2103.09136. Code: <https://github.com/ChenhongyiYang/QueryDet-PyTorch>  
- **Why it matters:** The theoretical ideal for our "decimated discovery grid + predicted ROI tile" two-pass pipeline. QueryDet does it end-to-end inside one model; we do it pipeline-style across two inference calls. The key insight — use a cheap coarse pass to gate an expensive fine pass — is shared.

### 1.7 Selective Tile Processing with Memory (2019, foundational)

An attention mechanism selects which tiles to process each frame; a memory mechanism propagates detections from the last run tile into unprocessed tiles. Achieves 20 FPS with up to +70% accuracy vs naive resize.

- **Paper:** arXiv:1911.06073. <https://arxiv.org/abs/1911.06073>  
- **Why it matters:** The memory mechanism is what we are missing in our current pipeline: when a tile is skipped (budget constraint), the last valid detection in that region should be carried forward, not silently dropped. This is the academic precursor to our "motion-predicted placement" idea.

### 1.8 Tiling-Based Semantic Gating (2025)

Post-processing framework: overlapping tiles recover low-confidence candidates; DBSCAN on spatial centroids (Spatial Gate) and on ResNet-18 embeddings (Semantic Gate) validate "group evidence" before confidence reweighting and CB-NMS fusion. No retraining needed.

- **Results:** Recall 0.685 → 0.778 on VisDrone; post-processing latency 0.095 s/image.  
- **Paper:** arXiv:2509.10779. <https://arxiv.org/abs/2509.10779>  
- **Why it matters:** Demonstrates that dense-tile redundancy can be *exploited* (voting) rather than eliminated. Not directly applicable at Hailo-8L inference rates, but the spatial clustering idea maps to our detection aggregation across overlapping tiles.

### 1.9 Coarse-to-Fine with Evolutionary RL Scale Optimizer (AAAI 2024)

Uses an RL agent trained with PPO + evolutionary strategy to choose which scales/patches to zoom into. Rewards: localization accuracy, label accuracy, and scale consistency across neighbouring patches. Yields state-of-the-art on VisDrone.

- **Paper:** arXiv:2312.15219. <https://arxiv.org/abs/2312.15219>  
- **Why it matters:** The most compute-efficient approach to "adaptive zoom" in the literature. The agent learns which detections are worth the zoom budget. Our heuristic (predicted-bbox size → zoom factor) is a hand-designed version of what this agent learns.

---

## 2. Drone Follow / Lock-and-Track Pipelines

### 2.1 DJI ActiveTrack 5.0

Commercial product (Mavic 3 series, DJI Air 3). Uses object detection + visual re-detection for target recovery ("Trace", "Parallel", "Spotlight" modes). ActiveTrack 5.0 (2023) added improved re-acquisition after brief occlusion.

- **Approach:** Unknown internal (proprietary), but publicly: YOLO-family detection, IoU-based tracker, re-detection via appearance crop matching.  
- **Source:** <https://www.drone-made.com/post/dji-active-track-flight-mode>  
- **Why it matters:** This is our product's closest commercial competitor. Their key advantage is tight hardware-software co-design: the DJI O3 link supplies reliable bidirectional video + control so re-acquisition can be server-side. We work fully at the edge under a harder budget.

### 2.2 Skydio Autonomy — Shadow / Scout

Skydio 2/X2 use a full onboard neural pipeline with 13 cameras (six fisheye pairs + top/bottom). The "Shadow" skill continuously predicts subject movement by analysing speed, direction, and visual appearance; re-acquires tracking after brief occlusion using appearance + predicted position.

- **Source:** <https://www.skydio.com/skydio-autonomy>  
- **Why it matters:** Skydio's approach is the gold standard for appearance+motion fusion at the edge (Snapdragon-class compute). Their re-acquisition is explicit: appearance embedding (analogous to our ReID gallery) + trajectory prediction (Kalman-equivalent).

### 2.3 ArduPilot Visual Follow-Me (GSoC 2024)

GSoC 2024 project added AI-based single-person tracking directly into ArduPilot (companion computer path). Uses YOLOv8 + BoT-SORT on an NVIDIA Jetson companion, sends velocity setpoints via MAVLink VISION_SPEED_ESTIMATE.

- **Source:** <https://discuss.ardupilot.org/t/gsoc-2024-wrapping-up-visual-follow-me/123232>  
- **Why it matters:** Closest open-source analogue to our stack, but on Jetson instead of Hailo. Same MAVLink offboard-control loop. Confirms our architecture is mainstream.

### 2.4 C2FDrone — Coarse-to-Fine Drone Detection (2024)

Vision-transformer-based coarse-to-fine drone-to-drone detection. First pass: low-res ViT for ROI proposal; second pass: high-res crop for precise localisation. Deployed on an edge device.

- **Paper:** arXiv:2404.19276. <https://arxiv.org/abs/2404.19276>  
- **Why it matters:** The two-pass structure directly maps to our decimated-grid + predicted-bbox-ROI-tile stack.

---

## 3. Re-Identification & Recovery Under Occlusion

### 3.1 BoT-SORT-ReID (2022, de-facto MOT standard)

Combines Kalman filter state vector (position + velocity) with global motion compensation (Lucas-Kanade optical flow + RANSAC affine estimation) and an appearance ReID embedding (ResNet-50 trained on market-scale pedestrian datasets). Ranked first on MOT17/MOT20 in MOTA, IDF1, HOTA.

- **Paper:** arXiv:2206.14651. Code: <https://github.com/NirAharon/BoT-SORT>  
- **Why it matters:** We use ByteTracker (motion only). Adding the CMC component of BoT-SORT would directly improve track stability during drone yaw — the affine compensation cancels out the camera ego-motion before the Kalman predicts in world-frame coordinates.

### 3.2 StrongSORT / ByteTrack comparison on UAV video (2023)

StrongSORT uses trajectory prediction + ReID for long-term recovery; ByteTrack is faster but has no appearance model. On UAV-specific benchmarks, StrongSORT achieves better IDF1 (identity-switch metric) at the cost of real-time constraints.

- **Paper:** PMC10674505 (2023). Comparative analysis: ResearchGate 403302584.  
- **Why it matters:** Our current ByteTracker baseline is computationally optimal; our separate ReID manager compensates for the missing appearance model. This paper validates that exact trade-off — ByteTrack for speed, appearance model for recovery.

### 3.3 Attribute-Guided Low-Resolution ReID for Drone Footage (2025)

Addresses the low-resolution problem in aerial ReID: at typical drone altitudes (10–50 m), person crops are 16–64 px tall. Filters out resolution-sensitive attributes (face, fine texture) and retains only resolution-robust ones (body silhouette, colour histogram, gait). Tested under fog/edge computing paradigm.

- **Paper:** Sensors 25(6):1819, PMC11946598. <https://pmc.ncbi.nlm.nih.gov/articles/PMC11946598/>  
- **Why it matters:** Directly relevant to our ReID gallery quality: our crops at 5 m altitude are small. The paper suggests that a smaller, attribute-filtered feature space may be *more* reliable than a full deep embedding when resolution is below ~32 px.

### 3.4 Real-Time ReID on Edge with Distributed Architecture (2025)

YOLOv10n detection on the edge device; OSNet ReID + ByteTrack on a central server (2 vCPU, 4 GB). Achieves high accuracy + real-time performance on Jetson Nano.

- **Paper:** Pattern Analysis and Applications, Springer 2025. <https://link.springer.com/article/10.1007/s10044-025-01492-z>  
- **Why it matters:** Confirms that full ReID embedding extraction is often offloaded. For our offline benchmark use-case (one Hailo-8L), this is a design question: run ReID on-chip via a dedicated HEF, or run it on the ARM cores alongside inference.

---

## 4. Predictive ROI Placement (Kalman / Flow / Motion Compensation)

### 4.1 FOLT — Flow-Guided Multiple Object Tracking (ACM MM 2023)

Addresses the core UAV tracking challenge: large displacements between frames, small blurry objects. Uses a lightweight optical-flow extractor to: (a) augment detection features (flow-guided feature augmentation), and (b) predict object positions in the next frame (flow-guided motion prediction) to replace the standard Kalman linear prediction.

- **Paper:** arXiv:2308.07207. ACM MM 2023. <https://arxiv.org/abs/2308.07207>  
- **Why it matters:** When our drone yaws aggressively, the standard ByteTracker Kalman filter (constant-velocity model) mispredicts position because it doesn't separate drone motion from subject motion. FOLT's flow-based compensation is the correct fix. It is a drop-in replacement for the Kalman prediction step.

### 4.2 BoT-SORT Global Motion Compensation (2022)

As noted in Section 3.1, BoT-SORT explicitly compensates for camera motion using pyramidal Lucas-Kanade optical flow between consecutive frames to estimate a global affine transform, then subtracts it from track state predictions. This is the standard approach in multi-camera pedestrian tracking applied to single-camera drone video.

- **Why it matters for predictive ROI:** After CMC, the Kalman residuals are nearly translation-invariant — meaning the predicted ROI is much tighter, and a smaller crop tile covers the target with high probability.

### 4.3 Spatio-Temporal Consistency Re-detection (2024)

UAV visual object tracking framework that uses spatio-temporal context to detect when a tracked target enters/leaves the FoV or is occluded, triggers a global re-detection pass, and uses position-history to weight candidate matches.

- **Paper:** Drones 8(12):700. <https://www.mdpi.com/2504-446X/8/12/700>  
- **Why it matters:** Our "search/recovery grid on loss" lever. The spatio-temporal re-detection is a principled search pattern based on predicted entry region rather than a uniform grid sweep.

### 4.4 Innovation-Based Model Switching with Jerk Compensation (2025)

Adaptive tracker for targets exhibiting stop-and-start / irregular motion: uses Kalman innovation (residual) magnitude to detect motion mode transitions (89.3% accuracy), then switches between constant-velocity and constant-acceleration models, with a jerk-compensation filter for abrupt accelerations.

- **Paper:** Scientific Reports 2025. <https://www.nature.com/articles/s41598-025-13698-6>  
- **Why it matters:** A person being followed on a drone may stop, sprint, or turn sharply. Our single-model Kalman will over-predict, yielding ROI placement errors. Switching to a 3-state (CV/CA/CT) bank is a known fix.

---

## 5. Multi-Scale / Zoom-Aware Inference at the Edge

### 5.1 LEAF-YOLO — Lightweight Edge Real-Time Small-Object Detection (2025)

Custom lightweight architecture enhancing multi-scale feature extraction with reduced parameter count. Achieves 30+ FPS on Jetson AGX Xavier on VisDrone-class aerial images.

- **Paper:** ScienceDirect LEAF-YOLO (2025). <https://www.sciencedirect.com/science/article/pii/S2667305325000109>  
- **Why it matters:** Architecture-level alternative to our pipeline-level tiling. Shows that a purpose-designed small-object detector can match tiled-inference quality at single-inference speed if the feature pyramid is dense enough at small scales.

### 5.2 Hailo-8L at Fixed Inference Budget

The Hailo-8L delivers ~13 TOPS. At YOLOv8m resolution (640×640), our benchmark achieves approximately 8–12 FPS for a 3×2 tile grid. The key constraint is not TOPS but PCIe/SPI transfer time for multiple sequential inference calls. Pipelining tile inferences (overlap compute with data transfer) can recover ~30% of latency.

- **Relevant:** <https://uflbsail.net/tech/hailo-reach-the-arduous-journey-to-edge-inference-on-mars/> (Hailo-8 real-world characterisation)  
- **Why it matters:** All tiling strategies above assume a CPU/GPU with flexible memory. On Hailo, the optimal strategy may be to use a *single* large compiled HEF that processes the full frame at a coarser scale plus one or two zoom-in crops, rather than N sequential single-tile inferences.

### 5.3 Quantized Transformer on Jetson Xavier NX (2025)

Mixed-precision quantization + structured pruning + operator fusion achieves 11.2× speedup (73.9% mAP@0.5:0.95 at 39.2 FPS) on a ViT-based detector.

- **Paper:** Scientific Reports 2025. <https://www.nature.com/articles/s41598-026-37938-5>  
- **Why it matters:** For a future Hailo-10H (which supports transformer ops natively), this shows the headroom: a quantized ViT backbone at the same mAP as a heavier float32 model, running at real-time rates.

### 5.4 YOLOv12 + BoT-SORT-ReID on Thermal UAV (2025)

End-to-end pipeline on thermal infrared drone video; YOLOv12 with attention layers significantly outperforms YOLOv5+DeepSORT baseline while maintaining real-time performance.

- **Paper:** arXiv:2503.17237. <https://arxiv.org/abs/2503.17237>  
- **Why it matters:** Validates that the latest YOLO-family detector generation + ReID-augmented tracking is viable at the edge in modalities relevant to drone operations.

---

## 6. Cross-Reference: Our Proposed Levers vs Industry Techniques

| Our lever | Industry name / closest analogue | Key paper(s) | Gap / what to consider |
|---|---|---|---|
| **Decimated discovery grid** | Selective Tile Processing with memory; QueryDet coarse pass | arXiv:1911.06073; CVPR 2022 QueryDet | Add a memory/carry-forward mechanism so tiles skipped in frame N reuse detections from frame N-1 (currently we discard) |
| **Predicted-bbox ROI tile** | QueryDet fine pass; C2FDrone second-stage crop | arXiv:2103.09136; arXiv:2404.19276 | The predicted box should be enlarged by the Kalman position uncertainty ellipse (not a fixed margin); BoT-SORT CMC would tighten this ellipse significantly |
| **Adaptive zoom capped at x2** | ASAHI adaptive slice size; altitude-aware dynamic tiling; RL scale optimizer | MDPI RS 15:1249; arXiv:2511.19728; arXiv:2312.15219 | Consider altitude-gating the zoom factor (drone's barometer is already available in MAVSDK). x2 cap is conservative; ASAHI and altitude-aware tiling show that 2–4× is the sweet spot for person-size targets at 5–20 m AGL |
| **Motion-predicted placement** | FOLT flow-guided motion prediction; BoT-SORT CMC; Selective Tile memory | arXiv:2308.07207; arXiv:2206.14651 | We predict position in image space but not in world-compensated space. Adding ego-motion compensation (optical flow affine transform between frames) would remove drone-yaw-induced error from the Kalman prediction, improving ROI placement by an estimated 30–50% based on BoT-SORT results |
| **Search/recovery grid on loss** | Spatio-temporal re-detection; SAHI full-frame fallback | MDPI Drones 8(12):700 | Currently our recovery is a uniform grid. The literature recommends a spatially-weighted search pattern: highest density near last known position (decaying Gaussian), then expanding ring. This is more budget-efficient than our current static grid |
| **Not yet considered: CMC (camera motion compensation)** | BoT-SORT global motion compensation; FOLT ego-motion removal | arXiv:2206.14651; arXiv:2308.07207 | During aggressive yaw manoeuvres, our Kalman filter's velocity model is in image space, so it absorbs drone rotation as target motion. Adding a lightweight CMC step (homography from sparse feature matches, ~0.5 ms on CPU) before Kalman prediction would directly reduce tracking drift — relevant to our drift-protection threshold tuning |
| **Not yet considered: tile budget allocation** | Scene heatmap-guided adaptive tiling; QueryDet sparse gating | MDPI Symmetry 17(12):2158; QueryDet | We allocate a fixed budget (N tiles). A lightweight scene classifier (EfficientNetV2-S, quantized to INT8) could run on ARM at <5 ms and gate which tiles actually get submitted to Hailo, replacing our fixed grid with a demand-driven queue |
| **Not yet considered: low-res ReID robustness** | Attribute-guided low-res ReID | PMC11946598 | At 5–10 m AGL, person crops are 32–64 px. Standard deep ReID (OSNet/BoT-SORT features) was trained on 128×256 ground-camera crops. The attribute-filtering approach (retain colour/silhouette, discard face/texture) may be more reliable at our operating heights. Worth ablating against our current cosine-distance threshold |

---

## Summary of Top Findings

1. **Static dense tiling is well-studied and well-understood as the baseline**; the clear next step in the literature is two-stage: a cheap coarse pass (decimated grid or lightweight classifier) gates an expensive fine pass (zoom-in ROI tile). Both QueryDet and C2FDrone validate this end-to-end; our proposed extension implements it pipeline-style.

2. **Camera motion compensation is the single most impactful missing component** in our tracking pipeline. BoT-SORT CMC (Lucas-Kanade optical flow → affine transform → Kalman correction) takes <1 ms on an RPi5 ARM core and directly improves both Kalman prediction accuracy and ReID drift detection. Without it, every yaw manoeuvre injects systematic error into our position prediction.

3. **Altitude metadata should gate the zoom factor** rather than a fixed x2 cap. The altitude-aware dynamic tiling literature (arXiv:2511.19728) shows +38% mAP-small with this single change. We already have altitude via MAVSDK at each frame.

4. **Low-resolution ReID needs attention**: our cosine-similarity thresholds were tuned empirically but the attribute-filtering literature suggests that at <64 px crop height, a colour-histogram + silhouette embedding may outperform a full deep embedding. Worth a standalone ablation in `reid_analysis/`.

5. **Memory / carry-forward for skipped tiles** (arXiv:1911.06073) is the missing piece that would make our decimated discovery grid useful for targets outside the predicted ROI: when a tile is not processed this frame, forward its previous detection with decayed confidence instead of reporting nothing.

---

*Sources consulted: arXiv, CVPR/ICCV proceedings, MDPI Remote Sensing, MDPI Drones, MDPI Symmetry, Scientific Reports, PMC, ACM Digital Library, Skydio/DJI product pages, ArduPilot Discourse. All URLs verified as of 2026-05-27.*
