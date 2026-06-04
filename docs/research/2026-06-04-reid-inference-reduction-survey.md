# ReID Inference-Cost Reduction for Tracking & Re-Acquisition: Survey

**Research date:** 2026-06-04
**Scope:** 2019–2026, MOT + selective/sparse ReID, UAV person-following, multi-camera VMS, edge-NPU ReID deployment.
**Context:** Drone single/multi-target follow on Hailo NPU. Detection runs tiled; ByteTracker associates by IoU/motion; a person-ReID embedding model can run on **person crops only** (never full frame, never vehicles). Running ReID on every crop every frame is the quality ceiling but too expensive. We want established patterns for deciding *when* to extract embeddings and *when* to match, to ablate offline against ground truth.

> Our current implementation already does several of these (see `drone_follow/pipeline_adapter/reid_manager.py`): event-driven recovery on tracker loss, drift/duplicate gallery gating, FIFO gallery, raw-detection fallback. This survey grounds those choices in the literature and ranks what to add/ablate next.

---

## 1. Taxonomy of ReID inference-reduction strategies

The mechanisms found cluster into seven families. Most production systems stack 2–4 of them.

| # | Family | Core idea | Saves | Risk |
|---|--------|-----------|-------|------|
| A | **Event-driven / occlusion-triggered ReID** | Only extract & match embeddings when the cheap motion/IoU associator is *uncertain* — track loss, gap, ambiguous match. Pure-IoU otherwise. | Most crops skipped in easy frames | Embedding may be missing exactly when the target is occluded/blurry |
| B | **Ambiguity-gated extraction (per-detection risk test)** | For each detection, run ReID only if it is "risky": 0 or ≥2 confirmed tracks overlap it above an IoU threshold; skip if exactly one clean match. | ~50–80% of crops in benign frames | Tuning θ_IoU; aspect-ratio confusables slip through |
| C | **Shared-backbone / zero-extra-inference embeddings** | Reuse detector feature maps (ROI-pool from a conv layer) as the appearance descriptor — no separate ReID forward pass at all. | The *entire* ReID model | Weak, low-dim descriptor; poor on small/low-texture aerial crops |
| D | **Temporal cadence / decay** | Sample an embedding every N frames per track (not every frame); decay the stored feature's weight between samples; EMA-smooth updates. | (N−1)/N of in-track extractions | Stale gallery if scene changes fast |
| E | **Gallery update policy (drift / duplicate / diversity gating)** | Don't blindly append every embedding. Reject likely-drift samples, skip near-duplicates, keep a *diverse* exemplar set (FIFO or pose/view-diverse), EMA-merge. | Match cost + storage; protects quality | Mis-gating can starve or poison the gallery |
| F | **Motion / geometry candidate pruning before embedding** | Use Kalman gate, optical-flow ROI, or spatial-temporal constraints to shortlist candidates, so ReID is computed/compared against few boxes — or replaced entirely by a cheap cue (color histogram). | Embedding count and/or N×M match matrix | Fast/erratic motion (agile drone) breaks the gate |
| G | **Cheap-embedding compression** | Smaller backbone (OSNet, RepVGG-A0), low input res, INT8/FP16 quantization, reduced embedding dim. Orthogonal to A–F: makes each extraction cheaper rather than rarer. | Per-extraction FLOPs/latency | Accuracy drop; calibration effort |

---

## 2. Per-source notes

### A / B — Selective & ambiguity-gated ReID

**When to Extract ReID Features: A Selective Approach for Improved MOT** — Hassan et al., 2024, arXiv:2409.06617.
Per-detection **risk test**: compute IoU of each detection against all confirmed tracklets; mark the detection **non-risky (skip ReID)** iff *exactly one* tracklet has IoU > θ. Otherwise (0 or ≥2 overlaps) it is **risky** → extract the embedding. Two refinements: an **Aspect-Ratio-Similarity** check forces extraction when the single IoU match has a suspicious shape change, and **feature decay** down-weights stored features during skipped frames. Applied to StrongSORT and Deep OC-SORT on MOT17/MOT20/DanceTrack.
- *Numbers:* MOT17-val StrongSORT 69.54→69.61 HOTA while FPS 0.96→1.73 (**+80% speed, accuracy flat/up**); DanceTrack-val 1.56→2.38 FPS (+52%) at +0.34 HOTA. A more aggressive "FSS 0.2" setting on MOT17-test ran ReID on only **42.7% of detections** for 62.7 vs 63.5 HOTA.
- *Applicability:* **High.** This is the canonical formalization of our intuition. The "exactly one clean IoU match → skip" rule maps directly onto ByteTracker's high-confidence first-association: when a locked/auto target has a single unambiguous IoU continuation, skip the embedding; only spend ReID when the box is contested or unmatched. The aspect-ratio guard is cheap and worth adopting for aerial pose changes.

**ByteTrack** (associate-every-box, appearance-free) — Zhang et al., 2022, ECCV.
Two-stage IoU/Kalman association with **no appearance model at all**; low-score boxes recovered in a second IoU pass. Community guidance (BoxMOT, Datature) crystallized the rule we should encode: **IoU is more robust than ReID under severe occlusion and for low-score boxes**; ReID helps most on **low frame-rate / large inter-frame motion**. Second-stage (low-score) association should *always* be IoU — occluded crops yield bad embeddings.
- *Applicability:* **High / foundational.** This is our tracker. Confirms: don't waste ReID on low-confidence or heavily-occluded boxes (their embeddings are noise). Reserve ReID for the large-motion / re-acquisition regime — exactly the drone case after a yaw slew or occlusion gap.

### C — Shared-backbone / zero-extra-inference embeddings

**LITE: A Paradigm Shift in MOT with Efficient ReID Feature Integration** — Jumabek et al., 2024, arXiv:2409.04187 (Springer NeurIP-W).
Eliminates the separate ReID network: ROI-crops the **first conv layer of YOLOv8** (48-channel, half-res feature map) at each detection box and averages across channels → a 48-D appearance descriptor, computed for free during detection.
- *Numbers:* LITE:DeepSORT on MOT17 43.0 HOTA / 50.1 IDF1 at **28.3 FPS vs DeepSORT 13.7 FPS (2×)**; MOT20 **4×** speedup at comparable HOTA.
- *Applicability:* **Medium, with caution.** Tempting on edge (zero extra NPU passes), but a 48-D shallow-feature descriptor is weak on small, low-texture aerial person crops — the regime where our dedicated ReID earns its keep. Worth an *ablation lower-bound* baseline ("free embedding") but unlikely to replace the real ReID for hard re-acquisition.

**FairMOT** — Zhang et al., IJCV 2021, arXiv:2004.01888.
Joint detection + embedding on a shared anchor-free CenterNet/DLA-34 backbone; one forward pass yields both boxes and a per-center ReID vector. Establishes that detection/ReID feature *fairness* needs an anchor-free, multi-scale backbone.
- *Applicability:* **Low (architectural).** Requires retraining a joint head — not compatible with our fixed Hailo 4-class detector HEF + separate ReID HEF. Useful as background for *why* a naively-shared backbone (LITE) underperforms a purpose-built ReID.

### D / E — Cadence, decay, and gallery update policy

**StrongSORT: Make DeepSORT Great Again** — Du et al., TMM 2023, arXiv:2202.13514.
Replaces DeepSORT's append-only feature **bank** with an **EMA appearance state** per track (smooth update, robust to detection noise), plus motion-distance gating in the cost. The EMA both stabilizes the descriptor against drift and removes the need to store/compare a whole gallery per track.
- *Applicability:* **High.** Directly informs our gallery policy. We use FIFO + drift/duplicate gates; an EMA-merged "anchor" embedding alongside the FIFO exemplars is a cheap, well-validated addition to ablate (EMA for stability vs FIFO diversity for view changes).

**Deep OC-SORT** — Maggiolino et al., ICIP 2023, arXiv:2302.11813.
Adds **adaptive (dynamic) appearance** on top of motion-only OC-SORT: appearance is fused into association with a confidence-weighted scheme and **down-weighted when detection confidence is low**, "adding negligible computation." Camera-Motion-Compensation helps on moving-camera sets (MOT17/DanceTrack) but not static (MOT20).
- *Numbers:* 64.9 / 63.9 HOTA on MOT17 / MOT20.
- *Applicability:* **High.** "Trust appearance less when the detection is weak" is a free, principled gate for our drift logic, and CMC is directly relevant to a yawing/translating drone — confirms motion-compensating the Kalman gate before trusting IoU.

**Self-Supervised Temporal-Coherence ReID** — Pathak et al., 2020, arXiv:2007.11064; and general MOT guidance (Joint Detection & Tracking w/ ID features, 2020). Consensus finding: **integrating ReID over a tracklet (temporal pooling) beats frame-by-frame matching**, and matching can be done at keyframes rather than every frame.
- *Applicability:* **Medium.** Supports cadence sampling (our `--update-interval`) and tracklet-level (rather than per-frame) gallery matching during re-acquisition.

### F — Motion/geometry candidate pruning (UAV-specific)

**SDG-Track: Heterogeneous Observer-Follower Framework for High-Res UAV Tracking on Embedded Platforms** — 2025, arXiv:2512.04883 (Jetson Orin Nano).
Observer/Follower split: a heavy detector runs at **low frequency** for position anchors; a CPU **sparse optical-flow ROI** interpolates trajectory at high frequency in between. Re-acquisition uses a **training-free Dual-Space Recovery** — fused Lab + HSV color histograms + geometric consistency — **instead of a ReID network entirely**.
- *Numbers:* 35.1 FPS, retains 97.2% of frame-by-frame detection precision; tracks agile FPV drones.
- *Applicability:* **High (pattern), Medium (method).** The low-freq-detector / high-freq-interpolation cadence is exactly a budget lever for tiled detection. The color-histogram recovery is a *cheap pre-filter*: shortlist re-acquisition candidates by histogram, then spend ReID only on the top-k — a concrete way to cut the re-acquisition embedding count.

**Multi-Feature ReID-Enhanced Dual Motion Modeling for Multi Small-Object Tracking** — 2025, PMC12473289.
For drone-scale small objects: Kalman + optical-flow dual motion with a **Kalman-guided dynamic ROI** to recover trajectories; ReID is one cue among motion features, not run blindly.
- *Applicability:* **High.** Validates motion-gated, ROI-constrained re-detection for small aerial targets — our predicted-bbox ROI tile feeds the same idea.

### G — Cheap embeddings on edge / NPU

**Hailo Multi-Camera Multi-Person ReID** (engineering write-up) — hailo.ai blog.
YOLOv5s detector + **RepVGG-A0 ReID** (2048-D embedding) on **Hailo-8**; ReID runs **only on cropped person regions**; a GStreamer **gallery-search plugin** adds/searches embeddings across cameras/time.
- *Numbers:* 30 FPS/stream end-to-end on 4× FHD; standalone ReID **1015 FPS**, detection 379 FPS; 90% Rank-1 on Market-1501.
- *Applicability:* **Very high (same vendor/stack).** Confirms crop-only ReID is already the standard Hailo pattern and that ReID is *not* the bottleneck once gated — detection/tiling is. The gallery-search plugin is a reusable building block for our gallery management.

**OSNet: Omni-Scale Feature Learning for ReID** — Zhou et al., ICCV 2019, arXiv:1905.00953.
2.2M-param multi-scale ReID backbone (vs ResNet-50 ~24M) with strong low-resolution performance; OSNet-x0.5 gives compact embeddings ~20 FPS on mobile NPUs.
- *Applicability:* **High.** Strong candidate if we ever swap the ReID HEF — multi-scale design is well-suited to the scale variation of aerial person crops.

**Real-time Person ReID at the Edge: A Mixed-Precision Approach** — Nikouei et al., 2019, arXiv:1908.07842.
MobileNet-v2 + **FP16 mixed precision**: 3.25× throughput (→27.8 FPS) and 1.45× lower power vs FP32 ResNet-50, at −5.6% accuracy.
- *Applicability:* **Medium.** Quantization is already implied by Hailo INT8 HEFs; the takeaway is the *accuracy-cost trade curve* to report when we ablate embedding precision/dim.

---

## 3. Ranked recommendations to implement / ablate (offline, GT available)

Ordered by expected (impact × low-risk × fit to our stack). Our offline harness has ground-truth tracks, so each can be scored against the "ReID-every-crop-every-frame" upper bound.

1. **Ambiguity-gated extraction (Family B, the 2409.06617 risk test).** Replace "extract on cadence + on loss" with per-detection risk: skip ReID when exactly one confirmed track cleanly overlaps the box (IoU > θ) with consistent aspect ratio; extract when 0 or ≥2 overlap. *Ablate θ_IoU and the aspect-ratio guard.* Highest expected crop-count reduction with near-zero quality loss; directly proven.
2. **Confidence-/occlusion-gated trust (Deep OC-SORT + ByteTrack rule).** Never extract ReID on low-score or 2nd-stage (occluded) boxes; down-weight appearance when detection confidence is low. Cheap, removes the worst (noisiest) embeddings. *Ablate the confidence floor.*
3. **EMA anchor alongside FIFO gallery (StrongSORT).** Add one EMA-smoothed anchor embedding to the current FIFO+drift/duplicate gallery; compare drift against the EMA, enrich the FIFO for diversity. *Ablate EMA-α and EMA-vs-FIFO-vs-both.*
4. **Cheap-cue pre-filter for re-acquisition (SDG-Track color histogram).** During track-loss recovery over visible persons, shortlist by Lab/HSV histogram (or detector-feature LITE descriptor) and run the real ReID only on top-k candidates. *Ablate k and pre-filter type.* Cuts the worst-case re-acquisition embedding count.
5. **Cadence + feature decay tuning (Family D).** We already have `--update-interval`; ablate it jointly with a feature-decay weight so a single sample every N frames suffices in-track.
6. **"Free embedding" lower bound (LITE).** Implement the detector-feature descriptor purely as an ablation floor — quantifies how much our dedicated ReID HEF buys us on small aerial crops, justifying its cost.

Strategies 1–2 are gating (run fewer extractions); 3–5 are quality/efficiency of the gallery+recovery; 6 is a baseline. They compose.

---

## 4. Metrics reported by these works (for comparable ablation tables)

Report these so our tables line up with the literature:

- **Tracking quality:** **HOTA** (primary in all recent MOT work — balances detection + association), **IDF1** (identity consistency / association), **MOTA** (detection-dominated), **ID-switches (IDsw)**, and **AssA / DetA** (HOTA's association/detection sub-scores). DanceTrack-style sets stress association; report HOTA there.
- **Re-acquisition specific:** track fragmentation / **Frag**, mostly-tracked / mostly-lost (**MT/ML**), and recovery latency (frames-to-reacquire after a gap) — most relevant to our follow use-case.
- **Cost / efficiency (the whole point):** **FPS / latency** end-to-end; **fraction of detections on which ReID was extracted** (2409.06617's headline lever — e.g. "42.7% coverage at 62.7 HOTA"); embeddings-per-frame; total ReID forward passes per sequence. Report quality *as a function of* extraction fraction — that curve is the deliverable.
- **Edge specifics:** power (W), device, embedding dim & precision (Hailo blog: 2048-D RepVGG-A0; LITE: 48-D; mixed-precision paper: FP16 vs FP32 accuracy delta).
- **ReID accuracy proxy (if we score embeddings directly):** **Rank-1 / mAP** on a held-out crop set (Market-1501 is the common reference; Hailo blog reports 90% Rank-1) — lets us tune the embedding/threshold independently of the tracker.

Our offline harness should produce a single table: *rows = gating strategy, columns = HOTA / IDF1 / IDsw / recovery-latency / ReID-extraction-fraction*, all against the every-crop-every-frame upper bound.

---

## Sources

- [When to Extract ReID Features: A Selective Approach for Improved MOT (2409.06617)](https://arxiv.org/abs/2409.06617)
- [LITE: A Paradigm Shift in MOT with Efficient ReID Feature Integration (2409.04187)](https://arxiv.org/abs/2409.04187)
- [ByteTrack (intro + IoU-vs-ReID guidance, Datature / BoxMOT)](https://datature.io/blog/introduction-to-bytetrack-multi-object-tracking-by-associating-every-detection-box)
- [StrongSORT: Make DeepSORT Great Again (2202.13514)](https://arxiv.org/pdf/2202.13514)
- [Deep OC-SORT: Multi-Pedestrian Tracking by Adaptive Re-Identification (2302.11813)](https://arxiv.org/abs/2302.11813)
- [FairMOT (2004.01888)](https://ar5iv.labs.arxiv.org/html/2004.01888)
- [SDG-Track: Observer-Follower UAV Tracking on Embedded Platforms (2512.04883)](https://arxiv.org/abs/2512.04883)
- [Multi-Feature ReID-Enhanced Dual Motion Modeling for Multi Small-Object Tracking (PMC12473289)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12473289/)
- [Hailo Multi-Camera Multi-Person Re-Identification (Hailo-8)](https://hailo.ai/blog/multi-camera-multi-person-re-identification/)
- [OSNet: Omni-Scale Feature Learning for Person Re-Identification (1905.00953)](https://arxiv.org/pdf/1905.00953)
- [Real-time Person ReID at the Edge: A Mixed Precision Approach (1908.07842)](https://arxiv.org/pdf/1908.07842)
- [Exploiting Temporal Coherence for Self-Supervised Video Re-ID (2007.11064)](https://arxiv.org/pdf/2007.11064)
