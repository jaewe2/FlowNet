# Priority-Based Data Preservation with Variable Packet Sizes
**A Size-Aware Maximum Weighted Flow Approach**  
Author: Jason Roe | Advisor: Professor Bin Tang, California State University Dominguez Hills  
Course: CSC 590 — M.S. Project

---

## Overview

This project extends the Maximum Weighted Flow (MWF-U) framework from Rivera & Tang (2024) to support packets of different sizes across data generator nodes. The core contribution is a new class of size-aware algorithms that maximize total preserved data priority in base station-less sensor networks (BSNs) under storage and energy constraints.

In the original paper, all data packets are assumed to be unit-sized, allowing Algorithm 3 (GOA) to be proven optimal. This project removes that assumption and studies what happens when packets from different source nodes have heterogeneous storage requirements.

**Key results:**
- GOA is provably suboptimal when packet sizes vary (counterexample: GOA=20, optimal=42)
- Five new heuristic algorithms designed, from simple density sorting to prefix enumeration
- Hybrid GOA and PSB-GOA reach 99.5–100% of optimal on 10-node networks
- DDR+-GOA (contention-aware extension) improves on DDR-GOA in sparse and heavily fragmented networks
- Dual Branch & Bound exact solver correctly bounds all heuristics with zero violations
- Codebase refactored from a 3,500-line monolith into 19 focused Java files

---

## Files

| File | Description |
|------|-------------|
| **Core Flow Network** | |
| `FlowNetwork.java` | BFN state, buildCFN, bfsFAP, augmentPath, computeDelta, snapCap/restoreFlow, computeReachablePerSink |
| `AugmentationEngine.java` | Shared augmentation helpers — augmentBestFit, runPSBTail (used by Hybrid and PSB) |
| **Algorithms** | |
| `GOA.java` | Greedy Optimal Algorithm — sort by priority, BFS augmentation (Algorithm 3 from paper) |
| `DensityGOA.java` | Sort by priority ÷ size (value density), BFS augmentation |
| `HybridGOA.java` | Prefix enumeration (κ=2) + density rollout + best-fit storage selection |
| `DDRGOA.java` | Dynamic Density Reordering — re-ranks DGs after each full augmentation step |
| `DDRPlusGOA.java` | Contention-aware DDR — shares reachable storage proportionally across competing DGs using √contention dampening |
| `PSBGOA.java` | Per-Step Best-path scoring — scores every (DG, path) pair at each step, interleaves packets |
| `ExactSolverNew.java` | Dual Branch & Bound — best-fit pass + BFS pass, shared global best |
| `ExactSolver.java` | Legacy exact solver (single-strategy, retained for reference) |
| **Experiment & I/O** | |
| `SimulationRunner.java` | Scaling loop, active-trial tracking, violation warnings, fallback `*` markers, per-algorithm win/loss counts |
| `TraceLogger.java` | Per-packet relay trace output with node type annotations |
| `AlgorithmResult.java` | Value object — total priority + flow edge list |
| `GraphLauncher.java` | Swing graph window launcher for visual runs |
| `Main.java` | Thin I/O shell — reads parameters, delegates to SimulationRunner |
| **Visualization** | |
| `SensorNetworkGraph.java` | Physical BSN graph (Swing) — draggable nodes, flow paths highlighted |
| `BFNGraph.java` | BFN flow network (Swing) — split nodes, draggable/bendable edges, pannable canvas |
| `Axis.java` | Simple x/y coordinate holder for graph nodes |
| **Legacy** | |
| `SensorStuff.java` | Original 3,500-line monolith — retained for reference, not used by Main |

---

## Requirements & Setup

### Java Version
Java SE 11 or higher. Visualization is built with Java Swing, included in the standard JDK — no additional libraries required.

### Installing Java (JDK)

**macOS:**
```bash
# Download JDK 21 from https://www.oracle.com/java/technologies/downloads/
# After installing the .dmg, verify with:
java -version
javac -version
```

**Windows:**  
Download the JDK 21 MSI installer from https://www.oracle.com/java/technologies/downloads/, run it, then verify in Command Prompt:
```cmd
java -version
javac -version
```

**Linux (Ubuntu/Debian):**
```bash
sudo apt update
sudo apt install openjdk-21-jdk
java -version
```

---

## Compile & Run

```bash
# Compile all files
javac *.java

# Run
java Main
```

### Sample Input
```
Width x and length y of sensor network: 100 100
Transmission range (m): 55
Graph structure - 1: adj-matrix, 2: adj-list: 1
Network sizes to test (comma-separated, e.g. 10,50,100): 8,10
Number of trials per network size: 30
Min node energy level: 20
Max node energy level: 20
Min overflow packets per DG: 4
Max overflow packets per DG: 8
Min packet size (storage units): 1
Max packet size (storage units): 6
Min storage capacity (storage units): 4
Max storage capacity (storage units): 10
Min packet priority: 1
Max packet priority: 20
Number of nodes for visual run: 8
```

> Setting min = max for any parameter produces the uniform (fixed) case.

---

## Problem Statement

In a BSN deployed in a harsh environment (e.g., underwater, underground, volcanic), sensors collect data and must store it locally until a drone or robot retrieves it. When sensor storage fills up, overflow data must be offloaded to nearby storage nodes. Because not all data can always be preserved, the system must decide which data to keep to maximize total preserved priority.

**This extension:** Each data generator (DG) node produces packets of a specific size `szᵢ` (storage units). Different DGs may have different packet sizes, making storage consumption heterogeneous. The goal is to maximize:

```
Vf = Σᵢ (vᵢ × |fᵢ|)
```

subject to energy and storage constraints.

---

## Parameters

| Parameter | Description |
|-----------|-------------|
| Width × Length | Physical dimensions of the sensor field (meters) |
| Transmission Range (TR) | Max distance between two connected nodes (meters) |
| Min/Max Node Energy | Energy budget `Eᵢ` — limits packets routed through a node |
| Min/Max Overflow Packets | `dᵢ` — number of overflow packets at each DG |
| Min/Max Packet Size | `szᵢ` — storage units one packet from DGᵢ occupies |
| Min/Max Storage Capacity | `mⱼ` — raw storage units at each storage node |
| Min/Max Packet Priority | `vᵢ` — importance/weight of packets from DGᵢ |
| Network Sizes | Comma-separated node counts to test (e.g. `8,10,20`) |
| Trials per Size | Number of random trials per network size |
| Visual Run Nodes | Node count for the single graphical run before scaling |

---

## BSN-Based Flow Network (BFN) Transformation

Following Section VI of Rivera & Tang (2024), each BSN graph `G(V, E)` is transformed into a directed flow network `G'(V', E')`.

**Node layout (CFN indices):**
```
0        = super source s
2i + 1   = in-node  i'   for BSN node i
2i + 2   = out-node i''  for BSN node i
2n + 1   = super sink t
```

**Edge construction:**
```
s  -> i'  :  capacity = dᵢ    (DG sending capacity)
i' -> i'' :  capacity = Eᵢ    (node energy budget)
u'' -> v' :  capacity = INF   (BSN routing edge)
j'' -> t  :  capacity = mⱼ    (raw storage units at storage node j)
```

**Size-aware sink check:** When augmenting flow for DGᵢ with packet size `szᵢ`, storage node `j` can only accept a packet if `sinkCap[j] >= szᵢ`. After pushing Δ packets, storage is reduced by `Δ × szᵢ`.

---

## Algorithms

### Algorithm 1 — GOA (Greedy Optimal Algorithm)
**Based on:** Algorithm 3 from Rivera & Tang (2024)  
**Sorting key:** Priority `vᵢ` descending  
**Routing:** BFS shortest augmenting path

Sorts DGs by priority weight and pushes maximum flow starting from the highest-priority source. Proven optimal for MWF-U (Theorem 2). Not optimal for MWF-S — a large high-priority packet may fill storage that could hold many small packets with greater combined priority.

**Counterexample:**
```
DG0: v=10, sz=3, d=2  vs  DG1: v=7, sz=1, d=6,  storage cap=6
GOA = 20   (pushes DG0 first, fills storage)
Optimal = 42  (pushing DG1 first fits 6 packets)
```

---

### Algorithm 2 — Density GOA (Value Density Greedy)
**New contribution**  
**Sorting key:** Value density `ρᵢ = vᵢ / szᵢ` descending  
**Routing:** BFS shortest augmenting path

Sorts by priority per storage unit consumed. Motivated by the fractional knapsack algorithm — maximizes priority per unit of the binding constraint (storage). Consistently beats GOA by 3–7 percentage points on variable-size instances. Also not always optimal — leftover storage fragments create bin-packing difficulty.

---

### Algorithm 3 — Approx GOA (2-Approximation)
**Adapted from:** Algorithm 5 of Rivera & Tang (2024)

Runs both GOA and Density GOA independently on fresh network copies and returns whichever result is higher. Always at least as good as either constituent. Provides a guaranteed ≥ ½ × optimal bound (adapted from Theorem 5).

---

### Algorithm 4 — Hybrid GOA (Prefix Enumeration + Best-Fit)
**New contribution**  
**Strategy:** Try every ordered pair of DGs as the first two to commit (κ=2), simulate the rest in density order, commit to the best starting combination  
**Routing:** Best-fit storage selection (route to the storage node where leftover space is smallest)

The most powerful heuristic. Addresses the fundamental weakness of all fixed-ordering algorithms: the best ordering depends on the network state after routing has begun. By trying k² starting combinations and measuring each one's full rollout, Hybrid GOA escapes bad orderings. Best-fit selection minimizes storage fragmentation by packing each node as tightly as possible. Reaches 99.6% of optimal at 10-node scale. Cost is O(k⁴), capped at MAX_HYBRID_DGS=12 with automatic fallback to Density GOA.

---

### Algorithm 5 — DDR-GOA (Dynamic Density Reordering)
**New contribution**  
**Strategy:** Re-rank all remaining DGs after each full augmentation step  
**Routing:** BFS shortest augmenting path

After routing each DG, recomputes effective priority for all remaining DGs: `effPri(i) = vᵢ × min(dᵢ, reachable_capacity / szᵢ)`. Routes whichever scores highest. Adapts to the changing residual network state. Performance is variable — works well when routing decisions strongly restrict future options, but the reachability estimate can be misleading when multiple DGs compete for the same storage.

---

### Algorithm 6 — DDR+-GOA (Contention-Aware DDR)
**New contribution — extension of DDR-GOA**  
**Strategy:** Re-rank with contention-weighted reachability  
**Routing:** BFS shortest augmenting path

Fixes DDR-GOA's competition-blindness: when k DGs can all reach the same storage node, plain DDR gives each one full credit for that storage. DDR+ computes a contention map — how many active DGs reach each sink — and shares each sink's capacity proportionally using √contention dampening:

```
share(i, j) = sinkCap(j) / √contention(j)
reachable+(i) = Σⱼ share(i, j)
effPri+(i) = vᵢ × min(dᵢ, ⌊reachable+(i) / szᵢ⌋)
```

Empirical results: DDR+ improves over DDR on sparse networks (3–1 win/loss at 8-node, TR=50) and heavily fragmented instances (10-node, sz 1–8: 95.4% vs 94.0%). On well-connected dense networks the contention signal can over-correct, making DDR+ slightly worse. The optimal dampening factor is instance-dependent — adaptive dampening is an open direction.

---

### Algorithm 7 — PSB-GOA (Per-Step Best-Path Scoring)
**New contribution**  
**Strategy:** Score every (DG, path) pair at each step, pick the globally best next move  
**Routing:** Best-fit storage selection with relay energy penalty

The most fine-grained algorithm. Unlike all others, PSB-GOA never commits to one DG for more than one step. At each routing decision it evaluates every active DG on every available path using:

```
score(i, path) = vᵢ / (waste + relayPenalty + 1)
```

This lets it interleave packets from different DGs in whatever order is locally optimal at each moment. The relay penalty preserves relay energy for later packets. Matches Hybrid GOA at 99.5% of optimal through an entirely different mechanism — local per-step scoring rather than global prefix enumeration. Same O(k⁴) cap with Density GOA fallback beyond 12 DGs.

---

### Exact Solver — Dual Branch & Bound
**Ground truth for evaluation**

Exhaustively searches all possible DG orderings using branch and bound with pruning. Warm-started with the best result from GOA, Density GOA, and Hybrid GOA. Children explored in density order for early pruning.

**Critical design:** Runs two independent B&B searches — one with best-fit routing, one with standard BFS routing — both updating a shared global best. This is necessary because both routing strategies find genuinely different valid solutions; a single-strategy solver allowed heuristics to exceed it. After the dual-search fix, zero violations across all experiments. Capped at MAX_EXACT_DGS=15; returns N/A beyond that.

---

## Key Engineering Fixes

### Fix 1 — Sink-Edge Unit Mismatch
The CFN sink edge capacity was in raw storage units, but the bottleneck walk treated it as a packet count — silently mixing units and over-augmenting every path. Fixed by skipping the sink edge in all bottleneck min-walks and augmenting it explicitly by Δ × szᵢ.

### Fix 2 — Dual B&B Exact Solver
Best-fit routing (Hybrid, PSB) and standard BFS routing (GOA, DDR) find genuinely different feasible solutions on the same instance. The original single-strategy B&B allowed heuristics to exceed it. Fixed by running two independent B&B passes, both updating one shared global best.

### Fix 3 — O(k⁴) Complexity Cap
Hybrid GOA and PSB-GOA enumerate k² prefixes with k-step rollouts → O(k⁴·n·m²). At k > 12 the 30-node trials ran indefinitely. Fixed by MAX_HYBRID_DGS=12: fall back to Density GOA automatically. Affected trials marked with `*` in output.

---

## Key Theoretical Findings

- GOA is **not optimal** with different sizes — shown by counterexample (GOA=20, optimal=42)
- Density GOA is **also not optimal** — shown by second counterexample (Density=16, optimal=17)
- The problem is **fundamentally harder** with sizes — leftover storage fragments create bin-packing difficulty on top of flow routing
- Approx GOA **guarantees ≥ ½ × optimal** — proven by adapting Theorem 5 from Rivera & Tang (2024)
- **Best-fit and BFS routing explore different feasible spaces** — the exact solver must cover both to guarantee correctness
- **Contention-aware reachability helps selectively** — DDR+ improves on DDR in sparse/fragmented settings but can over-correct in dense networks

---

## Experimental Results Summary

| Algorithm | 8-node avg | 10-node avg | Notes |
|-----------|-----------|------------|-------|
| GOA | ~92–95% | ~93–95% | Baseline; suboptimal for MWF-S |
| Density GOA | ~95–98% | ~96–97% | Simple density sort; consistently beats GOA |
| Approx GOA | ~97–99% | ~98–99% | max(GOA, Density); safe baseline |
| Hybrid GOA | ~99–100% | ~99–100% | Strongest heuristic; k⁴ cost |
| DDR-GOA | ~95–97% | ~94–98% | Variable; adapts well on some instances |
| DDR+-GOA | ~95–97% | ~95–96% | Improves DDR on sparse/fragmented networks |
| PSB-GOA | ~99–100% | ~99–100% | Matches Hybrid via per-step scoring |
| Exact | 100% | 100% | Dual B&B; N/A beyond k=15 |

All percentages relative to the dual B&B exact solver. Zero constraint violations after the dual-search fix.

---

## Randomized Trial Design

Each trial independently varies three factors:

**Node placement** — Gaussian clustering (1–3 centres) + 30% uniform outliers. Produces trials with tight clusters, sparse scatter, or a mix.

**DG/storage split** — Ratio drawn randomly between 20% and 70% per trial. Some trials are DG-heavy (storage bottleneck), others storage-heavy (energy bottleneck).

**Transmission range jitter** — Each trial applies ±25% random multiplier to base TR. The actual TR is printed per trial.

**Connectivity guaranteed** — `buildConnectedGraph()` regenerates up to 1000 times until BFS confirms full connectivity.

---

## Visualization

Each visual run opens graph windows per algorithm:

### Physical BSN Graph (`SensorNetworkGraph`)
- **Purple nodes** = Data Generators — shows `v`, `sz`, `d`, `E`, `(x,y)`
- **Green nodes** = Storage nodes — shows `cap`, `E`, `(x,y)`
- **Orange glow edges** = Flow paths chosen by the algorithm
- **Yellow dashed ring** = Relay-capable node
- Nodes are **draggable**; render at true physical coordinates

### BFN Flow Network (`BFNGraph`)
- Split pairs `i'` / `i''` for each BSN node
- Super source `s` left, super sink `t` right
- Edge labels: `d=X`, `E=X`, `m=X`, `inf`
- **Orange glow** on edges carrying active flow
- **Drag edges** to bend; **drag background** to pan

---

## Sample Output

### Visual Run
```
-- Feasibility Check --
  Total DG packets:    17
  Total storage need:  58
  Total storage cap:   25  (BOTTLENECK)
  Total node energy:   120  (sufficient)
  >> Bottleneck detected — running MWF.

===== VISUAL RUN SUMMARY =====
  ILP Optimal:                69.0
  GOA priority:        69.0  (100.0% of optimal)
  Density GOA priority: 69.0  (100.0% of optimal)
  Hybrid GOA priority: 69.0  (100.0% of optimal)
  DDR-GOA priority:    69.0  (100.0% of optimal)
  DDR+-GOA priority:   69.0  (100.0% of optimal)
  PSB-GOA priority:    69.0  (100.0% of optimal)
```

### Scaling Runs
```
==== Network Size: 8 nodes, 20 trials ====
  Trial  1 [TR=49, DGs=3, STs=5]: GOA=81.0  Density=85.0  Approx=85.0
    Hybrid=85.0  DDR=85.0  DDR+=85.0  PSB=85.0  Exact=85.0
  ...
-- Results over 19 active trials (1 skipped, 20 total) --
  ILP Optimal avg:      125.58  (19/19 active trials)
  GOA avg:              114.16  (90.9% of optimal)
  Density GOA avg:      121.95  (97.1% of optimal)
  Approx GOA avg:       122.16  (97.3% of optimal)
  Hybrid GOA avg:       125.32  (99.8% of optimal)
  DDR-GOA avg:          119.95  (95.5% of optimal)
  DDR+-GOA avg:         108.31  (96.4% of optimal)
  PSB-GOA avg:          125.32  (99.8% of optimal)
  DDR+ beat DDR:        5/16 active trials
  DDR beat DDR+:        2/16 active trials
  DDR+/DDR tied:        9/16 active trials
```

---

## Recommended Test Inputs

### Small & clean (see everything working)
```
100 100 / TR=55 / adj-matrix / sizes: 6,8 / 20 trials / energy 20-20
packets 4-8 / pkt-size 1-6 / storage 4-10 / priority 1-20 / visual: 6
```

### Maximum fragmentation pressure
```
100 100 / TR=65 / adj-matrix / sizes: 8,10 / 30 trials / energy 25-25
packets 5-10 / pkt-size 1-8 / storage 4-8 / priority 1-50 / visual: 8
```

### Large scale with complexity cap
```
300 300 / TR=70 / adj-matrix / sizes: 10,20,30 / 30 trials / energy 50-50
packets 5-10 / pkt-size 1-8 / storage 6-16 / priority 1-50 / visual: 10
```

---

## Reference

Rivera, G. & Tang, B. (2024). *Priority-Based Data Preservation in Challenging Environments: A Maximum Weighted Flow Approach.* California State University Dominguez Hills.  
Paper: https://csc.csudh.edu/btang/papers/priority_journal.pdf
