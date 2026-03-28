# Priority-Based Data Preservation with Variable Packet Sizes
**A Size-Aware Maximum Weighted Flow Approach**  
Author: Jason Roe | Advisor: Professor Bin Tang, California State University Dominguez Hills  
Course: CSC 590 — M.S. Project

---

## Overview

This project extends the Maximum Weighted Flow (MWF-U) framework from Rivera & Tang (2024) to support packets of different sizes across data generator nodes. The core contribution is a new class of size-aware algorithms that maximize total preserved data priority in base station-less sensor networks (BSNs) under storage and energy constraints.

In the original paper, all data packets are assumed to be unit-sized, allowing Algorithm 3 (GOA) to be proven optimal. This project removes that assumption and studies what happens when packets from different source nodes have heterogeneous storage requirements.

---

## Files

| File | Description |
|------|-------------|
| `SensorStuff.java` | Main class — BFN construction, all three algorithms, scaling loop, visualization wiring, connectivity enforcement |
| `SensorNetworkGraph.java` | Physical BSN graph visualization (Swing) — draggable nodes, fixed initial positions, flow paths highlighted |
| `BFNGraph.java` | BFN flow network visualization (Swing) — split nodes i'/i'', draggable/bendable edges, labels follow edges, pannable canvas |
| `Axis.java` | Simple x/y coordinate holder for graph nodes |

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
java SensorStuff
```

### Sample Input
```
Width x and length y of sensor network: 100 100
Transmission range (m): 80
Graph structure - 1: adj-matrix, 2: adj-list: 1
Network sizes to test (comma-separated, e.g. 10,50,100): 10,50,100
Number of trials per network size: 20
Min node energy level: 5
Max node energy level: 10
Min overflow packets per DG: 3
Max overflow packets per DG: 10
Min packet size (storage units): 1
Max packet size (storage units): 5
Min storage capacity (storage units): 10
Max storage capacity (storage units): 50
Min packet priority: 1
Max packet priority: 10
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
| Network Sizes | Comma-separated node counts to test (e.g. `10,50,100`) |
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

**Steps:**
1. Sort DGs by priority `vᵢ` descending
2. For each DG (highest priority first):
   - Find shortest Feasible Augmenting Path (FAP) through this DG
   - Compute `Δ = min(path residual, dᵢ, floor(mⱼ/szᵢ))`
   - Augment flow by Δ; update `dᵢ -= Δ`, `mⱼ -= Δ×szᵢ`
   - Repeat until no FAPs exist through this DG
3. Move to next DG; stop when no FAPs remain anywhere

**Limitation:** Not always optimal when sizes differ. A large high-priority packet may fill storage that could hold many small packets with greater combined priority.

**Counterexample:**
```
DG0: v=10, sz=3, d=2  vs  DG1: v=7, sz=1, d=6,  storage cap=6
GOA = 20   (pushes DG0 first, fills storage)
Optimal = 42  (pushing DG1 first fits 6 packets)
```

---

### Algorithm 2 — Density GOA (Value Density Greedy)
**New contribution of this project**  
**Sorting key:** Value density `ρᵢ = vᵢ / szᵢ` descending

Analogous to the fractional knapsack greedy — maximizes priority per unit of storage consumed, which is the binding constraint when packet sizes differ.

**Steps:**
1. Compute density `ρᵢ = vᵢ / szᵢ` for each DG
2. Sort DGs by `ρᵢ` descending
3. For each DG (highest density first):
   - Find shortest FAP through this DG
   - Compute `Δ = min(path residual, dᵢ, floor(mⱼ/szᵢ))`
   - Augment flow by Δ; update `dᵢ -= Δ`, `mⱼ -= Δ×szᵢ`
   - Repeat until no FAPs exist through this DG
4. Move to next DG; stop when no FAPs remain anywhere

**Limitation:** Also not always optimal. Leftover storage fragments after placing some packets may not fit remaining packets, creating bin-packing-like difficulty.

**Counterexample:**
```
DG0: v=9, sz=3  vs  DG1: v=8, sz=2,  storage cap=5
Density GOA = 16  (pushes DG1 first, ρ=4.0)
Optimal = 17  (one packet from each DG)
```

---

### Algorithm 3 — Approx GOA (2-Approximation)
**Adapted from:** Algorithm 5 of Rivera & Tang (2024)

Runs both GOA (sort by `vᵢ`) and Density GOA (sort by `vᵢ/szᵢ`) as sub-routines and returns whichever result is higher.

**Steps:**
1. Sub-routine A: greedy by `vᵢ` → result `VfA`
2. Sub-routine B: greedy by `vᵢ/szᵢ` → result `VfB`
3. Return `max(VfA, VfB)`

**Approximation guarantee (adapted from Theorem 5):**
```
Let Sw = sources that send in sub-routine B before first failure sⱼ
Vopt <= V'f + vⱼ×dⱼ <= V'f + Vf
Since Vf = max(Vf, V'f):   Vf >= Vopt / 2
```

The result is always at least half the theoretical optimal.

---

## Key Theoretical Findings

- GOA is **not optimal** with different sizes — shown by counterexample (GOA=20, optimal=42)
- Density GOA is **also not optimal** — shown by second counterexample (Density=16, optimal=17)
- The problem is **fundamentally harder** with sizes — leftover storage fragments create bin-packing difficulty on top of flow routing
- Approx GOA **guarantees ≥ ½ × optimal** — proven by adapting Theorem 5 from Rivera & Tang (2024)

---

## Randomized Trial Design

Each trial independently varies three factors to ensure GOA and Density GOA are tested across genuinely diverse network conditions:

**Node placement** — nodes are distributed using a mix of Gaussian clustering (1–3 random cluster centres, implemented in `randomNodes()`) and uniform scatter (30% outliers). This produces trials with tight dense clusters, sparse scatter, or a mix — directly changing which constraint (energy vs. storage) dominates.

**DG/storage split** — the ratio of DG nodes to storage nodes is drawn randomly between 20% and 70% per trial, rather than a fixed split. Some trials are DG-heavy (storage is the bottleneck), others are storage-heavy (routing and energy matter more).

**Transmission range jitter** — each trial applies a ±25% random multiplier to the base TR (`trialTR = TR × U[0.75, 1.25]`), varying graph density from sparse to dense. The actual TR used is printed per trial line in the scaling output.

**Connectivity guaranteed** — before any trial runs, `buildConnectedGraph()` regenerates node positions and edges up to 1000 times until the graph is fully connected (verified by BFS from node 0). This satisfies Professor Tang's connectivity requirement: packets must be routable from any DG to any storage node.

---

## Visualization

Each visual run opens 4 graph windows:

### Physical BSN Graph (`SensorNetworkGraph`)
- **Purple nodes** = Data Generators (DGs) — shows `v`, `sz`, `d`, `E`, `(x,y)`
- **Green nodes** = Storage nodes — shows `cap`, `E`, `(x,y)`
- **Blue/purple edges** = BSN communication links
- **Orange glow edges** = Flow paths chosen by the algorithm
- **Yellow dashed ring** = Relay-capable node
- Nodes are **draggable** to rearrange the layout
- Nodes render at their true physical `(x,y)` coordinates on first open

### BFN Flow Network (`BFNGraph`)
- Each BSN node shown as split pair `i'` (in-node) and `i''` (out-node)
- Super source `s` on the left, super sink `t` on the right
- **Edge labels:**
  - `s → i'` edges: labeled `d=X` (packets) and `E=X` (DG energy)
  - `i' → i"` edges: labeled `E=X` (node energy budget)
  - `u" → v'` routing edges: labeled `inf` (dashed, 70% opacity)
  - `j" → t` sink edges: labeled `m=X`, `E=X`, `(ST N)` — one per storage node
- **Orange glow** on edges carrying active flow
- **All labels attach to and move with their edge** when bent
- **Drag any edge line** to bend it — grab anywhere along the line, not just the midpoint
- **Drag the canvas background** to pan the entire view
- Labels stack cleanly with no overlap for multiple DGs or storage nodes
- Window height scales automatically with node count so no nodes are clipped

---

## Sample Output

### Visual Run
```
-- Feasibility Check --
  Total DG packets:    17
  Total storage need:  52
  Total storage cap:   54  (sufficient)
  Total node energy:   39  (BOTTLENECK)
  >> Bottleneck detected — running MWF.

  [TRACE GOA] DG 9 -> Storage 3 | Δ=3 | v=5 | sz=2 | path: 9 -> 3
  Pushed 3 packet(s) from DG 9 (v=5, sz=2) -> sink 8

Total Preserved Priority (GOA): 14.0

-- Relay Node Activity --
  Node 3 [Relay]: forwarded 3 packet(s), energy used: 3/6
```

If no bottleneck exists, the algorithm is skipped:
```
  >> All packets can be offloaded — MWF not needed.
```

### Scaling Runs
```
==== Network Size: 50 nodes, 20 trials ====
  Trial  1 [TR=84, DGs=18, STs=32]: GOA=210.0  Density=245.0  Approx=245.0
  Trial  2 [TR=71, DGs=29, STs=21]: GOA=198.0  Density=198.0  Approx=198.0
  ...
-- Averages over 20 trials --
  GOA avg:              205.30
  Density GOA avg:      231.10
  Approx GOA avg:       231.10
  Density beat GOA:     14/20 trials
  GOA beat Density:      3/20 trials
  Tied:                  3/20 trials
```

---

## Reference

Rivera, G. & Tang, B. (2024). *Priority-Based Data Preservation in Challenging Environments: A Maximum Weighted Flow Approach.* California State University Dominguez Hills.  
Paper: https://csc.csudh.edu/btang/papers/priority_journal.pdf
