# Priority-Based Data Preservation with Variable Packet Sizes
### A Size-Aware Maximum Weighted Flow Approach
**Author:** Jason Roe  
**Advisor:** Professor Bin Tang, California State University Dominguez Hills  
**Course:** CSC 590 — M.S. Project

---

## Overview

This project extends the Maximum Weighted Flow (MWF-U) framework from Rivera & Tang (2024) to support **packets of different sizes** across data generator nodes. The core contribution is a new class of size-aware algorithms that maximize total preserved data priority in base station-less sensor networks (BSNs) under storage and energy constraints.

In the original paper, all data packets are assumed to be unit-sized, allowing Algorithm 3 (GOA) to be proven optimal. This project removes that assumption and studies what happens when packets from different source nodes have heterogeneous storage requirements.

---

## Problem Statement

In a BSN deployed in a harsh environment (e.g., underwater, underground, volcanic), sensors collect data and must store it locally until a drone or robot arrives to retrieve it. When sensor storage fills up, overflow data must be offloaded to nearby storage nodes. Because not all data can always be preserved, the system must decide **which data to keep** to maximize total preserved priority.

**Our extension:** Each data generator (DG) node produces packets of a specific size `szᵢ` (storage units). Different DGs may have different packet sizes, making storage consumption heterogeneous. The goal remains to maximize total preserved priority `Vf = Σᵢ (vᵢ × |fᵢ|)` subject to energy and storage constraints.

---

## Parameters

All parameters are user-configurable at runtime:

| Parameter | Description |
|---|---|
| Width × Length | Physical dimensions of the sensor field (meters) |
| Transmission Range (TR) | Max distance between two connected nodes (meters) |
| Min/Max Node Energy | Energy budget Eᵢ per node — limits packets through that node |
| Min/Max Overflow Packets | dᵢ — number of overflow packets at each DG |
| Min/Max Packet Size | szᵢ — storage units one packet from DGᵢ occupies |
| Min/Max Storage Capacity | mⱼ — raw storage units available at each storage node |
| Min/Max Packet Priority | vᵢ — importance/weight of packets from DGᵢ |
| Network Sizes | Comma-separated list of node counts to test (e.g. 10,50,100) |
| Trials per Size | Number of random trials per network size |
| Visual Run Nodes | Node count for the single graphical run before scaling |

Setting min = max for any parameter produces the uniform case.

---

## BSN-Based Flow Network (BFN) Transformation

Following Section VI of Rivera & Tang (2024), each BSN graph `G(V, E)` is transformed into a flow network `G'(V', E')`:

**Node layout (CFN indices):**
```
0        = super source s
2i + 1   = in-node  i'  for BSN node i
2i + 2   = out-node i"  for BSN node i
2n + 1   = super sink t
```

**Edge construction:**
```
s  -> i'  :  capacity = dᵢ        (DG sending capacity)
i' -> i"  :  capacity = Eᵢ        (node energy budget, uniform cost = 1)
u" -> v'  :  capacity = INF       (BSN routing edge, uniform cost = 1)
j" -> t   :  capacity = mⱼ        (raw storage units at storage node j)
```

**Cost model:** All flows consume exactly 1 unit of edge capacity per packet (MWF-U uniform cost model). The i'→i" edge limits total packets through node i based on its energy budget Eᵢ.

**Size-aware sink check:** When augmenting flow for DG sᵢ with packet size szᵢ, a storage node j can accept a packet only if `sinkCap[j] >= szᵢ`. After pushing Δ packets, storage is reduced by `Δ × szᵢ`.

---

## Algorithms

### Algorithm 1: GOA — Size-Aware Greedy Optimal Algorithm

**Based on:** Algorithm 3 from Rivera & Tang (2024)  
**Sorting key:** Priority vᵢ descending

**Mathematical objective:**
```
maximize  Vf = Σᵢ (vᵢ × |fᵢ|)
subject to:
  |fᵢ| <= dᵢ                          (DG sending capacity)
  Σ flows through node i <= Eᵢ        (node energy)
  Σ (szᵢ × packets stored at j) <= mⱼ (storage capacity)
```

**Steps:**
1. Sort DGs by priority vᵢ in descending order
2. For each DG sᵢ (highest priority first):
   - Find shortest Feasible Augmenting Path (FAP) through sᵢ
   - Compute bottleneck Δ = min(path residual, dᵢ, floor(mⱼ/szᵢ))
   - Augment flow by Δ; update dᵢ -= Δ, mⱼ -= Δ×szᵢ
   - Repeat until no FAPs exist through sᵢ
3. Move to next DG; stop when no FAPs remain anywhere

**Limitation:** Not always optimal when sizes differ. A large high-priority packet may fill storage that would otherwise hold many small packets with greater combined priority.

**Counterexample:**  
DG0: v=10, sz=3, d=2 vs DG1: v=7, sz=1, d=6, storage cap=6  
GOA gives **20** (pushes DG0 first, fills storage)  
Optimal gives **42** (pushing DG1 first fits 6 packets)

---

### Algorithm 2: Density GOA — Value Density Greedy Algorithm

**New contribution of this project**  
**Sorting key:** Value density ρᵢ = vᵢ/szᵢ descending

**Mathematical basis:**  
Define value density ρᵢ = vᵢ / szᵢ — priority gained per storage unit consumed. Analogous to the fractional knapsack greedy, adapted to the network flow setting. Sorting by ρᵢ maximizes priority per unit of storage consumed, which is the binding constraint when sizes differ.

**Steps:**
1. Compute density ρᵢ = vᵢ / szᵢ for each DG
2. Sort DGs by ρᵢ in descending order (highest density first)
3. For each DG sᵢ (highest density first):
   - Find shortest FAP through sᵢ
   - Compute Δ = min(path residual, dᵢ, floor(mⱼ/szᵢ))
   - Augment flow by Δ; update dᵢ -= Δ, mⱼ -= Δ×szᵢ
   - Repeat until no FAPs exist through sᵢ
4. Move to next DG; stop when no FAPs remain anywhere

**Limitation:** Also not always optimal. Leftover storage fragments after placing some packets may not fit remaining packets, creating bin-packing-like difficulty.

**Second counterexample:**  
DG0: v=9, sz=3 vs DG1: v=8, sz=2, storage cap=5  
Density GOA gives **16** (pushes DG1 first, ρ=4.0)  
Optimal gives **17** (push DG0 one packet + DG1 one packet)

---

### Algorithm 3: Approx GOA — 2-Approximation Algorithm

**Adapted from:** Algorithm 5 of Rivera & Tang (2024)  
**Modification:** Replaces cost cᵢ with size szᵢ; uses splittable flows (not all-or-nothing)

**Mathematical basis:**  
Run two sub-routines and return the better result:
- Sub-routine A: greedy by vᵢ (same logic as Algorithm 1)
- Sub-routine B: greedy by vᵢ/szᵢ (same logic as Algorithm 2)

**Approximation guarantee (adapted from Theorem 5):**
```
Let Sw = sources that send in sub-routine B before first failure sⱼ
V'f = Σ(sᵢ∈Sw)(vᵢ×dᵢ)   and   vⱼ×dⱼ <= Vf  (A sends highest v first)
Vopt <= V'f + vⱼ×dⱼ <= V'f + Vf
Since Vf = max(Vf, V'f):   Vf >= Vopt / 2
```

**Steps:**
1. Run sub-routine A (sort by vᵢ, splittable) → result VfA
2. Run sub-routine B (sort by vᵢ/szᵢ, splittable) → result VfB
3. Return max(VfA, VfB)

**Guarantee:** Result is always at least half the optimal total priority.

---

## Key Theoretical Findings

1. **GOA is not optimal with different sizes** — shown by counterexample (GOA=20, optimal=42)
2. **Density GOA is not optimal either** — shown by second counterexample (Density=16, optimal=17)
3. **The problem is fundamentally harder with sizes** — leftover storage fragments create bin-packing difficulty on top of flow routing
4. **Approx GOA achieves at least ½ × optimal** — proven by adapting Theorem 5 from Rivera & Tang (2024)

---

## Randomized Trial Design

Each trial independently varies three factors to ensure GOA and Density GOA are tested across genuinely diverse network conditions rather than structurally similar graphs:

**Node placement** — nodes are distributed using a mix of Gaussian clustering (1–3 random cluster centres) and uniform scatter (30% outliers). This produces trials with tight dense clusters, sparse scatter, or a mix of both — directly changing which constraint (energy vs. storage) dominates.

**DG/storage split** — the ratio of data generator nodes to storage nodes is drawn randomly between 20% and 70% per trial, rather than a fixed 50/50 coin flip. Some trials are DG-heavy (storage is the bottleneck), others are storage-heavy (routing and energy matter more).

**Transmission range jitter** — each trial applies a ±25% random multiplier to the base TR, varying graph density from sparse (few routing paths) to dense (many alternative paths). The actual TR used is printed per trial line in the scaling output.

---

## Files

| File | Description |
|---|---|
| `SensorStuff.java` | Main class — BFN construction, all three algorithms, scaling loop, visualization wiring |
| `SensorNetworkGraph.java` | Physical BSN graph visualization (Swing) — draggable nodes, flow paths highlighted |
| `BFNGraph.java` | BFN flow network visualization (Swing) — split nodes i'/i", super source/sink, edge labels |
| `Axis.java` | Simple x/y coordinate holder for graph nodes |

---

## Requirements & Setup

### Java Version
This project uses **Java SE 11 or higher**. The visualization is built with **Java Swing**, which is included in the standard Java Development Kit (JDK) — no separate download required.

### Installing Java (JDK)

**macOS:**
1. Go to https://www.oracle.com/java/technologies/downloads/
2. Select **macOS** tab and download the **DMG installer** for JDK 21 (latest LTS)
3. Open the `.dmg` file and follow the installer
4. Verify installation by opening Terminal and running:
```bash
java -version
javac -version
```
You should see something like:
```
java version "21.0.x"
javac 21.0.x
```

**Windows:**
1. Go to https://www.oracle.com/java/technologies/downloads/
2. Select **Windows** tab and download the **MSI installer** for JDK 21
3. Run the installer and follow the prompts
4. Open Command Prompt and verify:
```bash
java -version
javac -version
```

**Linux (Ubuntu/Debian):**
```bash
sudo apt update
sudo apt install openjdk-21-jdk
java -version
```

### Note on Swing
Java Swing is part of the standard JDK and requires **no additional installation or configuration**. If you are using an IDE (IntelliJ, Eclipse, VS Code), Swing is available automatically once the JDK is installed. No external libraries or dependencies are needed for this project.

---

## Compile & Run

```bash
# Compile all files
javac *.java

# Run
java SensorStuff
```

**Sample input:**
```
Width x and length y of sensor network: 100 100
Transmission range (m): 80
Graph structure - 1: adj-matrix, 2: adj-list: 1
Network sizes to test (comma-separated): 10,50,100
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

---

## Visualization

Each visual run opens **4 graph windows:**

**Physical BSN Graph (SensorNetworkGraph):**
- 🔵 Blue nodes = Data Generators (DGs) — shows v, sz, d, (x,y)
- 🟢 Green nodes = Storage nodes — shows capacity, (x,y)
- Gray edges = BSN communication links
- Orange dashed edges = Flow paths chosen by the algorithm
- Draggable nodes to untangle overlapping layouts

**BFN Flow Network (BFNGraph):**
- Each BSN node shown as split pair i' (in-node) and i" (out-node)
- Super source s on the left, super sink t on the right
- Blue internal edge i'→i" labeled with node energy Eᵢ
- Dashed oval around each i'/i" pair
- Orange glow on edges carrying GOA flow
- Click and drag to pan the network

---

## Scaling Output Format

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
