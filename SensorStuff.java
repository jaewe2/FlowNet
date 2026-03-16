import java.util.*;
import javax.swing.SwingUtilities;

public class SensorStuff {
    private static final double Eelect     = .0001;
    private static final double Eamp       = 0.0000001;
    private static final int    PACKET_SIZE = 3200;

    // ── per-source arrays ─────────────────────────────────────────────────────
    protected int[] packetSize;      // szᵢ: storage units one packet from DGᵢ occupies
    protected int[] packetPriority;  // vᵢ:  priority/weight of packets from DGᵢ

    // ── per-node energy ───────────────────────────────────────────────────────
    // Eᵢ: how many packets can pass through node i total.
    // Each packet that passes through node i costs exactly 1 unit of Eᵢ
    // (uniform cost model, MWF-U — cost is the same for all edges).
    protected int[] nodeEnergy;

    // ── CFN matrices ──────────────────────────────────────────────────────────
    private int     cfnNodes;
    private int[][] cap;   // capacity matrix of the BFN
    private int[][] flow;  // current flow matrix of the BFN
    private Set<Integer> dgCFNIds = new HashSet<>();

    // ── CFN node index helpers ────────────────────────────────────────────────
    // Each BSN node i is split into:
    //   in-node  i' = 2i+1
    //   out-node i" = 2i+2
    //   super source s = 0
    //   super sink   t = 2n+1
    private int inNode(int i)  { return 2*i + 1; }
    private int outNode(int i) { return 2*i + 2; }
    private int superSink()    { return 2*nodeLoc.length + 1; }

    // ── existing fields ───────────────────────────────────────────────────────
    protected class AdjacentMatrix {
        protected int[][] adjM;
        protected int nodes;
        protected AdjacentMatrix(int nodes) {
            this.nodes = nodes;
            adjM = new int[nodes][nodes];
        }
        public void addE(int i, int j) { adjM[i][j] = 1; adjM[j][i] = 1; }
        public int[][] getAdjM()       { return adjM; }
    }

    protected class Edge implements Comparable<Edge> {
        int node1, node2;
        double edgeW;
        public Edge(int n1, int n2, double w) { node1=n1; node2=n2; edgeW=w; }
        public int compareTo(Edge o)          { return Double.compare(edgeW, o.edgeW); }
    }

    protected AdjacentMatrix             adjM;
    protected ArrayList<LinkedList<Integer>> adjList;
    protected double[][]                 nodeLoc;
    protected List<List<Integer>>        components;
    protected int[]                      packetsPerNode;
    protected List<Edge>                 mstEdges   = new ArrayList<>();
    protected List<Integer>              rendPoints = new ArrayList<>();
    protected double                     totalEDijkstra = 0.0;
    protected double                     totalEPrim     = 0.0;
    protected List<Double>               dijkstraPerComponentE = new ArrayList<>();
    protected List<Double>               primPerComponentE     = new ArrayList<>();

    // ── constructor ───────────────────────────────────────────────────────────
    public SensorStuff(int numNodes, int choice) {
        if (choice == 1) {
            adjM = new AdjacentMatrix(numNodes);
        } else {
            adjList = new ArrayList<>();
            for (int i = 0; i < numNodes; i++) adjList.add(new LinkedList<>());
        }
        nodeLoc        = new double[numNodes][2];
        packetsPerNode = new int[numNodes];
        packetSize     = new int[numNodes];
        packetPriority = new int[numNodes];
        nodeEnergy     = new int[numNodes];
    }

    // =========================================================================
    //  RANDOM INITIALISATION
    // =========================================================================

    public void randomPacketSizes(int minSz, int maxSz) {
        Random rand = new Random();
        for (int i = 0; i < packetSize.length; i++)
            packetSize[i] = (minSz == maxSz) ? minSz
                          : rand.nextInt(maxSz - minSz + 1) + minSz;
    }

    public void randomPacketPriorities(int minP, int maxP) {
        Random rand = new Random();
        for (int i = 0; i < packetPriority.length; i++)
            packetPriority[i] = (minP == maxP) ? minP
                              : rand.nextInt(maxP - minP + 1) + minP;
    }

    public void randomDataPackets(int min, int max) {
        Random r = new Random();
        for (int i = 0; i < packetsPerNode.length; i++)
            packetsPerNode[i] = (min == max) ? min
                              : r.nextInt(max - min + 1) + min;
    }

    /** Assign a random energy budget Eᵢ to each node.
     *  Each packet that passes through node i costs exactly 1 unit of Eᵢ
     *  (uniform cost model, MWF-U). */
    public void randomNodeEnergies(int minE, int maxE) {
        Random rand = new Random();
        for (int i = 0; i < nodeEnergy.length; i++)
            nodeEnergy[i] = (minE == maxE) ? minE
                          : rand.nextInt(maxE - minE + 1) + minE;
    }

    // =========================================================================
    //  BFN CONSTRUCTION  (Section VI of paper, uniform cost = 1 per packet)
    //
    //  Node layout:
    //    0      = super source s
    //    2i+1   = in-node  i'  for BSN node i
    //    2i+2   = out-node i"  for BSN node i
    //    2n+1   = super sink t
    //
    //  Edges:
    //    s  -> i'  : capacity di         (DG sending capacity)
    //    i' -> i"  : capacity Eᵢ         (node energy budget)
    //    u" -> v'  : capacity INF        (uniform cost = 1, BSN edge)
    //    j" -> t   : capacity mj         (raw storage units)
    // =========================================================================

    public void buildCFN(List<Integer> dgNodes,
                         List<Integer> storageNodes,
                         int[]         storageCapacity) {
        int n    = nodeLoc.length;
        int S    = 0;
        int T    = 2*n + 1;
        cfnNodes = 2*n + 2;
        cap      = new int[cfnNodes][cfnNodes];
        flow     = new int[cfnNodes][cfnNodes];
        dgCFNIds.clear();

        // s -> i' for each DG
        for (int dg : dgNodes) {
            cap[S][inNode(dg)] = packetsPerNode[dg];
            dgCFNIds.add(inNode(dg));
        }

        // i' -> i" with per-node energy (uniform cost = 1 per packet)
        for (int i = 0; i < n; i++)
            cap[inNode(i)][outNode(i)] = nodeEnergy[i];

        // u" -> v' for each BSN edge (uniform cost = 1, large capacity)
        int INF    = Integer.MAX_VALUE / 2;
        int[][] bsnAdj = (adjM != null) ? adjM.getAdjM() : null;
        for (int u = 0; u < n; u++)
            for (int v : getAdjNodes(bsnAdj != null ? bsnAdj : adjList, u))
                cap[outNode(u)][inNode(v)] = INF;

        // j" -> t with raw storage capacity
        for (int j = 0; j < storageNodes.size(); j++) {
            int st = storageNodes.get(j);
            cap[outNode(st)][T] = storageCapacity[j];
        }
    }

    // =========================================================================
    //  BFS FOR SHORTEST FEASIBLE AUGMENTING PATH (FAP)
    //
    //  Finds the shortest path from super source s to super sink t that
    //  passes through a specific DG source node (cfnSource).
    //
    //  Two-phase BFS:
    //    Phase 1: BFS from s to cfnSource only.
    //    Phase 2: BFS from cfnSource to t, blocked from entering other DG
    //             in-nodes to ensure flow is attributed to the correct DG.
    //
    //  Size-aware sink check: sinkCap[u] >= szI ensures at least one
    //  packet of size szI fits in the storage node before accepting the path.
    // =========================================================================

    private int[] bfsFAP(int cfnSource, int szI, int[] sinkCap) {
        int S = 0, T = superSink();
        int[]    parent = new int[cfnNodes];
        boolean[] vis   = new boolean[cfnNodes];
        Arrays.fill(parent, -1);

        // phase 1: BFS from s to cfnSource
        Queue<Integer> q = new LinkedList<>();
        q.add(S);
        vis[S]    = true;
        parent[S] = S;

        boolean reached = false;
        while (!q.isEmpty()) {
            int u = q.poll();
            if (u == cfnSource) { reached = true; break; }
            for (int v = 0; v < cfnNodes; v++) {
                if (vis[v] || v == T) continue;
                if (cap[u][v] - flow[u][v] <= 0) continue;
                vis[v]    = true;
                parent[v] = u;
                q.add(v);
            }
        }
        if (!reached) return null;

        // phase 2: BFS from cfnSource to t
        boolean[] vis2 = new boolean[cfnNodes];
        Queue<Integer> q2 = new LinkedList<>();
        q2.add(cfnSource);
        vis2[cfnSource] = true;

        while (!q2.isEmpty()) {
            int u = q2.poll();
            for (int v = 0; v < cfnNodes; v++) {
                if (vis2[v]) continue;
                if (cap[u][v] - flow[u][v] <= 0) continue;
                if (v != T && v != cfnSource && dgCFNIds.contains(v)) continue;
                if (v == T) {
                    // size-aware sink check: at least one packet of size szI must fit
                    if (sinkCap[u] < szI) continue;
                    parent[v] = u;
                    return parent;
                }
                vis2[v]   = true;
                parent[v] = u;
                q2.add(v);
            }
        }
        return null;
    }

    // =========================================================================
    //
    //  ALGORITHM 1: GOA (Greedy Optimal Algorithm) — Size-Aware Extension
    //
    //  Based on Algorithm 3 from Rivera & Tang (2024).
    //  Extended to handle packets of different sizes (our new contribution).
    //
    //  Mathematical basis:
    //    Objective: maximize Vf = Σᵢ (vᵢ × |fᵢ|)
    //    Subject to:
    //      |fᵢ| <= dᵢ              (DG sending capacity)
    //      Σ flows through node i <= Eᵢ  (node energy)
    //      Σ (szᵢ × packets stored at j) <= mⱼ  (storage capacity)
    //
    //  Steps:
    //    1. Sort DGs by priority vᵢ in descending order.
    //    2. For each DG sᵢ (highest priority first):
    //       a. Find shortest FAP through sᵢ in residual graph.
    //       b. Compute bottleneck Δ = min(path residual, dᵢ, floor(mⱼ/szᵢ)).
    //       c. Augment flow by Δ along the path.
    //       d. Update: dᵢ -= Δ, mⱼ -= Δ×szᵢ.
    //       e. Repeat until no more FAPs exist through sᵢ.
    //    3. Move to next DG. Stop when no FAPs remain anywhere.
    //
    //  Why it can fail with different sizes:
    //    In the original paper all packets are unit-sized, so the exchange
    //    argument holds: swapping flow from high-v to low-v always reduces
    //    total weight. With different sizes, a large high-v packet may fill
    //    storage that would otherwise hold many small lower-v packets with
    //    greater combined weight (see counterexample: v0=10,sz0=3,d0=2 vs
    //    v1=7,sz1=1,d1=6, cap=6 -> GOA gives 20, optimal gives 42).
    //
    // =========================================================================

    public double goa(List<Integer> dgNodes,
                      List<Integer> storageNodes,
                      int[]         storageCapacity) {

        buildCFN(dgNodes, storageNodes, storageCapacity);
        int S = 0, T = superSink();

        // remaining overflow packets per DG
        int[] remaining = new int[dgNodes.size()];
        for (int i = 0; i < dgNodes.size(); i++)
            remaining[i] = packetsPerNode[dgNodes.get(i)];

        // sinkCap[outNode(j)] = raw storage units remaining at storage node j
        int[] sinkCap = new int[cfnNodes];
        for (int j = 0; j < storageNodes.size(); j++)
            sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

        // step 1: sort DGs by priority vᵢ descending
        Integer[] order = new Integer[dgNodes.size()];
        for (int i = 0; i < order.length; i++) order[i] = i;
        Arrays.sort(order, (a, b) ->
                packetPriority[dgNodes.get(b)] - packetPriority[dgNodes.get(a)]);

        double      totalWeight  = 0.0;
        List<int[]> goaFlowEdges = new ArrayList<>();
        int         maxIter      = cfnNodes * cfnNodes;

        // step 2: greedily push flow from each DG in priority order
        for (int idx : order) {
            int dg     = dgNodes.get(idx);
            int cfnSrc = inNode(dg);
            int szI    = packetSize[dg];
            int vi     = packetPriority[dg];
            int iters  = 0;

            while (remaining[idx] > 0 && iters++ < maxIter) {

                // step 2a: find shortest FAP through this DG
                int[] parent = bfsFAP(cfnSrc, szI, sinkCap);
                if (parent == null) break;

                int sinkOut = parent[T];

                // step 2b: compute bottleneck Δ
                int pathBottleneck = Integer.MAX_VALUE;
                int cur = T, steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    pathBottleneck = Math.min(pathBottleneck,
                                             cap[prev][cur] - flow[prev][cur]);
                    cur = prev;
                }
                // Δ = min(path residual, remaining packets, floor(mⱼ/szᵢ))
                int delta = Math.min(pathBottleneck,
                            Math.min(remaining[idx], sinkCap[sinkOut] / szI));
                if (delta <= 0) break;

                // step 2c: augment flow along path
                cur = T; steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    flow[prev][cur] += delta;
                    flow[cur][prev] -= delta;  // update residual graph

                    // record BSN-level edge (skip s, t, internal i'->i" edges)
                    if (prev != S && prev != T && cur != S && cur != T) {
                        int bsnU = (prev-1)/2, bsnV = (cur-1)/2;
                        if (bsnU != bsnV) {
                            boolean exists = false;
                            for (int[] fe : goaFlowEdges)
                                if ((fe[0]==bsnU&&fe[1]==bsnV)||
                                    (fe[0]==bsnV&&fe[1]==bsnU))
                                    { exists=true; break; }
                            if (!exists) goaFlowEdges.add(new int[]{bsnU, bsnV});
                        }
                    }
                    cur = prev;
                }

                // step 2d: update capacities
                remaining[idx]   -= delta;           // dᵢ -= Δ
                sinkCap[sinkOut] -= delta * szI;     // mⱼ -= Δ × szᵢ
                cap[S][cfnSrc]   -= delta;
                cap[sinkOut][T]  -= delta;
                totalWeight      += (double) delta * vi;  // Vf += Δ × vᵢ

                System.out.printf(
                    "  Pushed %d packet(s) from DG %d (v=%d, sz=%d) -> sink %d%n",
                    delta, dg, vi, szI, sinkOut);
            }
            // step 2e: move to next DG when no more FAPs exist through this one
        }

        System.out.printf("%nTotal Preserved Priority (GOA): %.1f%n", totalWeight);
        launchGraph(dgNodes, storageNodes, goaFlowEdges, storageCapacity,
                    "GOA - Sort by Priority (v)");
        launchBFN(dgNodes, storageNodes, storageCapacity, goaFlowEdges,
                  "GOA - Sort by Priority (v)");
        return totalWeight;
    }

    // =========================================================================
    //
    //  ALGORITHM 2: DENSITY GOA — New Algorithm for Different Packet Sizes
    //
    //  Motivation:
    //    GOA fails when sizes differ because it ignores the storage cost
    //    of each packet. A high-priority large packet may consume storage
    //    that would otherwise hold many small packets with greater combined
    //    priority.
    //
    //  Mathematical basis:
    //    Define value density: ρᵢ = vᵢ / szᵢ
    //    This represents priority gained per storage unit consumed.
    //    Analogy: the fractional knapsack greedy, adapted to network flow.
    //
    //    Objective: maximize Vf = Σᵢ (vᵢ × |fᵢ|)
    //    Key insight: sorting by ρᵢ = vᵢ/szᵢ maximizes priority per unit
    //    of storage consumed, which is the binding constraint when sizes differ.
    //
    //  Steps:
    //    1. Compute density ρᵢ = vᵢ / szᵢ for each DG.
    //    2. Sort DGs by ρᵢ in descending order (highest density first).
    //    3. For each DG sᵢ (highest density first):
    //       a. Find shortest FAP through sᵢ in residual graph.
    //       b. Compute bottleneck Δ = min(path residual, dᵢ, floor(mⱼ/szᵢ)).
    //       c. Augment flow by Δ along the path.
    //       d. Update: dᵢ -= Δ, mⱼ -= Δ×szᵢ.
    //       e. Repeat until no more FAPs exist through sᵢ.
    //    4. Move to next DG. Stop when no FAPs remain anywhere.
    //
    //  Note: Density GOA is also not always optimal (second counterexample:
    //    v0=9,sz0=3 vs v1=8,sz1=2, cap=5 -> Density gives 16, optimal=17).
    //    However it outperforms GOA in cases where high-priority packets
    //    have large sizes relative to their priority gain per storage unit.
    //
    // =========================================================================

    public double goaDensity(List<Integer> dgNodes,
                             List<Integer> storageNodes,
                             int[]         storageCapacity) {

        buildCFN(dgNodes, storageNodes, storageCapacity);
        int S = 0, T = superSink();

        int[] remaining = new int[dgNodes.size()];
        for (int i = 0; i < dgNodes.size(); i++)
            remaining[i] = packetsPerNode[dgNodes.get(i)];

        int[] sinkCap = new int[cfnNodes];
        for (int j = 0; j < storageNodes.size(); j++)
            sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

        // step 1-2: compute and sort by density ρᵢ = vᵢ/szᵢ descending
        Integer[] order = new Integer[dgNodes.size()];
        for (int i = 0; i < order.length; i++) order[i] = i;
        Arrays.sort(order, (a, b) -> {
            double rhoA = (double) packetPriority[dgNodes.get(a)]
                        / packetSize[dgNodes.get(a)];
            double rhoB = (double) packetPriority[dgNodes.get(b)]
                        / packetSize[dgNodes.get(b)];
            return Double.compare(rhoB, rhoA);
        });

        double      totalWeight  = 0.0;
        List<int[]> goaFlowEdges = new ArrayList<>();
        int         maxIter      = cfnNodes * cfnNodes;

        System.out.println("\n-- Density Order (rho = v/sz) --");
        for (int idx : order) {
            int dg = dgNodes.get(idx);
            System.out.printf("  DG %d: v=%d, sz=%d, rho=%.2f%n",
                    dg, packetPriority[dg], packetSize[dg],
                    (double) packetPriority[dg] / packetSize[dg]);
        }

        // step 3: greedily push flow from each DG in density order
        for (int idx : order) {
            int dg     = dgNodes.get(idx);
            int cfnSrc = inNode(dg);
            int szI    = packetSize[dg];
            int vi     = packetPriority[dg];
            int iters  = 0;

            while (remaining[idx] > 0 && iters++ < maxIter) {

                // step 3a: find shortest FAP through this DG
                int[] parent = bfsFAP(cfnSrc, szI, sinkCap);
                if (parent == null) break;

                int sinkOut = parent[T];

                // step 3b: compute bottleneck Δ
                int pathBottleneck = Integer.MAX_VALUE;
                int cur = T, steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    pathBottleneck = Math.min(pathBottleneck,
                                             cap[prev][cur] - flow[prev][cur]);
                    cur = prev;
                }
                // Δ = min(path residual, remaining packets, floor(mⱼ/szᵢ))
                int delta = Math.min(pathBottleneck,
                            Math.min(remaining[idx], sinkCap[sinkOut] / szI));
                if (delta <= 0) break;

                // step 3c: augment flow along path
                cur = T; steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    flow[prev][cur] += delta;
                    flow[cur][prev] -= delta;  // update residual graph

                    if (prev != S && prev != T && cur != S && cur != T) {
                        int bsnU = (prev-1)/2, bsnV = (cur-1)/2;
                        if (bsnU != bsnV) {
                            boolean exists = false;
                            for (int[] fe : goaFlowEdges)
                                if ((fe[0]==bsnU&&fe[1]==bsnV)||
                                    (fe[0]==bsnV&&fe[1]==bsnU))
                                    { exists=true; break; }
                            if (!exists) goaFlowEdges.add(new int[]{bsnU, bsnV});
                        }
                    }
                    cur = prev;
                }

                // step 3d: update capacities
                remaining[idx]   -= delta;           // dᵢ -= Δ
                sinkCap[sinkOut] -= delta * szI;     // mⱼ -= Δ × szᵢ
                cap[S][cfnSrc]   -= delta;
                cap[sinkOut][T]  -= delta;
                totalWeight      += (double) delta * vi;  // Vf += Δ × vᵢ

                System.out.printf(
                    "  Pushed %d packet(s) from DG %d (v=%d, sz=%d, rho=%.2f) -> sink %d%n",
                    delta, dg, vi, szI, (double)vi/szI, sinkOut);
            }
            // step 3e: move to next DG when no more FAPs exist through this one
        }

        System.out.printf("%nTotal Preserved Priority (Density GOA): %.1f%n",
                          totalWeight);
        launchGraph(dgNodes, storageNodes, goaFlowEdges, storageCapacity,
                    "Density GOA - Sort by Density (v/sz)");
        launchBFN(dgNodes, storageNodes, storageCapacity, goaFlowEdges,
                  "Density GOA - Sort by Density (v/sz)");
        return totalWeight;
    }

    // =========================================================================
    //
    //  ALGORITHM 3: APPROX GOA — 2-Approximation (Adapted from Algorithm 5)
    //
    //  Motivation:
    //    Neither GOA nor Density GOA is always optimal with different sizes.
    //    We adapt Algorithm 5 from Rivera & Tang (2024) — originally designed
    //    for MWF-H — to our size-aware MWF-U setting by replacing cost cᵢ
    //    with size szᵢ in the sorting criteria.
    //
    //  Mathematical basis:
    //    Run two sub-routines and return the better result:
    //      Sub-routine A: greedy by vᵢ  (same as GOA)
    //      Sub-routine B: greedy by vᵢ/szᵢ (same as Density GOA)
    //
    //    Approximation guarantee (adapted from Theorem 5):
    //      Let Sw = sources that send in sub-routine B before first failure sⱼ.
    //      V'f = Σ(sᵢ∈Sw)(vᵢ×dᵢ)  and  vⱼ×dⱼ <= Vf (A sends highest v first)
    //      Vopt <= V'f + vⱼ×dⱼ <= V'f + Vf
    //      Since Vf = max(Vf, V'f):  Vf >= Vopt/2
    //
    //    Note: Splittable flows are used (unlike the all-or-nothing model in
    //    the original Algorithm 5), making this more practical for MWF-U
    //    where partial sends are naturally allowed.
    //
    //  Steps:
    //    1. Run sub-routine A: greedy by vᵢ, splittable flows -> result VfA
    //    2. Run sub-routine B: greedy by vᵢ/szᵢ, splittable flows -> result VfB
    //    3. Return max(VfA, VfB)
    //
    // =========================================================================

    public double goaApprox(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            int[]         storageCapacity) {

        // step 1: sub-routine A (sort by vᵢ)
        System.out.println("\n-- Approx Sub-routine A: sort by v --");
        double vfA = runGreedy(dgNodes, storageNodes, storageCapacity, false);

        // step 2: sub-routine B (sort by vᵢ/szᵢ)
        System.out.println("\n-- Approx Sub-routine B: sort by v/sz --");
        double vfB = runGreedy(dgNodes, storageNodes, storageCapacity, true);

        // step 3: return max(VfA, VfB)
        double best = Math.max(vfA, vfB);
        System.out.printf("%nApprox (A) by v:         %.1f%n", vfA);
        System.out.printf("Approx (B) by v/sz:      %.1f%n", vfB);
        System.out.printf("Approx result (max A,B):  %.1f%n", best);

        return best;
    }

    // -------------------------------------------------------------------------
    //  Shared greedy logic for both sub-routines of goaApprox.
    //  Identical structure to GOA/Density GOA — only the sort key differs.
    //  useDensity=false -> sort by vᵢ (sub-routine A)
    //  useDensity=true  -> sort by vᵢ/szᵢ (sub-routine B)
    // -------------------------------------------------------------------------

    private double runGreedy(List<Integer> dgNodes,
                             List<Integer> storageNodes,
                             int[]         storageCapacity,
                             boolean       useDensity) {

        buildCFN(dgNodes, storageNodes, storageCapacity);
        int S = 0, T = superSink();

        int[] remaining = new int[dgNodes.size()];
        for (int i = 0; i < dgNodes.size(); i++)
            remaining[i] = packetsPerNode[dgNodes.get(i)];

        int[] sinkCap = new int[cfnNodes];
        for (int j = 0; j < storageNodes.size(); j++)
            sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

        Integer[] order = new Integer[dgNodes.size()];
        for (int i = 0; i < order.length; i++) order[i] = i;
        if (useDensity) {
            Arrays.sort(order, (a, b) -> {
                double dA = (double) packetPriority[dgNodes.get(a)]
                          / packetSize[dgNodes.get(a)];
                double dB = (double) packetPriority[dgNodes.get(b)]
                          / packetSize[dgNodes.get(b)];
                return Double.compare(dB, dA);
            });
        } else {
            Arrays.sort(order, (a, b) ->
                packetPriority[dgNodes.get(b)] - packetPriority[dgNodes.get(a)]);
        }

        double totalWeight  = 0.0;
        List<int[]> flowEdges = new ArrayList<>();
        int maxIter = cfnNodes * cfnNodes;

        for (int idx : order) {
            int dg     = dgNodes.get(idx);
            int cfnSrc = inNode(dg);
            int szI    = packetSize[dg];
            int vi     = packetPriority[dg];
            int iters  = 0;

            while (remaining[idx] > 0 && iters++ < maxIter) {
                int[] parent = bfsFAP(cfnSrc, szI, sinkCap);
                if (parent == null) break;

                int sinkOut = parent[T];

                int pathBottleneck = Integer.MAX_VALUE;
                int cur = T, steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    pathBottleneck = Math.min(pathBottleneck,
                                             cap[prev][cur] - flow[prev][cur]);
                    cur = prev;
                }
                int delta = Math.min(pathBottleneck,
                            Math.min(remaining[idx], sinkCap[sinkOut] / szI));
                if (delta <= 0) break;

                cur = T; steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    flow[prev][cur] += delta;
                    flow[cur][prev] -= delta;
                    cur = prev;
                }

                remaining[idx]   -= delta;
                sinkCap[sinkOut] -= delta * szI;
                cap[S][cfnSrc]   -= delta;
                cap[sinkOut][T]  -= delta;
                totalWeight      += (double) delta * vi;

                System.out.printf(
                    "  Pushed %d packet(s) from DG %d (v=%d, sz=%d) -> sink %d%n",
                    delta, dg, vi, szI, sinkOut);
            }
        }

        System.out.printf("  Total: %.1f%n", totalWeight);
        return totalWeight;
    }

    // =========================================================================
    //  SILENT RUN — no console output, no graphs (used for scaling trials)
    // =========================================================================

    public double runSilent(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            int[]         storageCapacity,
                            boolean       useDensity) {

        buildCFN(dgNodes, storageNodes, storageCapacity);
        int S = 0, T = superSink();

        int[] remaining = new int[dgNodes.size()];
        for (int i = 0; i < dgNodes.size(); i++)
            remaining[i] = packetsPerNode[dgNodes.get(i)];

        int[] sinkCap = new int[cfnNodes];
        for (int j = 0; j < storageNodes.size(); j++)
            sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

        Integer[] order = new Integer[dgNodes.size()];
        for (int i = 0; i < order.length; i++) order[i] = i;
        if (useDensity) {
            Arrays.sort(order, (a, b) -> {
                double dA = (double) packetPriority[dgNodes.get(a)]
                          / packetSize[dgNodes.get(a)];
                double dB = (double) packetPriority[dgNodes.get(b)]
                          / packetSize[dgNodes.get(b)];
                return Double.compare(dB, dA);
            });
        } else {
            Arrays.sort(order, (a, b) ->
                packetPriority[dgNodes.get(b)] - packetPriority[dgNodes.get(a)]);
        }

        double totalWeight = 0.0;
        int maxIterations  = cfnNodes * cfnNodes;

        for (int idx : order) {
            int dg     = dgNodes.get(idx);
            int cfnSrc = inNode(dg);
            int szI    = packetSize[dg];
            int vi     = packetPriority[dg];
            int iters  = 0;

            while (remaining[idx] > 0 && iters++ < maxIterations) {
                int[] parent = bfsFAP(cfnSrc, szI, sinkCap);
                if (parent == null) break;

                int sinkOut = parent[T];

                int pathBottleneck = Integer.MAX_VALUE;
                int cur = T, steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    pathBottleneck = Math.min(pathBottleneck,
                                             cap[prev][cur] - flow[prev][cur]);
                    cur = prev;
                }
                int delta = Math.min(pathBottleneck,
                            Math.min(remaining[idx], sinkCap[sinkOut] / szI));
                if (delta <= 0) break;

                cur = T; steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    flow[prev][cur] += delta;
                    flow[cur][prev] -= delta;
                    cur = prev;
                }

                remaining[idx]   -= delta;
                sinkCap[sinkOut] -= delta * szI;
                cap[S][cfnSrc]   -= delta;
                cap[sinkOut][T]  -= delta;
                totalWeight      += (double) delta * vi;
            }
        }
        return totalWeight;
    }

    // =========================================================================
    //  SILENT RUN WITH FLOW EDGE COLLECTION (for visual run)
    // =========================================================================

    public double runSilentCollect(List<Integer> dgNodes,
                                   List<Integer> storageNodes,
                                   int[]         storageCapacity,
                                   boolean       useDensity,
                                   List<int[]>   flowEdgesOut) {

        buildCFN(dgNodes, storageNodes, storageCapacity);
        int S = 0, T = superSink();

        int[] remaining = new int[dgNodes.size()];
        for (int i = 0; i < dgNodes.size(); i++)
            remaining[i] = packetsPerNode[dgNodes.get(i)];

        int[] sinkCap = new int[cfnNodes];
        for (int j = 0; j < storageNodes.size(); j++)
            sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

        Integer[] order = new Integer[dgNodes.size()];
        for (int i = 0; i < order.length; i++) order[i] = i;
        if (useDensity) {
            Arrays.sort(order, (a, b) -> {
                double dA = (double) packetPriority[dgNodes.get(a)]
                          / packetSize[dgNodes.get(a)];
                double dB = (double) packetPriority[dgNodes.get(b)]
                          / packetSize[dgNodes.get(b)];
                return Double.compare(dB, dA);
            });
        } else {
            Arrays.sort(order, (a, b) ->
                packetPriority[dgNodes.get(b)] - packetPriority[dgNodes.get(a)]);
        }

        double totalWeight = 0.0;
        int maxIter        = cfnNodes * cfnNodes;

        for (int idx : order) {
            int dg     = dgNodes.get(idx);
            int cfnSrc = inNode(dg);
            int szI    = packetSize[dg];
            int vi     = packetPriority[dg];
            int iters  = 0;

            while (remaining[idx] > 0 && iters++ < maxIter) {
                int[] parent = bfsFAP(cfnSrc, szI, sinkCap);
                if (parent == null) break;

                int sinkOut = parent[T];

                int pathBottleneck = Integer.MAX_VALUE;
                int cur = T, steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    pathBottleneck = Math.min(pathBottleneck,
                                             cap[prev][cur] - flow[prev][cur]);
                    cur = prev;
                }
                int delta = Math.min(pathBottleneck,
                            Math.min(remaining[idx], sinkCap[sinkOut] / szI));
                if (delta <= 0) break;

                cur = T; steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    flow[prev][cur] += delta;
                    flow[cur][prev] -= delta;

                    // collect BSN flow edges for visualization
                    if (prev != S && prev != T && cur != S && cur != T) {
                        int bsnU = (prev-1)/2, bsnV = (cur-1)/2;
                        if (bsnU != bsnV) {
                            boolean exists = false;
                            for (int[] fe : flowEdgesOut)
                                if ((fe[0]==bsnU&&fe[1]==bsnV)||
                                    (fe[0]==bsnV&&fe[1]==bsnU))
                                    { exists=true; break; }
                            if (!exists) flowEdgesOut.add(new int[]{bsnU, bsnV});
                        }
                    }
                    cur = prev;
                }

                remaining[idx]   -= delta;
                sinkCap[sinkOut] -= delta * szI;
                cap[S][cfnSrc]   -= delta;
                cap[sinkOut][T]  -= delta;
                totalWeight      += (double) delta * vi;
            }
        }
        return totalWeight;
    }

    // =========================================================================
    //  BFN VISUALISATION
    // =========================================================================

    public void launchBFN(List<Integer> dgNodes,
                          List<Integer> storageNodes,
                          int[]         storageCapacity,
                          List<int[]>   goaFlowEdges,
                          String        algoTitle) {
        int n = nodeLoc.length;
        int[] storageSlots = Arrays.copyOf(storageCapacity, storageCapacity.length);

        int[][] adjMatrix = new int[n][n];
        if (adjM != null) {
            adjMatrix = adjM.getAdjM();
        } else {
            for (int u = 0; u < n; u++)
                for (int v : getAdjNodes(adjList, u))
                    adjMatrix[u][v] = 1;
        }

        BFNGraph panel = new BFNGraph();
        panel.setAlgoTitle(algoTitle);
        panel.build(n, nodeEnergy,
                    new HashSet<>(dgNodes), storageNodes,
                    nodeLoc, packetsPerNode, storageSlots,
                    adjMatrix, goaFlowEdges);

        SwingUtilities.invokeLater(panel);
    }

    // =========================================================================
    //  PHYSICAL BSN GRAPH VISUALISATION
    // =========================================================================

    public void launchGraph(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            List<int[]>   goaFlowEdges,
                            int[]         storageCapacity,
                            String        algoTitle) {
        int n = nodeLoc.length;

        Map<Integer, Axis> nodeMap = new LinkedHashMap<>();
        for (int i = 0; i < n; i++) {
            Axis a = new Axis();
            a.setxAxis(nodeLoc[i][0]);
            a.setyAxis(nodeLoc[i][1]);
            nodeMap.put(i + 1, a);
        }

        Map<Integer, Set<Integer>> adjMap = new LinkedHashMap<>();
        for (int i = 0; i < n; i++) adjMap.put(i + 1, new LinkedHashSet<>());
        int[][] bsnAdj = (adjM != null) ? adjM.getAdjM() : null;
        for (int u = 0; u < n; u++)
            for (int v : getAdjNodes(bsnAdj != null ? bsnAdj : adjList, u))
                adjMap.get(u + 1).add(v + 1);

        Set<Integer> dgIds      = new LinkedHashSet<>();
        Set<Integer> storageIds = new LinkedHashSet<>();
        for (int dg : dgNodes)      dgIds.add(dg + 1);
        for (int st : storageNodes) storageIds.add(st + 1);

        List<int[]> flowEdges1 = new ArrayList<>();
        for (int[] fe : goaFlowEdges)
            flowEdges1.add(new int[]{fe[0] + 1, fe[1] + 1});

        double maxX = 1, maxY = 1;
        for (double[] loc : nodeLoc) {
            maxX = Math.max(maxX, loc[0]);
            maxY = Math.max(maxY, loc[1]);
        }
        maxX *= 1.20;
        maxY *= 1.20;

        Map<Integer, Integer> priorityMap   = new LinkedHashMap<>();
        Map<Integer, Integer> packetSizeMap = new LinkedHashMap<>();
        Map<Integer, Integer> packetsMap    = new LinkedHashMap<>();
        Map<Integer, Integer> storageCapMap = new LinkedHashMap<>();

        for (int dg : dgNodes) {
            priorityMap.put(dg + 1,   packetPriority[dg]);
            packetSizeMap.put(dg + 1, packetSize[dg]);
            packetsMap.put(dg + 1,    packetsPerNode[dg]);
        }
        for (int j = 0; j < storageNodes.size(); j++)
            storageCapMap.put(storageNodes.get(j) + 1, storageCapacity[j]);

        SensorNetworkGraph panel = new SensorNetworkGraph();
        panel.setGraphWidth(maxX);
        panel.setGraphHeight(maxY);
        panel.setNodes(nodeMap);
        panel.setAdjList(adjMap);
        panel.setDgNodeIds(dgIds);
        panel.setStorageNodeIds(storageIds);
        panel.setFlowEdges(flowEdges1);
        panel.setNodePriority(priorityMap);
        panel.setNodePacketSize(packetSizeMap);
        panel.setNodePackets(packetsMap);
        panel.setNodeStorageCap(storageCapMap);
        panel.setAlgoTitle(algoTitle);
        panel.setConnected(components != null && components.size() == 1);

        SwingUtilities.invokeLater(panel);
    }

    // =========================================================================
    //  MAIN
    // =========================================================================

    public static void main(String[] args) {
        Scanner kb = new Scanner(System.in);

        System.out.print("Width x and length y of sensor network: ");
        int widthX = kb.nextInt(), lenY = kb.nextInt();
        System.out.print("Transmission range (m): ");
        int TR = kb.nextInt();
        System.out.print("Graph structure - 1: adj-matrix, 2: adj-list: ");
        int choice = kb.nextInt();

        System.out.print("Network sizes to test (comma-separated, e.g. 10,50,100): ");
        String[] sizeTokens = kb.next().split(",");
        int[] networkSizes = new int[sizeTokens.length];
        for (int i = 0; i < sizeTokens.length; i++) {
            int sz = Integer.parseInt(sizeTokens[i].trim());
            networkSizes[i] = Math.max(2, sz);
        }

        System.out.print("Number of trials per network size: ");
        int trials = kb.nextInt();

        System.out.print("Min node energy level: ");
        int minE = kb.nextInt();
        System.out.print("Max node energy level: ");
        int maxE = kb.nextInt();

        System.out.print("Min overflow packets per DG: ");
        int minPkt = kb.nextInt();
        System.out.print("Max overflow packets per DG: ");
        int maxPkt = kb.nextInt();

        System.out.print("Min packet size (storage units): ");
        int minSz = kb.nextInt();
        System.out.print("Max packet size (storage units): ");
        int maxSz = kb.nextInt();

        System.out.print("Min storage capacity (storage units): ");
        int minCap = kb.nextInt();
        System.out.print("Max storage capacity (storage units): ");
        int maxCap = kb.nextInt();

        System.out.print("Min packet priority: ");
        int minPri = kb.nextInt();
        System.out.print("Max packet priority: ");
        int maxPri = kb.nextInt();

        Random rand = new Random();

        // ── single visual run before scaling ──────────────────────────────────
        System.out.println("\n==== Visual Run (graphs for one trial) ====");
        System.out.print("Number of nodes for visual run: ");
        int visNodes = Math.max(2, kb.nextInt());

        SensorStuff visNet = new SensorStuff(visNodes, choice);
        visNet.randomNodes(widthX, lenY);
        visNet.createE(TR);
        visNet.randomNodeEnergies(minE, maxE);
        visNet.randomDataPackets(minPkt, maxPkt);
        visNet.randomPacketSizes(minSz, maxSz);
        visNet.randomPacketPriorities(minPri, maxPri);

        // find components
        boolean[] visVisited = new boolean[visNodes];
        visNet.components = new ArrayList<>();
        for (int i = 0; i < visNodes; i++)
            if (!visVisited[i])
                visNet.components.add(buildBFS(
                    visNet.adjM != null ? visNet.adjM.getAdjM()
                                       : visNet.adjList, i, visVisited));

        // random DG/storage split
        List<Integer> visDG  = new ArrayList<>();
        List<Integer> visST  = new ArrayList<>();
        List<Integer> visAll = new ArrayList<>();
        for (int i = 0; i < visNodes; i++) visAll.add(i);
        Collections.shuffle(visAll, rand);
        visDG.add(visAll.get(0));
        visST.add(visAll.get(1));
        for (int i = 2; i < visNodes; i++) {
            if (rand.nextBoolean()) visDG.add(visAll.get(i));
            else                   visST.add(visAll.get(i));
        }

        // storage capacity from user input range
        int visTotalCap = 0;
        int[] visCap    = new int[visST.size()];
        for (int j = 0; j < visCap.length; j++) {
            visCap[j]    = (minCap == maxCap) ? minCap
                         : rand.nextInt(maxCap - minCap + 1) + minCap;
            visTotalCap += visCap[j];
        }

        // print setup
        System.out.println("\n-- Visual Run DG Setup --");
        for (int dg : visDG)
            System.out.printf(
                "  DG %d: packets=%d, size=%d, priority=%d, energy=%d%n",
                dg, visNet.packetsPerNode[dg], visNet.packetSize[dg],
                visNet.packetPriority[dg], visNet.nodeEnergy[dg]);
        System.out.println("-- Visual Run Storage Setup --");
        for (int j = 0; j < visST.size(); j++)
            System.out.printf(
                "  Storage %d: capacity=%d, energy=%d%n",
                visST.get(j), visCap[j],
                visNet.nodeEnergy[visST.get(j)]);

        // run algorithms silently then launch all 4 graphs
        List<int[]> goaEdges     = new ArrayList<>();
        List<int[]> densityEdges = new ArrayList<>();
        double visGOA     = visNet.runSilentCollect(visDG, visST, visCap,
                                                    false, goaEdges);
        double visDensity = visNet.runSilentCollect(visDG, visST, visCap,
                                                    true,  densityEdges);
        System.out.printf("  GOA total priority:         %.1f%n", visGOA);
        System.out.printf("  Density GOA total priority: %.1f%n", visDensity);

        visNet.launchGraph(visDG, visST, goaEdges,     visCap,
                           "GOA - Sort by Priority (v)");
        visNet.launchBFN(visDG,   visST, visCap,
                         goaEdges, "GOA - Sort by Priority (v)");
        visNet.launchGraph(visDG, visST, densityEdges, visCap,
                           "Density GOA - Sort by Density (v/sz)");
        visNet.launchBFN(visDG,   visST, visCap,
                         densityEdges, "Density GOA - Sort by Density (v/sz)");

        System.out.println("\n-- Graphs launched. Starting scaling runs... --\n");

        // ── scaling loop ──────────────────────────────────────────────────────
        for (int numNodes : networkSizes) {
            System.out.printf(
                "%n==== Network Size: %d nodes, %d trials ====%n",
                numNodes, trials);

            double sumGOA = 0, sumDensity = 0, sumApprox = 0;
            int densityBeatGOA = 0, goaBeatDensity = 0, tied = 0;

            for (int t = 1; t <= trials; t++) {

                SensorStuff net = new SensorStuff(numNodes, choice);
                net.randomNodes(widthX, lenY);
                net.createE(TR);
                net.randomNodeEnergies(minE, maxE);
                net.randomDataPackets(minPkt, maxPkt);
                net.randomPacketSizes(minSz, maxSz);
                net.randomPacketPriorities(minPri, maxPri);

                // find components quietly
                boolean[] visited = new boolean[numNodes];
                net.components = new ArrayList<>();
                for (int i = 0; i < numNodes; i++)
                    if (!visited[i])
                        net.components.add(buildBFS(
                            net.adjM != null ? net.adjM.getAdjM()
                                             : net.adjList, i, visited));

                // random DG/storage split
                List<Integer> dgNodes      = new ArrayList<>();
                List<Integer> storageNodes = new ArrayList<>();
                List<Integer> allNodes     = new ArrayList<>();
                for (int i = 0; i < numNodes; i++) allNodes.add(i);
                Collections.shuffle(allNodes, rand);
                dgNodes.add(allNodes.get(0));
                storageNodes.add(allNodes.get(1));
                for (int i = 2; i < numNodes; i++) {
                    if (rand.nextBoolean()) dgNodes.add(allNodes.get(i));
                    else                   storageNodes.add(allNodes.get(i));
                }

                // storage capacity from user input range
                int[] storageCap = new int[storageNodes.size()];
                for (int j = 0; j < storageCap.length; j++)
                    storageCap[j] = (minCap == maxCap) ? minCap
                        : rand.nextInt(maxCap - minCap + 1) + minCap;

                // silent run
                double goaResult     = net.runSilent(dgNodes, storageNodes,
                                                     storageCap, false);
                double densityResult = net.runSilent(dgNodes, storageNodes,
                                                     storageCap, true);
                double approxResult  = Math.max(goaResult, densityResult);

                System.out.printf(
                    "  Trial %2d: GOA=%.1f  Density=%.1f  Approx=%.1f%n",
                    t, goaResult, densityResult, approxResult);

                sumGOA     += goaResult;
                sumDensity += densityResult;
                sumApprox  += approxResult;

                if (densityResult > goaResult)      densityBeatGOA++;
                else if (goaResult > densityResult) goaBeatDensity++;
                else                                tied++;
            }

            // print averages
            System.out.printf("%n-- Averages over %d trials --%n", trials);
            System.out.printf("  GOA avg:              %.2f%n",
                              sumGOA     / trials);
            System.out.printf("  Density GOA avg:      %.2f%n",
                              sumDensity / trials);
            System.out.printf("  Approx GOA avg:       %.2f%n",
                              sumApprox  / trials);
            System.out.printf("  Density beat GOA:     %d/%d trials%n",
                              densityBeatGOA, trials);
            System.out.printf("  GOA beat Density:     %d/%d trials%n",
                              goaBeatDensity, trials);
            System.out.printf("  Tied:                 %d/%d trials%n",
                              tied, trials);
        }

        kb.close();
    }

    // =========================================================================
    //  ORIGINAL METHODS
    // =========================================================================

    public static double distanceNodes(double[] n1, double[] n2) {
        double dx = n2[0]-n1[0], dy = n2[1]-n1[1];
        return Math.sqrt(dy*dy + dx*dx);
    }

    public void randomNodes(int w, int l) {
        Random r = new Random();
        for (int i = 0; i < nodeLoc.length; i++) {
            nodeLoc[i][0] = r.nextDouble() * w;
            nodeLoc[i][1] = r.nextDouble() * l;
        }
    }

    public void createE(int TR) {
        int n = nodeLoc.length;
        for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++)
                if (distanceNodes(nodeLoc[i], nodeLoc[j]) <= TR) {
                    if (adjM != null) adjM.addE(i, j);
                    else { adjList.get(i).add(j); adjList.get(j).add(i); }
                }
    }

    public void printConnectedBFS() {
        boolean[] visited = new boolean[nodeLoc.length];
        components = new ArrayList<>();
        for (int i = 0; i < visited.length; i++)
            if (!visited[i])
                components.add(buildBFS(
                    adjM != null ? adjM.getAdjM() : adjList, i, visited));
        System.out.println("Connected components (BFS):");
        components.forEach(System.out::println);
    }

    public void printConnectedDFS() {
        boolean[] visited = new boolean[nodeLoc.length];
        components = new ArrayList<>();
        for (int i = 0; i < visited.length; i++)
            if (!visited[i])
                components.add(buildDFS(
                    adjM != null ? adjM.getAdjM() : adjList, i, visited));
        System.out.println("Connected components (DFS):");
        components.forEach(System.out::println);
    }

    public static List<Integer> buildBFS(Object g, int src, boolean[] visited) {
        Queue<Integer> q = new LinkedList<>();
        List<Integer> comp = new ArrayList<>();
        visited[src] = true; q.add(src); comp.add(src);
        while (!q.isEmpty()) {
            int cur = q.poll();
            for (int nb : getAdjNodes(g, cur))
                if (!visited[nb]) {
                    visited[nb] = true; q.add(nb); comp.add(nb);
                }
        }
        return comp;
    }

    public List<Integer> buildDFS(Object g, int src, boolean[] visited) {
        List<Integer> comp = new ArrayList<>();
        Stack<Integer> stk = new Stack<>();
        stk.push(src);
        while (!stk.isEmpty()) {
            int cur = stk.pop();
            if (!visited[cur]) { visited[cur] = true; comp.add(cur); }
            for (int nb : getAdjNodes(g, cur))
                if (!visited[nb]) stk.push(nb);
        }
        return comp;
    }

    public static List<Integer> getAdjNodes(Object g, int cur) {
        List<Integer> adj = new ArrayList<>();
        if (g instanceof int[][]) {
            int[][] m = (int[][]) g;
            for (int v = 0; v < m.length; v++) if (m[cur][v] == 1) adj.add(v);
        } else if (g instanceof List) {
            adj = ((List<List<Integer>>) g).get(cur);
        }
        return adj;
    }

    public void selectRendPoints() {
        Random r = new Random();
        for (List<Integer> c : components)
            rendPoints.add(c.get(r.nextInt(c.size())));
    }

    public void computeEnergyConsumption() {
        for (int i = 0; i < components.size(); i++) {
            List<Integer> comp = components.get(i);
            int rend = rendPoints.get(i);
            mstEdges = new ArrayList<>();
            double dE = computeEDijkstra(rend, comp);
            double pE = primMST(adjM != null ? adjM.getAdjM() : adjList,
                                rend, comp);
            dijkstraPerComponentE.add(dE);
            primPerComponentE.add(pE);
            totalEDijkstra += dE;
            totalEPrim     += pE;
        }
        energyToString();
    }

    public double computeEDijkstra(int rend, List<Integer> comp) {
        double E = 0;
        for (int node : comp) {
            if (node == rend) continue;
            E += computePathE(shortestP(node, rend), node);
        }
        return E;
    }

    public double computePathE(List<Integer> path, int s) {
        double E = 0;
        int pkts = packetsPerNode[s];
        for (int i = 0; i < path.size() - 1; i++) {
            double d = distanceNodes(nodeLoc[path.get(i)],
                                     nodeLoc[path.get(i+1)]);
            E += computeTEnergy(pkts, d) + computeREnergy(pkts);
        }
        return E;
    }

    public double computeTEnergy(int n, double d) {
        double b = n * (double) PACKET_SIZE;
        return Eelect * b + Eamp * b * d * d;
    }

    public double computeREnergy(int n) {
        return Eelect * n * (double) PACKET_SIZE;
    }

    public double computeTourEnergy(List<Integer> tour) {
        double dist = 0;
        double[] depot = {0, 0};
        for (int i = 0; i < tour.size(); i++) {
            double[] cur  = tour.get(i) == -1 ? depot : nodeLoc[tour.get(i)];
            double[] next = (i+1 < tour.size())
                    ? (tour.get(i+1) == -1 ? depot : nodeLoc[tour.get(i+1)])
                    : depot;
            dist += distanceNodes(cur, next);
        }
        return dist * 100;
    }

    public double primMST(Object g, int rend, List<Integer> comp) {
        PriorityQueue<Edge> pq  = new PriorityQueue<>();
        HashSet<Integer>    vis = new HashSet<>();
        HashMap<Integer,Integer> ovPkts = new HashMap<>();
        double totalE = 0;
        for (int nd : comp) ovPkts.put(nd, packetsPerNode[nd]);
        vis.add(rend);
        for (int nb : getAdjNodes(g, rend))
            if (comp.contains(nb) && !vis.contains(nb))
                pq.add(new Edge(rend, nb,
                       distanceNodes(nodeLoc[rend], nodeLoc[nb])));
        while (!pq.isEmpty() && vis.size() < comp.size()) {
            Edge e = pq.poll();
            if (vis.contains(e.node2)) continue;
            vis.add(e.node2); mstEdges.add(e);
            int pkts = ovPkts.get(e.node2);
            totalE += computeTEnergy(pkts, e.edgeW) + computeREnergy(pkts);
            ovPkts.put(e.node1, ovPkts.get(e.node1) + pkts);
            for (int nb : getAdjNodes(g, e.node2))
                if (comp.contains(nb) && !vis.contains(nb))
                    pq.add(new Edge(e.node2, nb,
                           distanceNodes(nodeLoc[e.node2], nodeLoc[nb])));
        }
        return totalE;
    }

    public List<Integer> shortestP(int s, int dest) {
        double[]  dist = new double[nodeLoc.length];
        int[]     prev = new int[nodeLoc.length];
        boolean[] enc  = new boolean[nodeLoc.length];
        Arrays.fill(dist, Double.MAX_VALUE);
        Arrays.fill(prev, -1);
        dist[s] = 0;
        PriorityQueue<Integer> pq =
                new PriorityQueue<>(Comparator.comparingDouble(o -> dist[o]));
        pq.add(s);
        while (!pq.isEmpty()) {
            int u = pq.poll();
            if (u == dest) break;
            if (enc[u]) continue;
            enc[u] = true;
            for (int v : getAdjNodes(
                    adjM != null ? adjM.getAdjM() : adjList, u)) {
                double d = distanceNodes(nodeLoc[u], nodeLoc[v]);
                double e = computeTEnergy(1, d) + computeREnergy(1);
                if (dist[u]+e < dist[v]) {
                    dist[v] = dist[u]+e; prev[v] = u; pq.add(v);
                }
            }
        }
        List<Integer> path = new ArrayList<>();
        int cur = dest;
        if (prev[cur] != -1 || cur == s) {
            while (cur != -1 && cur != s) { path.add(cur); cur = prev[cur]; }
            path.add(s);
            Collections.reverse(path);
        }
        return path;
    }

    public void energyToString() {
        StringBuilder sb = new StringBuilder(
            "\nConnected Components and Rendezvous Points:\n");
        for (int i = 0; i < components.size(); i++) {
            sb.append("Component ").append(i+1).append(": ");
            for (int nd : components.get(i)) sb.append(nd+1).append(" ");
            sb.append(" Rendezvous: ").append(rendPoints.get(i)+1).append("\n");
            sb.append(String.format("  Energy (Dijkstra): %.3f mJ%n",
                      dijkstraPerComponentE.get(i)));
            sb.append(String.format("  Energy (Prim MST): %.3f mJ%n",
                      primPerComponentE.get(i)));
        }
        sb.append(String.format("Total Energy (Dijkstra): %.3f mJ%n",
                  totalEDijkstra));
        sb.append(String.format("Total Energy (Prim MST): %.3f mJ%n",
                  totalEPrim));
        System.out.print(sb);
    }

    public List<Integer> approxTSPConstruct(List<Integer> pts, int rend) {
        List<Integer> nodes = new ArrayList<>(pts);
        nodes.add(-1);
        Map<Integer,List<Integer>> mst = buildMST(nodes);
        List<Integer> order = new ArrayList<>();
        boolean[] vis = new boolean[nodeLoc.length + 1];
        preOrderTraversal(-1, mst, vis, order);
        order.add(-1);
        return order;
    }

    public Map<Integer,List<Integer>> buildMST(List<Integer> nodes) {
        Map<Integer,List<Integer>> mst = new HashMap<>();
        PriorityQueue<Edge> pq  = new PriorityQueue<>();
        Set<Integer>        vis = new HashSet<>();
        vis.add(-1);
        for (Integer nd : nodes)
            if (nd != -1)
                pq.add(new Edge(-1, nd,
                       distanceNodes(new double[]{0,0}, getNodeLoc(nd))));
        while (!pq.isEmpty() && vis.size() < nodes.size()) {
            Edge e = pq.poll();
            if (vis.contains(e.node2)) continue;
            vis.add(e.node2);
            mst.computeIfAbsent(e.node1, k->new ArrayList<>()).add(e.node2);
            mst.computeIfAbsent(e.node2, k->new ArrayList<>()).add(e.node1);
            for (Integer nd : nodes)
                if (!vis.contains(nd))
                    pq.add(new Edge(e.node2, nd,
                           distanceNodes(getNodeLoc(e.node2),
                                         getNodeLoc(nd))));
        }
        return mst;
    }

    private double[] getNodeLoc(int nd) {
        return nd == -1 ? new double[]{0,0} : nodeLoc[nd];
    }

    public void preOrderTraversal(int cur,
                                  Map<Integer,List<Integer>> mst,
                                  boolean[] vis, List<Integer> order) {
        int idx = cur == -1 ? nodeLoc.length : cur;
        vis[idx] = true;
        order.add(cur);
        for (int nb : mst.getOrDefault(cur, new ArrayList<>())) {
            int nIdx = nb == -1 ? nodeLoc.length : nb;
            if (!vis[nIdx]) preOrderTraversal(nb, mst, vis, order);
        }
    }

    public List<Integer> compWithNode(int node) {
        for (List<Integer> c : components)
            if (c.contains(node)) return c;
        return Collections.emptyList();
    }
}