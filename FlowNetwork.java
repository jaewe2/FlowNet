import java.util.*;

// =============================================================================
//  FlowNetwork — BSN-based Flow Network (BFN/CFN) state and core primitives
//
//  Holds the directed flow graph that all MWF algorithms operate on.
//  Encapsulates:
//    - cap[][], flow[][] adjacency arrays
//    - CFN construction (buildCFN)
//    - BFS for feasible augmenting paths (bfsFAP)
//    - Reachable residual storage computation (computeReachableResidual)
//    - Path augmentation (augmentPath)
//    - Deep copy utility (deepCopy2D)
//
//  NODE INDEX LAYOUT:
//    0        = super source s
//    2i + 1   = in-node  i'   for physical node i
//    2i + 2   = out-node i"   for physical node i
//    2n + 1   = super sink t
//
//  EDGE CONSTRUCTION (see buildCFN):
//    s  → i'  cap = dᵢ   (DG overflow packets)
//    i' → i"  cap = Eᵢ   (node energy — enforces relay cost)
//    u" → v'  cap = INF  (BSN communication edge)
//    j" → t   cap = mⱼ   (storage capacity in raw storage units)
// =============================================================================

public class FlowNetwork {

    // ── public fields (read/write by algorithms via snapshot/restore) ──────────
    public int     cfnNodes;
    public int[][] cap;
    public int[][] flow;

    // ── package-private references to BSN node data ───────────────────────────
    final int[]   packetSize;
    final int[]   packetPriority;
    final int[]   nodeEnergy;
    final int[]   packetsPerNode;
    final double[][] nodeLoc;

    // ── internal ──────────────────────────────────────────────────────────────
    private Set<Integer> dgCFNIds   = new HashSet<>();
    int                  cachedMaxSz = 1;

    // ── constructor ───────────────────────────────────────────────────────────

    public FlowNetwork(int[]    packetSize,
                       int[]    packetPriority,
                       int[]    nodeEnergy,
                       int[]    packetsPerNode,
                       double[][] nodeLoc) {
        this.packetSize     = packetSize;
        this.packetPriority = packetPriority;
        this.nodeEnergy     = nodeEnergy;
        this.packetsPerNode = packetsPerNode;
        this.nodeLoc        = nodeLoc;
    }

    // ── CFN node index helpers ─────────────────────────────────────────────────

    public int inNode(int i)  { return 2 * i + 1; }
    public int outNode(int i) { return 2 * i + 2; }
    public int superSink()    { return 2 * nodeLoc.length + 1; }

    // ── BFN construction ──────────────────────────────────────────────────────
    // Transforms the physical BSN graph into a directed flow network so that
    // standard max-flow algorithms can enforce the energy constraints on nodes.
    // Each physical node i is split into i' (in-node) and i" (out-node);
    // the directed edge i'→i" has capacity Eᵢ (energy budget).

    public void buildCFN(List<Integer> dgNodes,
                         List<Integer> storageNodes,
                         int[]         storageCapacity,
                         Object        adjGraph) {
        int n    = nodeLoc.length;
        int S    = 0;
        int T    = 2 * n + 1;
        cfnNodes = 2 * n + 2;
        cap      = new int[cfnNodes][cfnNodes];
        flow     = new int[cfnNodes][cfnNodes];
        dgCFNIds.clear();

        cachedMaxSz = 1;
        for (int sz : packetSize) cachedMaxSz = Math.max(cachedMaxSz, sz);

        // s → i'  for each DG
        for (int dg : dgNodes) {
            cap[S][inNode(dg)] = packetsPerNode[dg];
            dgCFNIds.add(inNode(dg));
        }

        // i' → i"  for every physical node (energy internal edge)
        for (int i = 0; i < n; i++)
            cap[inNode(i)][outNode(i)] = nodeEnergy[i];

        // u" → v'  for every BSN communication edge (infinite capacity)
        int INF = Integer.MAX_VALUE / 2;
        for (int u = 0; u < n; u++)
            for (int v : SensorStuff.getAdjNodes(adjGraph, u))
                cap[outNode(u)][inNode(v)] = INF;

        // j" → t  for each storage node
        for (int j = 0; j < storageNodes.size(); j++) {
            int st = storageNodes.get(j);
            cap[outNode(st)][T] = storageCapacity[j];
        }
    }

    // ── BFS for shortest feasible augmenting path ─────────────────────────────
    // Finds the shortest path s → cfnSource → t in the residual graph.
    // Blocked from entering other DG in-nodes so flow stays attributed to the
    // correct source. Size-aware: only accepts a storage sink if
    // sinkCap[j"] >= szI (at least one packet of size szI fits).
    //
    // Returns parent[] array for path reconstruction, or null if no path.

    public int[] bfsFAP(int cfnSource, int szI, int[] sinkCap) {
        int S = 0, T = superSink();
        int[]     parent = new int[cfnNodes];
        boolean[] vis    = new boolean[cfnNodes];
        Arrays.fill(parent, -1);

        // phase 1: BFS from S until we reach cfnSource
        Queue<Integer> q = new LinkedList<>();
        q.add(S); vis[S] = true; parent[S] = S;

        boolean reached = false;
        while (!q.isEmpty()) {
            int u = q.poll();
            if (u == cfnSource) { reached = true; break; }
            for (int v = 0; v < cfnNodes; v++) {
                if (vis[v] || v == T) continue;
                if (cap[u][v] - flow[u][v] <= 0) continue;
                vis[v] = true; parent[v] = u; q.add(v);
            }
        }
        if (!reached) return null;

        // phase 2: BFS from cfnSource to T, blocking other DG in-nodes
        boolean[] vis2 = new boolean[cfnNodes];
        Queue<Integer> q2 = new LinkedList<>();
        q2.add(cfnSource); vis2[cfnSource] = true;

        while (!q2.isEmpty()) {
            int u = q2.poll();
            for (int v = 0; v < cfnNodes; v++) {
                if (vis2[v]) continue;
                if (cap[u][v] - flow[u][v] <= 0) continue;
                if (v != T && v != cfnSource && dgCFNIds.contains(v)) continue;
                if (v == T) {
                    if (sinkCap[u] < szI) continue;
                    parent[v] = u;
                    return parent;
                }
                vis2[v] = true; parent[v] = u; q2.add(v);
            }
        }
        return null;
    }

    // ── Compute bottleneck delta without augmenting (used for pre-logging) ─────

    public int computeDelta(int[] parent, int idx, int[] remaining,
                            int[] sinkCap, int szI) {
        int S = 0, T = superSink();
        int sinkOut = parent[T];

        // skip the sink edge (sinkOut→T): its cap is in storage units,
        // not packet units, so including it corrupts the bottleneck min.
        // sinkCap[sinkOut]/szI correctly caps packet flow at the sink.
        int pathBottleneck = Integer.MAX_VALUE;
        int cur = T, steps = 0;
        while (cur != S && steps++ < cfnNodes) {
            int prev = parent[cur];
            if (prev == -1 || prev == cur) break;
            if (cur != T)
                pathBottleneck = Math.min(pathBottleneck,
                                         cap[prev][cur] - flow[prev][cur]);
            cur = prev;
        }
        return Math.min(pathBottleneck, Math.min(remaining[idx], sinkCap[sinkOut] / szI));
    }

    // ── Path augmentation ─────────────────────────────────────────────────────
    // Computes bottleneck Δ along parent[] path from S to T, augments flow,
    // updates remaining[idx] and sinkCap[sinkOut].
    // Returns priority gained (Δ × vi), or 0 if no progress.
    //
    // Optionally collects BSN-level flow edges into flowEdgesOut if non-null.

    public int augmentPath(int[] parent,
                           int   idx,
                           int[] remaining,
                           int[] sinkCap,
                           int   szI,
                           int   vi,
                           List<int[]> flowEdgesOut) {
        int S = 0, T = superSink();

        int sinkOut = parent[T];

        // compute bottleneck — skip the sink edge (sinkOut→T) because its
        // cap is in storage units while all other edges are in packet units;
        // sinkCap[sinkOut] already enforces the storage constraint correctly.
        int pathBottleneck = Integer.MAX_VALUE;
        int cur = T, steps = 0;
        while (cur != S && steps++ < cfnNodes) {
            int prev = parent[cur];
            if (prev == -1 || prev == cur) break;
            if (cur != T)   // skip the sink edge to avoid unit mismatch
                pathBottleneck = Math.min(pathBottleneck,
                                         cap[prev][cur] - flow[prev][cur]);
            cur = prev;
        }
        int delta = Math.min(pathBottleneck,
                    Math.min(remaining[idx], sinkCap[sinkOut] / szI));
        if (delta <= 0) return 0;

        // augment flow; the sink edge is augmented by delta*szI (storage units)
        // so the CFN edge cap correctly tracks remaining physical storage.
        cur = T; steps = 0;
        while (cur != S && steps++ < cfnNodes) {
            int prev = parent[cur];
            if (prev == -1 || prev == cur) break;
            int amount = (cur == T) ? delta * szI : delta;
            flow[prev][cur] += amount;
            flow[cur][prev] -= amount;

            // collect BSN-level edges if caller wants them
            if (flowEdgesOut != null
                    && prev != S && prev != T && cur != S && cur != T) {
                int bsnU = (prev - 1) / 2, bsnV = (cur - 1) / 2;
                if (bsnU != bsnV) {
                    boolean exists = false;
                    for (int[] fe : flowEdgesOut)
                        if (fe[0] == bsnU && fe[1] == bsnV) { exists = true; break; }
                    if (!exists) flowEdgesOut.add(new int[]{bsnU, bsnV});
                }
            }
            cur = prev;
        }

        remaining[idx]   -= delta;
        sinkCap[sinkOut] -= delta * szI;
        return delta * vi;
    }

    // ── Fully augment one DG (silent — no printing, no edge collection) ───────
    // Used by ExactSolver, rollout simulations, and warm-start.
    // Returns total priority gained from this DG.

    public int augmentDGSilent(int           idx,
                               List<Integer> dgNodes,
                               int[]         remaining,
                               int[]         sinkCap) {
        int dg     = dgNodes.get(idx);
        int cfnSrc = inNode(dg);
        int szI    = packetSize[dg];
        int vi     = packetPriority[dg];
        int total  = 0;
        int maxIter = cfnNodes * cfnNodes;
        int iters  = 0;

        while (remaining[idx] > 0 && iters++ < maxIter) {
            int[] parent = bfsFAP(cfnSrc, szI, sinkCap);
            if (parent == null) break;
            int gained = augmentPath(parent, idx, remaining, sinkCap, szI, vi, null);
            if (gained <= 0) break;
            total += gained;
        }
        return total;
    }

    // ── Reachable residual helper (used by DDR-GOA and exact solver UB) ───────
    // BFS from cfnSource on the residual CFN; sums residual capacity of all
    // j"→t edges where sinkCap[j"] >= szI.

    public int computeReachableResidual(int           cfnSource,
                                        int           szI,
                                        int[]         sinkCap) {
        int T = superSink();
        boolean[] vis = new boolean[cfnNodes];
        Queue<Integer> q = new LinkedList<>();
        q.add(cfnSource); vis[cfnSource] = true;

        int totalResidual = 0;
        while (!q.isEmpty()) {
            int u = q.poll();
            for (int v = 0; v < cfnNodes; v++) {
                if (vis[v]) continue;
                if (cap[u][v] - flow[u][v] <= 0) continue;
                if (v != T && v != cfnSource && dgCFNIds.contains(v)) continue;
                if (v == T) {
                    if (sinkCap[u] >= szI) totalResidual += sinkCap[u];
                    continue;
                }
                vis[v] = true;
                q.add(v);
            }
        }
        return totalResidual;
    }

    // ── Per-sink reachable residual (used by DDR+ contention sharing) ────────
    // BFS from cfnSource on residual CFN; returns map from sink-out-node to
    // the residual capacity at that sink (only sinks with sinkCap >= szI).
    // Differs from computeReachableResidual which sums all reachable sinks;
    // this version preserves the per-sink breakdown so a contention-aware
    // caller can share each sink's capacity across competing DGs.

    public Map<Integer, Integer> computeReachablePerSink(int   cfnSource,
                                                         int   szI,
                                                         int[] sinkCap) {
        int T = superSink();
        boolean[] vis = new boolean[cfnNodes];
        Queue<Integer> q = new LinkedList<>();
        q.add(cfnSource); vis[cfnSource] = true;

        Map<Integer, Integer> perSink = new LinkedHashMap<>();
        while (!q.isEmpty()) {
            int u = q.poll();
            for (int v = 0; v < cfnNodes; v++) {
                if (vis[v]) continue;
                if (cap[u][v] - flow[u][v] <= 0) continue;
                if (v != T && v != cfnSource && dgCFNIds.contains(v)) continue;
                if (v == T) {
                    if (sinkCap[u] >= szI) perSink.put(u, sinkCap[u]);
                    continue;
                }
                vis[v] = true;
                q.add(v);
            }
        }
        return perSink;
    }

    // ── Relay energy penalty (used by PSB-GOA scoring) ────────────────────────
    // Walks path from T back to S; for each internal relay node (not the DG
    // source), adds scale × delta / residualEnergy_r to the penalty.
    // Uses residual energy (cap - flow) so nearly-depleted relays score high.

    public double computeRelayPenalty(int[] parent, int dgNode, int delta) {
        int S = 0, T = superSink();
        double penalty = 0.0;
        double scale   = cachedMaxSz;

        int cur = T, steps = 0;
        while (cur != S && steps++ < cfnNodes) {
            int prev = parent[cur];
            if (prev == -1 || prev == cur) break;
            // detect internal edge i'→i": prev=inNode(r), cur=outNode(r)
            if (prev > 0 && cur > 0 && prev != T && cur != T) {
                int bsnPrev = (prev - 1) / 2;
                int bsnCur  = (cur  - 1) / 2;
                if (bsnPrev == bsnCur && bsnPrev != dgNode) {
                    int residualEnergy = cap[prev][cur] - flow[prev][cur];
                    if (residualEnergy > 0) {
                        penalty += scale * (double) delta / residualEnergy;
                    } else {
                        penalty += scale * delta * 10.0;
                    }
                }
            }
            cur = prev;
        }
        return penalty;
    }

    // ── State snapshot / restore (used by rollout and B&B) ────────────────────

    public int[][] snapCap()  { return deepCopy2D(cap);  }
    public int[][] snapFlow() { return deepCopy2D(flow); }

    public void restoreCap(int[][] snap)  { cap  = snap; }
    public void restoreFlow(int[][] snap) { flow = snap; }

    public static int[][] deepCopy2D(int[][] src) {
        int[][] copy = new int[src.length][];
        for (int i = 0; i < src.length; i++) copy[i] = src[i].clone();
        return copy;
    }

    // ── Initialise sinkCap array from storageCapacity ─────────────────────────

    public int[] makeSinkCap(List<Integer> storageNodes, int[] storageCapacity) {
        int[] sinkCap = new int[cfnNodes];
        for (int j = 0; j < storageNodes.size(); j++)
            sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];
        return sinkCap;
    }

    // ── Prefix builder used by Hybrid/PSB algorithms ──────────────────────────
    // Returns empty prefix + all k singletons + all k(k-1) ordered pairs.

    public static List<List<Integer>> buildPEPrefixes(int n) {
        List<List<Integer>> prefixes = new ArrayList<>();
        prefixes.add(new ArrayList<>());
        for (int a = 0; a < n; a++) {
            List<Integer> p = new ArrayList<>();
            p.add(a);
            prefixes.add(p);
        }
        for (int a = 0; a < n; a++) {
            for (int b = 0; b < n; b++) {
                if (a == b) continue;
                List<Integer> p = new ArrayList<>();
                p.add(a); p.add(b);
                prefixes.add(p);
            }
        }
        return prefixes;
    }

    // ── Path utility: extract BSN-level nodes from parent array ──────────────

    public List<Integer> extractBSNPath(int[] parent) {
        int S = 0, T = superSink();
        List<Integer> cfnPath = new ArrayList<>();
        int cur = T, guard = 0;
        cfnPath.add(T);
        while (cur != S && guard++ < cfnNodes) {
            int prev = parent[cur];
            if (prev == -1 || prev == cur) break;
            cfnPath.add(prev);
            cur = prev;
        }
        Collections.reverse(cfnPath);

        List<Integer> bsnPath = new ArrayList<>();
        for (int node : cfnPath) {
            int bsn = (node <= 0 || node == superSink()) ? -1 : (node - 1) / 2;
            if (bsn == -1) continue;
            if (bsnPath.isEmpty() || bsnPath.get(bsnPath.size() - 1) != bsn)
                bsnPath.add(bsn);
        }
        return bsnPath;
    }
}
