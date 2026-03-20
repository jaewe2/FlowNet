import java.util.*;
import javax.swing.SwingUtilities;

public class SensorStuff {
    private static final double Eelect     = .0001;
    private static final double Eamp       = 0.0000001;
    private static final int    PACKET_SIZE = 3200;

    protected int[] packetSize;
    protected int[] packetPriority;
    protected int[] nodeEnergy;

    private int     cfnNodes;
    private int[][] cap;
    private int[][] flow;
    private Set<Integer> dgCFNIds = new HashSet<>();

    private int inNode(int i)  { return 2*i + 1; }
    private int outNode(int i) { return 2*i + 2; }
    private int superSink()    { return 2*nodeLoc.length + 1; }

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

    protected AdjacentMatrix                 adjM;
    protected ArrayList<LinkedList<Integer>> adjList;
    protected double[][]                     nodeLoc;
    protected List<List<Integer>>            components;
    protected int[]                          packetsPerNode;
    protected List<Edge>                     mstEdges   = new ArrayList<>();
    protected List<Integer>                  rendPoints = new ArrayList<>();
    protected double                         totalEDijkstra = 0.0;
    protected double                         totalEPrim     = 0.0;
    protected List<Double>                   dijkstraPerComponentE = new ArrayList<>();
    protected List<Double>                   primPerComponentE     = new ArrayList<>();

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

    public void randomNodeEnergies(int minE, int maxE) {
        Random rand = new Random();
        for (int i = 0; i < nodeEnergy.length; i++)
            nodeEnergy[i] = (minE == maxE) ? minE
                          : rand.nextInt(maxE - minE + 1) + minE;
    }

    // =========================================================================
    //  IMPROVED: CLUSTERED + SCATTERED NODE PLACEMENT
    //
    //  Randomly picks 1–3 cluster centres per trial. 70% of nodes are placed
    //  near a cluster (Gaussian spread), 30% are fully random outliers.
    //  This produces varied topologies: some trials dense/clustered, others
    //  sparse/scattered — so GOA vs Density GOA see genuinely different graphs.
    // =========================================================================

    public void randomNodes(int w, int l) {
        Random r = new Random();
        int numClusters = 1 + r.nextInt(3);
        double[] cx = new double[numClusters];
        double[] cy = new double[numClusters];
        for (int k = 0; k < numClusters; k++) {
            cx[k] = r.nextDouble() * w;
            cy[k] = r.nextDouble() * l;
        }
        double spread = Math.min(w, l) * 0.25;
        for (int i = 0; i < nodeLoc.length; i++) {
            if (r.nextDouble() < 0.3) {
                nodeLoc[i][0] = r.nextDouble() * w;
                nodeLoc[i][1] = r.nextDouble() * l;
            } else {
                int k = r.nextInt(numClusters);
                nodeLoc[i][0] = Math.max(0, Math.min(w, cx[k] + r.nextGaussian() * spread));
                nodeLoc[i][1] = Math.max(0, Math.min(l, cy[k] + r.nextGaussian() * spread));
            }
        }
    }

    // =========================================================================
    // ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
    //  BSN-BASED FLOW NETWORK (BFN) — GRAPH TRANSFORMATION
    //
    //  WHAT IT DOES:
    //    Converts the physical sensor network graph G(V,E) into a directed
    //    flow network G'(V',E') so that standard max-flow algorithms can be
    //    applied. This transformation is the foundation that all three
    //    algorithms (GOA, Density GOA, Approx GOA) run on top of.
    //
    //  WHY WE NEED IT:
    //    In a raw sensor graph, energy constraints live on nodes — each node
    //    has a budget of how many packets it can forward. Max-flow only
    //    supports capacity constraints on edges, not nodes. The BFN solves
    //    this by splitting each physical node i into two virtual nodes:
    //      i'  (in-node)  — receives incoming flow
    //      i"  (out-node) — sends outgoing flow
    //    The directed edge i' → i" has capacity = Eᵢ (node energy budget).
    //    Any packet passing through node i must traverse this internal edge,
    //    so the energy constraint is enforced naturally by the flow.
    //
    //  NODE LAYOUT (CFN indices):
    //    0        = super source s   — pushes flow out to all DG in-nodes
    //    2i + 1   = in-node  i'      — receives packets arriving at node i
    //    2i + 2   = out-node i"      — sends packets leaving node i
    //    2n + 1   = super sink t     — collects flow from all storage out-nodes
    //
    //  EDGE CONSTRUCTION:
    //    s  → i'  : capacity = dᵢ    — how many packets DG i can send
    //    i' → i"  : capacity = Eᵢ    — node energy budget (enforces relay cost)
    //    u" → v'  : capacity = INF   — BSN communication edge (no extra cost)
    //    j" → t   : capacity = mⱼ    — raw storage units at storage node j
    //
    //  SIZE-AWARE SINK CHECK:
    //    Storage node j only accepts a packet from DG i if sinkCap[j] >= szᵢ.
    //    After Δ packets are pushed, storage is reduced by Δ × szᵢ.
    // ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
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

        for (int dg : dgNodes) {
            cap[S][inNode(dg)] = packetsPerNode[dg];
            dgCFNIds.add(inNode(dg));
        }

        for (int i = 0; i < n; i++)
            cap[inNode(i)][outNode(i)] = nodeEnergy[i];

        int INF    = Integer.MAX_VALUE / 2;
        int[][] bsnAdj = (adjM != null) ? adjM.getAdjM() : null;
        for (int u = 0; u < n; u++)
            for (int v : getAdjNodes(bsnAdj != null ? bsnAdj : adjList, u))
                cap[outNode(u)][inNode(v)] = INF;

        for (int j = 0; j < storageNodes.size(); j++) {
            int st = storageNodes.get(j);
            cap[outNode(st)][T] = storageCapacity[j];
        }
    }

    // =========================================================================
    //  BFS FOR SHORTEST FEASIBLE AUGMENTING PATH (FAP)
    //
    //  Finds the shortest path s → cfnSource → t in the residual graph,
    //  blocked from entering other DG in-nodes so flow stays attributed
    //  to the correct source. Size-aware: only accepts a storage sink if
    //  sinkCap[j] >= szᵢ (at least one packet of that size fits).
    // =========================================================================

    private int[] bfsFAP(int cfnSource, int szI, int[] sinkCap) {
        int S = 0, T = superSink();
        int[]    parent = new int[cfnNodes];
        boolean[] vis   = new boolean[cfnNodes];
        Arrays.fill(parent, -1);

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

    // =========================================================================
    //  PRE-FLIGHT FEASIBILITY CHECK
    //
    //  Before running MWF, check whether all DG packets can be offloaded.
    //  If total storage capacity >= total packets AND total node energy across
    //  all nodes >= total packets, there is no bottleneck — MWF is unnecessary.
    //
    //  As Prof. Tang describes: "if all the flows can be offloaded, there's no
    //  point to do this because all the packets can be offloaded, there's enough
    //  energy and storage, no worries."
    //
    //  Returns true if MWF is needed (bottleneck exists), false if not.
    // =========================================================================

    public boolean needsMWF(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            int[]         storageCapacity) {
        int totalPackets = 0;
        int totalStorageDemand = 0;
        for (int dg : dgNodes) {
            totalPackets += packetsPerNode[dg];
            totalStorageDemand += packetsPerNode[dg] * packetSize[dg];
        }

        int totalStorage = 0;
        int usableStorage = 0;
        boolean hasDeadStorage = false;
        for (int j = 0; j < storageNodes.size(); j++) {
            int st = storageNodes.get(j);
            int capJ = storageCapacity[j];
            totalStorage += capJ;
            if (capJ > 0 && nodeEnergy[st] <= 0) hasDeadStorage = true;
            if (nodeEnergy[st] > 0) usableStorage += capJ;
        }

        int totalEnergy = 0;
        for (int e : nodeEnergy) totalEnergy += e;

        boolean hasDeadSource = false;
        for (int dg : dgNodes)
            if (packetsPerNode[dg] > 0 && nodeEnergy[dg] <= 0)
                hasDeadSource = true;

        boolean storageSufficient = usableStorage >= totalStorageDemand;
        boolean energySufficient  = totalEnergy   >= totalPackets;

        System.out.printf("%n-- Feasibility Check --%n");
        System.out.printf("  Total DG packets:    %d%n", totalPackets);
        System.out.printf("  Total storage need:  %d%n", totalStorageDemand);
        System.out.printf("  Total storage cap:   %d%n", totalStorage);
        System.out.printf("  Usable storage cap:  %d  (%s)%n",
                          usableStorage, storageSufficient ? "sufficient" : "BOTTLENECK");
        System.out.printf("  Total node energy:   %d  (%s)%n",
                          totalEnergy, energySufficient ? "sufficient" : "BOTTLENECK");

        if (hasDeadSource)
            System.out.println("  Dead source detected: at least one DG has packets but no energy.");
        if (hasDeadStorage)
            System.out.println("  Dead storage detected: at least one storage node has capacity but no energy.");

        if (storageSufficient && energySufficient && !hasDeadSource && !hasDeadStorage) {
            System.out.println("  >> All packets can be offloaded — MWF not needed.");
            return false;
        }
        System.out.println("  >> Bottleneck detected — running MWF.");
        return true;
    }

    // =========================================================================
    //  RELAY NODE SUMMARY PRINTER
    //
    //  After augmentation, prints which BSN nodes acted purely as relays
    //  (carried flow but are neither a DG source nor a storage sink on that
    //  path). Shows packets forwarded and energy consumed per relay node.
    //  Both DG nodes and storage nodes can act as relays for other flows.
    // =========================================================================

    private void printRelayInfo(Map<Integer, Integer> relayCount,
                                Set<Integer>          dgSet,
                                Set<Integer>          storageSet) {
        System.out.println("\n-- Relay Node Activity --");
        boolean anyRelay = false;
        for (Map.Entry<Integer, Integer> e : relayCount.entrySet()) {
            int node = e.getKey();
            int pkts = e.getValue();
            String role = dgSet.contains(node)      ? " [DG-relay]"
                        : storageSet.contains(node) ? " [Storage-relay]"
                        : " [Relay]";
            System.out.printf("  Node %d%s: forwarded %d packet(s), energy used: %d/%d%n",
                              node, role, pkts, pkts, nodeEnergy[node]);
            anyRelay = true;
        }
        if (!anyRelay) System.out.println("  No relay nodes used.");
    }


    private int cfnToBSNNode(int cfnNode) {
        if (cfnNode <= 0 || cfnNode == superSink()) return -1;
        return (cfnNode - 1) / 2;
    }

    private List<Integer> extractBSNPathFromParent(int[] parent) {
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
            int bsn = cfnToBSNNode(node);
            if (bsn == -1) continue;
            if (bsnPath.isEmpty() || bsnPath.get(bsnPath.size() - 1) != bsn)
                bsnPath.add(bsn);
        }
        return bsnPath;
    }

    private String relayRoleTag(int node, Set<Integer> dgSet, Set<Integer> storageSet) {
        if (dgSet.contains(node)) return "DG-relay";
        if (storageSet.contains(node)) return "Storage-relay";
        return "Relay";
    }

    private void printRelayTrace(String algoTag,
                                 int dg,
                                 int sinkNode,
                                 int delta,
                                 int vi,
                                 int szI,
                                 List<Integer> bsnPath,
                                 Set<Integer> dgSet,
                                 Set<Integer> storageSet) {
        StringBuilder sb = new StringBuilder();
        sb.append("  [TRACE ").append(algoTag).append("] DG ").append(dg)
          .append(" -> Storage ").append(sinkNode)
          .append(" | Δ=").append(delta)
          .append(" | v=").append(vi)
          .append(" | sz=").append(szI)
          .append(" | path: ");

        for (int i = 0; i < bsnPath.size(); i++) {
            int node = bsnPath.get(i);
            if (i > 0) sb.append(" -> ");
            sb.append(node);
            if (i > 0 && i < bsnPath.size() - 1) {
                sb.append("[").append(relayRoleTag(node, dgSet, storageSet)).append("]");
            }
        }
        System.out.println(sb);

        if (bsnPath.size() <= 2) {
            System.out.println("    relays: none");
            return;
        }

        System.out.println("    relays:");
        for (int i = 1; i < bsnPath.size() - 1; i++) {
            int relay = bsnPath.get(i);
            System.out.printf("      node %d [%s] forwarded %d packet(s), energy impact %d/%d%n",
                              relay, relayRoleTag(relay, dgSet, storageSet),
                              delta, delta, nodeEnergy[relay]);
        }
    }

    // =========================================================================
    // ████████████████████████████████████████████████████████████████████████
    //
    //   ALGORITHMS — GOA, DENSITY GOA, APPROX GOA
    //
    //   All three algorithms share the same BFN graph and FAP-based flow
    //   augmentation loop. The only difference between them is the order
    //   in which DG sources are processed. Each algorithm:
    //     1. Runs a pre-flight feasibility check (needsMWF)
    //     2. Builds the BFN (buildCFN)
    //     3. Sorts DG nodes by its chosen key
    //     4. Iterates: find FAP → compute bottleneck Δ → augment flow
    //     5. Prints relay node activity and launches graph windows
    //
    // ████████████████████████████████████████████████████████████████████████
    // =========================================================================

    // ─────────────────────────────────────────────────────────────────────────
    // ALGORITHM 1 — GOA (Greedy Optimal Algorithm), Size-Aware Extension
    //
    //  WHAT IT DOES:
    //    Pushes the highest-priority packets first. Sorts DGs by vᵢ and
    //    greedily routes as many packets as possible before moving on.
    //
    //  WHEN IT WINS:
    //    When high-priority packets are also small — they don't waste storage.
    //
    //  WHEN IT LOSES:
    //    When a large high-priority packet fills storage that could have held
    //    many small packets with greater combined priority.
    //    Counterexample: DG0 v=10 sz=3 d=2 vs DG1 v=7 sz=1 d=6, cap=6
    //                    GOA=20, Optimal=42
    //
    //  STEPS:
    //    1. Sort DGs by priority vᵢ descending
    //    2. For each DG (highest priority first):
    //       a. Find shortest FAP through this DG in the residual graph
    //       b. Compute Δ = min(path residual, remaining packets, floor(mⱼ/szᵢ))
    //       c. Augment flow by Δ along the path
    //       d. Update: dᵢ -= Δ, mⱼ -= Δ×szᵢ
    //       e. Repeat until no FAPs exist through this DG
    //    3. Move to next DG; stop when no FAPs remain anywhere
    // ─────────────────────────────────────────────────────────────────────────

    public double goa(List<Integer> dgNodes,
                      List<Integer> storageNodes,
                      int[]         storageCapacity) {

        if (!needsMWF(dgNodes, storageNodes, storageCapacity)) return 0.0;

        buildCFN(dgNodes, storageNodes, storageCapacity);
        int S = 0, T = superSink();

        int[] remaining = new int[dgNodes.size()];
        for (int i = 0; i < dgNodes.size(); i++)
            remaining[i] = packetsPerNode[dgNodes.get(i)];

        int[] sinkCap = new int[cfnNodes];
        for (int j = 0; j < storageNodes.size(); j++)
            sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

        // step 1: sort by priority vᵢ descending
        Integer[] order = new Integer[dgNodes.size()];
        for (int i = 0; i < order.length; i++) order[i] = i;
        Arrays.sort(order, (a, b) ->
                packetPriority[dgNodes.get(b)] - packetPriority[dgNodes.get(a)]);

        double      totalWeight  = 0.0;
        List<int[]> goaFlowEdges = new ArrayList<>();
        Map<Integer, Integer> relayCount = new LinkedHashMap<>();
        int         maxIter      = cfnNodes * cfnNodes;

        Set<Integer> dgSet      = new HashSet<>(dgNodes);
        Set<Integer> storageSet = new HashSet<>(storageNodes);

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
                int delta = Math.min(pathBottleneck,
                            Math.min(remaining[idx], sinkCap[sinkOut] / szI));
                if (delta <= 0) break;

                // collect relay nodes along this path
                List<Integer> pathBsnNodes = extractBSNPathFromParent(parent);
                int sinkNode = pathBsnNodes.isEmpty() ? ((sinkOut - 1) / 2)
                                                      : pathBsnNodes.get(pathBsnNodes.size() - 1);
                for (int k = 1; k < pathBsnNodes.size() - 1; k++) {
                    int relay = pathBsnNodes.get(k);
                    relayCount.merge(relay, delta, Integer::sum);
                }
                printRelayTrace("GOA", dg, sinkNode, delta, vi, szI,
                                pathBsnNodes, dgSet, storageSet);

                // step 2c: augment flow along path
                cur = T; steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    flow[prev][cur] += delta;
                    flow[cur][prev] -= delta;

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
                remaining[idx]   -= delta;
                sinkCap[sinkOut] -= delta * szI;
                totalWeight      += (double) delta * vi;

                System.out.printf(
                    "  Pushed %d packet(s) from DG %d (v=%d, sz=%d) -> sink %d%n",
                    delta, dg, vi, szI, sinkOut);
            }
            // step 2e: move to next DG when no more FAPs exist through this one
        }

        System.out.printf("%nTotal Preserved Priority (GOA): %.1f%n", totalWeight);
        printRelayInfo(relayCount, dgSet, storageSet);
        launchGraph(dgNodes, storageNodes, goaFlowEdges, storageCapacity,
                    "GOA - Sort by Priority (v)");
        launchBFN(dgNodes, storageNodes, storageCapacity, goaFlowEdges,
                  "GOA - Sort by Priority (v)");
        return totalWeight;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // ALGORITHM 2 — DENSITY GOA (Value Density Greedy), New Contribution
    //
    //  WHAT IT DOES:
    //    Pushes packets with the highest priority-per-storage-unit (ρ = v/sz)
    //    first. Treats storage as the binding constraint and maximizes how
    //    much priority fits per unit consumed — like fractional knapsack.
    //
    //  WHEN IT WINS:
    //    When high-priority packets are large and low-priority packets are
    //    small — sorting by raw priority wastes storage on big packets.
    //
    //  WHEN IT LOSES:
    //    When leftover storage fragments can't fit remaining packets, creating
    //    bin-packing difficulty that density sorting can't resolve.
    //    Counterexample: DG0 v=9 sz=3 vs DG1 v=8 sz=2, cap=5
    //                    Density=16, Optimal=17
    //
    //  STEPS:
    //    1. Compute density ρᵢ = vᵢ / szᵢ for each DG
    //    2. Sort DGs by ρᵢ descending (highest density first)
    //    3. For each DG (highest density first):
    //       a. Find shortest FAP through this DG in the residual graph
    //       b. Compute Δ = min(path residual, remaining packets, floor(mⱼ/szᵢ))
    //       c. Augment flow by Δ along the path
    //       d. Update: dᵢ -= Δ, mⱼ -= Δ×szᵢ
    //       e. Repeat until no FAPs exist through this DG
    //    4. Move to next DG; stop when no FAPs remain anywhere
    // ─────────────────────────────────────────────────────────────────────────

    public double goaDensity(List<Integer> dgNodes,
                             List<Integer> storageNodes,
                             int[]         storageCapacity) {

        if (!needsMWF(dgNodes, storageNodes, storageCapacity)) return 0.0;

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
        Map<Integer, Integer> relayCount = new LinkedHashMap<>();
        int         maxIter      = cfnNodes * cfnNodes;

        Set<Integer> dgSet      = new HashSet<>(dgNodes);
        Set<Integer> storageSet = new HashSet<>(storageNodes);

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
                int delta = Math.min(pathBottleneck,
                            Math.min(remaining[idx], sinkCap[sinkOut] / szI));
                if (delta <= 0) break;

                // collect relay nodes along this path
                List<Integer> pathBsnNodes = extractBSNPathFromParent(parent);
                int sinkNode = pathBsnNodes.isEmpty() ? ((sinkOut - 1) / 2)
                                                      : pathBsnNodes.get(pathBsnNodes.size() - 1);
                for (int k = 1; k < pathBsnNodes.size() - 1; k++) {
                    int relay = pathBsnNodes.get(k);
                    relayCount.merge(relay, delta, Integer::sum);
                }
                printRelayTrace("DENS", dg, sinkNode, delta, vi, szI,
                                pathBsnNodes, dgSet, storageSet);

                // step 3c: augment flow along path
                cur = T; steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    flow[prev][cur] += delta;
                    flow[cur][prev] -= delta;

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
                remaining[idx]   -= delta;
                sinkCap[sinkOut] -= delta * szI;
                totalWeight      += (double) delta * vi;

                System.out.printf(
                    "  Pushed %d packet(s) from DG %d (v=%d, sz=%d, rho=%.2f) -> sink %d%n",
                    delta, dg, vi, szI, (double)vi/szI, sinkOut);
            }
            // step 3e: move to next DG when no more FAPs exist through this one
        }

        System.out.printf("%nTotal Preserved Priority (Density GOA): %.1f%n", totalWeight);
        printRelayInfo(relayCount, dgSet, storageSet);
        launchGraph(dgNodes, storageNodes, goaFlowEdges, storageCapacity,
                    "Density GOA - Sort by Density (v/sz)");
        launchBFN(dgNodes, storageNodes, storageCapacity, goaFlowEdges,
                  "Density GOA - Sort by Density (v/sz)");
        return totalWeight;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // ALGORITHM 3 — APPROX GOA (2-Approximation), Adapted from Alg. 5
    //
    //  WHAT IT DOES:
    //    Runs both GOA (sort by vᵢ) and Density GOA (sort by vᵢ/szᵢ) as
    //    sub-routines and returns whichever result is higher.
    //
    //  GUARANTEE:
    //    Always produces at least half the theoretical optimal — proven by
    //    adapting Theorem 5 from Rivera & Tang (2024) to the size-aware case.
    //
    //  WHY IT WORKS:
    //    One of the two orderings must capture at least half the optimal
    //    priority. Taking the max of both ensures the guarantee holds even
    //    when neither sub-routine alone is optimal.
    //
    //  STEPS:
    //    1. Run sub-routine A: greedy by vᵢ (same as GOA) → result VfA
    //    2. Run sub-routine B: greedy by vᵢ/szᵢ (same as Density GOA) → VfB
    //    3. Return max(VfA, VfB)
    //
    //  APPROXIMATION PROOF SKETCH:
    //    Let Sw = sources that send in sub-routine B before first failure sⱼ
    //    V'f = Σ(sᵢ∈Sw)(vᵢ×dᵢ)  and  vⱼ×dⱼ <= Vf  (A sends highest v first)
    //    Vopt <= V'f + vⱼ×dⱼ <= V'f + Vf
    //    Since Vf = max(Vf, V'f):  Vf >= Vopt / 2
    // ─────────────────────────────────────────────────────────────────────────

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

    // =========================================================================
    // ████████████████████████████████████████████████████████████████████████
    //   END OF ALGORITHMS
    // ████████████████████████████████████████████████████████████████████████
    // =========================================================================

    // -------------------------------------------------------------------------
    //  Shared greedy core used by both Approx GOA sub-routines.
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
    //  SILENT RUN (no output, no graphs — used for scaling trials)
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
                totalWeight      += (double) delta * vi;
            }
        }
        return totalWeight;
    }

    // =========================================================================
    //  SILENT RUN WITH FLOW EDGE COLLECTION (used by visual run)
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
        Map<Integer, Integer> energyMap     = new LinkedHashMap<>();

        for (int i = 0; i < n; i++) energyMap.put(i + 1, nodeEnergy[i]);

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
        panel.setNodeEnergy(energyMap);
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

        boolean[] visVisited = new boolean[visNodes];
        visNet.components = new ArrayList<>();
        for (int i = 0; i < visNodes; i++)
            if (!visVisited[i])
                visNet.components.add(buildBFS(
                    visNet.adjM != null ? visNet.adjM.getAdjM()
                                       : visNet.adjList, i, visVisited));

        // randomized DG/storage ratio instead of fixed 50/50
        List<Integer> visDG  = new ArrayList<>();
        List<Integer> visST  = new ArrayList<>();
        List<Integer> visAll = new ArrayList<>();
        for (int i = 0; i < visNodes; i++) visAll.add(i);
        Collections.shuffle(visAll, rand);
        visDG.add(visAll.get(0));
        visST.add(visAll.get(1));
        double visDgRatio = 0.2 + rand.nextDouble() * 0.5;
        for (int i = 2; i < visNodes; i++) {
            if (rand.nextDouble() < visDgRatio) visDG.add(visAll.get(i));
            else                               visST.add(visAll.get(i));
        }

        int[] visCap = new int[visST.size()];
        for (int j = 0; j < visCap.length; j++)
            visCap[j] = (minCap == maxCap) ? minCap
                      : rand.nextInt(maxCap - minCap + 1) + minCap;

        System.out.printf("  Visual run DG ratio: %.0f%% (%d DGs, %d storage)%n",
                          visDgRatio * 100, visDG.size(), visST.size());
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
                visST.get(j), visCap[j], visNet.nodeEnergy[visST.get(j)]);

        // use goa() and goaDensity() directly so feasibility check,
        // relay tracking, and console output all appear in the visual run
        System.out.println("\n-- Running GOA (visual) --");
        double visGOA     = visNet.goa(visDG, visST, visCap);
        System.out.println("\n-- Running Density GOA (visual) --");
        double visDensity = visNet.goaDensity(visDG, visST, visCap);
        System.out.printf("%n  GOA total priority:         %.1f%n", visGOA);
        System.out.printf("  Density GOA total priority: %.1f%n", visDensity);

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

                // per-trial TR jitter (75%–125% of base TR)
                double trJitter = 0.75 + rand.nextDouble() * 0.5;
                int trialTR = (int)(TR * trJitter);
                net.createE(trialTR);

                net.randomNodeEnergies(minE, maxE);
                net.randomDataPackets(minPkt, maxPkt);
                net.randomPacketSizes(minSz, maxSz);
                net.randomPacketPriorities(minPri, maxPri);

                boolean[] visited = new boolean[numNodes];
                net.components = new ArrayList<>();
                for (int i = 0; i < numNodes; i++)
                    if (!visited[i])
                        net.components.add(buildBFS(
                            net.adjM != null ? net.adjM.getAdjM()
                                             : net.adjList, i, visited));

                // randomized DG/storage ratio per trial
                List<Integer> dgNodes      = new ArrayList<>();
                List<Integer> storageNodes = new ArrayList<>();
                List<Integer> allNodes     = new ArrayList<>();
                for (int i = 0; i < numNodes; i++) allNodes.add(i);
                Collections.shuffle(allNodes, rand);
                dgNodes.add(allNodes.get(0));
                storageNodes.add(allNodes.get(1));
                double dgRatio = 0.2 + rand.nextDouble() * 0.5;
                for (int i = 2; i < numNodes; i++) {
                    if (rand.nextDouble() < dgRatio) dgNodes.add(allNodes.get(i));
                    else                             storageNodes.add(allNodes.get(i));
                }

                int[] storageCap = new int[storageNodes.size()];
                for (int j = 0; j < storageCap.length; j++)
                    storageCap[j] = (minCap == maxCap) ? minCap
                        : rand.nextInt(maxCap - minCap + 1) + minCap;

                double goaResult     = net.runSilent(dgNodes, storageNodes,
                                                     storageCap, false);
                double densityResult = net.runSilent(dgNodes, storageNodes,
                                                     storageCap, true);
                double approxResult  = Math.max(goaResult, densityResult);

                System.out.printf(
                    "  Trial %2d [TR=%d, DGs=%d, STs=%d]: GOA=%.1f  Density=%.1f  Approx=%.1f%n",
                    t, trialTR, dgNodes.size(), storageNodes.size(),
                    goaResult, densityResult, approxResult);

                sumGOA     += goaResult;
                sumDensity += densityResult;
                sumApprox  += approxResult;

                if (densityResult > goaResult)      densityBeatGOA++;
                else if (goaResult > densityResult) goaBeatDensity++;
                else                                tied++;
            }

            System.out.printf("%n-- Averages over %d trials --%n", trials);
            System.out.printf("  GOA avg:              %.2f%n", sumGOA     / trials);
            System.out.printf("  Density GOA avg:      %.2f%n", sumDensity / trials);
            System.out.printf("  Approx GOA avg:       %.2f%n", sumApprox  / trials);
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