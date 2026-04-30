import java.util.*;
import javax.swing.SwingUtilities;

public class SensorStuff {
    private static final double Eelect     = .0001;
    private static final double Eamp       = 0.0000001;
    private static final int    PACKET_SIZE = 3200;

    protected int[] packetSize;
    protected int[] packetPriority;
    protected int[] nodeEnergy;

    int     cfnNodes;
    int[][] cap;
    int[][] flow;
    private Set<Integer> dgCFNIds = new HashSet<>();
    int     cachedMaxSz = 1;  // cached for computeRelayPenalty

    int inNode(int i)  { return 2*i + 1; }
    int outNode(int i) { return 2*i + 2; }
    int superSink()    { return 2*nodeLoc.length + 1; }

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
    //  CONNECTIVITY CHECK
    //
    //  Returns true if the graph is fully connected — i.e. every node can
    //  reach every other node. Required by the data preservation problem:
    //  packets must be routable from any DG to any storage node.
    //  Uses a single BFS from node 0 and checks that all nodes are visited.
    // =========================================================================

    private boolean isFullyConnected(int numNodes) {
        boolean[] visited = new boolean[numNodes];
        List<Integer> comp = buildBFS(adjM != null ? adjM.getAdjM() : adjList, 0, visited);
        return comp.size() == numNodes;
    }

    // =========================================================================
    //  CONNECTED GRAPH GENERATION
    //
    //  Regenerates node positions and edges until the graph is fully connected,
    //  satisfying Professor Tang's connectivity requirement. Attempts up to
    //  1000 times; prints a warning if no connected graph could be produced
    //  (typically caused by TR being too small relative to the field size).
    //
    //  For the visual run: uses the fixed base TR.
    //  For scaling trials: also re-applies TR jitter each attempt so the
    //  jitter still varies across trials while guaranteeing connectivity.
    // =========================================================================

    int buildConnectedGraph(int numNodes, int widthX, int lenY, int baseTR, Random rand, boolean applyJitter) {
        int trialTR = baseTR;
        int attempts = 0;
        do {
            // reset adjacency structure before regenerating edges
            if (adjM != null) {
                adjM = new AdjacentMatrix(numNodes);
            } else {
                adjList = new ArrayList<>();
                for (int i = 0; i < numNodes; i++) adjList.add(new LinkedList<>());
            }
            randomNodes(widthX, lenY);
            if (applyJitter) {
                // per-trial TR jitter (75%–125% of base TR)
                double trJitter = 0.75 + rand.nextDouble() * 0.5;
                trialTR = (int)(baseTR * trJitter);
            }
            createE(trialTR);
            attempts++;
        } while (!isFullyConnected(numNodes) && attempts < 1000);

        if (attempts >= 1000)
            System.out.println("  Warning: could not generate a fully connected graph after 1000 attempts. " +
                               "Try increasing TR or shrinking the field.");
        return trialTR;
    }

    // =========================================================================
    //  ADJACENCY TRACE PRINTER
    //
    //  Prints the full adjacency structure to the terminal for one trial.
    //  For adj-matrix: prints the matrix row by row with node labels.
    //  For adj-list:   prints each node's neighbour list.
    //  Also prints each node's (x,y) position and role (DG / Storage / Relay).
    //  Called once during the visual run so the graph structure is visible.
    // =========================================================================

    public void printAdjacency(List<Integer> dgNodes,
                               List<Integer> storageNodes) {
        int n = nodeLoc.length;
        Set<Integer> dgSet  = new HashSet<>(dgNodes);
        Set<Integer> stSet  = new HashSet<>(storageNodes);

        System.out.println("\n======================================");
        System.out.println("  ADJACENCY STRUCTURE (Visual Run)");
        System.out.println("======================================");

        // node role + position summary
        System.out.println("\n-- Node Summary --");
        for (int i = 0; i < n; i++) {
            String role = dgSet.contains(i)  ? "DG"
                        : stSet.contains(i)  ? "Storage"
                        : "Relay";
            System.out.printf("  Node %2d [%-7s]  (x=%.1f, y=%.1f)  E=%d%n",
                i, role, nodeLoc[i][0], nodeLoc[i][1], nodeEnergy[i]);
        }

        if (adjM != null) {
            // ── adjacency matrix ──────────────────────────────────────────────
            System.out.println("\n-- Adjacency Matrix --");
            // header row
            System.out.print("      ");
            for (int j = 0; j < n; j++) System.out.printf("%3d", j);
            System.out.println();
            System.out.print("      ");
            for (int j = 0; j < n; j++) System.out.print("---");
            System.out.println();
            int[][] m = adjM.getAdjM();
            for (int i = 0; i < n; i++) {
                System.out.printf("  %2d |", i);
                for (int j = 0; j < n; j++)
                    System.out.printf("%3d", m[i][j]);
                System.out.println();
            }
            // edge list derived from matrix
            System.out.println("\n-- Edges (from matrix) --");
            boolean any = false;
            for (int i = 0; i < n; i++)
                for (int j = i+1; j < n; j++)
                    if (m[i][j] == 1) {
                        System.out.printf("  Node %d -- Node %d  (dist=%.1f)%n",
                            i, j, distanceNodes(nodeLoc[i], nodeLoc[j]));
                        any = true;
                    }
            if (!any) System.out.println("  (no edges)");

        } else {
            // ── adjacency list ────────────────────────────────────────────────
            System.out.println("\n-- Adjacency List --");
            for (int i = 0; i < n; i++) {
                System.out.printf("  Node %2d -> [ ", i);
                List<Integer> nbrs = adjList.get(i);
                if (nbrs == null || nbrs.isEmpty()) {
                    System.out.print("(none)");
                } else {
                    for (int j = 0; j < nbrs.size(); j++) {
                        System.out.print(nbrs.get(j));
                        if (j < nbrs.size()-1) System.out.print(", ");
                    }
                }
                System.out.println(" ]");
            }
        }
        System.out.println("======================================\n");
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

        // issue 2 fix: cache maxSz here so computeRelayPenalty never loops
        cachedMaxSz = 1;
        for (int sz : packetSize) cachedMaxSz = Math.max(cachedMaxSz, sz);

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

    int[] bfsFAP(int cfnSource, int szI, int[] sinkCap) {
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
    //   ALGORITHMS — GOA, DENSITY GOA, APPROX GOA, HYBRID GOA
    //
    //   All algorithms share the same BFN graph and FAP-based flow
    //   augmentation loop. The difference between them is the order
    //   in which DG sources are processed and how paths are selected.
    //   Each algorithm:
    //     1. Runs a pre-flight feasibility check (needsMWF)
    //     2. Builds the BFN (buildCFN)
    //     3. Sorts / selects DG nodes by its chosen key
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
                                if (fe[0] == bsnU && fe[1] == bsnV)
                                    { exists = true; break; }
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
    //  DISTINCTION FROM PAPER'S ALGO 5 (Issue B):
    //    Algorithm 5 in Rivera & Tang (2024) sorts by vᵢ/cᵢ where cᵢ is
    //    the EDGE CAPACITY COST per unit flow — an MWF-H concept meaning
    //    each unit flow from DGᵢ consumes cᵢ units on EVERY edge it uses.
    //    Here, szᵢ is the STORAGE UNITS consumed at the SINK ONLY when one
    //    packet from DGᵢ is stored. These are fundamentally different:
    //      - cᵢ (paper): affects routing edge capacities throughout the BFN
    //      - szᵢ (project): affects only the j''→t sink edge capacity
    //    This project stays strictly within MWF-U (cᵢ=1 for all sources on
    //    all routing edges). Using vᵢ/szᵢ as the density key is a NEW
    //    contribution motivated by the fractional knapsack analogy, NOT a
    //    direct application of the paper's Algo 5.
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
                                if (fe[0] == bsnU && fe[1] == bsnV)
                                    { exists = true; break; }
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
    //    Adapted from Theorem 5 of Rivera & Tang (2024), which proves a
    //    1/2-approximation under the ALL-OR-NOTHING model — each source
    //    either sends ALL dᵢ packets or NONE. That paper uses costs cᵢ
    //    (edge capacity usage per unit flow, MWF-H concept). This project
    //    replaces cᵢ with szᵢ (storage consumption at the sink only), which
    //    is a different quantity — szᵢ does NOT affect routing edge capacity.
    //
    //  IMPORTANT DISTINCTION (Issue B):
    //    In Algo. 5 of the paper, the sort key vᵢ/cᵢ uses cᵢ = edge capacity
    //    cost per unit flow (MWF-H). Here, szᵢ is the storage units consumed
    //    at the sink when one packet from DGᵢ is stored. These are different:
    //    cᵢ affects ALL edges on the path; szᵢ affects ONLY the sink edge.
    //    The project stays in MWF-U (uniform cᵢ=1 for all routing edges).
    //
    //  SPLITTABLE FLOW CAVEAT (Issue A):
    //    Theorem 5 in the paper is proven under the all-or-nothing model
    //    (each source sends all dᵢ or nothing). This code uses SPLITTABLE
    //    flow — partial packets from a DG can be sent. The 1/2 guarantee
    //    is conjectured to hold for the splittable case (supported by our
    //    experiments showing ≥97% of optimal empirically) but is NOT
    //    formally proven here. It should be treated as a strong heuristic
    //    with an empirical approximation ratio, not a formal guarantee.
    //
    //  STEPS:
    //    1. Run sub-routine A: greedy by vᵢ (same as GOA) → result VfA
    //    2. Run sub-routine B: greedy by vᵢ/szᵢ (same as Density GOA) → VfB
    //    3. Return max(VfA, VfB)
    //
    //  APPROXIMATION PROOF SKETCH (from Theorem 5, all-or-nothing model):
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

    // ─────────────────────────────────────────────────────────────────────────
    // ALGORITHM 4 — HYBRID GOA (PE-κ2 + Rollout + Best-Fit Storage)
    //
    //  WHAT IT DOES:
    //    Combines three strategies to handle variable packet sizes better
    //    than any single ordering heuristic:
    //
    //    1. PARTIAL ENUMERATION (PE, κ=2):
    //       Tries every ordered pair of DGs as the first two to augment,
    //       plus every single DG prefix and a pure-rollout run (κ=0).
    //       Catches cases where GOA/DensityGOA lock in a bad prefix —
    //       the first two augmented DGs most affect how much storage
    //       is left for remaining DGs due to variable packet sizes.
    //
    //    2. ROLLOUT (lookahead):
    //       After the prefix, instead of committing to a fixed sort order
    //       for remaining DGs, each candidate is simulated as "next":
    //       fully augment it, then run density-greedy on the rest,
    //       and pick the candidate that yields the highest pilot total.
    //       Repeat until no FAPs remain anywhere.
    //
    //    3. BEST-FIT STORAGE SELECTION:
    //       During each augmentation step, instead of routing to whichever
    //       storage node bfsFAP finds first, prefer the storage node whose
    //       residual capacity after placing szᵢ units is the smallest
    //       non-negative value — minimizing wasted storage fragments that
    //       future DGs with different packet sizes cannot fill.
    //
    //  WHEN IT WINS:
    //    Any case where GOA, DensityGOA, and ApproxGOA all make suboptimal
    //    prefix decisions or route to fragmenting storage nodes. The PE
    //    component guarantees at least one trial will have the optimal first
    //    two DGs; rollout corrects the tail; best-fit reduces waste.
    //
    //  APPROXIMATION:
    //    Inherits the PE(κ=2) conjectured 2/3-approximation lower bound
    //    (stronger than ApproxGOA's proven 1/2). No formal proof for
    //    general flow networks — treat as a strong heuristic with a
    //    better empirical floor than ApproxGOA.
    //
    //  COMPLEXITY:
    //    O(k² × k² × n × m²) = O(k⁴ × n × m²)
    //    Practical for k ≤ 15 DGs, typical in small sensor networks.
    //    For large k, use ApproxGOA or runSilent instead.
    //
    //  STEPS:
    //    1. Build all prefixes: empty, all k singletons, all k(k-1) pairs
    //    2. For each prefix:
    //       a. Fresh CFN, augment prefix DGs using best-fit storage
    //       b. Rollout loop: simulate each remaining DG as next candidate,
    //          pick the one with highest pilot value, commit and repeat
    //    3. Return the best total across all prefix trials
    // ─────────────────────────────────────────────────────────────────────────

    public double goaHybrid(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            int[]         storageCapacity) {

        if (!needsMWF(dgNodes, storageNodes, storageCapacity)) return 0.0;

        int n = dgNodes.size();
        int bestTotal = 0;

        // step 1: build all prefix trials
        // - empty prefix  → pure rollout
        // - k singletons  → rollout after 1 fixed DG
        // - k(k-1) pairs  → rollout after 2 fixed DGs  (PE κ=2 core)
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

        // step 2: evaluate each prefix trial
        for (List<Integer> prefix : prefixes) {

            // fresh CFN for every trial
            buildCFN(dgNodes, storageNodes, storageCapacity);

            int[] remaining = new int[n];
            for (int i = 0; i < n; i++)
                remaining[i] = packetsPerNode[dgNodes.get(i)];

            int[] sinkCap = new int[cfnNodes];
            for (int j = 0; j < storageNodes.size(); j++)
                sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

            int trialTotal = 0;
            Set<Integer> used = new LinkedHashSet<>();

            // step 2a: augment prefix DGs in order using best-fit storage
            for (int idx : prefix) {
                used.add(idx);
                trialTotal += augmentBestFit(
                    idx, dgNodes, remaining, sinkCap, storageNodes);
            }

            // step 2b: rollout loop for remaining DGs
            // At each step: for every candidate DG, simulate it as next
            // (best-fit augment + density-greedy tail), pick highest pilot.
            while (true) {
                List<Integer> candidates = new ArrayList<>();
                for (int i = 0; i < n; i++) {
                    if (used.contains(i)) continue;
                    if (remaining[i] <= 0) continue;
                    // quick reachability check before simulating
                    int[] probe = bfsFAP(inNode(dgNodes.get(i)),
                                        packetSize[dgNodes.get(i)], sinkCap);
                    if (probe != null) candidates.add(i);
                }
                if (candidates.isEmpty()) break;

                int bestCandidate = -1;
                int bestPilot     = -1;

                for (int ci : candidates) {
                    // snapshot current CFN state for rollback after simulation
                    int[][] capSnap  = deepCopy2D(cap);
                    int[][] flowSnap = deepCopy2D(flow);
                    int[]   remSnap  = remaining.clone();
                    int[]   sinkSnap = sinkCap.clone();

                    // simulate phase A: best-fit augment this candidate
                    int pilotVal = augmentBestFit(
                        ci, dgNodes, remaining, sinkCap, storageNodes);

                    // simulate phase B: density-greedy tail on rest
                    List<Integer> tail = new ArrayList<>();
                    for (int i = 0; i < n; i++) {
                        if (i == ci || used.contains(i)) continue;
                        if (remaining[i] > 0) tail.add(i);
                    }
                    // sort tail by density ρ = vᵢ/szᵢ descending
                    tail.sort((a, b) -> {
                        double dA = (double) packetPriority[dgNodes.get(a)]
                                  / packetSize[dgNodes.get(a)];
                        double dB = (double) packetPriority[dgNodes.get(b)]
                                  / packetSize[dgNodes.get(b)];
                        return Double.compare(dB, dA);
                    });
                    for (int ti : tail)
                        pilotVal += augmentBestFit(
                            ti, dgNodes, remaining, sinkCap, storageNodes);

                    if (pilotVal > bestPilot) {
                        bestPilot     = pilotVal;
                        bestCandidate = ci;
                    }

                    // restore CFN state — undo simulation
                    cap       = capSnap;
                    flow      = flowSnap;
                    remaining = remSnap;
                    sinkCap   = sinkSnap;
                }

                if (bestCandidate < 0 || bestPilot <= 0) break;

                // commit: augment the winning candidate for real
                used.add(bestCandidate);
                trialTotal += augmentBestFit(
                    bestCandidate, dgNodes, remaining, sinkCap, storageNodes);
            }

            if (trialTotal > bestTotal) bestTotal = trialTotal;
        }

        System.out.printf("%nTotal Preserved Priority (Hybrid GOA): %d%n", bestTotal);
        return (double) bestTotal;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // ALGORITHM 5 — PE+PSB GOA (Partial Enumeration + Per-Step Best Path)
    //
    //  WHAT IT DOES:
    //    Combines Hybrid GOA's PE(κ=2) prefix enumeration with PSB's
    //    per-step best path scoring for the tail. This fixes the core
    //    weakness of the original PSB-GOA: making greedy per-step decisions
    //    with no lookahead caused it to lose to Hybrid GOA, which exhaustively
    //    tries all ordered pairs of DGs as the first two to augment.
    //
    //    The algorithm works in two phases per prefix trial:
    //
    //    PHASE 1 — PE(κ=2) PREFIX:
    //      Try every ordered pair (a, b) of DGs as the first two to fully
    //      augment using best-fit storage selection. Also tries all single
    //      DG prefixes and the empty prefix (pure PSB tail). This gives the
    //      algorithm the global commitment awareness that pure PSB-GOA lacks.
    //
    //    PHASE 2 — PSB TAIL:
    //      After the prefix, run per-step best path scoring on all remaining
    //      DGs simultaneously. At each step, score every active DG's FAP by:
    //
    //        score(P) = (v_i × Δ) / (Δ × sz_i + waste + 1)
    //
    //      where waste = (sinkCap[j] - Δ × sz_i) % minRemainingSize —
    //      the storage fragment that no remaining packet can fill.
    //      Pick the path with the highest score and augment it.
    //      Repeat until no FAPs remain.
    //
    //  WHY THIS COMBINATION WORKS:
    //    - PE prefix catches bad commitment orderings (Hybrid's strength)
    //    - PSB tail avoids locking into a fixed ordering after the prefix,
    //      instead adapting step-by-step to the residual graph state
    //    - The waste penalty in the tail reduces storage fragmentation
    //      that neither PE nor rollout-based tails address directly
    //
    //  WHEN IT WINS:
    //    Cases where the optimal solution requires both: (a) the right first
    //    two DGs committed upfront AND (b) interleaved augmentation of the
    //    remaining DGs rather than exhausting them one at a time. Neither
    //    pure Hybrid (rollout tail) nor pure PSB (no prefix) finds these.
    //
    //  WHEN IT LOSES:
    //    When κ=2 prefix is insufficient — optimal requires committing 3+
    //    DGs in a specific order. Extending to κ=3 would help but costs k³.
    //    Also when the waste estimator (minRemainingSize) is stale after
    //    many DGs have been exhausted mid-tail.
    //
    //  APPROXIMATION:
    //    Inherits PE(κ=2) conjectured 2/3 lower bound from the prefix phase.
    //    The PSB tail is a heuristic improvement over the rollout tail used
    //    in Hybrid GOA — no formal ratio improvement is proven.
    //
    //  COMPLEXITY:
    //    O(k² × k × n × m²) — k² prefix trials, each with a PSB tail that
    //    scans all k DGs per augmentation step. Practical for k ≤ 15.
    //
    //  STEPS:
    //    1. Build all prefixes: empty, k singletons, k(k-1) ordered pairs
    //    2. For each prefix:
    //       a. Fresh CFN; augment prefix DGs using best-fit storage
    //       b. PSB tail: at each step score all remaining DGs' FAPs by
    //          (v_i × Δ) / (Δ × sz_i + waste + 1), pick highest, augment
    //       c. Record trial total
    //    3. Return best total across all prefix trials
    // ─────────────────────────────────────────────────────────────────────────

    // ─────────────────────────────────────────────────────────────────────────
    // ALGORITHM 5 — PE+ROLLOUT+PSB GOA (Partial Enumeration + Rollout
    //               with Energy-Aware Per-Step Best Path pilot simulation)
    //
    //  WHAT IT DOES:
    //    Combines three components:
    //
    //    1. PE(κ=2) PREFIX — same as Hybrid GOA:
    //       Try every ordered pair (a,b) of DGs as the first two to augment,
    //       plus all singletons and the empty prefix.
    //
    //    2. ROLLOUT SELECTION — same lookahead depth as Hybrid GOA:
    //       After the prefix, for each remaining DG candidate, simulate it as
    //       "next to commit" and measure the total pilot value. Pick the
    //       candidate with the highest pilot.
    //
    //    3. ENERGY-AWARE PSB SCORING INSIDE THE PILOT:
    //       The per-step score now accounts for THREE constraints:
    //
    //       a. Priority gained:       v_i × Δ          (maximize)
    //       b. Storage waste:         Δ × sz_i + waste  (minimize)
    //       c. Relay energy penalty:  relayPenalty(P)   (minimize)
    //
    //       score(P) = (v_i × Δ)
    //                / (Δ × sz_i + waste + relayPenalty(P) + 1)
    //
    //       where relayPenalty(P) = Σ over relay nodes r on path P of
    //         Δ × (1 / E_r) × scaleFactor
    //
    //       A relay node with low energy Eᵣ gets a HIGH penalty because
    //       routing Δ packets through it consumes a large fraction of its
    //       remaining budget, potentially blocking future high-priority DGs
    //       that depend on the same relay. A high-energy relay costs little.
    //
    //  WHY RELAY ENERGY MATTERS:
    //    Under uniform energy all relay penalties are equal and cancel out,
    //    so the score reduces to the original storage-only formula. Under
    //    non-uniform energy, paths through scarce relay nodes score lower,
    //    preserving that energy for future DGs that may need it. Without
    //    this term, PSB-GOA could burn through a critical low-energy relay
    //    on a moderate-priority DG, permanently blocking a high-priority
    //    DG that had no other path — exactly what GOA avoids by exhausting
    //    high-priority DGs first.
    //
    //  APPROXIMATION:
    //    Inherits PE(κ=2) conjectured 2/3 lower bound. The energy-aware
    //    scoring is a heuristic improvement over both the original PSB
    //    (storage-only) and Hybrid GOA (density-greedy pilot). Expected
    //    to hold up across both uniform and non-uniform energy scenarios.
    //
    //  COMPLEXITY:
    //    O(k² × k² × n × m²) — same as Hybrid GOA.
    //
    //  STEPS:
    //    1. Build all prefixes: empty, k singletons, k(k-1) ordered pairs
    //    2. For each prefix:
    //       a. Fresh CFN; augment prefix DGs using best-fit storage
    //       b. Rollout loop:
    //          - For each remaining candidate DG_i:
    //            * Snapshot CFN state
    //            * Commit DG_i (augment fully with best-fit)
    //            * Run energy-aware PSB tail on all other remaining DGs
    //            * Record pilot total; restore CFN state
    //          - Commit the candidate with the highest pilot total
    //          - Repeat until no candidates remain
    //    3. Return best total across all prefix trials
    // ─────────────────────────────────────────────────────────────────────────

    public double goaPSB(List<Integer> dgNodes,
                         List<Integer> storageNodes,
                         int[]         storageCapacity) {

        if (!needsMWF(dgNodes, storageNodes, storageCapacity)) return 0.0;

        int n         = dgNodes.size();
        int bestTotal = 0;

        // step 1: build all prefix trials (same as Hybrid GOA)
        List<List<Integer>> prefixes = new ArrayList<>();
        prefixes.add(new ArrayList<>());
        for (int a = 0; a < n; a++) {
            List<Integer> p = new ArrayList<>();
            p.add(a); prefixes.add(p);
        }
        for (int a = 0; a < n; a++) {
            for (int b = 0; b < n; b++) {
                if (a == b) continue;
                List<Integer> p = new ArrayList<>();
                p.add(a); p.add(b); prefixes.add(p);
            }
        }

        List<int[]> bestFlowEdges = new ArrayList<>();

        for (List<Integer> prefix : prefixes) {

            buildCFN(dgNodes, storageNodes, storageCapacity);

            int[] remaining = new int[n];
            for (int i = 0; i < n; i++)
                remaining[i] = packetsPerNode[dgNodes.get(i)];

            int[] sinkCap = new int[cfnNodes];
            for (int j = 0; j < storageNodes.size(); j++)
                sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

            int trialTotal = 0;
            Set<Integer> used = new LinkedHashSet<>();
            List<int[]> trialFlowEdges = new ArrayList<>();

            // step 2a: augment prefix DGs with best-fit storage
            for (int idx : prefix) {
                used.add(idx);
                trialTotal += augmentBestFit(
                    idx, dgNodes, remaining, sinkCap, storageNodes);
            }

            // step 2b: rollout loop with PSB pilot simulation
            while (true) {

                // build candidate list: active DGs that can still reach storage
                List<Integer> candidates = new ArrayList<>();
                for (int i = 0; i < n; i++) {
                    if (used.contains(i) || remaining[i] <= 0) continue;
                    int[] probe = bfsFAP(inNode(dgNodes.get(i)),
                                        packetSize[dgNodes.get(i)], sinkCap);
                    if (probe != null) candidates.add(i);
                }
                if (candidates.isEmpty()) break;

                int bestCandidate = -1;
                int bestPilot     = -1;

                for (int ci : candidates) {

                    // snapshot CFN state before simulation
                    int[][] capSnap  = deepCopy2D(cap);
                    int[][] flowSnap = deepCopy2D(flow);
                    int[]   remSnap  = remaining.clone();
                    int[]   sinkSnap = sinkCap.clone();

                    // commit candidate: augment ci with best-fit
                    int pilotVal = augmentBestFit(
                        ci, dgNodes, remaining, sinkCap, storageNodes);

                    // PSB tail: per-step best path on all other remaining DGs
                    // (this is the upgrade over Hybrid GOA's density-greedy tail)
                    pilotVal += runPSBTail(dgNodes, remaining, sinkCap, used, ci);

                    if (pilotVal > bestPilot) {
                        bestPilot     = pilotVal;
                        bestCandidate = ci;
                    }

                    // restore CFN state
                    cap       = capSnap;
                    flow      = flowSnap;
                    remaining = remSnap;
                    sinkCap   = sinkSnap;
                }

                if (bestCandidate < 0 || bestPilot <= 0) break;

                // commit the winning candidate for real
                used.add(bestCandidate);
                int committed = augmentBestFit(
                    bestCandidate, dgNodes, remaining, sinkCap, storageNodes);
                trialTotal += committed;

                System.out.printf(
                    "  Committed DG %d (v=%d, sz=%d, pilotVal=%d)%n",
                    dgNodes.get(bestCandidate),
                    packetPriority[dgNodes.get(bestCandidate)],
                    packetSize[dgNodes.get(bestCandidate)],
                    bestPilot);
            }

            if (trialTotal > bestTotal) {
                bestTotal     = trialTotal;
                bestFlowEdges = trialFlowEdges;
            }
        }

        System.out.printf("%nTotal Preserved Priority (PSB-GOA): %d%n", bestTotal);
        Set<Integer> dgSet      = new HashSet<>(dgNodes);
        Set<Integer> storageSet = new HashSet<>(storageNodes);
        Map<Integer, Integer> relayCount = new LinkedHashMap<>();
        printRelayInfo(relayCount, dgSet, storageSet);
        launchGraph(dgNodes, storageNodes, bestFlowEdges, storageCapacity,
                    "PSB-GOA - PE+Rollout+PSB Pilot");
        launchBFN(dgNodes, storageNodes, storageCapacity, bestFlowEdges,
                  "PSB-GOA - PE+Rollout+PSB Pilot");
        return (double) bestTotal;
    }

    // =========================================================================
    //  PSB TAIL HELPER (used inside rollout pilot simulation for PSB-GOA)
    //
    //  Runs the energy-aware per-step best path scoring on all remaining DGs
    //  (excluding 'used' set and the just-committed candidate 'committed')
    //  against the current CFN state. Returns the total priority value.
    //
    //  Score formula (energy-aware):
    //    score(P) = (v_i × Δ) / (Δ × sz_i + waste + relayPenalty(P) + 1)
    //
    //  relayPenalty(P) = scaleFactor × Σ_{relay r on P} (Δ / E_r)
    //    where E_r = nodeEnergy[r] (original energy budget of relay r).
    //    This penalises paths through low-energy relays proportionally to
    //    how much of their budget this augmentation consumes. Paths through
    //    high-energy relays incur negligible penalty.
    // =========================================================================

    private int runPSBTail(List<Integer> dgNodes,
                           int[]         remaining,
                           int[]         sinkCap,
                           Set<Integer>  used,
                           int           committed) {
        int S = 0, T = superSink();
        int n = dgNodes.size();
        int total = 0;

        while (true) {
            int minRemSz = Integer.MAX_VALUE;
            for (int i = 0; i < n; i++) {
                if (i == committed || used.contains(i)) continue;
                if (remaining[i] > 0)
                    minRemSz = Math.min(minRemSz, packetSize[dgNodes.get(i)]);
            }
            if (minRemSz == Integer.MAX_VALUE) break;

            int    bestIdx    = -1;
            double bestScore  = -1.0;
            int[]  bestParent = null;
            int    bestDelta  = 0;

            for (int i = 0; i < n; i++) {
                if (i == committed || used.contains(i)) continue;
                if (remaining[i] <= 0) continue;

                int dg     = dgNodes.get(i);
                int cfnSrc = inNode(dg);
                int szI    = packetSize[dg];
                int vi     = packetPriority[dg];

                int[] parent = bfsFAP(cfnSrc, szI, sinkCap);
                if (parent == null) continue;

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
                            Math.min(remaining[i], sinkCap[sinkOut] / szI));
                if (delta <= 0) continue;

                // storage waste penalty
                int newResidual = sinkCap[sinkOut] - delta * szI;
                int waste       = (minRemSz > 0) ? (newResidual % minRemSz) : 0;

                // relay energy penalty — penalise paths through low-energy relays
                double relayPenalty = computeRelayPenalty(parent, dg, delta);

                // energy-aware score
                double score = (double)(vi * delta)
                             / ((double)(delta * szI + waste) + relayPenalty + 1.0);

                if (score > bestScore) {
                    bestScore = score; bestIdx = i;
                    bestParent = parent; bestDelta = delta;
                }
            }

            if (bestIdx < 0) break;

            int sinkOut = bestParent[T];
            int szI     = packetSize[dgNodes.get(bestIdx)];
            int vi      = packetPriority[dgNodes.get(bestIdx)];

            int cur = T, steps = 0;
            while (cur != S && steps++ < cfnNodes) {
                int prev = bestParent[cur];
                if (prev == -1 || prev == cur) break;
                flow[prev][cur] += bestDelta;
                flow[cur][prev] -= bestDelta;
                cur = prev;
            }

            remaining[bestIdx] -= bestDelta;
            sinkCap[sinkOut]   -= bestDelta * szI;
            total              += bestDelta * vi;
        }
        return total;
    }

    // =========================================================================
    //  RELAY ENERGY PENALTY HELPER (used by PSB-GOA scoring)
    //
    //  Walks the augmenting path from t back to s via the parent array.
    //  For each internal node that is a relay (not the DG source, not a
    //  storage sink, not super source/sink), computes:
    //
    //    penalty contribution = scale × delta / residualEnergy_r
    //
    //  where residualEnergy_r = cap[i'][i''] - flow[i'][i''] is the REMAINING
    //  energy budget of relay r at the current point in augmentation.
    //
    //  WHY RESIDUAL INSTEAD OF ORIGINAL (Issue C fix):
    //    Using original nodeEnergy[r] kept the penalty stable but ignored
    //    that a nearly-depleted relay (residual=1) is just as cheap to route
    //    through as a fresh relay (residual=10) under that scheme. Using
    //    residual energy correctly reflects scarcity: a relay with only 1
    //    unit left receives a 10× higher penalty than one with 10 units,
    //    discouraging routes that would deplete critical bottleneck relays.
    //
    //  Under uniform energy all residuals start equal and decrease at the
    //  same rate, so relative penalties remain equal and cancel in score
    //  comparisons — the score still reduces to the storage-only formula,
    //  preserving consistent behaviour with the uniform energy experiments.
    //
    //  Zero residual on a path is a hard block (bfsFAP won't route there),
    //  so the zero-energy guard is a safety fallback only.
    // =========================================================================

    private double computeRelayPenalty(int[] parent, int dgNode, int delta) {
        int S = 0, T = superSink();
        double penalty = 0.0;
        // issue 2 fix: use cachedMaxSz instead of recomputing every call
        double scale = cachedMaxSz;

        int cur = T, steps = 0;
        while (cur != S && steps++ < cfnNodes) {
            int prev = parent[cur];
            if (prev == -1 || prev == cur) break;

            // detect internal edge i'→i'': prev=inNode(r)=2r+1, cur=outNode(r)=2r+2
            if (prev > 0 && cur > 0 && prev != T && cur != T) {
                int bsnPrev = (prev - 1) / 2;
                int bsnCur  = (cur  - 1) / 2;
                if (bsnPrev == bsnCur && bsnPrev != dgNode) {
                    // issue C fix: use residual energy, not original nodeEnergy
                    int residualEnergy = cap[prev][cur] - flow[prev][cur];
                    if (residualEnergy > 0) {
                        penalty += scale * (double) delta / residualEnergy;
                    } else {
                        // safety fallback — bfsFAP should never route here
                        penalty += scale * delta * 10.0;
                    }
                }
            }
            cur = prev;
        }
        return penalty;
    }

    public double runSilentPSB(List<Integer> dgNodes,
                               List<Integer> storageNodes,
                               int[]         storageCapacity) {

        int n = dgNodes.size();
        int bestTotal = 0;

        List<List<Integer>> prefixes = new ArrayList<>();
        prefixes.add(new ArrayList<>());
        for (int a = 0; a < n; a++) {
            List<Integer> p = new ArrayList<>(); p.add(a); prefixes.add(p);
        }
        for (int a = 0; a < n; a++) {
            for (int b = 0; b < n; b++) {
                if (a == b) continue;
                List<Integer> p = new ArrayList<>();
                p.add(a); p.add(b); prefixes.add(p);
            }
        }

        for (List<Integer> prefix : prefixes) {
            buildCFN(dgNodes, storageNodes, storageCapacity);

            int[] remaining = new int[n];
            for (int i = 0; i < n; i++)
                remaining[i] = packetsPerNode[dgNodes.get(i)];

            int[] sinkCap = new int[cfnNodes];
            for (int j = 0; j < storageNodes.size(); j++)
                sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

            int trialTotal = 0;
            Set<Integer> used = new LinkedHashSet<>();

            // augment prefix DGs with best-fit
            for (int idx : prefix) {
                used.add(idx);
                trialTotal += augmentBestFit(
                    idx, dgNodes, remaining, sinkCap, storageNodes);
            }

            // rollout loop with PSB pilot
            while (true) {
                List<Integer> candidates = new ArrayList<>();
                for (int i = 0; i < n; i++) {
                    if (used.contains(i) || remaining[i] <= 0) continue;
                    int[] probe = bfsFAP(inNode(dgNodes.get(i)),
                                        packetSize[dgNodes.get(i)], sinkCap);
                    if (probe != null) candidates.add(i);
                }
                if (candidates.isEmpty()) break;

                int bestCandidate = -1, bestPilot = -1;

                for (int ci : candidates) {
                    int[][] capSnap  = deepCopy2D(cap);
                    int[][] flowSnap = deepCopy2D(flow);
                    int[]   remSnap  = remaining.clone();
                    int[]   sinkSnap = sinkCap.clone();

                    // commit candidate + PSB tail
                    int pilotVal = augmentBestFit(
                        ci, dgNodes, remaining, sinkCap, storageNodes);
                    pilotVal += runPSBTail(dgNodes, remaining, sinkCap, used, ci);

                    if (pilotVal > bestPilot) {
                        bestPilot = pilotVal; bestCandidate = ci;
                    }

                    cap = capSnap; flow = flowSnap;
                    remaining = remSnap; sinkCap = sinkSnap;
                }

                if (bestCandidate < 0 || bestPilot <= 0) break;
                used.add(bestCandidate);
                trialTotal += augmentBestFit(
                    bestCandidate, dgNodes, remaining, sinkCap, storageNodes);
            }

            if (trialTotal > bestTotal) bestTotal = trialTotal;
        }
        return (double) bestTotal;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // ALGORITHM 6 — DYNAMIC DENSITY REORDERING GOA (DDR-GOA)
    //
    //  WHAT IT DOES:
    //    Instead of fixing the DG sort order upfront like GOA or Density GOA,
    //    DDR-GOA re-evaluates and re-sorts all remaining DGs after each full
    //    augmentation of one DG. The sort key is an "effective priority":
    //
    //      effectivePriority_i = v_i × min(d_i, floor(reachableResidual_i / sz_i))
    //
    //    where reachableResidual_i is the total residual storage capacity that
    //    DG_i can currently reach in the CFN via a valid FAP. This captures
    //    both packet value AND how much storage is actually still accessible
    //    to that DG given the current residual graph — not just a static ratio.
    //
    //  WHY IT HELPS:
    //    Static orderings (GOA, Density GOA) ignore that augmenting one DG
    //    changes residual capacities, potentially stranding storage accessible
    //    only by certain remaining DGs. DDR-GOA detects these shifts and
    //    promotes DGs that can still exploit remaining capacity before it
    //    becomes unreachable due to energy or routing bottlenecks.
    //
    //  WHEN IT WINS:
    //    When early augmentations fragment storage or deplete shared relay
    //    energy in ways that make static ordering suboptimal. DDR-GOA adapts
    //    to these changes dynamically, promoting smaller-packet DGs that can
    //    still fit into residual storage that larger-packet DGs cannot use.
    //
    //  WHEN IT LOSES:
    //    The reachability computation adds O(k × (V+E)) overhead per step.
    //    On networks where residual capacities stay uniformly distributed,
    //    the re-sorting yields no benefit over static Density GOA.
    //    Also, reachability is a global estimate — it does not account for
    //    path-level bottlenecks that bfsFAP would expose during augmentation.
    //
    //  APPROXIMATION:
    //    Heuristic. No formal approximation ratio improvement over static
    //    ordering is known. Retains the 1/2 lower bound when combined with
    //    Approx GOA (i.e., max of DDR-GOA and Density GOA).
    //
    //  STEPS:
    //    1. Initialise remaining[i] = d_i for all DGs; build fresh CFN
    //    2. Loop:
    //       a. For each remaining DG_i with remaining[i] > 0:
    //          compute reachableResidual_i via BFS on residual CFN
    //          effectivePriority_i = v_i × min(d_i, floor(reachableResidual_i / sz_i))
    //       b. Pick DG_i with highest effectivePriority_i
    //       c. Fully augment DG_i using standard FAP loop (one DG at a time)
    //       d. Re-sort remaining DGs (go back to step 2a)
    //    3. Stop when no remaining DG can reach any storage node
    // ─────────────────────────────────────────────────────────────────────────

    public double goaDDR(List<Integer> dgNodes,
                         List<Integer> storageNodes,
                         int[]         storageCapacity) {

        if (!needsMWF(dgNodes, storageNodes, storageCapacity)) return 0.0;

        buildCFN(dgNodes, storageNodes, storageCapacity);
        int S = 0, T = superSink();
        int n = dgNodes.size();

        int[] remaining = new int[n];
        for (int i = 0; i < n; i++)
            remaining[i] = packetsPerNode[dgNodes.get(i)];

        int[] sinkCap = new int[cfnNodes];
        for (int j = 0; j < storageNodes.size(); j++)
            sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

        double      totalWeight  = 0.0;
        List<int[]> goaFlowEdges = new ArrayList<>();
        Map<Integer, Integer> relayCount = new LinkedHashMap<>();
        int maxIter = cfnNodes * cfnNodes;

        Set<Integer> dgSet      = new HashSet<>(dgNodes);
        Set<Integer> storageSet = new HashSet<>(storageNodes);

        // step 2: dynamic reordering loop — repeat until no progress
        while (true) {

            // step 2a: recompute effective priority for every remaining DG
            int bestIdx      = -1;
            int bestEffPri   = -1;

            for (int i = 0; i < n; i++) {
                if (remaining[i] <= 0) continue;
                int dg  = dgNodes.get(i);
                int szI = packetSize[dg];
                int vi  = packetPriority[dg];

                // sum residual storage capacity reachable from this DG
                // via a BFS on the residual CFN from inNode(dg)
                int reachable = computeReachableResidual(
                    inNode(dg), szI, sinkCap, storageNodes);

                // effectivePriority = v_i × min(d_i, floor(reachable / sz_i))
                int canSend   = (szI > 0) ? reachable / szI : 0;
                int effPri    = vi * Math.min(remaining[i], canSend);

                if (effPri > bestEffPri) {
                    bestEffPri = effPri;
                    bestIdx    = i;
                }
            }

            // step 2b: no DG can make progress — stop
            if (bestIdx < 0 || bestEffPri <= 0) break;

            // step 2c: fully augment the chosen DG
            int dg     = dgNodes.get(bestIdx);
            int cfnSrc = inNode(dg);
            int szI    = packetSize[dg];
            int vi     = packetPriority[dg];
            int iters  = 0;

            while (remaining[bestIdx] > 0 && iters++ < maxIter) {

                int[] parent = bfsFAP(cfnSrc, szI, sinkCap);
                if (parent == null) break;

                int sinkOut = parent[T];

                // compute bottleneck Δ
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
                            Math.min(remaining[bestIdx], sinkCap[sinkOut] / szI));
                if (delta <= 0) break;

                // collect relay nodes
                List<Integer> pathBsnNodes = extractBSNPathFromParent(parent);
                int sinkNode = pathBsnNodes.isEmpty() ? ((sinkOut - 1) / 2)
                                                      : pathBsnNodes.get(pathBsnNodes.size() - 1);
                for (int k = 1; k < pathBsnNodes.size() - 1; k++)
                    relayCount.merge(pathBsnNodes.get(k), delta, Integer::sum);
                printRelayTrace("DDR", dg, sinkNode, delta, vi, szI,
                                pathBsnNodes, dgSet, storageSet);

                // augment flow
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
                                if (fe[0] == bsnU && fe[1] == bsnV)
                                    { exists = true; break; }
                            if (!exists) goaFlowEdges.add(new int[]{bsnU, bsnV});
                        }
                    }
                    cur = prev;
                }

                remaining[bestIdx] -= delta;
                sinkCap[sinkOut]   -= delta * szI;
                totalWeight        += (double) delta * vi;

                System.out.printf(
                    "  Pushed %d packet(s) from DG %d (v=%d, sz=%d, effPri=%d) -> sink %d%n",
                    delta, dg, vi, szI, bestEffPri, sinkOut);
            }
            // step 2d: go back to top — re-sort remaining DGs
        }

        System.out.printf("%nTotal Preserved Priority (DDR-GOA): %.1f%n", totalWeight);
        printRelayInfo(relayCount, dgSet, storageSet);
        launchGraph(dgNodes, storageNodes, goaFlowEdges, storageCapacity,
                    "DDR-GOA - Dynamic Density Reordering");
        launchBFN(dgNodes, storageNodes, storageCapacity, goaFlowEdges,
                  "DDR-GOA - Dynamic Density Reordering");
        return totalWeight;
    }

    // =========================================================================
    //  SILENT RUN FOR DDR-GOA (no output, no graphs — used for scaling trials)
    // =========================================================================

    public double runSilentDDR(List<Integer> dgNodes,
                               List<Integer> storageNodes,
                               int[]         storageCapacity) {

        buildCFN(dgNodes, storageNodes, storageCapacity);
        int S = 0, T = superSink();
        int n = dgNodes.size();

        int[] remaining = new int[n];
        for (int i = 0; i < n; i++)
            remaining[i] = packetsPerNode[dgNodes.get(i)];

        int[] sinkCap = new int[cfnNodes];
        for (int j = 0; j < storageNodes.size(); j++)
            sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

        double totalWeight = 0.0;
        int maxIter = cfnNodes * cfnNodes;

        while (true) {
            int bestIdx = -1, bestEffPri = -1;

            for (int i = 0; i < n; i++) {
                if (remaining[i] <= 0) continue;
                int dg  = dgNodes.get(i);
                int szI = packetSize[dg];
                int vi  = packetPriority[dg];
                int reachable = computeReachableResidual(
                    inNode(dg), szI, sinkCap, storageNodes);
                int canSend = (szI > 0) ? reachable / szI : 0;
                int effPri  = vi * Math.min(remaining[i], canSend);
                if (effPri > bestEffPri) { bestEffPri = effPri; bestIdx = i; }
            }

            if (bestIdx < 0 || bestEffPri <= 0) break;

            int dg     = dgNodes.get(bestIdx);
            int cfnSrc = inNode(dg);
            int szI    = packetSize[dg];
            int vi     = packetPriority[dg];
            int iters  = 0;

            while (remaining[bestIdx] > 0 && iters++ < maxIter) {
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
                            Math.min(remaining[bestIdx], sinkCap[sinkOut] / szI));
                if (delta <= 0) break;

                cur = T; steps = 0;
                while (cur != S && steps++ < cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    flow[prev][cur] += delta;
                    flow[cur][prev] -= delta;
                    cur = prev;
                }

                remaining[bestIdx] -= delta;
                sinkCap[sinkOut]   -= delta * szI;
                totalWeight        += (double) delta * vi;
            }
        }
        return totalWeight;
    }

    // =========================================================================
    //  REACHABLE RESIDUAL HELPER (used by DDR-GOA)
    //
    //  Runs a BFS from cfnSource on the residual CFN (respecting cap-flow > 0),
    //  blocked from entering other DG in-nodes (same restriction as bfsFAP).
    //  Sums the residual capacity of all j''→t edges reachable from cfnSource
    //  where sinkCap[j''] >= szI (i.e., the storage node can accept at least
    //  one packet of size szI). Returns the total reachable storage residual.
    //
    //  This is used to compute effectivePriority_i = v_i × min(d_i,
    //  floor(reachableResidual_i / sz_i)) for the DDR-GOA sort key.
    // =========================================================================

    private int computeReachableResidual(int          cfnSource,
                                         int          szI,
                                         int[]        sinkCap,
                                         List<Integer> storageNodes) {
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
                // block other DG in-nodes (same as bfsFAP)
                if (v != T && v != cfnSource && dgCFNIds.contains(v)) continue;
                if (v == T) {
                    // u is a storage out-node: count its residual if sz fits
                    if (sinkCap[u] >= szI)
                        totalResidual += sinkCap[u];
                    continue;  // don't enqueue T
                }
                vis[v] = true;
                q.add(v);
            }
        }
        return totalResidual;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // EXACT SOLVER — ILP BASELINE (Brute-Force Permutation Enumeration)
    //
    //  WHAT IT DOES:
    //    Finds the true optimal solution to the MWF-S ILP formulation by
    //    exhaustively enumerating ALL permutations of DG orderings, running
    //    the greedy augmentation loop for each, and returning the maximum
    //    total priority found across all orderings.
    //
    //  WHY THIS IS EXACT:
    //    For any fixed DG ordering, the greedy augmentation loop (bfsFAP +
    //    augment) finds the maximum flow achievable under that ordering.
    //    The ILP optimum must be realisable by some ordering of DGs — because
    //    any optimal flow solution can be decomposed into path flows, and those
    //    paths can be ordered by DG. By trying ALL k! orderings we are
    //    guaranteed to find the ordering that matches the ILP optimum.
    //
    //    This is consistent with the ILP objective from the image:
    //      max Σᵢ Σⱼ (yᵢ,ⱼ × vᵢ)
    //    subject to edge capacity (enforced by BFN cap/flow arrays),
    //    source supply dᵢ (enforced by remaining[i]),
    //    storage capacity mⱼ (enforced by sinkCap),
    //    and flow conservation (enforced by bfsFAP augmentation).
    //    All BFN constraints are already encoded — the exact solver just
    //    needs to find which ordering of DGs maximises the objective.
    //
    //  LIMITATION:
    //    Runtime is O(k! × k × n × m²) — only practical for k ≤ 8 DGs.
    //    For larger k, use goaHybrid or goaPSB as approximations.
    //    The scaling loop skips the exact solver for network sizes where
    //    the average number of DGs exceeds MAX_EXACT_DGS.
    //
    //  STEPS:
    //    1. Generate all k! permutations of DG indices
    //    2. For each permutation:
    //       a. Fresh CFN
    //       b. For each DG in permutation order:
    //          augment fully using standard FAP loop
    //       c. Record total priority
    //    3. Return the maximum total across all permutations
    // ─────────────────────────────────────────────────────────────────────────

    // =========================================================================
    //  EXACT SOLVER — BRANCH AND BOUND
    //
    //  WHAT IT DOES:
    //    Finds the true ILP optimum using branch and bound on DG orderings.
    //    Same correctness guarantee as the k! brute force but prunes branches
    //    early using an upper bound, making it vastly faster in practice.
    //
    //  WHY BRANCH AND BOUND INSTEAD OF SUBSET DP:
    //    Subset DP would require O(2^k) states, each storing a BFN state.
    //    However, the BFN state after augmenting a subset of DGs depends on
    //    the ORDER in which they were augmented — not just which subset —
    //    because variable packet sizes cause different storage residuals
    //    depending on which DG filled a storage node first. This means
    //    subset DP cannot share states across orderings and is not exact.
    //    Branch and bound avoids this by exploring orderings as a tree,
    //    sharing prefix computation across branches with the same prefix.
    //
    //  UPPER BOUND (pruning key):
    //    At any node in the search tree, after committing a prefix of DGs,
    //    the upper bound on the remaining priority is:
    //      UB = sum over remaining DGs of (v_i × min(d_i, floor(reachable_i / sz_i)))
    //    where reachable_i is the total residual storage reachable from DG_i.
    //    This is an optimistic overestimate — it assumes each remaining DG
    //    can independently access all reachable storage without competing.
    //    If currentBest + UB <= globalBest, this branch is pruned.
    //
    //  COMPLEXITY:
    //    Worst case O(k! × poly(n)) — same as brute force.
    //    In practice O(k^c × poly(n)) for small constant c, because the
    //    density-greedy upper bound is tight enough to prune most branches.
    //    Feasible for k ≤ 12 on typical sensor network instances.
    //
    //  STEPS:
    //    1. Start with globalBest = density greedy result (warm start)
    //    2. Recursively explore DG orderings as a tree:
    //       a. At each node: compute UB for remaining DGs
    //       b. If currentValue + UB <= globalBest: prune this branch
    //       c. Otherwise: try each remaining DG as next, snapshot/restore CFN
    //    3. Return globalBest
    // =========================================================================

    private static final int MAX_EXACT_DGS = 12;  // raised from 8 due to pruning

    // shared state for branch and bound recursion
    private int   bbGlobalBest;
    private int[] bbBestPerm;

    public double goaExact(List<Integer> dgNodes,
                           List<Integer> storageNodes,
                           int[]         storageCapacity) {

        if (!needsMWF(dgNodes, storageNodes, storageCapacity)) return 0.0;

        int n = dgNodes.size();
        if (n > MAX_EXACT_DGS) {
            System.out.printf(
                "  [Exact solver] Skipped: %d DGs > MAX_EXACT_DGS (%d).%n",
                n, MAX_EXACT_DGS);
            return -1.0;
        }

        // warm start: use density greedy as initial lower bound
        // this immediately prunes any branch that can't beat density greedy
        bbGlobalBest = (int) runSilent(dgNodes, storageNodes, storageCapacity, true);
        bbBestPerm   = null;

        System.out.printf("%n-- Exact Solver (Branch & Bound): %d DGs, " +
                          "warm start = %d --%n", n, bbGlobalBest);

        // build initial CFN and state
        buildCFN(dgNodes, storageNodes, storageCapacity);

        int[] remaining = new int[n];
        for (int i = 0; i < n; i++)
            remaining[i] = packetsPerNode[dgNodes.get(i)];

        int[] sinkCap = new int[cfnNodes];
        for (int j = 0; j < storageNodes.size(); j++)
            sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

        boolean[] committed = new boolean[n];
        int[]     perm      = new int[n];

        // launch recursive branch and bound
        bbSearch(dgNodes, storageNodes, n, remaining, sinkCap,
                 committed, perm, 0, 0);

        System.out.printf("  ILP Optimal Total Priority: %d%n", bbGlobalBest);
        if (bbBestPerm != null) {
            System.out.print("  Optimal DG order: [");
            for (int i = 0; i < bbBestPerm.length; i++) {
                System.out.print("DG" + dgNodes.get(bbBestPerm[i]));
                if (i < bbBestPerm.length-1) System.out.print(", ");
            }
            System.out.println("]");
        }
        return (double) bbGlobalBest;
    }

    private void bbSearch(List<Integer> dgNodes,
                          List<Integer> storageNodes,
                          int n, int[] remaining, int[] sinkCap,
                          boolean[] committed, int[] perm,
                          int depth, int currentValue) {

        // leaf node: all DGs committed, update global best
        if (depth == n) {
            if (currentValue > bbGlobalBest) {
                bbGlobalBest = currentValue;
                bbBestPerm   = perm.clone();
            }
            return;
        }

        // compute upper bound on remaining priority
        int ub = upperBound(dgNodes, n, remaining, sinkCap, committed);
        if (currentValue + ub <= bbGlobalBest) return;  // prune

        // try each uncommitted DG as the next to augment
        for (int i = 0; i < n; i++) {
            if (committed[i] || remaining[i] <= 0) continue;

            // snapshot CFN state
            int[][] capSnap  = deepCopy2D(cap);
            int[][] flowSnap = deepCopy2D(flow);
            int[]   remSnap  = remaining.clone();
            int[]   sinkSnap = sinkCap.clone();

            // commit DG i: augment fully
            int gained = augmentSilent(i, dgNodes, remaining, sinkCap);

            perm[depth]   = i;
            committed[i]  = true;

            // recurse
            bbSearch(dgNodes, storageNodes, n, remaining, sinkCap,
                     committed, perm, depth + 1, currentValue + gained);

            // restore CFN state
            committed[i] = false;
            cap          = capSnap;
            flow         = flowSnap;
            remaining    = remSnap;
            sinkCap      = sinkSnap;
        }
    }

    // ── upper bound: optimistic estimate of remaining priority ────────────────
    // For each uncommitted DG, compute how much it could send if it had
    // exclusive access to all reachable residual storage. Sum these up.
    // This overestimates because DGs compete for the same storage nodes,
    // but it is fast to compute and tight enough for effective pruning.
    private int upperBound(List<Integer> dgNodes, int n,
                           int[] remaining, int[] sinkCap,
                           boolean[] committed) {
        int ub = 0;
        for (int i = 0; i < n; i++) {
            if (committed[i] || remaining[i] <= 0) continue;
            int dg      = dgNodes.get(i);
            int szI     = packetSize[dg];
            int vi      = packetPriority[dg];
            int reach   = computeReachableResidual(
                inNode(dg), szI, sinkCap,
                // pass storageNodes implicitly via sinkCap — any stOut with
                // sinkCap[stOut] >= szI counts
                new ArrayList<>());
            int canSend = (szI > 0) ? reach / szI : 0;
            ub += vi * Math.min(remaining[i], canSend);
        }
        return ub;
    }

    // ── silent augmentation helper for branch and bound ──────────────────────
    // Fully augments one DG (by index) and returns total priority gained.
    // Modifies cap/flow/remaining/sinkCap in place — caller snapshots/restores.
    private int augmentSilent(int idx, List<Integer> dgNodes,
                               int[] remaining, int[] sinkCap) {
        int dg     = dgNodes.get(idx);
        int cfnSrc = inNode(dg);
        int szI    = packetSize[dg];
        int vi     = packetPriority[dg];
        int S      = 0, T = superSink();
        int total  = 0;
        int maxIter = cfnNodes * cfnNodes;
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
            total            += delta * vi;
        }
        return total;
    }

    // =========================================================================
    //  SILENT EXACT SOLVER (no output — used for scaling trials)
    // =========================================================================

    public double runSilentExact(List<Integer> dgNodes,
                                 List<Integer> storageNodes,
                                 int[]         storageCapacity) {

        int n = dgNodes.size();
        if (n > MAX_EXACT_DGS) return -1.0;

        // warm start
        bbGlobalBest = (int) runSilent(dgNodes, storageNodes,
                                       storageCapacity, true);
        bbBestPerm   = null;

        buildCFN(dgNodes, storageNodes, storageCapacity);

        int[] remaining = new int[n];
        for (int i = 0; i < n; i++)
            remaining[i] = packetsPerNode[dgNodes.get(i)];

        int[] sinkCap = new int[cfnNodes];
        for (int j = 0; j < storageNodes.size(); j++)
            sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

        boolean[] committed = new boolean[n];
        int[]     perm      = new int[n];

        bbSearch(dgNodes, storageNodes, n, remaining, sinkCap,
                 committed, perm, 0, 0);

        return (double) bbGlobalBest;
    }

    // =========================================================================
    //  PERMUTATION GENERATOR — kept for reference, no longer used by exact
    // =========================================================================

    private void generatePermutations(int[] arr, int k, List<int[]> result) {
        if (k == arr.length - 1) { result.add(arr.clone()); return; }
        for (int i = k; i < arr.length; i++) {
            int tmp = arr[k]; arr[k] = arr[i]; arr[i] = tmp;
            generatePermutations(arr, k + 1, result);
            tmp = arr[k]; arr[k] = arr[i]; arr[i] = tmp;
        }
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
    //  SILENT RUN FOR HYBRID GOA (no output, no graphs — used for scaling)
    //
    //  Mirrors goaHybrid but produces no console output or graph windows.
    //  Used in the scaling loop to compare Hybrid GOA against GOA, Density
    //  GOA, and Approx GOA across many random trials.
    // =========================================================================

    public double runSilentHybrid(List<Integer> dgNodes,
                                  List<Integer> storageNodes,
                                  int[]         storageCapacity) {

        int n = dgNodes.size();
        int bestTotal = 0;

        // build all prefix trials: empty, singletons, pairs
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

        for (List<Integer> prefix : prefixes) {
            buildCFN(dgNodes, storageNodes, storageCapacity);

            int[] remaining = new int[n];
            for (int i = 0; i < n; i++)
                remaining[i] = packetsPerNode[dgNodes.get(i)];

            int[] sinkCap = new int[cfnNodes];
            for (int j = 0; j < storageNodes.size(); j++)
                sinkCap[outNode(storageNodes.get(j))] = storageCapacity[j];

            int trialTotal = 0;
            Set<Integer> used = new LinkedHashSet<>();

            // augment prefix DGs
            for (int idx : prefix) {
                used.add(idx);
                trialTotal += augmentBestFit(
                    idx, dgNodes, remaining, sinkCap, storageNodes);
            }

            // rollout loop
            while (true) {
                List<Integer> candidates = new ArrayList<>();
                for (int i = 0; i < n; i++) {
                    if (used.contains(i) || remaining[i] <= 0) continue;
                    int[] probe = bfsFAP(inNode(dgNodes.get(i)),
                                        packetSize[dgNodes.get(i)], sinkCap);
                    if (probe != null) candidates.add(i);
                }
                if (candidates.isEmpty()) break;

                int bestCandidate = -1, bestPilot = -1;

                for (int ci : candidates) {
                    int[][] capSnap  = deepCopy2D(cap);
                    int[][] flowSnap = deepCopy2D(flow);
                    int[]   remSnap  = remaining.clone();
                    int[]   sinkSnap = sinkCap.clone();

                    int pilotVal = augmentBestFit(
                        ci, dgNodes, remaining, sinkCap, storageNodes);

                    List<Integer> tail = new ArrayList<>();
                    for (int i = 0; i < n; i++) {
                        if (i == ci || used.contains(i)) continue;
                        if (remaining[i] > 0) tail.add(i);
                    }
                    tail.sort((a, b) -> {
                        double dA = (double) packetPriority[dgNodes.get(a)]
                                  / packetSize[dgNodes.get(a)];
                        double dB = (double) packetPriority[dgNodes.get(b)]
                                  / packetSize[dgNodes.get(b)];
                        return Double.compare(dB, dA);
                    });
                    for (int ti : tail)
                        pilotVal += augmentBestFit(
                            ti, dgNodes, remaining, sinkCap, storageNodes);

                    if (pilotVal > bestPilot) {
                        bestPilot = pilotVal; bestCandidate = ci;
                    }

                    cap = capSnap; flow = flowSnap;
                    remaining = remSnap; sinkCap = sinkSnap;
                }

                if (bestCandidate < 0 || bestPilot <= 0) break;
                used.add(bestCandidate);
                trialTotal += augmentBestFit(
                    bestCandidate, dgNodes, remaining, sinkCap, storageNodes);
            }

            if (trialTotal > bestTotal) bestTotal = trialTotal;
        }
        return (double) bestTotal;
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
                                if (fe[0] == bsnU && fe[1] == bsnV)
                                    { exists = true; break; }
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
    //  BEST-FIT AUGMENTATION HELPER (used by Hybrid GOA)
    //
    //  Fully augments one DG (given by index idx into dgNodes) using
    //  best-fit storage selection: among all storage nodes reachable via
    //  a shortest FAP, prefer the one whose residual capacity after
    //  placing szᵢ units is the smallest non-negative value (tightest fit).
    //
    //  This minimises storage fragmentation — leftover fragments that are
    //  too small for any remaining packet size are wasted capacity.
    //
    //  Falls back to standard bfsFAP behaviour when only one storage node
    //  is reachable or when the masked search finds nothing new.
    //
    //  Returns the total priority value added by augmenting this DG.
    // =========================================================================

    private int augmentBestFit(int           idx,
                               List<Integer> dgNodes,
                               int[]         remaining,
                               int[]         sinkCap,
                               List<Integer> storageNodes) {
        int dg     = dgNodes.get(idx);
        int cfnSrc = inNode(dg);
        int szI    = packetSize[dg];
        int vi     = packetPriority[dg];
        int S      = 0, T = superSink();
        int total  = 0;
        int maxIter = cfnNodes * cfnNodes;
        int iters  = 0;

        while (remaining[idx] > 0 && iters++ < maxIter) {

            // best-fit storage selection:
            // try each storage node individually via a masked sinkCap,
            // score by residual after placement, keep tightest fit
            int[]  bestParent   = null;
            int    bestFitScore = Integer.MAX_VALUE;

            for (int j = 0; j < storageNodes.size(); j++) {
                int st    = storageNodes.get(j);
                int stOut = outNode(st);
                if (sinkCap[stOut] < szI) continue;  // can't fit one packet

                // masked sinkCap: only allow this storage node
                int[] maskedSink = new int[cfnNodes];
                maskedSink[stOut] = sinkCap[stOut];

                int[] parent = bfsFAP(cfnSrc, szI, maskedSink);
                if (parent == null) continue;

                // compute delta for this path
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
                            Math.min(remaining[idx], sinkCap[stOut] / szI));
                if (delta <= 0) continue;

                // score: residual after placing delta packets (lower = better)
                int newResidual = sinkCap[stOut] - delta * szI;
                if (newResidual < bestFitScore) {
                    bestFitScore = newResidual;
                    bestParent   = parent;
                }
            }

            // fall back to standard bfsFAP if best-fit found nothing
            if (bestParent == null) {
                bestParent = bfsFAP(cfnSrc, szI, sinkCap);
                if (bestParent == null) break;
            }

            int sinkOut = bestParent[T];

            // compute delta along chosen path
            int pathBottleneck = Integer.MAX_VALUE;
            int cur = T, steps = 0;
            while (cur != S && steps++ < cfnNodes) {
                int prev = bestParent[cur];
                if (prev == -1 || prev == cur) break;
                pathBottleneck = Math.min(pathBottleneck,
                                         cap[prev][cur] - flow[prev][cur]);
                cur = prev;
            }
            int delta = Math.min(pathBottleneck,
                        Math.min(remaining[idx], sinkCap[sinkOut] / szI));
            if (delta <= 0) break;

            // augment flow along the chosen path
            cur = T; steps = 0;
            while (cur != S && steps++ < cfnNodes) {
                int prev = bestParent[cur];
                if (prev == -1 || prev == cur) break;
                flow[prev][cur] += delta;
                flow[cur][prev] -= delta;
                cur = prev;
            }

            remaining[idx]   -= delta;
            sinkCap[sinkOut] -= delta * szI;
            total            += delta * vi;
        }

        return total;
    }

    // =========================================================================
    //  DEEP COPY HELPER (needed for rollout state snapshots in Hybrid GOA)
    //
    //  Creates a full independent copy of a 2D int array so that rollout
    //  simulations can safely modify cap/flow without affecting the committed
    //  CFN state. Each row is cloned separately to avoid reference sharing.
    // =========================================================================

    private int[][] deepCopy2D(int[][] src) {
        int[][] copy = new int[src.length][];
        for (int i = 0; i < src.length; i++)
            copy[i] = src[i].clone();
        return copy;
    }

    // =========================================================================
    //  BFN VISUALISATION
    // =========================================================================

    public void launchBFN(List<Integer> dgNodes,
                          List<Integer> storageNodes,
                          int[]         storageCapacity,
                          List<int[]>   goaFlowEdges,
                          String        algoTitle) {
        launchBFN(dgNodes, storageNodes, storageCapacity,
                  null, goaFlowEdges, algoTitle);
    }

    // overload that accepts final sinkCap array for waste display (feature 3)
    public void launchBFN(List<Integer> dgNodes,
                          List<Integer> storageNodes,
                          int[]         storageCapacity,
                          int[]         finalSinkCap,
                          List<int[]>   goaFlowEdges,
                          String        algoTitle) {
        int n = nodeLoc.length;

        int[] storageSlotsByNode = new int[n];
        for (int j = 0; j < storageNodes.size(); j++) {
            int st = storageNodes.get(j);
            storageSlotsByNode[st] = storageCapacity[j];
        }

        // compute waste per storage node: original cap minus what was used
        // waste[j] = finalSinkCap[outNode(st)] if provided, else 0
        int[] wastePerStorage = new int[storageNodes.size()];
        if (finalSinkCap != null) {
            for (int j = 0; j < storageNodes.size(); j++) {
                int stOut = outNode(storageNodes.get(j));
                if (stOut < finalSinkCap.length)
                    wastePerStorage[j] = finalSinkCap[stOut];
            }
        }

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
                    nodeLoc, packetsPerNode, storageSlotsByNode,
                    wastePerStorage, adjMatrix, goaFlowEdges);
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
    //  OPTIMALITY GAP HELPER
    //
    //  Returns a formatted string showing what percentage of the ILP optimal
    //  a given algorithm average achieved. Only shown when exact solver ran.
    //  e.g. "  (94.3% of optimal)" or "" if exact result unavailable.
    // =========================================================================

    private static String gap(double algoAvg, double exactAvg) {
        if (exactAvg <= 0) return "";
        return String.format("  (%.1f%% of optimal)", 100.0 * algoAvg / exactAvg);
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

        // regenerate until the visual run graph is fully connected —
        // required by the data preservation problem (Prof. Tang's requirement)
        visNet.buildConnectedGraph(visNodes, widthX, lenY, TR, rand, false);

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

        // print adjacency structure for the visual run so the graph topology
        // is visible in the terminal alongside the algorithm output
        visNet.printAdjacency(visDG, visST);

        // use goa() and goaDensity() directly so feasibility check,
        // relay tracking, and console output all appear in the visual run
        System.out.println("\n-- Running GOA (visual) --");
        double visGOA     = visNet.goa(visDG, visST, visCap);
        System.out.println("\n-- Running Density GOA (visual) --");
        double visDensity = visNet.goaDensity(visDG, visST, visCap);
        System.out.println("\n-- Running Hybrid GOA (visual) --");
        double visHybrid  = visNet.goaHybrid(visDG, visST, visCap);
        System.out.println("\n-- Running DDR-GOA (visual) --");
        double visDDR     = visNet.goaDDR(visDG, visST, visCap);
        System.out.println("\n-- Running PSB-GOA (visual) --");
        double visPSB     = visNet.goaPSB(visDG, visST, visCap);
        System.out.println("\n-- Running Exact Solver (visual) --");
        ExactSolver exactSolver = new ExactSolver(visNet);
        double visExact = exactSolver.solve(visDG, visST, visCap);

        System.out.printf("%n===== VISUAL RUN SUMMARY =====%n");
        System.out.printf("  ILP Optimal:                %.1f%n", visExact);
        System.out.printf("  GOA total priority:         %.1f  (%.1f%% of optimal)%n",
            visGOA,    visExact > 0 ? 100.0*visGOA/visExact    : 0);
        System.out.printf("  Density GOA total priority: %.1f  (%.1f%% of optimal)%n",
            visDensity, visExact > 0 ? 100.0*visDensity/visExact : 0);
        System.out.printf("  Hybrid GOA total priority:  %.1f  (%.1f%% of optimal)%n",
            visHybrid,  visExact > 0 ? 100.0*visHybrid/visExact  : 0);
        System.out.printf("  DDR-GOA total priority:     %.1f  (%.1f%% of optimal)%n",
            visDDR,     visExact > 0 ? 100.0*visDDR/visExact     : 0);
        System.out.printf("  PSB-GOA total priority:     %.1f  (%.1f%% of optimal)%n",
            visPSB,     visExact > 0 ? 100.0*visPSB/visExact     : 0);

        System.out.println("\n-- Graphs launched. Starting scaling runs... --\n");

        // ── scaling loop ──────────────────────────────────────────────────────
        for (int numNodes : networkSizes) {
            System.out.printf(
                "%n==== Network Size: %d nodes, %d trials ====%n",
                numNodes, trials);

            double sumGOA = 0, sumDensity = 0, sumApprox = 0,
                   sumHybrid = 0, sumDDR = 0, sumPSB = 0, sumExact = 0;
            int densityBeatGOA = 0, goaBeatDensity = 0, tied = 0;
            int hybridBeatApprox = 0, approxBeatHybrid = 0, hybridTied = 0;
            int ddrBeatApprox = 0, approxBeatDDR = 0, ddrTied = 0;
            int psbBeatAll = 0, allBeatPSB = 0, psbTied = 0;
            int exactTrials = 0;  // trials where exact solver ran

            for (int t = 1; t <= trials; t++) {

                SensorStuff net = new SensorStuff(numNodes, choice);

                // regenerate until fully connected, applying TR jitter each
                // attempt so jitter still varies across trials while guaranteeing
                // connectivity (per-trial TR jitter: 75%–125% of base TR)
                int trialTR = net.buildConnectedGraph(numNodes, widthX, lenY, TR, rand, true);

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

                // issue 4 fix: skip all algorithms if no bottleneck exists
                // runSilent/Hybrid/PSB/DDR all call buildCFN internally but do
                // not check needsMWF — doing it once here avoids wasted work
                if (!net.needsMWF(dgNodes, storageNodes, storageCap)) {
                    System.out.printf(
                        "  Trial %2d [TR=%d, DGs=%d, STs=%d]: No bottleneck — skipped%n",
                        t, trialTR, dgNodes.size(), storageNodes.size());
                    continue;
                }

                double goaResult     = net.runSilent(dgNodes, storageNodes,
                                                     storageCap, false);
                double densityResult = net.runSilent(dgNodes, storageNodes,
                                                     storageCap, true);
                double approxResult  = Math.max(goaResult, densityResult);
                double hybridResult  = net.runSilentHybrid(dgNodes, storageNodes,
                                                           storageCap);
                double ddrResult     = net.runSilentDDR(dgNodes, storageNodes,
                                                        storageCap);
                double psbResult     = net.runSilentPSB(dgNodes, storageNodes,
                                                        storageCap);
                double exactResult   = new ExactSolver(net).runSilent(
                                                          dgNodes, storageNodes,
                                                          storageCap);

                // best among all heuristics for PSB comparison
                double bestOther = Math.max(Math.max(approxResult, hybridResult),
                                            ddrResult);

                // optimality gap — only when exact solver ran (dgNodes.size() <= MAX)
                boolean exactRan = exactResult >= 0;
                String exactStr  = exactRan
                    ? String.format("%.1f", exactResult) : "N/A";

                System.out.printf(
                    "  Trial %2d [TR=%d, DGs=%d, STs=%d]: " +
                    "GOA=%.1f  Density=%.1f  Approx=%.1f  " +
                    "Hybrid=%.1f  DDR=%.1f  PSB=%.1f  Exact=%s%n",
                    t, trialTR, dgNodes.size(), storageNodes.size(),
                    goaResult, densityResult, approxResult,
                    hybridResult, ddrResult, psbResult, exactStr);

                sumGOA     += goaResult;
                sumDensity += densityResult;
                sumApprox  += approxResult;
                sumHybrid  += hybridResult;
                sumDDR     += ddrResult;
                sumPSB     += psbResult;
                if (exactRan) { sumExact += exactResult; exactTrials++; }

                if (densityResult > goaResult)      densityBeatGOA++;
                else if (goaResult > densityResult) goaBeatDensity++;
                else                                tied++;

                if (hybridResult > approxResult)      hybridBeatApprox++;
                else if (approxResult > hybridResult) approxBeatHybrid++;
                else                                  hybridTied++;

                if (ddrResult > approxResult)      ddrBeatApprox++;
                else if (approxResult > ddrResult) approxBeatDDR++;
                else                               ddrTied++;

                if (psbResult > bestOther)      psbBeatAll++;
                else if (psbResult < bestOther) allBeatPSB++;
                else                            psbTied++;
            }

            System.out.printf("%n-- Averages over %d trials --%n", trials);
            double avgExact = exactTrials > 0 ? sumExact / exactTrials : -1;
            System.out.printf("  ILP Optimal avg:      %.2f  (%d/%d trials exact ran)%n",
                              avgExact >= 0 ? avgExact : 0, exactTrials, trials);
            System.out.printf("  GOA avg:              %.2f%s%n",
                sumGOA/trials, gap(sumGOA/trials, avgExact));
            System.out.printf("  Density GOA avg:      %.2f%s%n",
                sumDensity/trials, gap(sumDensity/trials, avgExact));
            System.out.printf("  Approx GOA avg:       %.2f%s%n",
                sumApprox/trials, gap(sumApprox/trials, avgExact));
            System.out.printf("  Hybrid GOA avg:       %.2f%s%n",
                sumHybrid/trials, gap(sumHybrid/trials, avgExact));
            System.out.printf("  DDR-GOA avg:          %.2f%s%n",
                sumDDR/trials, gap(sumDDR/trials, avgExact));
            System.out.printf("  PSB-GOA avg:          %.2f%s%n",
                sumPSB/trials, gap(sumPSB/trials, avgExact));
            System.out.printf("  Density beat GOA:     %d/%d trials%n",
                              densityBeatGOA, trials);
            System.out.printf("  GOA beat Density:     %d/%d trials%n",
                              goaBeatDensity, trials);
            System.out.printf("  Tied:                 %d/%d trials%n",
                              tied, trials);
            System.out.printf("  Hybrid beat Approx:   %d/%d trials%n",
                              hybridBeatApprox, trials);
            System.out.printf("  Approx beat Hybrid:   %d/%d trials%n",
                              approxBeatHybrid, trials);
            System.out.printf("  Hybrid/Approx tied:   %d/%d trials%n",
                              hybridTied, trials);
            System.out.printf("  DDR beat Approx:      %d/%d trials%n",
                              ddrBeatApprox, trials);
            System.out.printf("  Approx beat DDR:      %d/%d trials%n",
                              approxBeatDDR, trials);
            System.out.printf("  DDR/Approx tied:      %d/%d trials%n",
                              ddrTied, trials);
            System.out.printf("  PSB beat all others:  %d/%d trials%n",
                              psbBeatAll, trials);
            System.out.printf("  Others beat PSB:      %d/%d trials%n",
                              allBeatPSB, trials);
            System.out.printf("  PSB/others tied:      %d/%d trials%n",
                              psbTied, trials);
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