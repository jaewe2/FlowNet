import java.util.*;

// =============================================================================
//  GOA — Greedy Optimal Algorithm (size-aware extension of paper Algo. 3)
//
//  Pushes the highest-priority packets first. Sorts DGs by vᵢ and greedily
//  routes as many packets as possible before moving on.
//
//  Optimal for the original MWF-U problem (all packets unit size).
//  May be suboptimal when packet sizes vary (see DensityGOA).
//
//  WHEN IT WINS:  high-priority packets are also small.
//  WHEN IT LOSES: large high-priority packets fragment storage.
//    Counterexample: DG0 v=10 sz=3 d=2 vs DG1 v=7 sz=1 d=6, cap=6
//                    GOA=20, Optimal=42
// =============================================================================

public class GOA {

    private final FlowNetwork  fn;
    private final TraceLogger  log;

    public GOA(FlowNetwork fn, TraceLogger log) {
        this.fn  = fn;
        this.log = log;
    }

    // ── Run ───────────────────────────────────────────────────────────────────

    public AlgorithmResult run(List<Integer> dgNodes,
                               List<Integer> storageNodes,
                               int[]         storageCapacity,
                               Object        adjGraph) {

        fn.buildCFN(dgNodes, storageNodes, storageCapacity, adjGraph);

        int[] remaining = initRemaining(dgNodes);
        int[] sinkCap   = fn.makeSinkCap(storageNodes, storageCapacity);

        // sort DGs by priority vᵢ descending
        Integer[] order = sortedIndices(dgNodes, false);

        double      totalWeight  = 0.0;
        List<int[]> flowEdges    = new ArrayList<>();
        Map<Integer, Integer> relayCount = new LinkedHashMap<>();
        Set<Integer> dgSet      = new HashSet<>(dgNodes);
        Set<Integer> storageSet = new HashSet<>(storageNodes);
        int maxIter = fn.cfnNodes * fn.cfnNodes;

        for (int idx : order) {
            int dg     = dgNodes.get(idx);
            int cfnSrc = fn.inNode(dg);
            int szI    = fn.packetSize[dg];
            int vi     = fn.packetPriority[dg];
            int iters  = 0;

            while (remaining[idx] > 0 && iters++ < maxIter) {
                int[] parent = fn.bfsFAP(cfnSrc, szI, sinkCap);
                if (parent == null) break;

                int sinkOut = parent[fn.superSink()];
                int delta   = fn.computeDelta(parent, idx, remaining, sinkCap, szI);
                if (delta <= 0) break;

                List<Integer> bsnPath = fn.extractBSNPath(parent);
                int sinkNode = bsnPath.isEmpty() ? (sinkOut - 1) / 2
                                                 : bsnPath.get(bsnPath.size() - 1);

                for (int k = 1; k < bsnPath.size() - 1; k++)
                    relayCount.merge(bsnPath.get(k), delta, Integer::sum);

                log.logTrace("GOA", dg, sinkNode, delta, vi, szI,
                             bsnPath, dgSet, storageSet, fn.nodeEnergy);

                int gained = fn.augmentPath(parent, idx, remaining, sinkCap,
                                            szI, vi, flowEdges);
                if (gained <= 0) break;

                log.logPush(delta, dg, vi, szI, sinkNode);
                totalWeight += gained;
            }
        }

        log.logf("%nTotal Preserved Priority (GOA): %.1f%n", totalWeight);
        log.logRelayInfo(relayCount, dgSet, storageSet);
        return new AlgorithmResult(totalWeight, flowEdges);
    }

    // ── Silent run (no output — scaling trials) ───────────────────────────────

    public double runSilent(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            int[]         storageCapacity,
                            Object        adjGraph) {
        fn.buildCFN(dgNodes, storageNodes, storageCapacity, adjGraph);
        int[] remaining = initRemaining(dgNodes);
        int[] sinkCap   = fn.makeSinkCap(storageNodes, storageCapacity);
        Integer[] order = sortedIndices(dgNodes, false);

        double totalWeight = 0.0;
        int maxIter = fn.cfnNodes * fn.cfnNodes;

        for (int idx : order) {
            int dg     = dgNodes.get(idx);
            int cfnSrc = fn.inNode(dg);
            int szI    = fn.packetSize[dg];
            int vi     = fn.packetPriority[dg];
            int iters  = 0;

            while (remaining[idx] > 0 && iters++ < maxIter) {
                int[] parent = fn.bfsFAP(cfnSrc, szI, sinkCap);
                if (parent == null) break;
                int gained = fn.augmentPath(parent, idx, remaining, sinkCap,
                                            szI, vi, null);
                if (gained <= 0) break;
                totalWeight += gained;
            }
        }
        return totalWeight;
    }

    // ── Sort helpers (shared with DensityGOA via package-private access) ──────

    Integer[] sortedIndices(List<Integer> dgNodes, boolean useDensity) {
        Integer[] order = new Integer[dgNodes.size()];
        for (int i = 0; i < order.length; i++) order[i] = i;
        if (useDensity) {
            Arrays.sort(order, (a, b) -> {
                double dA = (double) fn.packetPriority[dgNodes.get(a)]
                          / fn.packetSize[dgNodes.get(a)];
                double dB = (double) fn.packetPriority[dgNodes.get(b)]
                          / fn.packetSize[dgNodes.get(b)];
                return Double.compare(dB, dA);
            });
        } else {
            Arrays.sort(order, (a, b) ->
                fn.packetPriority[dgNodes.get(b)] - fn.packetPriority[dgNodes.get(a)]);
        }
        return order;
    }

    int[] initRemaining(List<Integer> dgNodes) {
        int[] r = new int[dgNodes.size()];
        for (int i = 0; i < dgNodes.size(); i++)
            r[i] = fn.packetsPerNode[dgNodes.get(i)];
        return r;
    }
}
