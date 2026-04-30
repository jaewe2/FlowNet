import java.util.*;

// =============================================================================
//  DDRGOA — Dynamic Density Reordering GOA
//
//  Instead of fixing the DG sort order upfront, re-evaluates all remaining
//  DGs after each full augmentation using an "effective priority":
//
//    effectivePriority_i = v_i × min(d_i, floor(reachableResidual_i / sz_i))
//
//  where reachableResidual_i is the total residual storage capacity that
//  DG_i can currently reach in the CFN via a valid FAP.
//
//  WHEN IT WINS: early augmentations fragment storage or deplete shared relay
//    energy in ways that make static ordering suboptimal.
//
//  WHEN IT LOSES: overhead of reachability BFS per step, no benefit when
//    residual capacities remain uniformly distributed.
//
//  COMPLEXITY: O(k² × (n + m) × n × m²) per run.
// =============================================================================

public class DDRGOA {

    private final FlowNetwork fn;
    private final TraceLogger log;
    private final GOA         goa;

    public DDRGOA(FlowNetwork fn, TraceLogger log) {
        this.fn  = fn;
        this.log = log;
        this.goa = new GOA(fn, TraceLogger.SILENT);
    }

    // ── Run ───────────────────────────────────────────────────────────────────

    public AlgorithmResult run(List<Integer> dgNodes,
                               List<Integer> storageNodes,
                               int[]         storageCapacity,
                               Object        adjGraph) {

        fn.buildCFN(dgNodes, storageNodes, storageCapacity, adjGraph);
        int n = dgNodes.size();

        int[] remaining = goa.initRemaining(dgNodes);
        int[] sinkCap   = fn.makeSinkCap(storageNodes, storageCapacity);

        double      totalWeight  = 0.0;
        List<int[]> flowEdges    = new ArrayList<>();
        Map<Integer, Integer> relayCount = new LinkedHashMap<>();
        Set<Integer> dgSet      = new HashSet<>(dgNodes);
        Set<Integer> storageSet = new HashSet<>(storageNodes);
        int maxIter = fn.cfnNodes * fn.cfnNodes;

        while (true) {
            // recompute effective priority for every remaining DG
            int bestIdx = -1, bestEffPri = -1;
            for (int i = 0; i < n; i++) {
                if (remaining[i] <= 0) continue;
                int dg       = dgNodes.get(i);
                int szI      = fn.packetSize[dg];
                int vi       = fn.packetPriority[dg];
                int reachable = fn.computeReachableResidual(
                    fn.inNode(dg), szI, sinkCap);
                int canSend  = (szI > 0) ? reachable / szI : 0;
                int effPri   = vi * Math.min(remaining[i], canSend);
                if (effPri > bestEffPri) { bestEffPri = effPri; bestIdx = i; }
            }
            if (bestIdx < 0 || bestEffPri <= 0) break;

            // fully augment the chosen DG
            int dg     = dgNodes.get(bestIdx);
            int cfnSrc = fn.inNode(dg);
            int szI    = fn.packetSize[dg];
            int vi     = fn.packetPriority[dg];
            int iters  = 0;

            while (remaining[bestIdx] > 0 && iters++ < maxIter) {
                int[] parent = fn.bfsFAP(cfnSrc, szI, sinkCap);
                if (parent == null) break;

                int sinkOut = parent[fn.superSink()];
                int delta   = fn.computeDelta(parent, bestIdx, remaining, sinkCap, szI);
                if (delta <= 0) break;

                List<Integer> bsnPath = fn.extractBSNPath(parent);
                int sinkNode = bsnPath.isEmpty() ? (sinkOut - 1) / 2
                                                 : bsnPath.get(bsnPath.size() - 1);

                for (int k = 1; k < bsnPath.size() - 1; k++)
                    relayCount.merge(bsnPath.get(k), delta, Integer::sum);

                log.logTrace("DDR", dg, sinkNode, delta, vi, szI,
                             bsnPath, dgSet, storageSet, fn.nodeEnergy);

                int gained = fn.augmentPath(parent, bestIdx, remaining, sinkCap,
                                            szI, vi, flowEdges);
                if (gained <= 0) break;
                totalWeight += gained;
            }
        }

        log.logf("%nTotal Preserved Priority (DDR-GOA): %.1f%n", totalWeight);
        log.logRelayInfo(relayCount, dgSet, storageSet);
        return new AlgorithmResult(totalWeight, flowEdges);
    }

    // ── Silent run ────────────────────────────────────────────────────────────

    public double runSilent(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            int[]         storageCapacity,
                            Object        adjGraph) {

        fn.buildCFN(dgNodes, storageNodes, storageCapacity, adjGraph);
        int n = dgNodes.size();

        int[] remaining = goa.initRemaining(dgNodes);
        int[] sinkCap   = fn.makeSinkCap(storageNodes, storageCapacity);

        double totalWeight = 0.0;
        int maxIter = fn.cfnNodes * fn.cfnNodes;

        while (true) {
            int bestIdx = -1, bestEffPri = -1;
            for (int i = 0; i < n; i++) {
                if (remaining[i] <= 0) continue;
                int dg       = dgNodes.get(i);
                int szI      = fn.packetSize[dg];
                int vi       = fn.packetPriority[dg];
                int reachable = fn.computeReachableResidual(
                    fn.inNode(dg), szI, sinkCap);
                int canSend  = (szI > 0) ? reachable / szI : 0;
                int effPri   = vi * Math.min(remaining[i], canSend);
                if (effPri > bestEffPri) { bestEffPri = effPri; bestIdx = i; }
            }
            if (bestIdx < 0 || bestEffPri <= 0) break;

            int dg     = dgNodes.get(bestIdx);
            int cfnSrc = fn.inNode(dg);
            int szI    = fn.packetSize[dg];
            int vi     = fn.packetPriority[dg];
            int iters  = 0;

            while (remaining[bestIdx] > 0 && iters++ < maxIter) {
                int[] parent = fn.bfsFAP(cfnSrc, szI, sinkCap);
                if (parent == null) break;
                int gained = fn.augmentPath(parent, bestIdx, remaining, sinkCap,
                                            szI, vi, null);
                if (gained <= 0) break;
                totalWeight += gained;
            }
        }
        return totalWeight;
    }
}
