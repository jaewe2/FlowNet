import java.util.*;

// =============================================================================
//  DensityGOA — Sort by priority density ρ = vᵢ/szᵢ
//
//  Treats storage as the binding constraint and maximises how much priority
//  fits per storage unit consumed — analogous to fractional knapsack.
//
//  WHEN IT WINS:  large high-priority packets waste storage; small low-priority
//                 packets have much better density.
//  WHEN IT LOSES: storage fragmentation when leftover gaps can't fit remaining
//                 packets (bin-packing difficulty).
//    Counterexample: DG0 v=9 sz=3 vs DG1 v=8 sz=2, cap=5
//                    Density=16, Optimal=17
//
// =============================================================================
//  ApproxGOA — 2-approximation: max(GOA, DensityGOA)
//
//  Runs both sub-routines and returns the larger result.
//  Guaranteed to be within 2× of optimal under the all-or-nothing model,
//  analogous to Algo. 5 in Rivera & Tang (2024).
// =============================================================================

public class DensityGOA {

    private final FlowNetwork fn;
    private final TraceLogger log;
    private final GOA         goa;  // reuses GOA's sort/init helpers

    public DensityGOA(FlowNetwork fn, TraceLogger log) {
        this.fn  = fn;
        this.log = log;
        this.goa = new GOA(fn, log);
    }

    // ── DensityGOA run ────────────────────────────────────────────────────────

    public AlgorithmResult run(List<Integer> dgNodes,
                               List<Integer> storageNodes,
                               int[]         storageCapacity,
                               Object        adjGraph) {

        fn.buildCFN(dgNodes, storageNodes, storageCapacity, adjGraph);

        int[] remaining = goa.initRemaining(dgNodes);
        int[] sinkCap   = fn.makeSinkCap(storageNodes, storageCapacity);

        // sort by density ρᵢ = vᵢ/szᵢ descending
        Integer[] order = goa.sortedIndices(dgNodes, true);

        log.log("\n-- Density Order (rho = v/sz) --");
        for (int idx : order) {
            int dg = dgNodes.get(idx);
            log.logf("  DG %d: v=%d, sz=%d, rho=%.2f%n",
                     dg, fn.packetPriority[dg], fn.packetSize[dg],
                     (double) fn.packetPriority[dg] / fn.packetSize[dg]);
        }

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

                log.logTrace("DENS", dg, sinkNode, delta, vi, szI,
                             bsnPath, dgSet, storageSet, fn.nodeEnergy);

                int gained = fn.augmentPath(parent, idx, remaining, sinkCap,
                                            szI, vi, flowEdges);
                if (gained <= 0) break;
                totalWeight += gained;
            }
        }

        log.logf("%nTotal Preserved Priority (Density GOA): %.1f%n", totalWeight);
        log.logRelayInfo(relayCount, dgSet, storageSet);
        return new AlgorithmResult(totalWeight, flowEdges);
    }

    // ── Silent run ────────────────────────────────────────────────────────────

    public double runSilent(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            int[]         storageCapacity,
                            Object        adjGraph) {
        fn.buildCFN(dgNodes, storageNodes, storageCapacity, adjGraph);
        int[] remaining = goa.initRemaining(dgNodes);
        int[] sinkCap   = fn.makeSinkCap(storageNodes, storageCapacity);
        Integer[] order = goa.sortedIndices(dgNodes, true);

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

    // ─────────────────────────────────────────────────────────────────────────
    //  ApproxGOA — nested static class (runs both sub-routines, returns max)
    // ─────────────────────────────────────────────────────────────────────────

    public static class ApproxGOA {

        private final FlowNetwork fn;
        private final TraceLogger log;

        public ApproxGOA(FlowNetwork fn, TraceLogger log) {
            this.fn  = fn;
            this.log = log;
        }

        public double run(List<Integer> dgNodes,
                          List<Integer> storageNodes,
                          int[]         storageCapacity,
                          Object        adjGraph) {

            log.log("\n-- Approx Sub-routine A: sort by v --");
            double vfA = new GOA(fn, log).runSilent(
                dgNodes, storageNodes, storageCapacity, adjGraph);

            log.log("\n-- Approx Sub-routine B: sort by v/sz --");
            double vfB = new DensityGOA(fn, log).runSilent(
                dgNodes, storageNodes, storageCapacity, adjGraph);

            double best = Math.max(vfA, vfB);
            log.logf("%nApprox (A) by v:         %.1f%n", vfA);
            log.logf("Approx (B) by v/sz:      %.1f%n", vfB);
            log.logf("Approx result (max A,B):  %.1f%n", best);
            return best;
        }

        public double runSilent(List<Integer> dgNodes,
                                List<Integer> storageNodes,
                                int[]         storageCapacity,
                                Object        adjGraph) {
            double vfA = new GOA(fn, TraceLogger.SILENT).runSilent(
                dgNodes, storageNodes, storageCapacity, adjGraph);
            double vfB = new DensityGOA(fn, TraceLogger.SILENT).runSilent(
                dgNodes, storageNodes, storageCapacity, adjGraph);
            return Math.max(vfA, vfB);
        }
    }
}
