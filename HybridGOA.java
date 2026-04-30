import java.util.*;

// =============================================================================
//  HybridGOA — PE(κ=2) + Rollout + Best-Fit Storage Selection
//
//  Combines three strategies to handle variable packet sizes better than
//  any single ordering heuristic:
//
//  1. PARTIAL ENUMERATION (PE, κ=2):
//     Tries every ordered pair of DGs as the first two to augment,
//     plus every single DG prefix and a pure-rollout run (κ=0).
//     Catches cases where GOA/DensityGOA lock in a bad prefix.
//
//  2. ROLLOUT (lookahead):
//     After the prefix, instead of committing to a fixed sort order,
//     each candidate is simulated as "next": fully augment it, then
//     run density-greedy on the rest, pick the highest pilot total.
//
//  3. BEST-FIT STORAGE SELECTION:
//     During augmentation, prefer the storage node whose residual
//     after placing szᵢ units is the smallest non-negative value,
//     minimising wasted storage fragments.
//
//  COMPLEXITY: O(k⁴ × n × m²) — practical for k ≤ 15.
// =============================================================================

public class HybridGOA {

    // PE(κ=2) generates k²+1 prefixes — cost is O(k⁴·n·m²).
    // Beyond this cap, fall back to DensityGOA to avoid multi-minute hangs.
    static final int MAX_HYBRID_DGS = 12;

    private final FlowNetwork       fn;
    private final TraceLogger       log;
    private final AugmentationEngine ae;
    private final GOA               goa;

    public HybridGOA(FlowNetwork fn, TraceLogger log) {
        this.fn  = fn;
        this.log = log;
        this.ae  = new AugmentationEngine(fn);
        this.goa = new GOA(fn, TraceLogger.SILENT);
    }

    // ── Run ───────────────────────────────────────────────────────────────────

    public AlgorithmResult run(List<Integer> dgNodes,
                               List<Integer> storageNodes,
                               int[]         storageCapacity,
                               Object        adjGraph) {

        if (dgNodes.size() > MAX_HYBRID_DGS) {
            log.logf("%n[Hybrid GOA] k=%d > MAX_HYBRID_DGS (%d) — using DensityGOA fallback.%n",
                     dgNodes.size(), MAX_HYBRID_DGS);
            return new DensityGOA(fn, log).run(dgNodes, storageNodes, storageCapacity, adjGraph);
        }
        int bestTotal = runCore(dgNodes, storageNodes, storageCapacity, adjGraph);
        log.logf("%nTotal Preserved Priority (Hybrid GOA): %d%n", bestTotal);
        return new AlgorithmResult((double) bestTotal);
    }

    public double runSilent(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            int[]         storageCapacity,
                            Object        adjGraph) {
        if (dgNodes.size() > MAX_HYBRID_DGS)
            return new DensityGOA(fn, TraceLogger.SILENT)
                       .runSilent(dgNodes, storageNodes, storageCapacity, adjGraph);
        return (double) runCore(dgNodes, storageNodes, storageCapacity, adjGraph);
    }

    // ── Core (shared by run and runSilent) ────────────────────────────────────

    private int runCore(List<Integer> dgNodes,
                        List<Integer> storageNodes,
                        int[]         storageCapacity,
                        Object        adjGraph) {
        int n = dgNodes.size();
        int bestTotal = 0;

        for (List<Integer> prefix : FlowNetwork.buildPEPrefixes(n)) {

            fn.buildCFN(dgNodes, storageNodes, storageCapacity, adjGraph);

            int[] remaining = goa.initRemaining(dgNodes);
            int[] sinkCap   = fn.makeSinkCap(storageNodes, storageCapacity);
            int   trialTotal = 0;
            Set<Integer> used = new LinkedHashSet<>();

            // augment prefix DGs with best-fit storage
            for (int idx : prefix) {
                used.add(idx);
                trialTotal += ae.augmentBestFit(
                    idx, dgNodes, remaining, sinkCap, storageNodes);
            }

            // rollout loop: simulate each remaining candidate as "next"
            while (true) {
                List<Integer> candidates = new ArrayList<>();
                for (int i = 0; i < n; i++) {
                    if (used.contains(i) || remaining[i] <= 0) continue;
                    if (fn.bfsFAP(fn.inNode(dgNodes.get(i)),
                                  fn.packetSize[dgNodes.get(i)], sinkCap) != null)
                        candidates.add(i);
                }
                if (candidates.isEmpty()) break;

                int bestCandidate = -1, bestPilot = -1;

                for (int ci : candidates) {
                    int[][] capSnap  = fn.snapCap();
                    int[][] flowSnap = fn.snapFlow();
                    int[]   remSnap  = remaining.clone();
                    int[]   sinkSnap = sinkCap.clone();

                    // simulate: augment candidate then density-greedy tail
                    int pilotVal = ae.augmentBestFit(
                        ci, dgNodes, remaining, sinkCap, storageNodes);

                    List<Integer> tail = new ArrayList<>();
                    for (int i = 0; i < n; i++) {
                        if (i == ci || used.contains(i)) continue;
                        if (remaining[i] > 0) tail.add(i);
                    }
                    tail.sort((a, b) -> {
                        double dA = (double) fn.packetPriority[dgNodes.get(a)]
                                  / fn.packetSize[dgNodes.get(a)];
                        double dB = (double) fn.packetPriority[dgNodes.get(b)]
                                  / fn.packetSize[dgNodes.get(b)];
                        return Double.compare(dB, dA);
                    });
                    for (int ti : tail)
                        pilotVal += ae.augmentBestFit(
                            ti, dgNodes, remaining, sinkCap, storageNodes);

                    if (pilotVal > bestPilot) {
                        bestPilot = pilotVal; bestCandidate = ci;
                    }

                    fn.restoreCap(capSnap); fn.restoreFlow(flowSnap);
                    remaining = remSnap;    sinkCap = sinkSnap;
                }

                if (bestCandidate < 0 || bestPilot <= 0) break;
                used.add(bestCandidate);
                trialTotal += ae.augmentBestFit(
                    bestCandidate, dgNodes, remaining, sinkCap, storageNodes);
            }

            if (trialTotal > bestTotal) bestTotal = trialTotal;
        }
        return bestTotal;
    }
}
