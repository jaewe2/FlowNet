import java.util.*;

// =============================================================================
//  PSBGOA — PE(κ=2) + Rollout + Energy-Aware Per-Step Best Path
//
//  Combines Hybrid GOA's PE(κ=2) prefix enumeration with PSB's per-step
//  best path scoring for the tail.
//
//  PHASE 1 — PE(κ=2) PREFIX: same as Hybrid GOA.
//
//  PHASE 2 — PSB TAIL (via AugmentationEngine.runPSBTail):
//    At each step, score every active DG's FAP by:
//      score(P) = (v_i × Δ) / (Δ × sz_i + waste + relayPenalty(P) + 1)
//    where waste = fragment left in storage that no remaining packet fits,
//    and relayPenalty penalises paths through low-residual-energy relays.
//    Pick the highest-scoring path and augment it. Repeat.
//
//  WHEN IT WINS:
//    Cases where optimal requires both the right first-two DGs committed
//    upfront AND interleaved augmentation of remaining DGs.
//
//  COMPLEXITY: O(k² × k² × n × m²) — same as Hybrid GOA.
// =============================================================================

public class PSBGOA {

    // Same cap as HybridGOA — both have O(k⁴·n·m²) complexity.
    static final int MAX_HYBRID_DGS = HybridGOA.MAX_HYBRID_DGS;

    private final FlowNetwork       fn;
    private final TraceLogger       log;
    private final AugmentationEngine ae;
    private final GOA               goa;

    public PSBGOA(FlowNetwork fn, TraceLogger log) {
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
            log.logf("%n[PSB-GOA] k=%d > MAX_HYBRID_DGS (%d) — using DensityGOA fallback.%n",
                     dgNodes.size(), MAX_HYBRID_DGS);
            return new DensityGOA(fn, log).run(dgNodes, storageNodes, storageCapacity, adjGraph);
        }
        int[] bestPrefixHolder = new int[1];
        int bestTotal = runCore(dgNodes, storageNodes, storageCapacity,
                                adjGraph, bestPrefixHolder);

        // Phase 2: replay the winning prefix once with relay/flow tracking
        fn.buildCFN(dgNodes, storageNodes, storageCapacity, adjGraph);
        int n = dgNodes.size();
        int[] remaining = goa.initRemaining(dgNodes);
        int[] sinkCap   = fn.makeSinkCap(storageNodes, storageCapacity);
        List<List<Integer>> allPrefixes = FlowNetwork.buildPEPrefixes(n);
        List<Integer> bestPrefix = allPrefixes.get(bestPrefixHolder[0]);

        List<int[]> flowEdges = new ArrayList<>();
        Map<Integer, Integer> relayCount = new LinkedHashMap<>();
        Set<Integer> dgSet      = new HashSet<>(dgNodes);
        Set<Integer> storageSet = new HashSet<>(storageNodes);
        Set<Integer> used = new LinkedHashSet<>();

        for (int idx : bestPrefix) {
            used.add(idx);
            replayWithTracking(idx, dgNodes, remaining, sinkCap,
                               flowEdges, relayCount, dgSet, storageSet);
        }
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

                int pilotVal = ae.augmentBestFit(ci, dgNodes, remaining, sinkCap, storageNodes);
                pilotVal += ae.runPSBTail(dgNodes, remaining, sinkCap, used, ci);
                if (pilotVal > bestPilot) { bestPilot = pilotVal; bestCandidate = ci; }

                fn.restoreCap(capSnap); fn.restoreFlow(flowSnap);
                remaining = remSnap;    sinkCap = sinkSnap;
            }
            if (bestCandidate < 0 || bestPilot <= 0) break;

            log.logf("  Committed DG %d (v=%d, sz=%d, pilotVal=%d)%n",
                     dgNodes.get(bestCandidate),
                     fn.packetPriority[dgNodes.get(bestCandidate)],
                     fn.packetSize[dgNodes.get(bestCandidate)],
                     bestPilot);
            used.add(bestCandidate);
            replayWithTracking(bestCandidate, dgNodes, remaining, sinkCap,
                               flowEdges, relayCount, dgSet, storageSet);
        }

        log.logf("%nTotal Preserved Priority (PSB-GOA): %d%n", bestTotal);
        log.logRelayInfo(relayCount, dgSet, storageSet);
        return new AlgorithmResult((double) bestTotal, flowEdges);
    }

    // Augment one DG fully via best-fit, collecting flow edges and relay counts.
    private void replayWithTracking(int idx, List<Integer> dgNodes,
                                    int[] remaining, int[] sinkCap,
                                    List<int[]> flowEdges,
                                    Map<Integer, Integer> relayCount,
                                    Set<Integer> dgSet, Set<Integer> storageSet) {
        int dg     = dgNodes.get(idx);
        int cfnSrc = fn.inNode(dg);
        int szI    = fn.packetSize[dg];
        int vi     = fn.packetPriority[dg];
        int maxIter = fn.cfnNodes * fn.cfnNodes;
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

            log.logTrace("PSB", dg, sinkNode, delta, vi, szI,
                         bsnPath, dgSet, storageSet, fn.nodeEnergy);

            fn.augmentPath(parent, idx, remaining, sinkCap, szI, vi, flowEdges);
        }
    }

    public double runSilent(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            int[]         storageCapacity,
                            Object        adjGraph) {
        if (dgNodes.size() > MAX_HYBRID_DGS)
            return new DensityGOA(fn, TraceLogger.SILENT)
                       .runSilent(dgNodes, storageNodes, storageCapacity, adjGraph);
        return (double) runCore(dgNodes, storageNodes, storageCapacity, adjGraph, null);
    }

    // ── Core (shared by run and runSilent) ────────────────────────────────────
    // bestPrefixIdxHolder[0] is set to the index into buildPEPrefixes() of the
    // winning prefix, so run() can replay it with full tracking.

    private int runCore(List<Integer> dgNodes,
                        List<Integer> storageNodes,
                        int[]         storageCapacity,
                        Object        adjGraph,
                        int[]         bestPrefixIdxHolder) {
        int n = dgNodes.size();
        int bestTotal = 0;
        int bestPrefixIdx = 0;

        List<List<Integer>> allPrefixes = FlowNetwork.buildPEPrefixes(n);
        for (int pi = 0; pi < allPrefixes.size(); pi++) {
            List<Integer> prefix = allPrefixes.get(pi);

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

            // rollout loop with PSB pilot
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

                    int pilotVal = ae.augmentBestFit(
                        ci, dgNodes, remaining, sinkCap, storageNodes);
                    pilotVal += ae.runPSBTail(dgNodes, remaining, sinkCap, used, ci);

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

            if (trialTotal > bestTotal) {
                bestTotal = trialTotal;
                bestPrefixIdx = pi;
            }
        }
        if (bestPrefixIdxHolder != null) bestPrefixIdxHolder[0] = bestPrefixIdx;
        return bestTotal;
    }
}
