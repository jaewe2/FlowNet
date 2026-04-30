import java.util.*;

// =============================================================================
//  DDRPlusGOA — Contention-Aware Dynamic Density Reordering
//
//  Extension of DDR-GOA. Fixes DDR's over-commitment failure mode:
//  when k DGs can all reach the same storage node j, plain DDR gives
//  every one of them full credit for j's capacity in its reachability
//  estimate, then over-commits to whichever looks best first.
//
//  DDRPlusGOA computes, before each reranking:
//    contention(j) = count of active DGs that currently reach sink j
//    share(i, j)   = sinkCap(j) / sqrt(contention(j))   [dampened]
//    reachable+(i) = sum over reachable j of share(i, j)
//    effectivePriority+(i) = v_i × min(d_i, floor(reachable+(i) / sz_i))
//
//  Uses sqrt(contention) rather than full contention because DGs are
//  served sequentially: after one DG is routed, contention drops for
//  the next reranking. Full division over-penalizes shared storage.
//
//  Everything else (augmentation, BFS routing, stopping condition) matches
//  DDR-GOA exactly — the difference is ONLY in the effective-priority score.
//
//  COMPLEXITY: O(k² × (n + m) × n × m²) — same asymptotic as DDR; the
//  contention map adds O(k × sinks) per recomputation which is dominated.
// =============================================================================

public class DDRPlusGOA {

    private final FlowNetwork fn;
    private final TraceLogger log;
    private final GOA         goa;

    public DDRPlusGOA(FlowNetwork fn, TraceLogger log) {
        this.fn  = fn;
        this.log = log;
        this.goa = new GOA(fn, TraceLogger.SILENT);
    }

    // ── Contention-aware effective priority ──────────────────────────────────
    // Returns the index of the DG with the highest contention-shared
    // effective priority; -1 if none viable.

    private int pickBestDG(List<Integer> dgNodes, int[] remaining, int[] sinkCap) {
        int n = dgNodes.size();

        // Step 1: compute per-sink reachability for every active DG
        List<Map<Integer, Integer>> reachable = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            if (remaining[i] <= 0) {
                reachable.add(Collections.emptyMap());
                continue;
            }
            int dg  = dgNodes.get(i);
            int szI = fn.packetSize[dg];
            reachable.add(fn.computeReachablePerSink(fn.inNode(dg), szI, sinkCap));
        }

        // Step 2: build contention map — for each sink, count how many active DGs reach it
        Map<Integer, Integer> contention = new LinkedHashMap<>();
        for (int i = 0; i < n; i++) {
            if (remaining[i] <= 0) continue;
            for (Integer sink : reachable.get(i).keySet()) {
                contention.merge(sink, 1, Integer::sum);
            }
        }

        // Step 3: compute shared reachable capacity per DG and score
        int bestIdx = -1;
        double bestEffPri = -1.0;
        for (int i = 0; i < n; i++) {
            if (remaining[i] <= 0) continue;
            int dg  = dgNodes.get(i);
            int szI = fn.packetSize[dg];
            int vi  = fn.packetPriority[dg];
            if (szI <= 0) continue;

            double sharedCap = 0.0;
            for (Map.Entry<Integer, Integer> e : reachable.get(i).entrySet()) {
                int sink   = e.getKey();
                int capJ   = e.getValue();
                int kShare = contention.getOrDefault(sink, 1);
                // Dampened: sqrt(contention) instead of full contention.
                // Full division over-penalizes shared storage because DGs
                // are served sequentially, not simultaneously — after one
                // DG is routed the contention drops for the next reranking.
                sharedCap += (double) capJ / Math.sqrt(kShare);
            }

            int canSend  = (int) Math.floor(sharedCap / szI);
            double effPri = vi * Math.min(remaining[i], canSend);
            if (effPri > bestEffPri) { bestEffPri = effPri; bestIdx = i; }
        }

        return (bestEffPri > 0) ? bestIdx : -1;
    }

    // ── Run ───────────────────────────────────────────────────────────────────

    public AlgorithmResult run(List<Integer> dgNodes,
                               List<Integer> storageNodes,
                               int[]         storageCapacity,
                               Object        adjGraph) {

        fn.buildCFN(dgNodes, storageNodes, storageCapacity, adjGraph);

        int[] remaining = goa.initRemaining(dgNodes);
        int[] sinkCap   = fn.makeSinkCap(storageNodes, storageCapacity);

        double      totalWeight  = 0.0;
        List<int[]> flowEdges    = new ArrayList<>();
        Map<Integer, Integer> relayCount = new LinkedHashMap<>();
        Set<Integer> dgSet      = new HashSet<>(dgNodes);
        Set<Integer> storageSet = new HashSet<>(storageNodes);
        int maxIter = fn.cfnNodes * fn.cfnNodes;

        while (true) {
            int bestIdx = pickBestDG(dgNodes, remaining, sinkCap);
            if (bestIdx < 0) break;

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

                log.logTrace("DDR+", dg, sinkNode, delta, vi, szI,
                             bsnPath, dgSet, storageSet, fn.nodeEnergy);

                int gained = fn.augmentPath(parent, bestIdx, remaining, sinkCap,
                                            szI, vi, flowEdges);
                if (gained <= 0) break;
                totalWeight += gained;
            }
        }

        log.logf("%nTotal Preserved Priority (DDR+-GOA): %.1f%n", totalWeight);
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

        double totalWeight = 0.0;
        int maxIter = fn.cfnNodes * fn.cfnNodes;

        while (true) {
            int bestIdx = pickBestDG(dgNodes, remaining, sinkCap);
            if (bestIdx < 0) break;

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
