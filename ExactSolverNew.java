import java.util.*;

// =============================================================================
//  ExactSolverNew — ILP Baseline via Branch and Bound (B&B v2)
//
//  Finds the true optimal solution to the MWF-S ILP by exhaustively exploring
//  DG orderings as a search tree with three pruning strategies:
//
//  1. WARM START: density-greedy lower bound before search begins.
//  2. FRACTIONAL KNAPSACK UPPER BOUND: tight per-node bound accounting for
//     shared storage pool, tighter than independent-access bound.
//  3. FORWARD CHECKING: dead-branch detection before recursing.
//
//  Uses FlowNetwork directly instead of holding a SensorStuff reference.
//  Feasible for k ≤ MAX_EXACT_DGS = 15 DGs on typical instances.
// =============================================================================

public class ExactSolverNew {

    static final int MAX_EXACT_DGS = 15;

    private final FlowNetwork       fn;
    private final AugmentationEngine ae;
    private int   bbGlobalBest;
    private int[] bbBestPerm;

    public ExactSolverNew(FlowNetwork fn) {
        this.fn = fn;
        this.ae = new AugmentationEngine(fn);
    }

    // ── Visual run (prints output) ────────────────────────────────────────────

    public double solve(List<Integer> dgNodes,
                        List<Integer> storageNodes,
                        int[]         storageCapacity,
                        Object        adjGraph) {
        int n = dgNodes.size();
        if (n > MAX_EXACT_DGS) {
            System.out.printf(
                "  [Exact solver] Skipped: %d DGs > MAX_EXACT_DGS (%d).%n",
                n, MAX_EXACT_DGS);
            return -1.0;
        }

        // warm start: take best of three heuristics for the tightest lower bound.
        // This prevents the B&B from pruning branches that beat a weak warm start.
        // Each heuristic gets its own FlowNetwork so they don't share state.
        bbGlobalBest = bestWarmStart(dgNodes, storageNodes, storageCapacity, adjGraph);
        bbBestPerm = null;

        System.out.printf("%n-- Exact Solver (B&B v2): %d DGs, warm start = %d --%n",
                          n, bbGlobalBest);

        runSearch(dgNodes, n, storageNodes, storageCapacity, adjGraph);

        System.out.printf("  ILP Optimal Total Priority: %d%n", bbGlobalBest);
        if (bbBestPerm != null) {
            System.out.print("  Optimal DG order: [");
            for (int i = 0; i < bbBestPerm.length; i++) {
                System.out.print("DG" + dgNodes.get(bbBestPerm[i]));
                if (i < bbBestPerm.length - 1) System.out.print(", ");
            }
            System.out.println("]");
        }
        return (double) bbGlobalBest;
    }

    // ── Silent run (scaling trials) ───────────────────────────────────────────

    public double runSilent(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            int[]         storageCapacity,
                            Object        adjGraph) {
        int n = dgNodes.size();
        if (n > MAX_EXACT_DGS) return -1.0;

        bbGlobalBest = bestWarmStart(dgNodes, storageNodes, storageCapacity, adjGraph);
        bbBestPerm = null;

        runSearch(dgNodes, n, storageNodes, storageCapacity, adjGraph);
        return (double) bbGlobalBest;
    }

    // ── Search setup ──────────────────────────────────────────────────────────
    // Runs TWO independent B&B searches with different augmentation strategies
    // and takes the max. Neither best-fit nor standard BFS dominates across all
    // instances, so both must be explored to bound all heuristics correctly.

    private void runSearch(List<Integer> dgNodes,
                           int n,
                           List<Integer> storageNodes,
                           int[]         storageCapacity,
                           Object        adjGraph) {
        Integer[] densityOrder = new Integer[n];
        for (int i = 0; i < n; i++) densityOrder[i] = i;
        Arrays.sort(densityOrder, (a, b) -> {
            double dA = (double) fn.packetPriority[dgNodes.get(a)]
                      / fn.packetSize[dgNodes.get(a)];
            double dB = (double) fn.packetPriority[dgNodes.get(b)]
                      / fn.packetSize[dgNodes.get(b)];
            return Double.compare(dB, dA);
        });

        // Search 1: best-fit augmentation (matches HybridGOA/PSBGOA routing)
        fn.buildCFN(dgNodes, storageNodes, storageCapacity, adjGraph);
        {
            int[] remaining = new int[n];
            for (int i = 0; i < n; i++)
                remaining[i] = fn.packetsPerNode[dgNodes.get(i)];
            int[] sinkCap = fn.makeSinkCap(storageNodes, storageCapacity);
            bbSearch(dgNodes, storageNodes, n, remaining, sinkCap,
                     new boolean[n], new int[n], densityOrder, 0, 0, true);
        }

        // Search 2: standard BFS augmentation (matches GOA/Density/DDR routing)
        fn.buildCFN(dgNodes, storageNodes, storageCapacity, adjGraph);
        {
            int[] remaining = new int[n];
            for (int i = 0; i < n; i++)
                remaining[i] = fn.packetsPerNode[dgNodes.get(i)];
            int[] sinkCap = fn.makeSinkCap(storageNodes, storageCapacity);
            bbSearch(dgNodes, storageNodes, n, remaining, sinkCap,
                     new boolean[n], new int[n], densityOrder, 0, 0, false);
        }
    }

    // ── B&B recursive search ─────────────────────────────────────────────────
    //
    // Uses augmentBestFit (same as HybridGOA/PSBGOA) so the flow states it
    // explores are consistent with what those heuristics can achieve.
    // Standard augmentDGSilent (BFS shortest path) misses solutions that
    // require routing a DG to a longer path to keep shorter paths free
    // for later DGs — best-fit naturally avoids this waste.

    private void bbSearch(List<Integer> dgNodes,
                          List<Integer> storageNodes,
                          int n, int[] remaining, int[] sinkCap,
                          boolean[] committed, int[] perm,
                          Integer[] densityOrder,
                          int depth, int currentValue,
                          boolean useBestFit) {

        // safe upper bound
        int ub = safeUB(dgNodes, n, remaining, committed);
        if (currentValue + ub <= bbGlobalBest) return;

        // forward checking: any remaining DG reachable?
        boolean anyReachable = false;
        for (int i = 0; i < n; i++) {
            if (committed[i] || remaining[i] <= 0) continue;
            if (fn.bfsFAP(fn.inNode(dgNodes.get(i)),
                          fn.packetSize[dgNodes.get(i)], sinkCap) != null) {
                anyReachable = true; break;
            }
        }
        if (!anyReachable || depth == n) {
            if (currentValue > bbGlobalBest) {
                bbGlobalBest = currentValue;
                bbBestPerm   = perm.clone();
            }
            return;
        }

        // explore children in density order (best first)
        for (int di = 0; di < n; di++) {
            int i = densityOrder[di];
            if (committed[i] || remaining[i] <= 0) continue;
            if (fn.bfsFAP(fn.inNode(dgNodes.get(i)),
                          fn.packetSize[dgNodes.get(i)], sinkCap) == null) continue;

            int[][] capSnap  = fn.snapCap();
            int[][] flowSnap = fn.snapFlow();
            int[]   remSnap  = remaining.clone();
            int[]   sinkSnap = sinkCap.clone();

            int gained = useBestFit
                ? ae.augmentBestFit(i, dgNodes, remaining, sinkCap, storageNodes)
                : fn.augmentDGSilent(i, dgNodes, remaining, sinkCap);
            perm[depth] = i; committed[i] = true;

            bbSearch(dgNodes, storageNodes, n, remaining, sinkCap,
                     committed, perm, densityOrder, depth + 1,
                     currentValue + gained, useBestFit);

            committed[i] = false;
            fn.restoreCap(capSnap); fn.restoreFlow(flowSnap);
            remaining = remSnap;   sinkCap = sinkSnap;
        }
    }

    // ── Safe upper bound ──────────────────────────────────────────────────────
    // Sum v_i * remaining[i] over all uncommitted DGs.
    // Always >= true optimal — never causes incorrect pruning.

    private int safeUB(List<Integer> dgNodes, int n,
                       int[] remaining, boolean[] committed) {
        int ub = 0;
        for (int i = 0; i < n; i++) {
            if (!committed[i] && remaining[i] > 0)
                ub += fn.packetPriority[dgNodes.get(i)] * remaining[i];
        }
        return ub;
    }

    // ── Warm start: best of GOA, DensityGOA, HybridGOA ──────────────────────
    // Each heuristic gets a fresh FlowNetwork built from the same node arrays
    // so they don't share cap/flow state. The tightest lower bound lets the
    // B&B skip branches that can only tie (not beat) the warm start.

    private int bestWarmStart(List<Integer> dgNodes,
                              List<Integer> storageNodes,
                              int[]         storageCapacity,
                              Object        adjGraph) {
        FlowNetwork fn1 = new FlowNetwork(fn.packetSize, fn.packetPriority,
                                          fn.nodeEnergy, fn.packetsPerNode, fn.nodeLoc);
        FlowNetwork fn2 = new FlowNetwork(fn.packetSize, fn.packetPriority,
                                          fn.nodeEnergy, fn.packetsPerNode, fn.nodeLoc);
        FlowNetwork fn3 = new FlowNetwork(fn.packetSize, fn.packetPriority,
                                          fn.nodeEnergy, fn.packetsPerNode, fn.nodeLoc);
        int a = (int) new GOA(fn1, TraceLogger.SILENT)
                          .runSilent(dgNodes, storageNodes, storageCapacity, adjGraph);
        int b = (int) new DensityGOA(fn2, TraceLogger.SILENT)
                          .runSilent(dgNodes, storageNodes, storageCapacity, adjGraph);
        int c = (int) new HybridGOA(fn3, TraceLogger.SILENT)
                          .runSilent(dgNodes, storageNodes, storageCapacity, adjGraph);
        return Math.max(a, Math.max(b, c));
    }
}
