import java.util.*;

// =========================================================================
//  ExactSolver — ILP Baseline via Branch and Bound (B&B v2)
//
//  WHAT IT DOES:
//    Finds the true optimal solution to the MWF-S ILP formulation:
//
//      max Σᵢ Σⱼ (yᵢ,ⱼ × vᵢ)
//
//    subject to:
//      i)   edge capacity constraint:  Σᵢ xᵢ,(u,v) ≤ ca(u,v)  ∀(u,v)∈E
//      ii)  source supply constraint:  Σ xᵢ,(sᵢ,v) ≤ dᵢ        ∀i
//      iii) flow conservation at all intermediate nodes
//      iv)  size-aware sink constraint: sinkCap[j] ≥ szᵢ before routing
//
//    All constraints are enforced by the BFN (buildCFN + bfsFAP).
//    The solver finds the optimal by exhaustively exploring DG orderings
//    as a search tree with three pruning strategies.
//
//  WHY BRANCH AND BOUND (not subset DP):
//    The BFN state after augmenting a subset of DGs depends on the ORDER
//    in which they were augmented — variable packet sizes cause different
//    storage residuals depending on which DG filled a storage node first.
//    Subset DP cannot share states across orderings and is therefore not
//    exact. Branch and bound explores orderings as a tree, sharing prefix
//    computation across branches with the same prefix.
//
//  THREE PRUNING STRATEGIES (B&B v2):
//
//  1. WARM START — density greedy lower bound:
//     Before search begins, run density greedy (sort by vᵢ/szᵢ) to get
//     a strong initial lower bound. Any branch that cannot beat this is
//     pruned immediately at the root level.
//
//  2. FRACTIONAL KNAPSACK UPPER BOUND (tighter than v1 independent bound):
//     At each node, solve the fractional relaxation of storage packing:
//       - total residual storage R = sum of reachable sinkCap[j] where
//         sinkCap[j] ≥ minSzRemaining (only count usable storage)
//       - sort remaining DGs by vᵢ/szᵢ descending
//       - greedily fill R, allowing a fractional last packet
//     This is tighter than the independent-access bound because it
//     correctly accounts for DGs competing for the same storage pool.
//     If currentValue + UB ≤ globalBest: prune entire subtree.
//
//  3. FORWARD CHECKING — dead branch detection:
//     Before recursing, check if any remaining DG can reach any storage
//     node in the current residual CFN. If none can, the subtree yields
//     exactly currentValue — no children need to be explored.
//
//  CHILD ORDERING — best-first within depth-first search:
//     Children are explored in descending order of vᵢ/szᵢ. This means
//     the most promising DG is committed first, raising bbGlobalBest
//     faster and making pruning more aggressive for all sibling subtrees.
//
//  COMPLEXITY:
//    Worst case O(k! × poly(n)) — same as brute force.
//    In practice O(kᶜ × poly(n)) for small constant c, because the
//    fractional knapsack bound is tight enough to prune most branches.
//    Feasible for k ≤ MAX_EXACT_DGS = 15 DGs on typical instances.
//
//  RELATIONSHIP TO PAPER:
//    The ILP objective matches Rivera & Tang (2024) Section VI adapted
//    for variable packet sizes (MWF-S). The BFN enforces all paper
//    constraints (Definition 1, 2, 3 and Section VI steps 1–4).
//    The exact solver is NOT in the paper — it is added as a project
//    contribution to measure empirical approximation ratios.
// =========================================================================

public class ExactSolver {

    // maximum DG count for which exact solver is feasible
    // 15! is large but pruning typically reduces explored tree to k^3–k^4
    static final int MAX_EXACT_DGS = 15;

    // reference to parent SensorStuff — gives access to BFN state
    private final SensorStuff net;

    // branch and bound shared state
    private int   bbGlobalBest;
    private int[] bbBestPerm;

    public ExactSolver(SensorStuff net) {
        this.net = net;
    }

    // =========================================================================
    //  PUBLIC ENTRY POINTS
    // =========================================================================

    // Visual run: prints output, returns optimal total priority
    public double solve(List<Integer> dgNodes,
                        List<Integer> storageNodes,
                        int[]         storageCapacity) {

        int n = dgNodes.size();
        if (n > MAX_EXACT_DGS) {
            System.out.printf(
                "  [Exact solver] Skipped: %d DGs > MAX_EXACT_DGS (%d).%n",
                n, MAX_EXACT_DGS);
            return -1.0;
        }

        // warm start: density greedy gives a strong initial lower bound
        // this immediately prunes any branch that cannot beat it
        bbGlobalBest = (int) net.runSilent(dgNodes, storageNodes,
                                           storageCapacity, true);
        bbBestPerm   = null;

        System.out.printf("%n-- Exact Solver (B&B v2): %d DGs, " +
                          "warm start = %d --%n", n, bbGlobalBest);

        // build initial CFN and run search
        net.buildCFN(dgNodes, storageNodes, storageCapacity);
        runSearch(dgNodes, storageNodes, n, storageCapacity);

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

    // Scaling trials: no output, returns optimal total priority or -1
    public double runSilent(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            int[]         storageCapacity) {

        int n = dgNodes.size();
        if (n > MAX_EXACT_DGS) return -1.0;

        bbGlobalBest = (int) net.runSilent(dgNodes, storageNodes,
                                           storageCapacity, true);
        bbBestPerm   = null;

        net.buildCFN(dgNodes, storageNodes, storageCapacity);
        runSearch(dgNodes, storageNodes, n, storageCapacity);
        return (double) bbGlobalBest;
    }

    // =========================================================================
    //  SEARCH SETUP
    // =========================================================================

    private void runSearch(List<Integer> dgNodes,
                           List<Integer> storageNodes,
                           int n, int[] storageCapacity) {

        int[] remaining = new int[n];
        for (int i = 0; i < n; i++)
            remaining[i] = net.packetsPerNode[dgNodes.get(i)];

        int[] sinkCap = new int[net.cfnNodes];
        for (int j = 0; j < storageNodes.size(); j++)
            sinkCap[net.outNode(storageNodes.get(j))] = storageCapacity[j];

        // child ordering: explore by density vᵢ/szᵢ descending (best first)
        Integer[] densityOrder = new Integer[n];
        for (int i = 0; i < n; i++) densityOrder[i] = i;
        Arrays.sort(densityOrder, (a, b) -> {
            double dA = (double) net.packetPriority[dgNodes.get(a)]
                      / net.packetSize[dgNodes.get(a)];
            double dB = (double) net.packetPriority[dgNodes.get(b)]
                      / net.packetSize[dgNodes.get(b)];
            return Double.compare(dB, dA);
        });

        boolean[] committed = new boolean[n];
        int[]     perm      = new int[n];

        bbSearch(dgNodes, n, remaining, sinkCap,
                 committed, perm, densityOrder, 0, 0);
    }

    // =========================================================================
    //  BRANCH AND BOUND RECURSIVE SEARCH
    // =========================================================================

    private void bbSearch(List<Integer> dgNodes,
                          int n, int[] remaining, int[] sinkCap,
                          boolean[] committed, int[] perm,
                          Integer[] densityOrder,
                          int depth, int currentValue) {

        // leaf node: all DGs committed
        if (depth == n) {
            if (currentValue > bbGlobalBest) {
                bbGlobalBest = currentValue;
                bbBestPerm   = perm.clone();
            }
            return;
        }

        // pruning 2: fractional knapsack upper bound
        int ub = fractionalKnapsackUB(dgNodes, n, remaining, sinkCap, committed);
        if (currentValue + ub <= bbGlobalBest) return;

        // pruning 3: forward checking — any DG still reachable?
        boolean anyReachable = false;
        for (int i = 0; i < n && !anyReachable; i++) {
            if (committed[i] || remaining[i] <= 0) continue;
            int[] probe = net.bfsFAP(net.inNode(dgNodes.get(i)),
                                     net.packetSize[dgNodes.get(i)], sinkCap);
            if (probe != null) anyReachable = true;
        }
        if (!anyReachable) {
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

            // quick reachability check before snapshotting
            int[] probe = net.bfsFAP(net.inNode(dgNodes.get(i)),
                                     net.packetSize[dgNodes.get(i)], sinkCap);
            if (probe == null) continue;

            // snapshot BFN state
            int[][] capSnap  = deepCopy2D(net.cap);
            int[][] flowSnap = deepCopy2D(net.flow);
            int[]   remSnap  = remaining.clone();
            int[]   sinkSnap = sinkCap.clone();

            int gained   = augmentSilent(i, dgNodes, remaining, sinkCap);
            perm[depth]  = i;
            committed[i] = true;

            bbSearch(dgNodes, n, remaining, sinkCap,
                     committed, perm, densityOrder,
                     depth + 1, currentValue + gained);

            // restore BFN state
            committed[i] = false;
            net.cap       = capSnap;
            net.flow      = flowSnap;
            remaining     = remSnap;
            sinkCap       = sinkSnap;
        }
    }

    // =========================================================================
    //  FRACTIONAL KNAPSACK UPPER BOUND
    //
    //  Computes a tight upper bound on remaining priority by solving the
    //  fractional relaxation of the storage packing problem:
    //    - R = total residual storage across storage nodes where
    //      sinkCap[v] ≥ minSzRemaining (only count usable storage — Issue 3 fix)
    //    - sort remaining DGs by vᵢ/szᵢ descending
    //    - greedily fill R, allowing a fractional last packet
    //  Tighter than independent-access bound: correctly accounts for DGs
    //  competing for the same storage pool. Runs in O(k log k) per call.
    // =========================================================================

    private int fractionalKnapsackUB(List<Integer> dgNodes, int n,
                                     int[] remaining, int[] sinkCap,
                                     boolean[] committed) {
        // minimum packet size among remaining DGs
        // only count storage nodes that can fit at least one remaining packet
        int minSzRemaining = Integer.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            if (!committed[i] && remaining[i] > 0)
                minSzRemaining = Math.min(minSzRemaining,
                                          net.packetSize[dgNodes.get(i)]);
        }
        if (minSzRemaining == Integer.MAX_VALUE) return 0;

        // total usable residual storage
        int totalResidual = 0;
        for (int v = 0; v < net.cfnNodes; v++) {
            if (sinkCap[v] > 0 && net.cap[v][net.superSink()] > 0
                    && sinkCap[v] >= minSzRemaining)
                totalResidual += sinkCap[v];
        }
        if (totalResidual <= 0) return 0;

        // build density-sorted list of remaining DGs
        int active = 0;
        for (int i = 0; i < n; i++)
            if (!committed[i] && remaining[i] > 0) active++;

        int[]    vi   = new int[active];
        int[]    szi  = new int[active];
        int[]    rem  = new int[active];
        double[] dens = new double[active];
        int k = 0;
        for (int i = 0; i < n; i++) {
            if (committed[i] || remaining[i] <= 0) continue;
            int dg  = dgNodes.get(i);
            vi[k]   = net.packetPriority[dg];
            szi[k]  = net.packetSize[dg];
            rem[k]  = remaining[i];
            dens[k] = (szi[k] > 0) ? (double) vi[k] / szi[k] : 0;
            k++;
        }

        Integer[] idx = new Integer[active];
        for (int i = 0; i < active; i++) idx[i] = i;
        Arrays.sort(idx, (a, b) -> Double.compare(dens[b], dens[a]));

        // fractional knapsack fill
        double ub = 0.0;
        int R = totalResidual;
        for (int ii = 0; ii < active && R > 0; ii++) {
            int j     = idx[ii];
            int szJ   = szi[j];
            if (szJ <= 0) continue;
            int canFit = Math.min(rem[j], R / szJ);
            if (canFit > 0) {
                ub += (double) vi[j] * canFit;
                R  -= canFit * szJ;
            }
            // fractional last packet
            if (R > 0 && rem[j] > canFit) {
                ub += (double) vi[j] * R / szJ;
                R   = 0;
            }
        }
        return (int) Math.ceil(ub);
    }

    // =========================================================================
    //  AUGMENTATION HELPER
    //
    //  Fully augments one DG (by index into dgNodes) against the current
    //  BFN state. Modifies net.cap, net.flow, remaining, sinkCap in place.
    //  Caller is responsible for snapshot/restore via deepCopy2D.
    //  Returns total priority gained from this DG's augmentation.
    // =========================================================================

    private int augmentSilent(int idx, List<Integer> dgNodes,
                               int[] remaining, int[] sinkCap) {
        int dg     = dgNodes.get(idx);
        int cfnSrc = net.inNode(dg);
        int szI    = net.packetSize[dg];
        int vi     = net.packetPriority[dg];
        int S      = 0, T = net.superSink();
        int total  = 0;
        int maxIter = net.cfnNodes * net.cfnNodes;
        int iters  = 0;

        while (remaining[idx] > 0 && iters++ < maxIter) {
            int[] parent = net.bfsFAP(cfnSrc, szI, sinkCap);
            if (parent == null) break;

            int sinkOut        = parent[T];
            int pathBottleneck = Integer.MAX_VALUE;
            int cur = T, steps = 0;
            while (cur != S && steps++ < net.cfnNodes) {
                int prev = parent[cur];
                if (prev == -1 || prev == cur) break;
                pathBottleneck = Math.min(pathBottleneck,
                                         net.cap[prev][cur] - net.flow[prev][cur]);
                cur = prev;
            }
            int delta = Math.min(pathBottleneck,
                        Math.min(remaining[idx], sinkCap[sinkOut] / szI));
            if (delta <= 0) break;

            cur = T; steps = 0;
            while (cur != S && steps++ < net.cfnNodes) {
                int prev = parent[cur];
                if (prev == -1 || prev == cur) break;
                net.flow[prev][cur] += delta;
                net.flow[cur][prev] -= delta;
                cur = prev;
            }

            remaining[idx]   -= delta;
            sinkCap[sinkOut] -= delta * szI;
            total            += delta * vi;
        }
        return total;
    }

    // =========================================================================
    //  DEEP COPY HELPER
    // =========================================================================

    private int[][] deepCopy2D(int[][] src) {
        int[][] copy = new int[src.length][];
        for (int i = 0; i < src.length; i++)
            copy[i] = src[i].clone();
        return copy;
    }
}