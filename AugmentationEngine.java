import java.util.*;

// =============================================================================
//  AugmentationEngine — Shared augmentation helpers used across algorithms
//
//  Contains augmentation variants that go beyond the basic augmentPath loop
//  in FlowNetwork:
//
//    augmentBestFit  — Hybrid/PSB: pick storage node with tightest remaining
//                      capacity after placement (minimise wasted fragments).
//
//    runPSBTail      — PSB-GOA: energy-aware per-step best-path scoring on
//                      all remaining DGs simultaneously.
//
//  All methods operate on a FlowNetwork instance passed by reference and
//  modify cap/flow/remaining/sinkCap in place; callers snapshot/restore.
// =============================================================================

public class AugmentationEngine {

    private final FlowNetwork fn;

    public AugmentationEngine(FlowNetwork fn) {
        this.fn = fn;
    }

    // ── Best-fit augmentation ─────────────────────────────────────────────────
    // Fully augments one DG (index idx into dgNodes) using best-fit storage
    // selection: among all storage nodes reachable via a shortest FAP, prefer
    // the one whose residual capacity after placing szᵢ units is the smallest
    // non-negative value (tightest fit).
    //
    // This minimises storage fragmentation — leftover fragments that are too
    // small for any remaining packet size are wasted capacity.
    //
    // Returns total priority value gained.

    public int augmentBestFit(int           idx,
                              List<Integer> dgNodes,
                              int[]         remaining,
                              int[]         sinkCap,
                              List<Integer> storageNodes) {
        int dg     = dgNodes.get(idx);
        int cfnSrc = fn.inNode(dg);
        int szI    = fn.packetSize[dg];
        int vi     = fn.packetPriority[dg];
        int S      = 0, T = fn.superSink();
        int total  = 0;
        int maxIter = fn.cfnNodes * fn.cfnNodes;
        int iters  = 0;

        while (remaining[idx] > 0 && iters++ < maxIter) {

            // try each storage node individually with a masked sinkCap;
            // score by residual after placement (lower = tighter = better)
            int[]  bestParent   = null;
            int    bestFitScore = Integer.MAX_VALUE;

            for (int j = 0; j < storageNodes.size(); j++) {
                int st    = storageNodes.get(j);
                int stOut = fn.outNode(st);
                if (sinkCap[stOut] < szI) continue;

                int[] maskedSink = new int[fn.cfnNodes];
                maskedSink[stOut] = sinkCap[stOut];

                int[] parent = fn.bfsFAP(cfnSrc, szI, maskedSink);
                if (parent == null) continue;

                // compute delta for this path — skip sink edge (storage-unit cap)
                int pathBottleneck = Integer.MAX_VALUE;
                int cur = T, steps = 0;
                while (cur != S && steps++ < fn.cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    if (cur != T)
                        pathBottleneck = Math.min(pathBottleneck,
                                                 fn.cap[prev][cur] - fn.flow[prev][cur]);
                    cur = prev;
                }
                int delta = Math.min(pathBottleneck,
                            Math.min(remaining[idx], sinkCap[stOut] / szI));
                if (delta <= 0) continue;

                int newResidual = sinkCap[stOut] - delta * szI;
                if (newResidual < bestFitScore) {
                    bestFitScore = newResidual;
                    bestParent   = parent;
                }
            }

            // fall back to standard bfsFAP if best-fit found nothing
            if (bestParent == null) {
                bestParent = fn.bfsFAP(cfnSrc, szI, sinkCap);
                if (bestParent == null) break;
            }

            int gained = fn.augmentPath(bestParent, idx, remaining, sinkCap,
                                        szI, vi, null);
            if (gained <= 0) break;
            total += gained;
        }
        return total;
    }

    // ── PSB tail ──────────────────────────────────────────────────────────────
    // Runs the energy-aware per-step best path scoring on all remaining DGs
    // (excluding 'used' set and the just-committed candidate 'committed')
    // against the current FlowNetwork state. Returns total priority value.
    //
    // Score formula (energy-aware):
    //   score(P) = (v_i × Δ) / (Δ × sz_i + waste + relayPenalty(P) + 1)
    //
    // relayPenalty(P) = scale × Σ_{relay r on P} (Δ / residualEnergy_r)

    public int runPSBTail(List<Integer> dgNodes,
                          int[]         remaining,
                          int[]         sinkCap,
                          Set<Integer>  used,
                          int           committed) {
        int S = 0, T = fn.superSink();
        int n = dgNodes.size();
        int total = 0;

        while (true) {
            // find minimum remaining packet size among active DGs
            int minRemSz = Integer.MAX_VALUE;
            for (int i = 0; i < n; i++) {
                if (i == committed || used.contains(i)) continue;
                if (remaining[i] > 0)
                    minRemSz = Math.min(minRemSz, fn.packetSize[dgNodes.get(i)]);
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
                int cfnSrc = fn.inNode(dg);
                int szI    = fn.packetSize[dg];
                int vi     = fn.packetPriority[dg];

                int[] parent = fn.bfsFAP(cfnSrc, szI, sinkCap);
                if (parent == null) continue;

                int sinkOut = parent[T];
                int pathBottleneck = Integer.MAX_VALUE;
                int cur = T, steps = 0;
                while (cur != S && steps++ < fn.cfnNodes) {
                    int prev = parent[cur];
                    if (prev == -1 || prev == cur) break;
                    if (cur != T)
                        pathBottleneck = Math.min(pathBottleneck,
                                                 fn.cap[prev][cur] - fn.flow[prev][cur]);
                    cur = prev;
                }
                int delta = Math.min(pathBottleneck,
                            Math.min(remaining[i], sinkCap[sinkOut] / szI));
                if (delta <= 0) continue;

                int    newResidual  = sinkCap[sinkOut] - delta * szI;
                int    waste        = (minRemSz > 0) ? (newResidual % minRemSz) : 0;
                double relayPenalty = fn.computeRelayPenalty(parent, dg, delta);
                double score        = (double)(vi * delta)
                                    / ((double)(delta * szI + waste) + relayPenalty + 1.0);

                if (score > bestScore) {
                    bestScore = score; bestIdx = i;
                    bestParent = parent; bestDelta = delta;
                }
            }

            if (bestIdx < 0) break;

            int sinkOut = bestParent[T];
            int szI     = fn.packetSize[dgNodes.get(bestIdx)];
            int vi      = fn.packetPriority[dgNodes.get(bestIdx)];

            int cur = T, steps = 0;
            while (cur != S && steps++ < fn.cfnNodes) {
                int prev = bestParent[cur];
                if (prev == -1 || prev == cur) break;
                // sink edge (prev→T) is in storage units; all others in packets
                int amount = (cur == T) ? bestDelta * szI : bestDelta;
                fn.flow[prev][cur] += amount;
                fn.flow[cur][prev] -= amount;
                cur = prev;
            }

            remaining[bestIdx] -= bestDelta;
            sinkCap[sinkOut]   -= bestDelta * szI;
            total              += bestDelta * vi;
        }
        return total;
    }
}
