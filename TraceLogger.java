import java.util.*;

// =============================================================================
//  TraceLogger — relay path tracing and relay info printing
//
//  Extracted from SensorStuff so that algorithm classes do not directly call
//  System.out for relay diagnostics. Pass a TraceLogger to algorithms that
//  want verbose output; algorithms call logTrace() and logRelayInfo().
//  Pass TraceLogger.SILENT to suppress all output (scaling trials).
// =============================================================================

public class TraceLogger {

    /** Shared no-op instance — zero overhead in silent runs. */
    public static final TraceLogger SILENT = new TraceLogger(false);

    private final boolean verbose;

    public TraceLogger(boolean verbose) {
        this.verbose = verbose;
    }

    // ── Relay trace (one augmentation step) ──────────────────────────────────

    public void logTrace(String        algoTag,
                         int           dg,
                         int           sinkNode,
                         int           delta,
                         int           vi,
                         int           szI,
                         List<Integer> bsnPath,
                         Set<Integer>  dgSet,
                         Set<Integer>  storageSet,
                         int[]         nodeEnergy) {
        if (!verbose) return;

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
            if (i > 0 && i < bsnPath.size() - 1)
                sb.append("[").append(roleTag(node, dgSet, storageSet)).append("]");
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
                              relay, roleTag(relay, dgSet, storageSet),
                              delta, delta, nodeEnergy[relay]);
        }
    }

    // ── Push summary (one augmentation step) ─────────────────────────────────

    public void logPush(int delta, int dg, int vi, int szI, int sinkOut) {
        if (!verbose) return;
        System.out.printf(
            "  Pushed %d packet(s) from DG %d (v=%d, sz=%d) -> sink %d%n",
            delta, dg, vi, szI, sinkOut);
    }

    // ── Relay node activity summary (after all augmentations) ────────────────

    public void logRelayInfo(Map<Integer, Integer> relayCount,
                             Set<Integer>          dgSet,
                             Set<Integer>          storageSet) {
        if (!verbose) return;
        System.out.println("\n-- Relay Node Activity --");
        boolean any = false;
        for (Map.Entry<Integer, Integer> e : relayCount.entrySet()) {
            int node = e.getKey(), pkts = e.getValue();
            String role = dgSet.contains(node)      ? " [DG-relay]"
                        : storageSet.contains(node) ? " [Storage-relay]"
                        : " [Relay]";
            System.out.printf("  Node %d%s: forwarded %d packet(s)%n",
                              node, role, pkts);
            any = true;
        }
        if (!any) System.out.println("  No relay nodes used.");
    }

    // ── Generic message ───────────────────────────────────────────────────────

    public void log(String msg) {
        if (verbose) System.out.println(msg);
    }

    public void logf(String fmt, Object... args) {
        if (verbose) System.out.printf(fmt, args);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private static String roleTag(int node, Set<Integer> dgSet,
                                  Set<Integer> storageSet) {
        if (dgSet.contains(node))      return "DG-relay";
        if (storageSet.contains(node)) return "Storage-relay";
        return "Relay";
    }
}
