import java.util.*;

// =============================================================================
//  AlgorithmResult — value object returned by every MWF algorithm
//
//  Bundles the total preserved priority with the BSN-level flow edges used,
//  so callers can visualise results without the algorithm knowing about Swing.
// =============================================================================

public class AlgorithmResult {

    public final double      totalPriority;
    public final List<int[]> flowEdges;     // each int[] = {bsnU, bsnV}

    public AlgorithmResult(double totalPriority, List<int[]> flowEdges) {
        this.totalPriority = totalPriority;
        this.flowEdges     = flowEdges != null ? flowEdges : new ArrayList<>();
    }

    public AlgorithmResult(double totalPriority) {
        this(totalPriority, new ArrayList<>());
    }
}
