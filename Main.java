import java.util.*;

// =============================================================================
//  Main — entry point
//
//  Reads configuration from stdin and delegates to SimulationRunner.
//  Mode 1: Regular MWF-S simulation (all algorithms + scaling trials)
//  Mode 2: Professor's ILP test case (20 nodes, fixed structure, LP file output)
// =============================================================================

public class Main {

    public static void main(String[] args) {
        Scanner kb = new Scanner(System.in);

        System.out.println("Select mode:");
        System.out.println("  1 — Regular MWF-S simulation");
        System.out.println("  2 — Professor's ILP test (20 nodes, generates .lp file)");
        System.out.print("Mode: ");
        int mode = kb.nextInt();

        if (mode == 2) {
            System.out.println();
            ILPSolver.runProfessorTest();
            kb.close();
            return;
        }

        // ── Mode 1: Regular simulation ────────────────────────────────────────
        SimulationRunner runner = new SimulationRunner();

        System.out.print("Width x and length y of sensor network: ");
        runner.widthX = kb.nextInt();
        runner.lenY   = kb.nextInt();

        System.out.print("Transmission range (m): ");
        runner.TR = kb.nextInt();

        System.out.print("Graph structure - 1: adj-matrix, 2: adj-list: ");
        runner.choice = kb.nextInt();

        System.out.print("Network sizes to test (comma-separated, e.g. 10,50,100): ");
        String[] sizeTokens = kb.next().split(",");
        runner.networkSizes = new int[sizeTokens.length];
        for (int i = 0; i < sizeTokens.length; i++)
            runner.networkSizes[i] = Math.max(2, Integer.parseInt(sizeTokens[i].trim()));

        System.out.print("Number of trials per network size: ");
        runner.trials = kb.nextInt();

        System.out.print("Min node energy level: ");
        runner.minE = kb.nextInt();
        System.out.print("Max node energy level: ");
        runner.maxE = kb.nextInt();

        System.out.print("Min overflow packets per DG: ");
        runner.minPkt = kb.nextInt();
        System.out.print("Max overflow packets per DG: ");
        runner.maxPkt = kb.nextInt();

        System.out.print("Min packet size (storage units): ");
        runner.minSz = kb.nextInt();
        System.out.print("Max packet size (storage units): ");
        runner.maxSz = kb.nextInt();

        System.out.print("Min storage capacity (storage units): ");
        runner.minCap = kb.nextInt();
        System.out.print("Max storage capacity (storage units): ");
        runner.maxCap = kb.nextInt();

        System.out.print("Min packet priority: ");
        runner.minPri = kb.nextInt();
        System.out.print("Max packet priority: ");
        runner.maxPri = kb.nextInt();

        System.out.print("Number of nodes for visual run: ");
        runner.visNodes = Math.max(2, kb.nextInt());

        kb.close();

        runner.runVisual();
        runner.runScaling();
    }
}
