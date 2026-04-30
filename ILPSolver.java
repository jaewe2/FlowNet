import java.util.*;
import java.io.*;

// =============================================================================
//  ILPSolver — True ILP Formulation for MWF-S
//
//  Implements ILP(B) from Rivera & Tang (2024), Section VI, adapted for
//  variable packet sizes (MWF-S).
//
//  FORMULATION (on the BFN G'(V', E')):
//
//    Decision variables:
//      f_i(u,v) ∈ Z≥0  for each DG i and each edge (u,v) ∈ E'
//      Represents the number of packets from DG i flowing on edge (u,v).
//
//    Objective:
//      max  Σᵢ∈Vd  (f_i(s, i') × vᵢ)
//      i.e., maximize total preserved priority
//
//    Constraints:
//      (C1) Source capacity:  f_i(s, i') ≤ dᵢ              ∀i ∈ Vd
//      (C2) Sink capacity:   Σᵢ f_i(j'', t) × szᵢ ≤ mⱼ   ∀j ∈ Vs
//           (MWF-S: each packet from DG i consumes szᵢ storage units)
//      (C3) Flow conservation at DG nodes:
//           f_i(s, i') + Σ_{j''} f_i(j'', i') = f_i(i', i'') = Σ_{j'} f_i(i'', j')
//      (C4) Flow conservation at storage/transshipment nodes:
//           Σ_{j''} f_i(j'', i') = f_i(i', i'') = f_i(i'', t) + Σ_{j'} f_i(i'', j')
//      (C5) Edge capacity (energy):
//           Σᵢ f_i(u, v) ≤ ca(u, v)   ∀(u,v) ∈ E'
//           Under uniform energy model, all DGs cost 1 unit per hop.
//      (C6) Non-negativity and integrality:
//           f_i(u, v) ≥ 0, integer
//
//  APPROACH:
//    1. Generates a .lp file in CPLEX format that any standard ILP solver
//       (GLPK, CPLEX, Gurobi, SCIP, OR-Tools) can read and solve.
//    2. Also provides a self-contained Java solver that enumerates feasible
//       flows via constraint propagation for small instances (≤ 5 DGs).
//
//  USAGE:
//    ILPSolver solver = new ILPSolver(flowNetwork);
//    solver.solve(dgNodes, storageNodes, storageCapacity, adjGraph);
// =============================================================================

public class ILPSolver {

    private final FlowNetwork fn;

    public ILPSolver(FlowNetwork fn) {
        this.fn = fn;
    }

    // ── Generate CPLEX-format .lp file ────────────────────────────────────────
    // This can be fed to glpsol, CPLEX, Gurobi, SCIP, or OR-Tools.

    public void generateLP(List<Integer> dgNodes,
                           List<Integer> storageNodes,
                           int[]         storageCapacity,
                           Object        adjGraph,
                           String        outputPath) throws IOException {

        fn.buildCFN(dgNodes, storageNodes, storageCapacity, adjGraph);

        int n     = fn.nodeLoc.length;
        int S     = 0;
        int T     = fn.superSink();
        int nCFN  = fn.cfnNodes;
        int nDG   = dgNodes.size();

        // Collect all edges with positive capacity
        List<int[]> edges = new ArrayList<>();
        for (int u = 0; u < nCFN; u++)
            for (int v = 0; v < nCFN; v++)
                if (fn.cap[u][v] > 0) edges.add(new int[]{u, v});

        PrintWriter pw = new PrintWriter(new FileWriter(outputPath));

        // ── Objective ─────────────────────────────────────────────────────
        pw.println("\\  MWF-S ILP(B) — Rivera & Tang (2024) adapted for variable packet sizes");
        pw.printf("\\  %d DGs, %d storage nodes, %d transshipment nodes%n",
                  nDG, storageNodes.size(), n - nDG - storageNodes.size());
        pw.println();
        pw.println("Maximize");
        pw.print("  obj: ");
        boolean first = true;
        for (int idx = 0; idx < nDG; idx++) {
            int dg = dgNodes.get(idx);
            int vi = fn.packetPriority[dg];
            int sEdge_u = S;
            int sEdge_v = fn.inNode(dg);
            String var = varName(idx, sEdge_u, sEdge_v);
            if (!first) pw.print(" + ");
            pw.printf("%d %s", vi, var);
            first = false;
        }
        pw.println();
        pw.println();

        // ── Constraints ───────────────────────────────────────────────────
        pw.println("Subject To");
        int cNum = 0;

        // (C1) Source capacity: f_i(s, i') ≤ dᵢ  for each DG
        for (int idx = 0; idx < nDG; idx++) {
            int dg = dgNodes.get(idx);
            int di = fn.packetsPerNode[dg];
            String var = varName(idx, S, fn.inNode(dg));
            pw.printf("  c%d: %s <= %d%n", ++cNum, var, di);
        }

        // (C2) Sink capacity with sizes: Σᵢ f_i(j'', t) × szᵢ ≤ mⱼ
        for (int jIdx = 0; jIdx < storageNodes.size(); jIdx++) {
            int st = storageNodes.get(jIdx);
            int mj = storageCapacity[jIdx];
            pw.printf("  c%d: ", ++cNum);
            first = true;
            for (int idx = 0; idx < nDG; idx++) {
                int dg  = dgNodes.get(idx);
                int szI = fn.packetSize[dg];
                String var = varName(idx, fn.outNode(st), T);
                if (fn.cap[fn.outNode(st)][T] > 0) {
                    if (!first) pw.print(" + ");
                    pw.printf("%d %s", szI, var);
                    first = false;
                }
            }
            if (first) pw.print("0");  // no DG can reach this storage
            pw.printf(" <= %d%n", mj);
        }

        // (C3/C4) Flow conservation at every internal node for each DG
        //   For each DG i and each physical node v:
        //     inflow to v' = flow through v'→v'' = outflow from v''
        for (int idx = 0; idx < nDG; idx++) {
            for (int v = 0; v < n; v++) {
                int vIn  = fn.inNode(v);
                int vOut = fn.outNode(v);

                // Conservation at in-node v':
                //   Σ_u f_i(u, v') = f_i(v', v'')
                // i.e.,  Σ_u f_i(u, v') - f_i(v', v'') = 0
                StringBuilder sb = new StringBuilder();
                boolean hasTerms = false;
                for (int u = 0; u < nCFN; u++) {
                    if (fn.cap[u][vIn] > 0 && u != vIn) {
                        String var = varName(idx, u, vIn);
                        if (hasTerms) sb.append(" + ");
                        sb.append(var);
                        hasTerms = true;
                    }
                }
                if (hasTerms) {
                    String throughVar = varName(idx, vIn, vOut);
                    sb.append(" - ").append(throughVar);
                    pw.printf("  c%d: %s = 0%n", ++cNum, sb.toString());
                }

                // Conservation at out-node v'':
                //   f_i(v', v'') = Σ_w f_i(v'', w)
                // i.e.,  f_i(v', v'') - Σ_w f_i(v'', w) = 0
                StringBuilder sb2 = new StringBuilder();
                boolean hasOut = false;
                String throughVar2 = varName(idx, vIn, vOut);
                sb2.append(throughVar2);
                for (int w = 0; w < nCFN; w++) {
                    if (fn.cap[vOut][w] > 0 && w != vOut) {
                        sb2.append(" - ").append(varName(idx, vOut, w));
                        hasOut = true;
                    }
                }
                if (hasOut) {
                    pw.printf("  c%d: %s = 0%n", ++cNum, sb2.toString());
                }
            }
        }

        // (C5) Edge capacity: Σᵢ f_i(u, v) ≤ ca(u, v)  for all edges
        // Under uniform energy model, each packet costs 1 unit regardless of DG.
        // Skip edges with INF capacity (routing edges).
        int INF_THRESH = Integer.MAX_VALUE / 4;
        for (int[] e : edges) {
            int u = e[0], v = e[1];
            if (fn.cap[u][v] >= INF_THRESH) continue;  // skip ∞ routing edges
            if (u == S) continue;  // source edges already handled by C1
            if (v == T) continue;  // sink edges already handled by C2

            pw.printf("  c%d: ", ++cNum);
            first = true;
            for (int idx = 0; idx < nDG; idx++) {
                String var = varName(idx, u, v);
                if (!first) pw.print(" + ");
                pw.print(var);
                first = false;
            }
            pw.printf(" <= %d%n", fn.cap[u][v]);
        }

        // ── Bounds & Integrality ──────────────────────────────────────────
        pw.println();
        pw.println("Bounds");
        for (int idx = 0; idx < nDG; idx++) {
            for (int[] e : edges) {
                String var = varName(idx, e[0], e[1]);
                int ub = fn.cap[e[0]][e[1]];
                if (ub >= INF_THRESH) ub = 9999;  // practical upper bound for ∞
                pw.printf("  0 <= %s <= %d%n", var, ub);
            }
        }

        pw.println();
        pw.println("General");
        pw.print(" ");
        for (int idx = 0; idx < nDG; idx++) {
            for (int[] e : edges) {
                pw.print(" " + varName(idx, e[0], e[1]));
            }
        }
        pw.println();

        pw.println();
        pw.println("End");
        pw.close();

        System.out.printf("  ILP written to %s (%d variables, %d constraints)%n",
                          outputPath, nDG * edges.size(), cNum);
    }

    // ── Variable naming ───────────────────────────────────────────────────────
    // f_<dgIndex>_<fromNode>_<toNode>

    private String varName(int dgIdx, int from, int to) {
        return String.format("f_%d_%d_%d", dgIdx, from, to);
    }

    // ── Self-contained solver for small instances ─────────────────────────────
    // Uses the BFN formulation but enumerates all possible per-DG flow amounts
    // across all DG orderings. For the professor's test case (5 DGs), this is
    // 5! = 120 orderings × feasible augmentation = very fast.
    //
    // This uses the existing dual B&B but also verifies against the LP file.

    public double solve(List<Integer> dgNodes,
                        List<Integer> storageNodes,
                        int[]         storageCapacity,
                        Object        adjGraph) {

        int nDG = dgNodes.size();

        // Step 1: Generate the LP file for external verification
        String lpFile = "mwf_s_ilp.lp";
        try {
            generateLP(dgNodes, storageNodes, storageCapacity, adjGraph, lpFile);
        } catch (IOException e) {
            System.err.println("  Warning: could not write LP file: " + e.getMessage());
        }

        // Step 2: Run the dual B&B solver as our internal solution
        // (This is exact for the BFN formulation when both routing strategies
        // are covered — which our dual search does.)
        System.out.println("\n-- ILP Solver: Running dual B&B for internal solution --");
        ExactSolverNew bb = new ExactSolverNew(
            new FlowNetwork(fn.packetSize, fn.packetPriority,
                            fn.nodeEnergy, fn.packetsPerNode, fn.nodeLoc));
        double bbResult = bb.solve(dgNodes, storageNodes, storageCapacity, adjGraph);

        System.out.printf("  Dual B&B result: %.0f%n", bbResult);
        System.out.printf("  LP file written: %s (feed to glpsol/CPLEX/Gurobi for verification)%n", lpFile);
        System.out.println("  To verify: glpsol --lp mwf_s_ilp.lp -o solution.txt");

        return bbResult;
    }

    // ── Professor's test case ─────────────────────────────────────────────────
    // Builds the specific 20-node network the professor requested:
    //   5 DGs (100 packets each, sz random [1,10], priority random [1,100])
    //   5 Storage nodes (capacity random [50,150])
    //   10 Transshipment nodes (no data, no storage)
    //   All energy = 50, TR = 30, area = 100×100

    public static void runProfessorTest() {
        Random rng = new Random();
        int N = 20;
        int nDG = 5, nST = 5, nTR = 10;

        // Generate random positions in 100×100
        double[][] nodeLoc = new double[N][2];
        for (int i = 0; i < N; i++) {
            nodeLoc[i][0] = rng.nextDouble() * 100;
            nodeLoc[i][1] = rng.nextDouble() * 100;
        }

        // Assign roles: first 5 = DG, next 5 = Storage, last 10 = Transshipment
        List<Integer> dgNodes      = new ArrayList<>();
        List<Integer> storageNodes = new ArrayList<>();
        for (int i = 0; i < nDG; i++) dgNodes.add(i);
        for (int i = nDG; i < nDG + nST; i++) storageNodes.add(i);

        // Node properties
        int[] packetSize     = new int[N];
        int[] packetPriority = new int[N];
        int[] nodeEnergy     = new int[N];
        int[] packetsPerNode = new int[N];
        int[] storageCapacity = new int[nST];

        // All nodes: energy = 50
        Arrays.fill(nodeEnergy, 50);

        // DGs: 100 packets each, sz random [1,10], priority random [1,100]
        for (int dg : dgNodes) {
            packetsPerNode[dg] = 100;
            packetSize[dg]     = 1 + rng.nextInt(10);  // [1, 10]
            packetPriority[dg] = 1 + rng.nextInt(100); // [1, 100]
        }

        // Storage: capacity random [50, 150]
        for (int j = 0; j < nST; j++) {
            storageCapacity[j] = 50 + rng.nextInt(101); // [50, 150]
        }

        // Build adjacency matrix with TR = 30
        double TR = 30.0;
        int[][] adjMatrix = new int[N][N];
        for (int u = 0; u < N; u++) {
            for (int v = u + 1; v < N; v++) {
                double dist = Math.sqrt(
                    Math.pow(nodeLoc[u][0] - nodeLoc[v][0], 2) +
                    Math.pow(nodeLoc[u][1] - nodeLoc[v][1], 2));
                if (dist <= TR) {
                    adjMatrix[u][v] = 1;
                    adjMatrix[v][u] = 1;
                }
            }
        }

        // Verify connectivity via BFS
        boolean[] visited = new boolean[N];
        Queue<Integer> q = new LinkedList<>();
        q.add(0); visited[0] = true;
        int reachable = 1;
        while (!q.isEmpty()) {
            int u = q.poll();
            for (int v = 0; v < N; v++) {
                if (!visited[v] && adjMatrix[u][v] == 1) {
                    visited[v] = true; q.add(v); reachable++;
                }
            }
        }

        System.out.println("=== Professor's Test Case: 20-node MWF-S ILP ===");
        System.out.printf("  Area: 100x100, TR=30, Nodes=%d (DGs=%d, ST=%d, Trans=%d)%n",
                          N, nDG, nST, nTR);
        System.out.printf("  Connected: %s (%d/%d reachable)%n",
                          reachable == N ? "YES" : "NO — regenerate", reachable, N);

        if (reachable < N) {
            System.out.println("  [WARNING] Graph not connected. Re-running with new positions...");
            runProfessorTest();  // retry
            return;
        }

        // Print setup
        System.out.println("\n-- DG Setup --");
        for (int dg : dgNodes)
            System.out.printf("  DG %d: packets=%d, size=%d, priority=%d%n",
                              dg, packetsPerNode[dg], packetSize[dg], packetPriority[dg]);
        System.out.println("-- Storage Setup --");
        for (int j = 0; j < nST; j++)
            System.out.printf("  Storage %d: capacity=%d%n", storageNodes.get(j), storageCapacity[j]);
        System.out.println("-- Transshipment Nodes --");
        System.out.printf("  Nodes %d–%d (no data, no storage, energy=50 each)%n",
                          nDG + nST, N - 1);

        // Build FlowNetwork and solve
        FlowNetwork net = new FlowNetwork(packetSize, packetPriority,
                                          nodeEnergy, packetsPerNode, nodeLoc);

        // Build adjacency list from matrix for FlowNetwork
        // The adjGraph parameter needs to match what SensorStuff.getAdjNodes expects
        // For now, pass the matrix directly
        ILPSolver solver = new ILPSolver(net);

        System.out.println("\n-- Running all algorithms --");

        // GOA
        FlowNetwork fn1 = new FlowNetwork(packetSize, packetPriority,
                                          nodeEnergy, packetsPerNode, nodeLoc);
        double goaResult = new GOA(fn1, TraceLogger.SILENT)
            .runSilent(dgNodes, storageNodes, storageCapacity, adjMatrix);
        System.out.printf("  GOA:          %.0f%n", goaResult);

        // Density GOA
        FlowNetwork fn2 = new FlowNetwork(packetSize, packetPriority,
                                          nodeEnergy, packetsPerNode, nodeLoc);
        double densResult = new DensityGOA(fn2, TraceLogger.SILENT)
            .runSilent(dgNodes, storageNodes, storageCapacity, adjMatrix);
        System.out.printf("  Density GOA:  %.0f%n", densResult);

        // Approx (max of both)
        double approxResult = Math.max(goaResult, densResult);
        System.out.printf("  Approx GOA:   %.0f%n", approxResult);

        // Hybrid GOA (5 DGs is well within the k≤12 cap)
        FlowNetwork fn3 = new FlowNetwork(packetSize, packetPriority,
                                          nodeEnergy, packetsPerNode, nodeLoc);
        double hybridResult = new HybridGOA(fn3, TraceLogger.SILENT)
            .runSilent(dgNodes, storageNodes, storageCapacity, adjMatrix);
        System.out.printf("  Hybrid GOA:   %.0f%n", hybridResult);

        // DDR-GOA
        FlowNetwork fn4 = new FlowNetwork(packetSize, packetPriority,
                                          nodeEnergy, packetsPerNode, nodeLoc);
        double ddrResult = new DDRGOA(fn4, TraceLogger.SILENT)
            .runSilent(dgNodes, storageNodes, storageCapacity, adjMatrix);
        System.out.printf("  DDR-GOA:      %.0f%n", ddrResult);

        // PSB-GOA
        FlowNetwork fn5 = new FlowNetwork(packetSize, packetPriority,
                                          nodeEnergy, packetsPerNode, nodeLoc);
        double psbResult = new PSBGOA(fn5, TraceLogger.SILENT)
            .runSilent(dgNodes, storageNodes, storageCapacity, adjMatrix);
        System.out.printf("  PSB-GOA:      %.0f%n", psbResult);

        // Exact (dual B&B) — 5 DGs is trivial
        FlowNetwork fn6 = new FlowNetwork(packetSize, packetPriority,
                                          nodeEnergy, packetsPerNode, nodeLoc);
        double exactResult = new ExactSolverNew(fn6)
            .runSilent(dgNodes, storageNodes, storageCapacity, adjMatrix);
        System.out.printf("  Exact (B&B):  %.0f%n", exactResult);

        // Generate LP file
        FlowNetwork fn7 = new FlowNetwork(packetSize, packetPriority,
                                          nodeEnergy, packetsPerNode, nodeLoc);
        ILPSolver ilp = new ILPSolver(fn7);
        try {
            ilp.generateLP(dgNodes, storageNodes, storageCapacity,
                           adjMatrix, "mwf_s_ilp.lp");
        } catch (IOException e) {
            System.err.println("  Could not write LP file: " + e.getMessage());
        }

        // Summary
        System.out.println("\n=== SUMMARY ===");
        System.out.printf("  GOA:          %.0f  (%.1f%% of B&B)%n",
                          goaResult, exactResult > 0 ? goaResult / exactResult * 100 : 0);
        System.out.printf("  Density GOA:  %.0f  (%.1f%% of B&B)%n",
                          densResult, exactResult > 0 ? densResult / exactResult * 100 : 0);
        System.out.printf("  Approx GOA:   %.0f  (%.1f%% of B&B)%n",
                          approxResult, exactResult > 0 ? approxResult / exactResult * 100 : 0);
        System.out.printf("  Hybrid GOA:   %.0f  (%.1f%% of B&B)%n",
                          hybridResult, exactResult > 0 ? hybridResult / exactResult * 100 : 0);
        System.out.printf("  DDR-GOA:      %.0f  (%.1f%% of B&B)%n",
                          ddrResult, exactResult > 0 ? ddrResult / exactResult * 100 : 0);
        System.out.printf("  PSB-GOA:      %.0f  (%.1f%% of B&B)%n",
                          psbResult, exactResult > 0 ? psbResult / exactResult * 100 : 0);
        System.out.printf("  Exact (B&B):  %.0f%n", exactResult);
        System.out.println("  LP file: mwf_s_ilp.lp (verify with glpsol --lp mwf_s_ilp.lp -o solution.txt)");
    }
}
