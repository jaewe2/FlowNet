import java.util.*;

// =============================================================================
//  SimulationRunner — visual run + scaling trial loop
//
//  Extracted from SensorStuff.main() so that the entry point contains only
//  I/O and orchestration logic, with no algorithm or graph logic mixed in.
//
//  Usage:
//    SimulationRunner runner = new SimulationRunner(config);
//    runner.runVisual();
//    runner.runScaling();
// =============================================================================

public class SimulationRunner {

    // ── Configuration (set from main() via Scanner) ───────────────────────────

    public int     widthX, lenY, TR, choice;
    public int[]   networkSizes;
    public int     trials;
    public int     minE, maxE;
    public int     minPkt, maxPkt;
    public int     minSz, maxSz;
    public int     minCap, maxCap;
    public int     minPri, maxPri;
    public int     visNodes;
    public Random  rand = new Random();

    // ── Visual run ────────────────────────────────────────────────────────────

    public void runVisual() {
        System.out.println("\n==== Visual Run (graphs for one trial) ====");

        // retry until we find a trial that actually has a bottleneck
        SensorStuff visNet = null;
        List<Integer> visDG = null, visST = null;
        int[] visCap = null;
        int attempts = 0;

        while (true) {
            attempts++;
            visNet = new SensorStuff(visNodes, choice);
            visNet.buildConnectedGraph(visNodes, widthX, lenY, TR, rand, false);
            visNet.randomNodeEnergies(minE, maxE);
            visNet.randomDataPackets(minPkt, maxPkt);
            visNet.randomPacketSizes(minSz, maxSz);
            visNet.randomPacketPriorities(minPri, maxPri);

            boolean[] visVisited = new boolean[visNodes];
            visNet.components = new ArrayList<>();
            for (int i = 0; i < visNodes; i++)
                if (!visVisited[i])
                    visNet.components.add(SensorStuff.buildBFS(
                        visNet.adjM != null ? visNet.adjM.getAdjM() : visNet.adjList,
                        i, visVisited));

            visDG = new ArrayList<>();
            visST = new ArrayList<>();
            List<Integer> visAll = new ArrayList<>();
            for (int i = 0; i < visNodes; i++) visAll.add(i);
            Collections.shuffle(visAll, rand);
            visDG.add(visAll.get(0)); visST.add(visAll.get(1));
            double visDgRatio = 0.2 + rand.nextDouble() * 0.5;
            for (int i = 2; i < visNodes; i++) {
                if (rand.nextDouble() < visDgRatio) visDG.add(visAll.get(i));
                else                               visST.add(visAll.get(i));
            }

            visCap = new int[visST.size()];
            for (int j = 0; j < visCap.length; j++)
                visCap[j] = (minCap == maxCap) ? minCap
                          : rand.nextInt(maxCap - minCap + 1) + minCap;

            if (visNet.needsMWF(visDG, visST, visCap)) break;

            if (attempts >= 50) {
                System.out.println(
                    "  Warning: could not find a bottleneck instance after 50 attempts.\n" +
                    "  Try reducing storage capacity or increasing packets per DG.\n" +
                    "  Skipping visual run.");
                return;
            }
            System.out.printf("  (attempt %d: no bottleneck, retrying...)%n", attempts);
        }

        System.out.printf("  Visual run DG ratio: %.0f%% (%d DGs, %d storage) — found in %d attempt(s)%n",
                          100.0 * visDG.size() / visNodes, visDG.size(), visST.size(), attempts);
        System.out.println("\n-- Visual Run DG Setup --");
        for (int dg : visDG)
            System.out.printf("  DG %d: packets=%d, size=%d, priority=%d, energy=%d%n",
                dg, visNet.packetsPerNode[dg], visNet.packetSize[dg],
                visNet.packetPriority[dg], visNet.nodeEnergy[dg]);
        System.out.println("-- Visual Run Storage Setup --");
        for (int j = 0; j < visST.size(); j++)
            System.out.printf("  Storage %d: capacity=%d, energy=%d%n",
                visST.get(j), visCap[j], visNet.nodeEnergy[visST.get(j)]);

        visNet.printAdjacency(visDG, visST);

        Object adjGraph = visNet.adjM != null ? visNet.adjM.getAdjM() : visNet.adjList;
        GraphLauncher launcher = new GraphLauncher(visNet);
        TraceLogger verbose = new TraceLogger(true);

        // each algorithm gets its own FlowNetwork so shared cap/flow state
        // is never corrupted between algorithm calls
        System.out.println("\n-- Running GOA (visual) --");
        AlgorithmResult resGOA = new GOA(newFN(visNet), verbose)
            .run(visDG, visST, visCap, adjGraph);
        launcher.launchGraph(visDG, visST, resGOA.flowEdges, visCap, "GOA - Sort by Priority (v)");
        launcher.launchBFN(visDG, visST, visCap, resGOA.flowEdges, "GOA - Sort by Priority (v)");

        System.out.println("\n-- Running Density GOA (visual) --");
        AlgorithmResult resDens = new DensityGOA(newFN(visNet), verbose)
            .run(visDG, visST, visCap, adjGraph);
        launcher.launchGraph(visDG, visST, resDens.flowEdges, visCap, "Density GOA");
        launcher.launchBFN(visDG, visST, visCap, resDens.flowEdges, "Density GOA");

        System.out.println("\n-- Running Hybrid GOA (visual) --");
        AlgorithmResult resHybrid = new HybridGOA(newFN(visNet), verbose)
            .run(visDG, visST, visCap, adjGraph);
        launcher.launchGraph(visDG, visST, resHybrid.flowEdges, visCap, "Hybrid GOA");
        launcher.launchBFN(visDG, visST, visCap, resHybrid.flowEdges, "Hybrid GOA");

        System.out.println("\n-- Running DDR-GOA (visual) --");
        AlgorithmResult resDDR = new DDRGOA(newFN(visNet), verbose)
            .run(visDG, visST, visCap, adjGraph);
        launcher.launchGraph(visDG, visST, resDDR.flowEdges, visCap, "DDR-GOA");
        launcher.launchBFN(visDG, visST, visCap, resDDR.flowEdges, "DDR-GOA");

        System.out.println("\n-- Running DDR+-GOA (visual) --");
        AlgorithmResult resDDRP = new DDRPlusGOA(newFN(visNet), verbose)
            .run(visDG, visST, visCap, adjGraph);
        launcher.launchGraph(visDG, visST, resDDRP.flowEdges, visCap, "DDR+-GOA");
        launcher.launchBFN(visDG, visST, visCap, resDDRP.flowEdges, "DDR+-GOA");

        System.out.println("\n-- Running PSB-GOA (visual) --");
        AlgorithmResult resPSB = new PSBGOA(newFN(visNet), verbose)
            .run(visDG, visST, visCap, adjGraph);
        launcher.launchGraph(visDG, visST, resPSB.flowEdges, visCap, "PSB-GOA");
        launcher.launchBFN(visDG, visST, visCap, resPSB.flowEdges, "PSB-GOA");

        System.out.println("\n-- Running Exact Solver (visual) --");
        double visExact = new ExactSolverNew(newFN(visNet))
            .solve(visDG, visST, visCap, adjGraph);

        System.out.printf("%n===== VISUAL RUN SUMMARY =====%n");
        System.out.printf("  ILP Optimal:                %.1f%n", visExact);
        boolean visFull = visDG.size() <= HybridGOA.MAX_HYBRID_DGS;
        pct("GOA",         resGOA.totalPriority,    visExact);
        pct("Density GOA", resDens.totalPriority,   visExact);
        pct(visFull ? "Hybrid GOA" : "Hybrid GOA*", resHybrid.totalPriority, visExact);
        pct("DDR-GOA",     resDDR.totalPriority,    visExact);
        pct("DDR+-GOA",    resDDRP.totalPriority,   visExact);
        pct(visFull ? "PSB-GOA" : "PSB-GOA*",       resPSB.totalPriority,    visExact);
        if (!visFull)
            System.out.printf("  [*] k=%d > MAX_HYBRID_DGS (%d): Hybrid/PSB used DensityGOA%n",
                              visDG.size(), HybridGOA.MAX_HYBRID_DGS);

        // post-hoc sanity check on visual run
        if (visExact >= 0) {
            double bestVis = Math.max(Math.max(resGOA.totalPriority, resDens.totalPriority),
                             Math.max(Math.max(resHybrid.totalPriority, resDDR.totalPriority),
                                      Math.max(resDDRP.totalPriority, resPSB.totalPriority)));
            if (bestVis > visExact + 0.5)
                System.out.printf("  [WARNING] Heuristic (%.1f) > ILP Optimal (%.1f) " +
                                  "— constraint violation!%n", bestVis, visExact);
        }

        System.out.println("\n-- Graphs launched. Starting scaling runs... --\n");
    }

    // ── Scaling loop ──────────────────────────────────────────────────────────

    public void runScaling() {
        for (int numNodes : networkSizes) {
            System.out.printf("%n==== Network Size: %d nodes, %d trials ====%n",
                              numNodes, trials);

            double sumGOA = 0, sumDensity = 0, sumApprox = 0,
                   sumHybrid = 0, sumDDR = 0, sumDDRP = 0, sumPSB = 0, sumExact = 0;
            int densityBeatGOA = 0, goaBeatDensity = 0, tied = 0;
            int hybridBeatApprox = 0, approxBeatHybrid = 0, hybridTied = 0;
            int ddrBeatApprox = 0, approxBeatDDR = 0, ddrTied = 0;
            int ddrpBeatDDR = 0, ddrBeatDDRP = 0, ddrpTied = 0;
            int psbBeatAll = 0, allBeatPSB = 0, psbTied = 0;
            int exactTrials = 0, activeTrials = 0, violations = 0;
            boolean anyFallback = false;

            for (int t = 1; t <= trials; t++) {
                SensorStuff net = new SensorStuff(numNodes, choice);
                int trialTR = net.buildConnectedGraph(
                    numNodes, widthX, lenY, TR, rand, true);

                net.randomNodeEnergies(minE, maxE);
                net.randomDataPackets(minPkt, maxPkt);
                net.randomPacketSizes(minSz, maxSz);
                net.randomPacketPriorities(minPri, maxPri);

                boolean[] visited = new boolean[numNodes];
                net.components = new ArrayList<>();
                for (int i = 0; i < numNodes; i++)
                    if (!visited[i])
                        net.components.add(SensorStuff.buildBFS(
                            net.adjM != null ? net.adjM.getAdjM() : net.adjList,
                            i, visited));

                List<Integer> dgNodes = new ArrayList<>(), storageNodes = new ArrayList<>();
                List<Integer> allNodes = new ArrayList<>();
                for (int i = 0; i < numNodes; i++) allNodes.add(i);
                Collections.shuffle(allNodes, rand);
                dgNodes.add(allNodes.get(0)); storageNodes.add(allNodes.get(1));
                double dgRatio = 0.2 + rand.nextDouble() * 0.5;
                for (int i = 2; i < numNodes; i++) {
                    if (rand.nextDouble() < dgRatio) dgNodes.add(allNodes.get(i));
                    else                             storageNodes.add(allNodes.get(i));
                }

                int[] storageCap = new int[storageNodes.size()];
                for (int j = 0; j < storageCap.length; j++)
                    storageCap[j] = (minCap == maxCap) ? minCap
                        : rand.nextInt(maxCap - minCap + 1) + minCap;

                if (!net.needsMWF(dgNodes, storageNodes, storageCap)) {
                    System.out.printf("  Trial %2d [TR=%d, DGs=%d, STs=%d]: " +
                                      "No bottleneck — skipped%n",
                                      t, trialTR, dgNodes.size(), storageNodes.size());
                    continue;
                }

                activeTrials++;
                Object adjGraph = net.adjM != null ? net.adjM.getAdjM() : net.adjList;

                double goaResult     = new GOA(newFN(net), TraceLogger.SILENT)
                                           .runSilent(dgNodes, storageNodes, storageCap, adjGraph);
                double densityResult = new DensityGOA(newFN(net), TraceLogger.SILENT)
                                           .runSilent(dgNodes, storageNodes, storageCap, adjGraph);
                double approxResult  = Math.max(goaResult, densityResult);
                double hybridResult  = new HybridGOA(newFN(net), TraceLogger.SILENT)
                                           .runSilent(dgNodes, storageNodes, storageCap, adjGraph);
                double ddrResult     = new DDRGOA(newFN(net), TraceLogger.SILENT)
                                           .runSilent(dgNodes, storageNodes, storageCap, adjGraph);
                double ddrpResult    = new DDRPlusGOA(newFN(net), TraceLogger.SILENT)
                                           .runSilent(dgNodes, storageNodes, storageCap, adjGraph);
                double psbResult     = new PSBGOA(newFN(net), TraceLogger.SILENT)
                                           .runSilent(dgNodes, storageNodes, storageCap, adjGraph);
                double exactResult   = new ExactSolverNew(newFN(net))
                                           .runSilent(dgNodes, storageNodes, storageCap, adjGraph);

                boolean exactRan   = exactResult >= 0;
                boolean hybridFull = dgNodes.size() <= HybridGOA.MAX_HYBRID_DGS;
                if (!hybridFull) anyFallback = true;
                String exactStr    = exactRan   ? String.format("%.1f", exactResult) : "N/A";
                String hybridStr   = hybridFull ? String.format("%.1f", hybridResult) : String.format("%.1f*", hybridResult);
                String psbStr      = hybridFull ? String.format("%.1f", psbResult)    : String.format("%.1f*", psbResult);

                // post-hoc sanity check: no heuristic should exceed exact optimal
                if (exactRan) {
                    double bestHeuristic = Math.max(Math.max(Math.max(goaResult, densityResult),
                                                             Math.max(hybridResult, ddrResult)),
                                                    Math.max(ddrpResult, psbResult));
                    if (bestHeuristic > exactResult + 0.5) {
                        violations++;
                        System.out.printf(
                            "  [WARNING] Trial %2d: heuristic (%.1f) > exact (%.1f) " +
                            "— constraint violation!%n",
                            t, bestHeuristic, exactResult);
                    }
                }

                double bestOther = Math.max(Math.max(approxResult, hybridResult), ddrResult);

                System.out.printf(
                    "  Trial %2d [TR=%d, DGs=%d, STs=%d]: " +
                    "GOA=%.1f  Density=%.1f  Approx=%.1f  " +
                    "Hybrid=%s  DDR=%.1f  DDR+=%.1f  PSB=%s  Exact=%s%n",
                    t, trialTR, dgNodes.size(), storageNodes.size(),
                    goaResult, densityResult, approxResult,
                    hybridStr, ddrResult, ddrpResult, psbStr, exactStr);

                sumGOA     += goaResult;
                sumDensity += densityResult;
                sumApprox  += approxResult;
                sumHybrid  += hybridResult;
                sumDDR     += ddrResult;
                sumDDRP    += ddrpResult;
                sumPSB     += psbResult;
                if (exactRan) { sumExact += exactResult; exactTrials++; }

                if (densityResult > goaResult)      densityBeatGOA++;
                else if (goaResult > densityResult) goaBeatDensity++;
                else                                tied++;

                if (hybridResult > approxResult)      hybridBeatApprox++;
                else if (approxResult > hybridResult) approxBeatHybrid++;
                else                                  hybridTied++;

                if (ddrResult > approxResult)      ddrBeatApprox++;
                else if (approxResult > ddrResult) approxBeatDDR++;
                else                               ddrTied++;

                if (ddrpResult > ddrResult)      ddrpBeatDDR++;
                else if (ddrResult > ddrpResult) ddrBeatDDRP++;
                else                             ddrpTied++;

                if (psbResult > bestOther)      psbBeatAll++;
                else if (psbResult < bestOther) allBeatPSB++;
                else                            psbTied++;
            }

            // use activeTrials as denominator for averages and win/loss counts
            int D = activeTrials > 0 ? activeTrials : 1;
            System.out.printf("%n-- Results over %d active trials (%d skipped, %d total) --%n",
                              activeTrials, trials - activeTrials, trials);
            double avgExact = exactTrials > 0 ? sumExact / exactTrials : -1;
            System.out.printf("  ILP Optimal avg:      %.2f  (%d/%d active trials)%n",
                              avgExact >= 0 ? avgExact : 0, exactTrials, activeTrials);
            System.out.printf("  GOA avg:              %.2f%s%n",
                sumGOA/D, gap(sumGOA/D, avgExact));
            System.out.printf("  Density GOA avg:      %.2f%s%n",
                sumDensity/D, gap(sumDensity/D, avgExact));
            System.out.printf("  Approx GOA avg:       %.2f%s%n",
                sumApprox/D, gap(sumApprox/D, avgExact));
            System.out.printf("  Hybrid GOA avg:       %.2f%s%n",
                sumHybrid/D, gap(sumHybrid/D, avgExact));
            System.out.printf("  DDR-GOA avg:          %.2f%s%n",
                sumDDR/D, gap(sumDDR/D, avgExact));
            System.out.printf("  DDR+-GOA avg:         %.2f%s%n",
                sumDDRP/D, gap(sumDDRP/D, avgExact));
            System.out.printf("  PSB-GOA avg:          %.2f%s%n",
                sumPSB/D, gap(sumPSB/D, avgExact));
            System.out.printf("  Density beat GOA:     %d/%d active trials%n", densityBeatGOA, activeTrials);
            System.out.printf("  GOA beat Density:     %d/%d active trials%n", goaBeatDensity, activeTrials);
            System.out.printf("  Tied:                 %d/%d active trials%n", tied, activeTrials);
            System.out.printf("  Hybrid beat Approx:   %d/%d active trials%n", hybridBeatApprox, activeTrials);
            System.out.printf("  Approx beat Hybrid:   %d/%d active trials%n", approxBeatHybrid, activeTrials);
            System.out.printf("  Hybrid/Approx tied:   %d/%d active trials%n", hybridTied, activeTrials);
            System.out.printf("  DDR beat Approx:      %d/%d active trials%n", ddrBeatApprox, activeTrials);
            System.out.printf("  Approx beat DDR:      %d/%d active trials%n", approxBeatDDR, activeTrials);
            System.out.printf("  DDR/Approx tied:      %d/%d active trials%n", ddrTied, activeTrials);
            System.out.printf("  DDR+ beat DDR:        %d/%d active trials%n", ddrpBeatDDR, activeTrials);
            System.out.printf("  DDR beat DDR+:        %d/%d active trials%n", ddrBeatDDRP, activeTrials);
            System.out.printf("  DDR+/DDR tied:        %d/%d active trials%n", ddrpTied, activeTrials);
            System.out.printf("  PSB beat all others:  %d/%d active trials%n", psbBeatAll, activeTrials);
            System.out.printf("  Others beat PSB:      %d/%d active trials%n", allBeatPSB, activeTrials);
            System.out.printf("  PSB/others tied:      %d/%d active trials%n", psbTied, activeTrials);
            if (violations > 0)
                System.out.printf("  [!] Constraint violations: %d (heuristic > exact)%n", violations);
            if (anyFallback)
                System.out.printf("  [*] Hybrid/PSB marked with * fell back to DensityGOA " +
                                  "(k > MAX_HYBRID_DGS=%d)%n", HybridGOA.MAX_HYBRID_DGS);
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    // Creates a fresh FlowNetwork from a SensorStuff instance.
    // Called once per algorithm per trial so no algorithm can corrupt
    // another's cap/flow state through the shared reference.
    static FlowNetwork newFN(SensorStuff net) {
        return new FlowNetwork(net.packetSize, net.packetPriority,
                               net.nodeEnergy, net.packetsPerNode,
                               net.nodeLoc);
    }

    private static void pct(String name, double val, double opt) {
        System.out.printf("  %-20s %.1f  (%.1f%% of optimal)%n",
                          name + " priority:", val,
                          opt > 0 ? 100.0 * val / opt : 0);
    }

    private static String gap(double algoAvg, double exactAvg) {
        if (exactAvg <= 0) return "";
        return String.format("  (%.1f%% of optimal)", 100.0 * algoAvg / exactAvg);
    }
}
