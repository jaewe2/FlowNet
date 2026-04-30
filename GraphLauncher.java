import java.awt.*;
import java.util.*;
import java.util.List;
import javax.swing.*;

// =============================================================================
//  GraphLauncher — Swing visualisation wiring
//
//  Extracted from SensorStuff so that algorithm classes have no Swing imports.
//  Builds SensorNetworkGraph and BFNGraph panels from BSN state and launches
//  them in separate JFrame windows on the Event Dispatch Thread.
// =============================================================================

public class GraphLauncher {

    private final SensorStuff net;

    public GraphLauncher(SensorStuff net) {
        this.net = net;
    }

    // ── Physical BSN graph window ─────────────────────────────────────────────

    public void launchGraph(List<Integer> dgNodes,
                            List<Integer> storageNodes,
                            List<int[]>   flowEdges,
                            int[]         storageCapacity,
                            String        algoTitle) {
        int n = net.nodeLoc.length;
        Object adjGraph = net.adjM != null ? net.adjM.getAdjM() : net.adjList;

        Map<Integer, Axis> nodeMap = new LinkedHashMap<>();
        for (int i = 0; i < n; i++) {
            Axis a = new Axis();
            a.setxAxis(net.nodeLoc[i][0]);
            a.setyAxis(net.nodeLoc[i][1]);
            nodeMap.put(i + 1, a);
        }

        Map<Integer, Set<Integer>> adjMap = new LinkedHashMap<>();
        for (int i = 0; i < n; i++) adjMap.put(i + 1, new LinkedHashSet<>());
        for (int u = 0; u < n; u++)
            for (int v : SensorStuff.getAdjNodes(adjGraph, u))
                adjMap.get(u + 1).add(v + 1);

        Set<Integer> dgIds      = new LinkedHashSet<>();
        Set<Integer> storageIds = new LinkedHashSet<>();
        for (int dg : dgNodes)      dgIds.add(dg + 1);
        for (int st : storageNodes) storageIds.add(st + 1);

        List<int[]> flowEdges1 = new ArrayList<>();
        for (int[] fe : flowEdges) flowEdges1.add(new int[]{fe[0] + 1, fe[1] + 1});

        double maxX = 1, maxY = 1;
        for (double[] loc : net.nodeLoc) {
            maxX = Math.max(maxX, loc[0]);
            maxY = Math.max(maxY, loc[1]);
        }
        maxX *= 1.20; maxY *= 1.20;

        Map<Integer, Integer> priorityMap   = new LinkedHashMap<>();
        Map<Integer, Integer> packetSizeMap = new LinkedHashMap<>();
        Map<Integer, Integer> packetsMap    = new LinkedHashMap<>();
        Map<Integer, Integer> storageCapMap = new LinkedHashMap<>();
        Map<Integer, Integer> energyMap     = new LinkedHashMap<>();

        for (int i = 0; i < n; i++) energyMap.put(i + 1, net.nodeEnergy[i]);
        for (int dg : dgNodes) {
            priorityMap.put(dg + 1,   net.packetPriority[dg]);
            packetSizeMap.put(dg + 1, net.packetSize[dg]);
            packetsMap.put(dg + 1,    net.packetsPerNode[dg]);
        }
        for (int j = 0; j < storageNodes.size(); j++)
            storageCapMap.put(storageNodes.get(j) + 1, storageCapacity[j]);

        SensorNetworkGraph panel = new SensorNetworkGraph();
        panel.setGraphWidth(maxX);
        panel.setGraphHeight(maxY);
        panel.setNodes(nodeMap);
        panel.setAdjList(adjMap);
        panel.setDgNodeIds(dgIds);
        panel.setStorageNodeIds(storageIds);
        panel.setFlowEdges(flowEdges1);
        panel.setNodePriority(priorityMap);
        panel.setNodePacketSize(packetSizeMap);
        panel.setNodePackets(packetsMap);
        panel.setNodeStorageCap(storageCapMap);
        panel.setNodeEnergy(energyMap);
        panel.setAlgoTitle(algoTitle);
        panel.setConnected(net.components != null && net.components.size() == 1);
        SwingUtilities.invokeLater(panel);
    }

    // ── BFN flow network window ───────────────────────────────────────────────

    public void launchBFN(List<Integer> dgNodes,
                          List<Integer> storageNodes,
                          int[]         storageCapacity,
                          List<int[]>   flowEdges,
                          String        algoTitle) {
        launchBFN(dgNodes, storageNodes, storageCapacity, null, flowEdges, algoTitle);
    }

    public void launchBFN(List<Integer> dgNodes,
                          List<Integer> storageNodes,
                          int[]         storageCapacity,
                          int[]         finalSinkCap,
                          List<int[]>   flowEdges,
                          String        algoTitle) {
        int n = net.nodeLoc.length;
        Object adjGraph = net.adjM != null ? net.adjM.getAdjM() : net.adjList;

        int[] storageSlotsByNode = new int[n];
        for (int j = 0; j < storageNodes.size(); j++)
            storageSlotsByNode[storageNodes.get(j)] = storageCapacity[j];

        int[] wastePerStorage = new int[storageNodes.size()];
        if (finalSinkCap != null) {
            for (int j = 0; j < storageNodes.size(); j++) {
                int stOut = 2 * storageNodes.get(j) + 2;  // outNode(st)
                if (stOut < finalSinkCap.length)
                    wastePerStorage[j] = finalSinkCap[stOut];
            }
        }

        int[][] adjMatrix = new int[n][n];
        if (net.adjM != null) {
            adjMatrix = net.adjM.getAdjM();
        } else {
            for (int u = 0; u < n; u++)
                for (int v : SensorStuff.getAdjNodes(net.adjList, u))
                    adjMatrix[u][v] = 1;
        }

        BFNGraph panel = new BFNGraph();
        panel.setAlgoTitle(algoTitle);
        panel.build(n, net.nodeEnergy,
                    new HashSet<>(dgNodes), storageNodes,
                    net.nodeLoc, net.packetsPerNode, storageSlotsByNode,
                    wastePerStorage, adjMatrix, flowEdges);
        SwingUtilities.invokeLater(panel);
    }
}
