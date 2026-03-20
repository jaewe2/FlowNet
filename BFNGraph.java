import java.awt.*;
import java.awt.event.*;
import java.util.*;
import java.util.List;
import javax.swing.*;

public class BFNGraph extends JPanel implements Runnable {
    private static final long serialVersionUID = 1L;

    // ── OR-Tools inspired palette ─────────────────────────────────────────────
    private static final Color COL_BG        = new Color(248, 248, 252);
    private static final Color COL_DG        = new Color(130, 50,  200);
    private static final Color COL_DG_LIGHT  = new Color(200, 165, 240);
    private static final Color COL_ST        = new Color(20,  150, 80);
    private static final Color COL_ST_LIGHT  = new Color(120, 210, 160);
    private static final Color COL_ST_NODE   = new Color(80,  80,  100);
    private static final Color COL_EDGE_INT  = new Color(100, 60,  180);
    private static final Color COL_EDGE_RT   = new Color(170, 145, 210);
    private static final Color COL_EDGE_ST   = new Color(60,  60,  100);
    private static final Color COL_FLOW      = new Color(220, 80,  20);
    private static final Color COL_FLOW_GLOW = new Color(255, 140, 40, 80);
    private static final Color COL_TEXT      = new Color(30,  15,  60);

    private static final int NODE_R = 22;
    private static final int PAD_TOP = 90;
    private static final int PAD_BOT = 90;
    private static final int PAD_SIDE = 100;
    private static final int ROW_GAP = 72;
    private static final int IN_X = 320;
    private static final int OUT_X = 560;

    // ── data ──────────────────────────────────────────────────────────────────
    private int n;
    private int[] nodeEnergies;
    private String algoTitle = "BFN";

    private Set<Integer> dgSet = new HashSet<>();
    private Set<Integer> storageSet = new HashSet<>();
    private int[] packetsPerNode;
    private int[] storageSlots;
    private List<Integer> storageList;
    private int[][] adjMatrix;

    private List<Integer> nodeOrder = new ArrayList<>();
    private Map<Integer, Point> inPts = new LinkedHashMap<>();
    private Map<Integer, Point> outPts = new LinkedHashMap<>();
    private Point sPoint, tPoint;

    private List<int[]> edges = new ArrayList<>();
    private List<String> edgeCaps = new ArrayList<>();
    private Set<Integer> flowIdx = new HashSet<>();
    private Set<Integer> activeFlowNodes = new HashSet<>();
    private Set<Integer> relayNodes = new HashSet<>();

    // pan
    private int offsetX = 0, offsetY = 0, dragX, dragY;
    private boolean dragging = false;

    public void setAlgoTitle(String t) { algoTitle = t; }

    // ── build ─────────────────────────────────────────────────────────────────
    public void build(int n, int[] nodeEnergies,
                      Set<Integer> dgNodes, List<Integer> storageList,
                      double[][] nodeLoc,
                      int[] packetsPerNode, int[] storageSlots,
                      int[][] adjMatrix,
                      List<int[]> goaFlowBSN) {
        this.n = n;
        this.nodeEnergies = nodeEnergies;
        this.dgSet = new HashSet<>(dgNodes);
        this.storageSet = new HashSet<>(storageList);
        this.packetsPerNode = packetsPerNode;
        this.storageSlots = storageSlots;
        this.storageList = storageList;
        this.adjMatrix = adjMatrix;

        edges.clear();
        edgeCaps.clear();
        flowIdx.clear();
        activeFlowNodes.clear();
        relayNodes.clear();
        inPts.clear();
        outPts.clear();
        nodeOrder.clear();

        buildNodeOrder();
        buildEdges(dgNodes, storageList, adjMatrix, packetsPerNode,
                   storageSlots, goaFlowBSN);
    }

    private void buildNodeOrder() {
        List<Integer> dgRows = new ArrayList<>();
        List<Integer> relayRows = new ArrayList<>();
        List<Integer> stRows = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            if (dgSet.contains(i)) dgRows.add(i);
            else if (storageSet.contains(i)) stRows.add(i);
            else relayRows.add(i);
        }
        Collections.sort(dgRows);
        Collections.sort(relayRows);
        Collections.sort(stRows);
        nodeOrder.addAll(dgRows);
        nodeOrder.addAll(relayRows);
        nodeOrder.addAll(stRows);
    }

    private void buildEdges(Set<Integer> dgNodes, List<Integer> storageList,
                            int[][] adjMatrix, int[] packetsPerNode,
                            int[] storageSlots, List<int[]> goaFlowBSN) {

        for (int dg : dgNodes) {
            edges.add(new int[]{0, -1, 1, dg});
            edgeCaps.add("d=" + packetsPerNode[dg]);
        }
        for (int i = 0; i < n; i++) {
            edges.add(new int[]{1, i, 2, i});
            edgeCaps.add("E=" + nodeEnergies[i]);
        }
        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (adjMatrix[u][v] == 1) {
                    edges.add(new int[]{2, u, 1, v});
                    edgeCaps.add("inf");
                }
            }
        }
        for (int j = 0; j < storageList.size(); j++) {
            int st = storageList.get(j);
            edges.add(new int[]{2, st, 3, -1});
            edgeCaps.add("m=" + storageSlots[j] + "\nE=" + nodeEnergies[st]);
        }

        Set<String> flowSet = new HashSet<>();
        for (int[] fe : goaFlowBSN) {
            int a = Math.min(fe[0], fe[1]);
            int b = Math.max(fe[0], fe[1]);
            flowSet.add(a + "-" + b);
            activeFlowNodes.add(fe[0]);
            activeFlowNodes.add(fe[1]);
        }

        relayNodes.addAll(activeFlowNodes);
        relayNodes.removeAll(dgSet);

        for (int ei = 0; ei < edges.size(); ei++) {
            int[] e = edges.get(ei);
            if (e[0] == 2 && e[2] == 1) {
                String k = Math.min(e[1], e[3]) + "-" + Math.max(e[1], e[3]);
                if (flowSet.contains(k)) flowIdx.add(ei);
            } else if (e[0] == 1 && e[2] == 2 && activeFlowNodes.contains(e[1])) {
                flowIdx.add(ei);
            } else if (e[0] == 0 && activeFlowNodes.contains(e[3])) {
                flowIdx.add(ei);
            } else if (e[2] == 3 && activeFlowNodes.contains(e[1]) && storageSet.contains(e[1])) {
                flowIdx.add(ei);
            }
        }
    }

    // ── layout ────────────────────────────────────────────────────────────────
    private void computeLayout(int W, int H) {
        if (n == 0) return;

        int sourceX = PAD_SIDE + 30;
        int inX = Math.max(IN_X, W / 4);
        int outX = Math.max(OUT_X, W / 2 + 40);
        int sinkX = W - PAD_SIDE - 30;

        int totalH = (Math.max(1, n) - 1) * ROW_GAP;
        int startY = Math.max(PAD_TOP, (H - totalH) / 2);

        for (int row = 0; row < nodeOrder.size(); row++) {
            int node = nodeOrder.get(row);
            int y = startY + row * ROW_GAP;
            inPts.put(node, new Point(inX, y));
            outPts.put(node, new Point(outX, y));
        }

        int midY = startY + totalH / 2;
        sPoint = new Point(sourceX, midY);
        tPoint = new Point(sinkX, midY);
    }

    private Point px(int type, int id) {
        Point b;
        switch (type) {
            case 0:  b = sPoint; break;
            case 3:  b = tPoint; break;
            case 1:  b = inPts.get(id); break;
            default: b = outPts.get(id); break;
        }
        return new Point(b.x + offsetX, b.y + offsetY);
    }

    // ── paint ─────────────────────────────────────────────────────────────────
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                            RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,
                            RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

        int W = getWidth(), H = getHeight();
        computeLayout(W, H);

        g2.setColor(COL_BG);
        g2.fillRect(0, 0, W, H);
        g2.setColor(new Color(245, 242, 252));
        g2.fillRoundRect(30, 30, W - 60, H - 60, 20, 20);
        g2.setColor(new Color(180, 160, 210));
        g2.setStroke(new BasicStroke(1.2f));
        g2.drawRoundRect(30, 30, W - 60, H - 60, 20, 20);

        if (n == 0) return;
        Stroke def = g2.getStroke();
        List<LabelDraw> deferredLabels = new ArrayList<>();

        for (int ei = 0; ei < edges.size(); ei++) {
            int[] e = edges.get(ei);
            Point p1 = px(e[0], e[1]);
            Point p2 = px(e[2], e[3]);
            String cap = edgeCaps.get(ei);
            boolean isFlow = flowIdx.contains(ei);
            boolean isInternal = e[0] == 1 && e[2] == 2 && e[1] == e[3];
            boolean isInf = "inf".equals(cap);
            boolean isS = e[0] == 0;
            boolean isT = e[2] == 3;

            if (isFlow) {
                g2.setColor(COL_FLOW_GLOW);
                g2.setStroke(new BasicStroke(16f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                drawArrow(g2, p1, p2, false, isInf, ei);
                g2.setColor(COL_FLOW);
                g2.setStroke(new BasicStroke(4f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            } else if (isInternal) {
                g2.setColor(COL_EDGE_INT);
                g2.setStroke(new BasicStroke(2f));
            } else if (isS || isT) {
                g2.setColor(COL_EDGE_ST);
                g2.setStroke(new BasicStroke(1.8f));
            } else if (isInf) {
                g2.setColor(COL_EDGE_RT);
                g2.setStroke(new BasicStroke(1f, BasicStroke.CAP_BUTT,
                    BasicStroke.JOIN_MITER, 1f, new float[]{6, 4}, 0));
            } else {
                g2.setColor(COL_EDGE_ST);
                g2.setStroke(new BasicStroke(1.5f));
            }
            drawArrow(g2, p1, p2, true, isInf, ei);
            g2.setStroke(def);

            int rowRef = isS ? nodeOrder.indexOf(e[3]) : nodeOrder.indexOf(e[1]);
            if (rowRef < 0) rowRef = 0;

            if (isInf) {
                // show only one inf label per reciprocal pair
                if (e[1] < e[3]) {
                    Point lp = getRoutingLabelPoint(p1, p2, ei, rowRef);
                    deferredLabels.add(new LabelDraw("inf", lp.x, lp.y, Font.PLAIN, COL_TEXT));
                }
            } else {
                Point lp = getLabelPoint(p1, p2, isInternal, isS, isT, rowRef, e);
                deferredLabels.add(new LabelDraw(cap, lp.x, lp.y,
                    isFlow ? Font.BOLD : Font.PLAIN,
                    isFlow ? COL_FLOW : isInternal ? COL_EDGE_INT : COL_TEXT));
            }
        }

        drawColumnHeaders(g2);
        drawNode(g2, px(0, -1), "s", COL_ST_NODE, COL_ST_NODE.darker());
        drawNode(g2, px(3, -1), "t", COL_ST_NODE, COL_ST_NODE.darker());

        for (int i = 0; i < n; i++) {
            boolean isDG = dgSet.contains(i);
            boolean isRelay = relayNodes.contains(i);
            boolean isStorage = storageSet.contains(i);
            Color fill = isDG ? COL_DG : isStorage ? COL_ST : COL_ST_NODE;
            Color light = isDG ? COL_DG_LIGHT : isStorage ? COL_ST_LIGHT : new Color(180, 180, 195);
            Point pi = px(1, i), po = px(2, i);

            if (isRelay) {
                g2.setColor(new Color(255, 170, 70, 55));
                g2.fillOval(pi.x - NODE_R - 8, pi.y - NODE_R - 8, NODE_R * 2 + 16, NODE_R * 2 + 16);
                g2.fillOval(po.x - NODE_R - 8, po.y - NODE_R - 8, NODE_R * 2 + 16, NODE_R * 2 + 16);
                g2.setColor(new Color(230, 120, 20, 160));
                g2.setStroke(new BasicStroke(2f));
                g2.drawOval(pi.x - NODE_R - 8, pi.y - NODE_R - 8, NODE_R * 2 + 16, NODE_R * 2 + 16);
                g2.drawOval(po.x - NODE_R - 8, po.y - NODE_R - 8, NODE_R * 2 + 16, NODE_R * 2 + 16);
                g2.setStroke(def);
            }

            drawNode(g2, pi, (i + 1) + "'", fill, light);
            drawNode(g2, po, (i + 1) + "\"", fill, light);
        }

        for (LabelDraw label : deferredLabels) {
            drawEdgeLabel(g2, label);
        }

        drawLegend(g2, def, W, H);
    }

    private void drawColumnHeaders(Graphics2D g2) {
        g2.setColor(new Color(110, 95, 145));
        g2.setFont(new Font("SansSerif", Font.BOLD, 14));
        FontMetrics fm = g2.getFontMetrics();
        String[] labels = {"Super Source", "In-Nodes", "Out-Nodes", "Super Sink"};
        int[] xs = {sPoint.x, inPts.get(nodeOrder.get(0)).x, outPts.get(nodeOrder.get(0)).x, tPoint.x};
        for (int i = 0; i < labels.length; i++) {
            g2.drawString(labels[i], xs[i] - fm.stringWidth(labels[i]) / 2, 55 + offsetY);
        }
    }

    private Point getLabelPoint(Point p1, Point p2, boolean isInternal, boolean isS,
                                boolean isT, int rowRef, int[] edge) {
        int mx = (p1.x + p2.x) / 2;
        int my = (p1.y + p2.y) / 2;
        if (isInternal) {
            return new Point(mx, my - 22);
        }
        if (isS) {
            return new Point(mx - 8, my - 14 - (rowRef % 2) * 10);
        }
        if (isT) {
            int idx = storageList.indexOf(edge[1]);
            return new Point(mx + 6, my - 16 + (idx % 3) * 14);
        }
        return new Point(mx, my - 14);
    }

    private Point getRoutingLabelPoint(Point p1, Point p2, int edgeIndex, int rowRef) {
        int mx = (p1.x + p2.x) / 2;
        int my = (p1.y + p2.y) / 2;
        int side = (edgeIndex % 2 == 0) ? 1 : -1;
        int laneX = mx + side * 26;
        int laneY = my + ((rowRef % 3) - 1) * 10;
        return new Point(laneX, laneY);
    }

    private static class LabelDraw {
        String text;
        int x;
        int y;
        int fontStyle;
        Color textColor;

        LabelDraw(String text, int x, int y, int fontStyle, Color textColor) {
            this.text = text;
            this.x = x;
            this.y = y;
            this.fontStyle = fontStyle;
            this.textColor = textColor;
        }
    }

    private void drawEdgeLabel(Graphics2D g2, LabelDraw label) {
        g2.setFont(new Font("SansSerif", label.fontStyle, 10));
        FontMetrics fm = g2.getFontMetrics();
        String[] lines = label.text.split("\\n");

        int maxW = 0;
        for (String line : lines) maxW = Math.max(maxW, fm.stringWidth(line));

        int lineH = fm.getHeight();
        int padX = 6;
        int padY = 4;
        int boxW = maxW + padX * 2;
        int boxH = lines.length * lineH + padY * 2;
        int boxX = label.x - boxW / 2;
        int boxY = label.y - boxH / 2;

        g2.setColor(new Color(255, 255, 255, 245));
        g2.fillRoundRect(boxX, boxY, boxW, boxH, 9, 9);
        g2.setColor(new Color(185, 176, 206));
        g2.setStroke(new BasicStroke(1.1f));
        g2.drawRoundRect(boxX, boxY, boxW, boxH, 9, 9);

        g2.setColor(label.textColor);
        int y = boxY + padY + fm.getAscent();
        for (String line : lines) {
            int tw = fm.stringWidth(line);
            g2.drawString(line, label.x - tw / 2, y);
            y += lineH;
        }
    }

    private void drawNode(Graphics2D g2, Point p, String lbl,
                          Color fill, Color light) {
        g2.setColor(new Color(0, 0, 0, 30));
        g2.fillOval(p.x - NODE_R + 2, p.y - NODE_R + 3, NODE_R * 2, NODE_R * 2);

        RadialGradientPaint rgp = new RadialGradientPaint(
            new Point(p.x - NODE_R / 3, p.y - NODE_R / 3), NODE_R * 2,
            new float[]{0f, 1f}, new Color[]{light, fill});
        g2.setPaint(rgp);
        g2.fillOval(p.x - NODE_R, p.y - NODE_R, NODE_R * 2, NODE_R * 2);
        g2.setPaint(null);

        g2.setColor(fill.darker());
        g2.setStroke(new BasicStroke(1.8f));
        g2.drawOval(p.x - NODE_R, p.y - NODE_R, NODE_R * 2, NODE_R * 2);

        g2.setColor(Color.WHITE);
        g2.setFont(new Font("SansSerif", Font.BOLD, 12));
        FontMetrics fm = g2.getFontMetrics();
        g2.drawString(lbl, p.x - fm.stringWidth(lbl) / 2, p.y + fm.getAscent() / 2 - 1);
    }

    private void drawArrow(Graphics2D g2, Point p1, Point p2,
                           boolean arrowHead, boolean isRouting, int edgeIndex) {
        double dx = p2.x - p1.x, dy = p2.y - p1.y;
        double len = Math.sqrt(dx * dx + dy * dy);
        if (len < 2) return;
        double ux = dx / len, uy = dy / len;
        int x1 = (int) (p1.x + ux * NODE_R), y1 = (int) (p1.y + uy * NODE_R);
        int x2 = (int) (p2.x - ux * (NODE_R + 4)), y2 = (int) (p2.y - uy * (NODE_R + 4));
        if (x1 == x2 && y1 == y2) return;

        if (isRouting) {
            int bend = (edgeIndex % 2 == 0) ? 18 : -18;
            int ctrlX = (x1 + x2) / 2;
            int ctrlY = (y1 + y2) / 2 + bend;
            g2.draw(new java.awt.geom.QuadCurve2D.Double(x1, y1, ctrlX, ctrlY, x2, y2));
        } else {
            g2.drawLine(x1, y1, x2, y2);
        }

        if (arrowHead) {
            int aw = 9, ah = 5;
            g2.fillPolygon(
                new int[]{x2, (int) (x2 - aw * ux + ah * uy), (int) (x2 - aw * ux - ah * uy)},
                new int[]{y2, (int) (y2 - aw * uy - ah * ux), (int) (y2 - aw * uy + ah * ux)}, 3);
        }
    }

    private void drawLegend(Graphics2D g2, Stroke def, int W, int H) {
        String[] lbl = {
            "DG (source) - i', i\"",
            "Storage (sink) - i', i\"",
            "Relay node used by flow",
            "s / t  super source / sink",
            "i' -> i\"  energy edge (Eᵢ)",
            "BSN routing edge (inf cap)",
            "GOA flow path"
        };
        Color[] col = { COL_DG, COL_ST, new Color(230, 120, 20), COL_ST_NODE,
                        COL_EDGE_INT, COL_EDGE_RT, COL_FLOW };

        g2.setFont(new Font("SansSerif", Font.PLAIN, 12));
        FontMetrics fm = g2.getFontMetrics();
        int maxW = 0;
        for (String s : lbl) maxW = Math.max(maxW, fm.stringWidth(s));
        int bw = 16, bh = 16, gap = 8, px = 12, py = 10;
        int boxW = maxW + bw + px * 2 + 10, boxH = lbl.length * (bh + gap) + py * 2;
        int lx = W - boxW - 12, ly = H - boxH - 12;

        g2.setColor(new Color(0, 0, 0, 30));
        g2.fillRoundRect(lx + 3, ly + 3, boxW, boxH, 12, 12);
        g2.setColor(new Color(255, 255, 255, 245));
        g2.fillRoundRect(lx, ly, boxW, boxH, 12, 12);
        g2.setColor(new Color(180, 160, 210));
        g2.setStroke(new BasicStroke(1.2f));
        g2.drawRoundRect(lx, ly, boxW, boxH, 12, 12);

        for (int i = 0; i < lbl.length; i++) {
            int cy = ly + py + i * (bh + gap), ix = lx + px;
            if (i < 2 || i == 3) {
                RadialGradientPaint rgp = new RadialGradientPaint(
                    new Point(ix + bw / 2 - 2, cy + bh / 2 - 2), bw,
                    new float[]{0f, 1f}, new Color[]{col[i].brighter(), col[i]});
                g2.setPaint(rgp);
                g2.fillOval(ix, cy, bw, bh);
                g2.setPaint(null);
                g2.setColor(col[i].darker());
                g2.setStroke(new BasicStroke(1f));
                g2.drawOval(ix, cy, bw, bh);
            } else if (i == 2) {
                g2.setColor(new Color(255, 170, 70, 90));
                g2.fillOval(ix - 2, cy - 2, bw + 4, bh + 4);
                g2.setColor(col[i]);
                g2.setStroke(new BasicStroke(2f));
                g2.drawOval(ix - 2, cy - 2, bw + 4, bh + 4);
            } else if (i == 4) {
                g2.setColor(col[i]);
                g2.setStroke(new BasicStroke(2f));
                g2.drawLine(ix, cy + bh / 2, ix + bw, cy + bh / 2);
                g2.fillPolygon(new int[]{ix + bw, ix + bw - 6, ix + bw - 6},
                               new int[]{cy + bh / 2, cy + bh / 2 - 3, cy + bh / 2 + 3}, 3);
                g2.setStroke(def);
            } else if (i == 5) {
                g2.setColor(col[i]);
                g2.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_BUTT,
                    BasicStroke.JOIN_MITER, 1f, new float[]{5, 4}, 0));
                g2.drawArc(ix, cy, bw, bh, 20, 140);
                g2.setStroke(def);
            } else {
                g2.setColor(COL_FLOW_GLOW);
                g2.setStroke(new BasicStroke(8f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                g2.drawLine(ix, cy + bh / 2, ix + bw, cy + bh / 2);
                g2.setColor(col[i]);
                g2.setStroke(new BasicStroke(2.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                g2.drawLine(ix, cy + bh / 2, ix + bw, cy + bh / 2);
                g2.setStroke(def);
            }
            g2.setFont(new Font("SansSerif", Font.PLAIN, 12));
            g2.setColor(COL_TEXT);
            g2.drawString(lbl[i], ix + bw + 8, cy + bh - 2);
        }
    }

    private void attachMouseListeners() {
        addMouseListener(new MouseAdapter() {
            @Override public void mousePressed(MouseEvent e) {
                dragging = true;
                dragX = e.getX() - offsetX;
                dragY = e.getY() - offsetY;
                setCursor(Cursor.getPredefinedCursor(Cursor.MOVE_CURSOR));
            }
            @Override public void mouseReleased(MouseEvent e) {
                dragging = false;
                setCursor(Cursor.getDefaultCursor());
            }
        });
        addMouseMotionListener(new MouseMotionAdapter() {
            @Override public void mouseDragged(MouseEvent e) {
                if (dragging) {
                    offsetX = e.getX() - dragX;
                    offsetY = e.getY() - dragY;
                    repaint();
                }
            }
        });
    }

    @Override
    public void run() {
        attachMouseListeners();
        JFrame frame = new JFrame("BFN Flow Network - " + algoTitle);
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);

        int prefH = Math.max(760, n * ROW_GAP + PAD_TOP + PAD_BOT + 120);
        frame.setPreferredSize(new Dimension(1500, prefH));

        JLabel banner = new JLabel("BFN: " + algoTitle, SwingConstants.CENTER);
        banner.setOpaque(true);
        banner.setBackground(new Color(116, 72, 168));
        banner.setForeground(Color.WHITE);
        banner.setFont(new Font("SansSerif", Font.BOLD, 18));
        banner.setBorder(BorderFactory.createEmptyBorder(8, 10, 8, 10));

        frame.setLayout(new BorderLayout());
        frame.add(banner, BorderLayout.NORTH);
        frame.add(this, BorderLayout.CENTER);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
