import java.awt.*;
import java.awt.event.*;
import java.awt.geom.RoundRectangle2D;
import java.util.*;
import java.util.List;
import javax.swing.*;

public class SensorNetworkGraph extends JPanel implements Runnable {
    private static final long serialVersionUID = 1L;

    private Map<Integer, Axis> nodes;
    private double graphWidth;
    private double graphHeight;
    private int    scaling   = 70;   // more left/bottom margin for axis labels
    private int    legendW   = 280;  // reserved width on right for legend
    private int    nodeW     = 84;
    private int    nodeH     = 76;
    private int    gridCount = 10;
    private boolean connected;
    private Map<Integer, Set<Integer>> adjList;
    private String algoTitle = "GOA";

    // ── GOA display data ──────────────────────────────────────────────────────
    private Set<Integer> dgNodeIds      = new HashSet<>();
    private Set<Integer> storageNodeIds = new HashSet<>();
    private List<int[]>  flowEdges      = new ArrayList<>();

    // ── per-node info ─────────────────────────────────────────────────────────
    private Map<Integer, Integer> nodePriority   = new HashMap<>();
    private Map<Integer, Integer> nodePacketSize = new HashMap<>();
    private Map<Integer, Integer> nodePackets    = new HashMap<>();
    private Map<Integer, Integer> nodeStorageCap = new HashMap<>();

    // ── drag state ────────────────────────────────────────────────────────────
    private Map<Integer, Point> nodePixels     = new HashMap<>();
    private int     dragNodeId  = -1;
    private int     dragOffsetX = 0;
    private int     dragOffsetY = 0;
    private boolean pixelsInit  = false;

    // ── setters ───────────────────────────────────────────────────────────────
    public void setDgNodeIds(Set<Integer> ids)             { dgNodeIds      = ids; repaint(); }
    public void setStorageNodeIds(Set<Integer> ids)        { storageNodeIds = ids; repaint(); }
    public void setFlowEdges(List<int[]> edges)            { flowEdges      = edges; repaint(); }
    public void setNodePriority(Map<Integer, Integer> m)   { nodePriority   = m; repaint(); }
    public void setNodePacketSize(Map<Integer, Integer> m) { nodePacketSize = m; repaint(); }
    public void setNodePackets(Map<Integer, Integer> m)    { nodePackets    = m; repaint(); }
    public void setNodeStorageCap(Map<Integer, Integer> m) { nodeStorageCap = m; repaint(); }
    public void setAlgoTitle(String t)                     { algoTitle = t; }
    public boolean isConnected()                           { return connected; }
    public void setConnected(boolean c)                    { connected = c; }
    public Map<Integer, Set<Integer>> getAdjList()         { return adjList; }
    public void setAdjList(Map<Integer, Set<Integer>> a)   { adjList = a; }
    public void setNodes(Map<Integer, Axis> n)             { nodes = n; pixelsInit = false; invalidate(); repaint(); }
    public Map<Integer, Axis> getNodes()                   { return nodes; }
    public double getGraphWidth()                          { return graphWidth; }
    public void   setGraphWidth(double w)                  { graphWidth = w; }
    public double getGraphHeight()                         { return graphHeight; }
    public void   setGraphHeight(double h)                 { graphHeight = h; }

    // ── graph drawing area (excludes legend strip on right) ───────────────────
    private int graphAreaWidth()  { return getWidth() - legendW - scaling; }
    private int graphAreaHeight() { return getHeight() - 2*scaling - 30; }

    // ── init pixel positions ──────────────────────────────────────────────────
    private void initPixels() {
        if (getWidth() == 0 || getHeight() == 0) return;
        double xScale = graphAreaWidth()  / graphWidth;
        double yScale = graphAreaHeight() / graphHeight;
        nodePixels.clear();
        for (Integer key : nodes.keySet()) {
            int px = (int)(nodes.get(key).getxAxis() * xScale) + scaling;
            int py = (int)((graphHeight - nodes.get(key).getyAxis()) * yScale) + scaling;
            nodePixels.put(key, new Point(px, py));
        }
        spreadOverlappingNodes();
        pixelsInit = true;
    }

    /**
     * Simple force-based spread: repeatedly push nodes apart if they overlap,
     * until no two nodes are closer than (nodeW + 10) horizontally and
     * (nodeH + 10) vertically.
     */
    private void spreadOverlappingNodes() {
        int minDX = nodeW  + 12;
        int minDY = nodeH  + 12;

        // strict bounds: node centre must stay this far from graph edges
        int xMin = scaling + nodeW/2 + 4;
        int xMax = scaling + graphAreaWidth()  - nodeW/2 - 4;
        int yMin = scaling + nodeH/2 + 4;
        int yMax = scaling + graphAreaHeight() - nodeH/2 - 4;

        List<Integer> keys = new ArrayList<>(nodePixels.keySet());

        // clamp first so we start inside
        for (Point p : nodePixels.values()) {
            p.x = Math.max(xMin, Math.min(xMax, p.x));
            p.y = Math.max(yMin, Math.min(yMax, p.y));
        }

        for (int iter = 0; iter < 300; iter++) {
            boolean moved = false;
            for (int i = 0; i < keys.size(); i++) {
                for (int j = i+1; j < keys.size(); j++) {
                    Point a = nodePixels.get(keys.get(i));
                    Point b = nodePixels.get(keys.get(j));
                    int dx = b.x - a.x, dy = b.y - a.y;
                    int overX = minDX - Math.abs(dx);
                    int overY = minDY - Math.abs(dy);
                    if (overX > 0 && overY > 0) {
                        if (overX < overY) {
                            int push = overX/2 + 1;
                            a.x -= (dx>=0 ? push : -push);
                            b.x += (dx>=0 ? push : -push);
                        } else {
                            int push = overY/2 + 1;
                            a.y -= (dy>=0 ? push : -push);
                            b.y += (dy>=0 ? push : -push);
                        }
                        // clamp immediately after each push
                        a.x = Math.max(xMin, Math.min(xMax, a.x));
                        a.y = Math.max(yMin, Math.min(yMax, a.y));
                        b.x = Math.max(xMin, Math.min(xMax, b.x));
                        b.y = Math.max(yMin, Math.min(yMax, b.y));
                        moved = true;
                    }
                }
            }
            if (!moved) break;
        }
    }

    // ── hit-test ──────────────────────────────────────────────────────────────
    private int hitNode(int mx, int my) {
        for (Map.Entry<Integer, Point> e : nodePixels.entrySet()) {
            Point p = e.getValue();
            if (mx >= p.x-nodeW/2 && mx <= p.x+nodeW/2 &&
                my >= p.y-nodeH/2 && my <= p.y+nodeH/2)
                return e.getKey();
        }
        return -1;
    }

    // ── mouse listeners ───────────────────────────────────────────────────────
    private void attachMouseListeners() {
        addMouseListener(new MouseAdapter() {
            @Override public void mousePressed(MouseEvent e) {
                if (!pixelsInit) initPixels();
                dragNodeId = hitNode(e.getX(), e.getY());
                if (dragNodeId != -1) {
                    Point p = nodePixels.get(dragNodeId);
                    dragOffsetX = e.getX() - p.x;
                    dragOffsetY = e.getY() - p.y;
                }
            }
            @Override public void mouseReleased(MouseEvent e) { dragNodeId = -1; }
        });
        addMouseMotionListener(new MouseMotionAdapter() {
            @Override public void mouseDragged(MouseEvent e) {
                if (dragNodeId != -1) {
                    int xMin = scaling + nodeW/2 + 4;
                    int xMax = scaling + graphAreaWidth()  - nodeW/2 - 4;
                    int yMin = scaling + nodeH/2 + 4;
                    int yMax = scaling + graphAreaHeight() - nodeH/2 - 4;
                    int nx = Math.max(xMin, Math.min(xMax, e.getX()-dragOffsetX));
                    int ny = Math.max(yMin, Math.min(yMax, e.getY()-dragOffsetY));
                    nodePixels.get(dragNodeId).setLocation(nx, ny);
                    repaint();
                }
            }
            @Override public void mouseMoved(MouseEvent e) {
                if (!pixelsInit) return;
                setCursor(hitNode(e.getX(), e.getY()) != -1
                    ? Cursor.getPredefinedCursor(Cursor.HAND_CURSOR)
                    : Cursor.getDefaultCursor());
            }
        });
    }

    // ── painting ──────────────────────────────────────────────────────────────
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        if (!pixelsInit) initPixels();
        if (nodePixels.isEmpty()) return;

        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,      RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

        int gaw = graphAreaWidth();
        int gah = graphAreaHeight();

        // ── full background ───────────────────────────────────────────────────
        g2.setColor(new Color(240,242,246));
        g2.fillRect(0, 0, getWidth(), getHeight());

        // ── graph background ──────────────────────────────────────────────────
        g2.setColor(Color.WHITE);
        g2.fillRect(scaling, scaling, gaw, gah);
        g2.setColor(new Color(200,200,200));
        g2.setStroke(new BasicStroke(1f));
        g2.drawRect(scaling, scaling, gaw, gah);

        // ── grid lines ────────────────────────────────────────────────────────
        g2.setFont(new Font("SansSerif", Font.PLAIN, 11));
        for (int i = 0; i <= gridCount; i++) {
            // horizontal
            int y = scaling + i*gah/gridCount;
            g2.setColor(new Color(220,220,220));
            g2.drawLine(scaling, y, scaling+gaw, y);
            g2.setColor(new Color(80,80,80));
            double val = graphHeight*(1.0 - (double)i/gridCount);
            String lbl = String.format("%.1f", val);
            FontMetrics fm = g2.getFontMetrics();
            g2.drawString(lbl, scaling-fm.stringWidth(lbl)-5, y+fm.getAscent()/2);

            // vertical
            int x = scaling + i*gaw/gridCount;
            g2.setColor(new Color(220,220,220));
            g2.drawLine(x, scaling, x, scaling+gah);
            g2.setColor(new Color(80,80,80));
            double xval = graphWidth*(double)i/gridCount;
            String xlbl = String.format("%.1f", xval);
            FontMetrics fmx = g2.getFontMetrics();
            g2.drawString(xlbl, x-fmx.stringWidth(xlbl)/2, scaling+gah+fmx.getAscent()+4);
        }

        Stroke def = g2.getStroke();

        // ── regular edges (thin gray, semi-transparent) ───────────────────────
        g2.setColor(new Color(160,160,160,140));
        g2.setStroke(new BasicStroke(1.5f));
        Set<String> drawnEdges = new HashSet<>();
        for (int node : adjList.keySet()) {
            if (adjList.get(node)==null) continue;
            for (int adj : adjList.get(node)) {
                String key = Math.min(node,adj)+"-"+Math.max(node,adj);
                if (drawnEdges.contains(key)) continue;
                drawnEdges.add(key);
                Point p1 = nodePixels.get(node), p2 = nodePixels.get(adj);
                if (p1==null||p2==null) continue;
                g2.drawLine(p1.x, p1.y, p2.x, p2.y);
            }
        }

        // ── flow edges (thick orange solid, drawn on top) ─────────────────────
        g2.setStroke(new BasicStroke(4f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        for (int[] fe : flowEdges) {
            Point p1 = nodePixels.get(fe[0]), p2 = nodePixels.get(fe[1]);
            if (p1==null||p2==null) continue;
            // glow effect: draw wider semi-transparent line first
            g2.setColor(new Color(255,165,0,60));
            g2.setStroke(new BasicStroke(10f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g2.drawLine(p1.x, p1.y, p2.x, p2.y);
            // solid line
            g2.setColor(new Color(230,100,0));
            g2.setStroke(new BasicStroke(3.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g2.drawLine(p1.x, p1.y, p2.x, p2.y);
        }
        g2.setStroke(def);

        // ── nodes ─────────────────────────────────────────────────────────────
        List<Integer> keyList = new ArrayList<>(nodes.keySet());
        for (int nodeId : keyList) {
            Point p = nodePixels.get(nodeId);
            if (p==null) continue;
            int cx=p.x, cy=p.y, bx=cx-nodeW/2, by=cy-nodeH/2;
            boolean isDG = dgNodeIds.contains(nodeId);
            boolean isST = storageNodeIds.contains(nodeId);

            Color base = isDG ? new Color(79,130,220) : isST ? new Color(46,162,100) : Color.RED;
            Color top  = isDG ? new Color(130,175,255): isST ? new Color(90,210,145) : Color.PINK;

            // shadow
            g2.setColor(new Color(0,0,0,50));
            g2.fillRoundRect(bx+4, by+5, nodeW, nodeH, 14, 14);

            // gradient fill
            GradientPaint gp = new GradientPaint(cx, by, top, cx, by+nodeH, base);
            g2.setPaint(gp);
            g2.fillRoundRect(bx, by, nodeW, nodeH, 14, 14);
            g2.setPaint(null);

            // border
            g2.setColor(dragNodeId==nodeId ? Color.YELLOW : base.darker());
            g2.setStroke(new BasicStroke(dragNodeId==nodeId ? 2.5f : 1.5f));
            g2.drawRoundRect(bx, by, nodeW, nodeH, 14, 14);
            g2.setStroke(def);

            // header bar
            g2.setColor(new Color(0,0,0,40));
            g2.fillRoundRect(bx, by, nodeW, 20, 14, 14);
            g2.fillRect(bx, by+10, nodeW, 10);

            // text
            g2.setColor(Color.WHITE);
            String prefix = isDG ? "DG" : isST ? "ST" : "N";
            g2.setFont(new Font("SansSerif", Font.BOLD, 12));
            drawCentered(g2, prefix+" "+nodeId, cx, by+15);

            g2.setColor(new Color(255,255,255,200));
            g2.drawLine(bx+6, by+21, bx+nodeW-6, by+21);

            g2.setColor(Color.WHITE);
            g2.setFont(new Font("SansSerif", Font.PLAIN, 11));
            if (isDG) {
                int ly = by+33;
                if (nodePriority.containsKey(nodeId))   { drawCentered(g2,"v="+nodePriority.get(nodeId),cx,ly);   ly+=13; }
                if (nodePacketSize.containsKey(nodeId))  { drawCentered(g2,"sz="+nodePacketSize.get(nodeId),cx,ly); ly+=13; }
                if (nodePackets.containsKey(nodeId))     { drawCentered(g2,"d="+nodePackets.get(nodeId),cx,ly);     ly+=13; }
                if (nodes.containsKey(nodeId)) {
                    g2.setFont(new Font("SansSerif", Font.PLAIN, 9));
                    g2.setColor(new Color(255,255,255,180));
                    drawCentered(g2, String.format("(%.1f, %.1f)",
                        nodes.get(nodeId).getxAxis(), nodes.get(nodeId).getyAxis()), cx, ly);
                }
            } else if (isST) {
                int ly = by+38;
                if (nodeStorageCap.containsKey(nodeId)) { drawCentered(g2,"cap="+nodeStorageCap.get(nodeId),cx,ly); ly+=13; }
                if (nodes.containsKey(nodeId)) {
                    g2.setFont(new Font("SansSerif", Font.PLAIN, 9));
                    g2.setColor(new Color(255,255,255,180));
                    drawCentered(g2, String.format("(%.1f, %.1f)",
                        nodes.get(nodeId).getxAxis(), nodes.get(nodeId).getyAxis()), cx, ly);
                }
            }
        }

        // ── axis labels ───────────────────────────────────────────────────────
        g2.setColor(new Color(50,50,50));
        g2.setFont(new Font("SansSerif", Font.BOLD, 13));
        FontMetrics fmAx = g2.getFontMetrics();
        String xLbl = "X Position (m)";
        g2.drawString(xLbl, scaling+gaw/2-fmAx.stringWidth(xLbl)/2, getHeight()-8);

        Graphics2D g2r = (Graphics2D) g2.create();
        g2r.setFont(new Font("SansSerif", Font.BOLD, 13));
        g2r.rotate(-Math.PI/2);
        String yLbl = "Y Position (m)";
        FontMetrics fmY = g2r.getFontMetrics();
        g2r.drawString(yLbl, -(scaling+gah/2+fmY.stringWidth(yLbl)/2), 16);
        g2r.dispose();

        // ── legend (outside graph, right side) ────────────────────────────────
        drawLegend(g2, def);
    }

    private void drawCentered(Graphics2D g2, String s, int cx, int y) {
        FontMetrics fm = g2.getFontMetrics();
        g2.drawString(s, cx-fm.stringWidth(s)/2, y);
    }

    private void drawLegend(Graphics2D g2, Stroke def) {
        int bw=16, bh=16, gap=10, padX=14, padY=12;
        String[] labels = {
            "DG (source)  - v, sz, d, (x,y)",
            "Storage (sink)  - cap, (x,y)",
            "GOA flow path",
            "Drag nodes to rearrange"
        };
        Color[] colors = {
            new Color(79,130,220), new Color(46,162,100),
            new Color(230,100,0),  new Color(80,80,80)
        };

        g2.setFont(new Font("SansSerif", Font.PLAIN, 13));
        FontMetrics fm = g2.getFontMetrics();
        int maxW = 0;
        for (String s : labels) maxW = Math.max(maxW, fm.stringWidth(s));
        int boxW = maxW + bw + padX*2 + 10;
        int boxH = labels.length*(bh+gap) + padY*2;

        // position: right strip, vertically centered
        int rx = getWidth() - legendW/2 - boxW/2;
        int ry = (getHeight()-boxH)/2;

        // shadow
        g2.setColor(new Color(0,0,0,40));
        g2.fillRoundRect(rx+3, ry+3, boxW, boxH, 12, 12);
        // white background
        g2.setColor(new Color(255,255,255,250));
        g2.fillRoundRect(rx, ry, boxW, boxH, 12, 12);
        // border
        g2.setColor(new Color(180,180,180));
        g2.setStroke(new BasicStroke(1.2f));
        g2.drawRoundRect(rx, ry, boxW, boxH, 12, 12);

        for (int i = 0; i < labels.length; i++) {
            int cy = ry+padY+i*(bh+gap);
            int ix = rx+padX;

            if (i==0||i==1) {
                g2.setColor(colors[i]);
                g2.fillRoundRect(ix, cy, bw, bh, 4, 4);
                g2.setColor(colors[i].darker());
                g2.setStroke(new BasicStroke(1f));
                g2.drawRoundRect(ix, cy, bw, bh, 4, 4);
            } else if (i==2) {
                // glow swatch
                g2.setColor(new Color(255,165,0,80));
                g2.setStroke(new BasicStroke(8f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                g2.drawLine(ix, cy+bh/2, ix+bw, cy+bh/2);
                g2.setColor(colors[i]);
                g2.setStroke(new BasicStroke(3f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                g2.drawLine(ix, cy+bh/2, ix+bw, cy+bh/2);
                g2.setStroke(def);
            } else {
                g2.setFont(new Font("SansSerif", Font.PLAIN, 15));
                g2.setColor(new Color(80,80,80));
                g2.drawString("\u2725", ix, cy+bh-1);
                g2.setFont(new Font("SansSerif", Font.PLAIN, 13));
            }

            g2.setFont(new Font("SansSerif", Font.PLAIN, 13));
            g2.setColor(new Color(30,30,30));
            g2.drawString(labels[i], ix+bw+8, cy+bh-2);
        }
    }

    // ── Runnable ──────────────────────────────────────────────────────────────
    @Override
    public void run() {
        attachMouseListeners();
        JFrame frame = new JFrame("Sensor Network Graph - "+algoTitle);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setPreferredSize(new Dimension(1600, 950));

        JLabel banner = new JLabel(algoTitle, SwingConstants.CENTER);
        banner.setFont(new Font("SansSerif", Font.BOLD, 18));
        banner.setOpaque(true);
        banner.setBackground(new Color(30,30,30));
        banner.setForeground(Color.WHITE);
        banner.setBorder(BorderFactory.createEmptyBorder(10,0,10,0));

        frame.getContentPane().setLayout(new BorderLayout());
        frame.getContentPane().add(banner, BorderLayout.NORTH);
        frame.getContentPane().add(this,   BorderLayout.CENTER);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}