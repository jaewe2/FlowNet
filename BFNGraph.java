import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.util.*;
import java.util.List;
import javax.swing.*;

public class BFNGraph extends JPanel implements Runnable {
    private static final long serialVersionUID = 1L;

    private static final int NODE_R   = 22;
    private static final int PAIR_H   = 72;
    private static final int PAD_TOP  = 90;
    private static final int PAD_BOT  = 90;
    private static final int PAD_SIDE = 110;

    // ── data ──────────────────────────────────────────────────────────────────
    private int      n;
    private int[]    nodeEnergies;
    private String   algoTitle = "BFN";

    private Set<Integer>  dgSet      = new HashSet<>();
    private Set<Integer>  storageSet = new HashSet<>();
    private int[]         packetsPerNode;
    private int[]         storageSlots;
    private List<Integer> storageList;
    private int[][]       adjMatrix;

    private List<Integer> nodeOrder = new ArrayList<>();

    private Map<Integer,Point> inPts  = new LinkedHashMap<>();
    private Map<Integer,Point> outPts = new LinkedHashMap<>();
    private Point sPoint, tPoint;

    private List<int[]>  edges    = new ArrayList<>();
    private List<String> edgeCaps = new ArrayList<>();
    private Set<Integer> flowIdx  = new HashSet<>();

    // pan
    private int offsetX=0, offsetY=0, dragX, dragY;
    private boolean dragging=false;

    // ── setters ───────────────────────────────────────────────────────────────
    public void setAlgoTitle(String t) { algoTitle = t; }

    // ── build ─────────────────────────────────────────────────────────────────
    public void build(int n, int[] nodeEnergies,
                      Set<Integer> dgNodes, List<Integer> storageList,
                      double[][] nodeLoc,
                      int[] packetsPerNode, int[] storageSlots,
                      int[][] adjMatrix,
                      List<int[]> goaFlowBSN) {
        this.n              = n;
        this.nodeEnergies   = nodeEnergies;
        this.dgSet          = new HashSet<>(dgNodes);
        this.storageSet     = new HashSet<>(storageList);
        this.packetsPerNode = packetsPerNode;
        this.storageSlots   = storageSlots;
        this.storageList    = storageList;
        this.adjMatrix      = adjMatrix;

        edges.clear(); edgeCaps.clear(); flowIdx.clear();
        inPts.clear(); outPts.clear(); nodeOrder.clear();

        // order nodes top-to-bottom by descending y
        Integer[] order = new Integer[n];
        for (int i=0; i<n; i++) order[i] = i;
        Arrays.sort(order, (a,b) ->
            Double.compare(nodeLoc[b][1], nodeLoc[a][1]));
        for (int i : order) nodeOrder.add(i);

        // store row index temporarily
        for (int row=0; row<nodeOrder.size(); row++) {
            inPts.put(nodeOrder.get(row),  new Point(row, 0));
            outPts.put(nodeOrder.get(row), new Point(row, 0));
        }

        buildEdges(dgNodes, storageList, adjMatrix,
                   packetsPerNode, storageSlots, goaFlowBSN);
    }

    private void buildEdges(Set<Integer> dgNodes, List<Integer> storageList,
                            int[][] adjMatrix, int[] packetsPerNode,
                            int[] storageSlots, List<int[]> goaFlowBSN) {

        // 1. s -> i' for each DG
        for (int dg : dgNodes) {
            edges.add(new int[]{0, -1, 1, dg});
            edgeCaps.add("d=" + packetsPerNode[dg] + " E=" + nodeEnergies[dg]);
        }

        // 2. i' -> i"  per-node energy edge (uniform cost = 1 per packet)
        for (int i=0; i<n; i++) {
            edges.add(new int[]{1, i, 2, i});
            edgeCaps.add("E=" + nodeEnergies[i]);
        }

        // 3. u" -> v'  BSN routing edges (cost = 1, capacity = inf)
        Set<String> added = new HashSet<>();
        for (int u=0; u<n; u++)
            for (int v=0; v<n; v++)
                if (adjMatrix[u][v] == 1) {
                    String k = u + "-" + v;
                    if (!added.contains(k)) {
                        added.add(k);
                        edges.add(new int[]{2, u, 1, v});
                        edgeCaps.add("inf");
                    }
                }

        // 4. j" -> t
        for (int j=0; j<storageList.size(); j++) {
            int st = storageList.get(j);
            edges.add(new int[]{2, st, 3, -1});
            edgeCaps.add("m=" + storageSlots[j] + " E=" + nodeEnergies[st]);
        }

        // mark flow edges
        Set<String> flowSet = new HashSet<>();
        for (int[] fe : goaFlowBSN)
            flowSet.add(Math.min(fe[0],fe[1]) + "-" + Math.max(fe[0],fe[1]));
        for (int ei=0; ei<edges.size(); ei++) {
            int[] e = edges.get(ei);
            if (e[0]==2 && e[2]==1) {
                String k = Math.min(e[1],e[3]) + "-" + Math.max(e[1],e[3]);
                if (flowSet.contains(k)) flowIdx.add(ei);
            }
        }
    }

    // ── layout ────────────────────────────────────────────────────────────────
    private void computeLayout(int W, int H) {
        if (n==0) return;
        int colX  = W/2;
        int GAP   = 30;
        int slotH = NODE_R*2 + PAIR_H + NODE_R*2 + GAP;
        int totalH = n*slotH - GAP;
        int startY = Math.max(PAD_TOP, (H - totalH) / 2);

        for (int row=0; row<nodeOrder.size(); row++) {
            int bsn  = nodeOrder.get(row);
            int inY  = startY + row*slotH + NODE_R;
            int outY = inY + PAIR_H;
            inPts.put(bsn,  new Point(colX, inY));
            outPts.put(bsn, new Point(colX, outY));
        }

        int topFirst = inPts.get(nodeOrder.get(0)).y;
        int botLast  = outPts.get(nodeOrder.get(n-1)).y;
        int midY     = (topFirst + botLast) / 2;
        sPoint = new Point(PAD_SIDE,     midY);
        tPoint = new Point(W - PAD_SIDE, midY);
    }

    private Point px(int type, int id) {
        Point b;
        switch(type) {
            case 0:  b = sPoint;         break;
            case 3:  b = tPoint;         break;
            case 1:  b = inPts.get(id);  break;
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

        // background
        g2.setColor(new Color(240,243,250));
        g2.fillRect(0,0,W,H);
        g2.setColor(new Color(250,251,255));
        g2.fillRoundRect(30,30,W-60,H-60,20,20);
        g2.setColor(new Color(195,200,215));
        g2.setStroke(new BasicStroke(1f));
        g2.drawRoundRect(30,30,W-60,H-60,20,20);

        if (n==0) return;
        Stroke def = g2.getStroke();

        // ── edges ─────────────────────────────────────────────────────────────
        for (int ei=0; ei<edges.size(); ei++) {
            int[]   e    = edges.get(ei);
            Point   p1   = px(e[0], e[1]);
            Point   p2   = px(e[2], e[3]);
            String  cap  = edgeCaps.get(ei);
            boolean isFlow     = flowIdx.contains(ei);
            boolean isInternal = e[0]==1 && e[2]==2 && e[1]==e[3];
            boolean isInf      = "inf".equals(cap);
            boolean isS        = e[0]==0;
            boolean isT        = e[2]==3;

            if (isFlow) {
                g2.setColor(new Color(255,150,0,55));
                g2.setStroke(new BasicStroke(12f,
                    BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                drawArrow(g2, p1, p2);
                g2.setColor(new Color(210,90,0));
                g2.setStroke(new BasicStroke(3f,
                    BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            } else if (isInternal) {
                g2.setColor(new Color(60,80,200));
                g2.setStroke(new BasicStroke(2f));
            } else if (isS || isT) {
                g2.setColor(new Color(70,70,70));
                g2.setStroke(new BasicStroke(1.8f));
            } else if (isInf) {
                g2.setColor(new Color(160,160,160,130));
                g2.setStroke(new BasicStroke(1f,
                    BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER,
                    1f, new float[]{6,4}, 0));
            } else {
                g2.setColor(new Color(100,100,100));
                g2.setStroke(new BasicStroke(1.5f));
            }
            drawArrow(g2, p1, p2);
            g2.setStroke(def);

            // labels: skip inf routing edges to reduce clutter
            if (!isInf || isFlow) {
                int mx = (p1.x+p2.x)/2, my = (p1.y+p2.y)/2;
                int lx = isInternal ? mx + NODE_R + 6 : mx;
                g2.setFont(new Font("SansSerif",
                    isFlow ? Font.BOLD : Font.PLAIN, 10));
                FontMetrics fm = g2.getFontMetrics();
                int tw = fm.stringWidth(cap);
                g2.setColor(new Color(255,255,255,220));
                g2.fillRoundRect(lx-tw/2-3, my-fm.getAscent()-1,
                                 tw+6, fm.getHeight()+2, 5, 5);
                g2.setColor(isFlow     ? new Color(170,60,0)  :
                            isInternal ? new Color(40,60,180)  :
                                         new Color(50,50,50));
                g2.drawString(cap, lx-tw/2, my);
            }
        }

        // ── nodes ─────────────────────────────────────────────────────────────
        drawNode(g2, px(0,-1), "s",
                 new Color(80,80,90), new Color(60,60,70));
        drawNode(g2, px(3,-1), "t",
                 new Color(80,80,90), new Color(60,60,70));

        for (int i=0; i<n; i++) {
            boolean isDG = dgSet.contains(i);
            Color fill   = isDG ? new Color(72,125,215) : new Color(42,155,95);
            Color dark   = isDG ? new Color(40,85,165)  : new Color(20,110,65);
            Point pi = px(1,i), po = px(2,i);

            // dashed oval around pair
            int pairCX = (pi.x+po.x)/2, pairCY = (pi.y+po.y)/2;
            int dist   = Math.abs(po.y - pi.y);
            int pairR  = dist/2 + NODE_R + 8;
            g2.setColor(isDG ? new Color(72,125,215,40)
                              : new Color(42,155,95,40));
            g2.fillOval(pairCX-pairR, pairCY-pairR, pairR*2, pairR*2);
            g2.setColor(isDG ? new Color(72,125,215,120)
                              : new Color(42,155,95,120));
            g2.setStroke(new BasicStroke(1.5f,
                BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER,
                1f, new float[]{5,3}, 0));
            g2.drawOval(pairCX-pairR, pairCY-pairR, pairR*2, pairR*2);
            g2.setStroke(def);

            drawNode(g2, pi, (i+1)+"'",  fill, dark);
            drawNode(g2, po, (i+1)+"\"", fill, dark);
        }

        drawLegend(g2, def, W, H);
    }

    private void drawNode(Graphics2D g2, Point p, String lbl,
                          Color fill, Color border) {
        g2.setColor(new Color(0,0,0,35));
        g2.fillOval(p.x-NODE_R+2, p.y-NODE_R+3, NODE_R*2, NODE_R*2);
        GradientPaint gp = new GradientPaint(
            p.x, p.y-NODE_R, fill.brighter(),
            p.x, p.y+NODE_R, fill.darker());
        g2.setPaint(gp);
        g2.fillOval(p.x-NODE_R, p.y-NODE_R, NODE_R*2, NODE_R*2);
        g2.setPaint(null);
        g2.setColor(border);
        g2.setStroke(new BasicStroke(1.8f));
        g2.drawOval(p.x-NODE_R, p.y-NODE_R, NODE_R*2, NODE_R*2);
        g2.setColor(Color.WHITE);
        g2.setFont(new Font("SansSerif", Font.BOLD, 11));
        FontMetrics fm = g2.getFontMetrics();
        g2.drawString(lbl, p.x-fm.stringWidth(lbl)/2,
                           p.y+fm.getAscent()/2-1);
    }

    private void drawArrow(Graphics2D g2, Point p1, Point p2) {
        double dx = p2.x-p1.x, dy = p2.y-p1.y;
        double len = Math.sqrt(dx*dx+dy*dy);
        if (len < 2) return;
        double ux = dx/len, uy = dy/len;
        int x1 = (int)(p1.x + ux*NODE_R);
        int y1 = (int)(p1.y + uy*NODE_R);
        int x2 = (int)(p2.x - ux*(NODE_R+4));
        int y2 = (int)(p2.y - uy*(NODE_R+4));
        if (x1==x2 && y1==y2) return;
        g2.drawLine(x1,y1,x2,y2);
        int aw=9, ah=5;
        g2.fillPolygon(
            new int[]{x2,(int)(x2-aw*ux+ah*uy),(int)(x2-aw*ux-ah*uy)},
            new int[]{y2,(int)(y2-aw*uy-ah*ux),(int)(y2-aw*uy+ah*ux)}, 3);
    }

    private void drawLegend(Graphics2D g2, Stroke def, int W, int H) {
        String[] lbl = {
            "DG (source) — i', i\"",
            "Storage (sink) — i', i\"",
            "s / t  super source / sink",
            "i' -> i\"  node energy edge (Eᵢ)",
            "BSN routing edge (uniform cost=1, inf cap)",
            "GOA flow path"
        };
        Color[] col = {
            new Color(72,125,215), new Color(42,155,95),
            new Color(80,80,90),   new Color(60,80,200),
            new Color(150,150,150), new Color(210,90,0)
        };

        g2.setFont(new Font("SansSerif", Font.PLAIN, 12));
        FontMetrics fm = g2.getFontMetrics();
        int maxW = 0;
        for (String s : lbl) maxW = Math.max(maxW, fm.stringWidth(s));
        int bw=16, bh=16, gap=8, px2=12, py2=10;
        int boxW = maxW + bw + px2*2 + 10;
        int boxH = lbl.length*(bh+gap) + py2*2;
        int lx = W-boxW-12, ly = H-boxH-12;

        g2.setColor(new Color(0,0,0,35));
        g2.fillRoundRect(lx+3, ly+3, boxW, boxH, 12, 12);
        g2.setColor(new Color(255,255,255,245));
        g2.fillRoundRect(lx, ly, boxW, boxH, 12, 12);
        g2.setColor(new Color(180,185,200));
        g2.setStroke(new BasicStroke(1f));
        g2.drawRoundRect(lx, ly, boxW, boxH, 12, 12);

        for (int i=0; i<lbl.length; i++) {
            int cy = ly+py2+i*(bh+gap), ix = lx+px2;
            if (i < 3) {
                g2.setColor(col[i]);
                g2.fillOval(ix, cy, bw, bh);
                g2.setColor(col[i].darker());
                g2.setStroke(new BasicStroke(1f));
                g2.drawOval(ix, cy, bw, bh);
            } else if (i==3) {
                g2.setColor(col[i]);
                g2.setStroke(new BasicStroke(2f));
                g2.drawLine(ix, cy+bh/2, ix+bw, cy+bh/2);
                g2.fillPolygon(
                    new int[]{ix+bw, ix+bw-6, ix+bw-6},
                    new int[]{cy+bh/2, cy+bh/2-3, cy+bh/2+3}, 3);
                g2.setStroke(def);
            } else if (i==4) {
                g2.setColor(col[i]);
                g2.setStroke(new BasicStroke(1.5f,
                    BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER,
                    1f, new float[]{5,4}, 0));
                g2.drawLine(ix, cy+bh/2, ix+bw, cy+bh/2);
                g2.setStroke(def);
            } else {
                g2.setColor(new Color(255,150,0,60));
                g2.setStroke(new BasicStroke(8f,
                    BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                g2.drawLine(ix, cy+bh/2, ix+bw, cy+bh/2);
                g2.setColor(col[i]);
                g2.setStroke(new BasicStroke(2.5f,
                    BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                g2.drawLine(ix, cy+bh/2, ix+bw, cy+bh/2);
                g2.setStroke(def);
            }
            g2.setFont(new Font("SansSerif", Font.PLAIN, 12));
            g2.setColor(new Color(30,30,30));
            g2.drawString(lbl[i], ix+bw+8, cy+bh-2);
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

        int slotH = NODE_R*2 + PAIR_H + NODE_R*2 + 30;
        int prefH = Math.max(700, n*slotH + PAD_TOP + PAD_BOT + 60);
        frame.setPreferredSize(new Dimension(1500, prefH));

        JLabel banner = new JLabel("BFN: " + algoTitle, SwingConstants.CENTER);
        banner.setFont(new Font("SansSerif", Font.BOLD, 18));
        banner.setOpaque(true);
        banner.setBackground(new Color(20,50,110));
        banner.setForeground(Color.WHITE);
        banner.setBorder(BorderFactory.createEmptyBorder(10,0,10,0));

        JLabel hint = new JLabel(
            "Click and drag to pan  |  " +
            "i' = in-node   i\" = out-node   " +
            "E = node energy budget   cost = 1 per packet (MWF-U)",
            SwingConstants.CENTER);
        hint.setFont(new Font("SansSerif", Font.ITALIC, 11));
        hint.setForeground(new Color(80,80,100));
        hint.setBorder(BorderFactory.createEmptyBorder(4,0,4,0));

        frame.getContentPane().setLayout(new BorderLayout());
        frame.getContentPane().add(banner, BorderLayout.NORTH);
        frame.getContentPane().add(this,   BorderLayout.CENTER);
        frame.getContentPane().add(hint,   BorderLayout.SOUTH);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}