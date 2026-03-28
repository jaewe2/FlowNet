import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.util.*;
import java.util.List;
import javax.swing.*;

public class BFNGraph extends JPanel implements Runnable {
    private static final long serialVersionUID = 1L;

    // ── palette ───────────────────────────────────────────────────────────────
    private static final Color COL_BG        = new Color(248, 248, 252);
    private static final Color COL_DG        = new Color(130, 50,  200);
    private static final Color COL_DG_LIGHT  = new Color(200, 165, 240);
    private static final Color COL_ST        = new Color(20,  150, 80);
    private static final Color COL_ST_LIGHT  = new Color(120, 210, 160);
    private static final Color COL_ST_NODE   = new Color(80,  80,  100);
    private static final Color COL_EDGE_INT  = new Color(100, 60,  180);
    private static final Color COL_EDGE_RT   = new Color(130, 100, 200);
    private static final Color COL_EDGE_ST   = new Color(60,  60,  100);
    private static final Color COL_FLOW      = new Color(220, 80,  20);
    private static final Color COL_FLOW_GLOW = new Color(255, 140, 40, 80);
    private static final Color COL_TEXT      = new Color(30,  15,  60);

    // ── layout ────────────────────────────────────────────────────────────────
    private static final int NODE_R   = 20;
    private static final int PAD_TOP  = 100;
    private static final int PAD_BOT  = 140;
    private static final int PAD_SIDE = 80;
    private static final int ROW_GAP  = 160;
    private static final int IN_X     = 350;
    private static final int OUT_X    = 620;

    // ── data ──────────────────────────────────────────────────────────────────
    private int n;
    private int[] nodeEnergies;
    private String algoTitle = "BFN";

    private Set<Integer>  dgSet      = new HashSet<>();
    private List<Integer> dgList     = new ArrayList<>();
    private Set<Integer>  storageSet = new HashSet<>();
    private int[]         packetsPerNode;
    private int[]         storageSlots;
    private List<Integer> storageList;
    private int[][]       adjMatrix;

    private List<Integer>      nodeOrder = new ArrayList<>();
    private Map<Integer,Point> inPts     = new LinkedHashMap<>();
    private Map<Integer,Point> outPts    = new LinkedHashMap<>();
    private Point sPoint, tPoint;

    // ── edge data ─────────────────────────────────────────────────────────────
    // Each edge is int[]{srcType, srcId, dstType, dstId}
    // types: 0=superSource, 1=inNode, 2=outNode, 3=superSink
    private List<int[]>  edges    = new ArrayList<>();
    private List<String> edgeCaps = new ArrayList<>();
    private Set<Integer> flowIdx         = new HashSet<>();
    private Set<Integer> activeFlowNodes = new HashSet<>();
    private Set<Integer> relayNodes      = new HashSet<>();

    // ── drag / pan ────────────────────────────────────────────────────────────
    private Map<Integer,Point> edgeCtrl = new HashMap<>();
    private int  dragEdgeIdx = -1;
    private int  dragOffX = 0, dragOffY = 0;
    private int  lastW = -1, lastH = -1;
    private int  offsetX = 0, offsetY = 0, panDragX, panDragY;
    private boolean panning = false;

    public void setAlgoTitle(String t) { algoTitle = t; }

    // ── build ─────────────────────────────────────────────────────────────────
    public void build(int n, int[] nodeEnergies,
                      Set<Integer> dgNodes, List<Integer> storageList,
                      double[][] nodeLoc,
                      int[] packetsPerNode, int[] storageSlots,
                      int[][] adjMatrix, List<int[]> goaFlowBSN) {
        this.n              = n;
        this.nodeEnergies   = nodeEnergies;
        this.dgSet          = new HashSet<>(dgNodes);
        this.dgList         = new ArrayList<>(dgNodes);
        Collections.sort(this.dgList);
        this.storageSet     = new HashSet<>(storageList);
        this.packetsPerNode = packetsPerNode;
        this.storageSlots   = storageSlots;
        this.storageList    = storageList;
        this.adjMatrix      = adjMatrix;

        edges.clear(); edgeCaps.clear(); flowIdx.clear();
        activeFlowNodes.clear(); relayNodes.clear();
        inPts.clear(); outPts.clear(); nodeOrder.clear();
        edgeCtrl.clear();
        lastW = -1; lastH = -1;

        buildNodeOrder();
        buildEdges(dgNodes, storageList, adjMatrix, packetsPerNode,
                   storageSlots, goaFlowBSN);
    }

    private void buildNodeOrder() {
        List<Integer> dg = new ArrayList<>(), relay = new ArrayList<>(),
                      st = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            if      (dgSet.contains(i))      dg.add(i);
            else if (storageSet.contains(i)) st.add(i);
            else                             relay.add(i);
        }
        Collections.sort(dg); Collections.sort(relay); Collections.sort(st);
        nodeOrder.addAll(dg); nodeOrder.addAll(relay); nodeOrder.addAll(st);
    }

    private void buildEdges(Set<Integer> dgNodes, List<Integer> storageList,
                            int[][] adjMatrix, int[] packetsPerNode,
                            int[] storageSlots, List<int[]> goaFlowBSN) {

        // s → i'  capacity = d (packets per DG) — iterate in sorted order
        for (int dg : dgList) {
            edges.add(new int[]{0, -1, 1, dg});
            edgeCaps.add("d=" + packetsPerNode[dg] + "\nE=" + nodeEnergies[dg]);
        }

        // i' → i"  capacity = E (node energy)
        for (int i = 0; i < n; i++) {
            edges.add(new int[]{1, i, 2, i});
            edgeCaps.add("E=" + nodeEnergies[i]);
        }

        // u" → v'  capacity = inf (BSN routing edges)
        for (int u = 0; u < n; u++)
            for (int v = 0; v < n; v++)
                if (adjMatrix[u][v] == 1) {
                    edges.add(new int[]{2, u, 1, v});
                    edgeCaps.add("inf");
                }

        // j" → t  capacity = m (storage capacity)
        for (int j = 0; j < storageList.size(); j++) {
            int st = storageList.get(j);
            edges.add(new int[]{2, st, 3, -1});
            edgeCaps.add("m=" + storageSlots[j] + "\nE=" + nodeEnergies[st]
                         + "\n(ST " + (st+1) + ")");
        }

        // mark flow edges
        Set<String> flowSet = new HashSet<>();
        for (int[] fe : goaFlowBSN) {
            int a = Math.min(fe[0],fe[1]), b = Math.max(fe[0],fe[1]);
            flowSet.add(a + "-" + b);
            activeFlowNodes.add(fe[0]); activeFlowNodes.add(fe[1]);
        }
        relayNodes.addAll(activeFlowNodes); relayNodes.removeAll(dgSet);

        for (int ei = 0; ei < edges.size(); ei++) {
            int[] e = edges.get(ei);
            if (e[0]==2 && e[2]==1) {
                String k = Math.min(e[1],e[3]) + "-" + Math.max(e[1],e[3]);
                if (flowSet.contains(k)) flowIdx.add(ei);
            } else if (e[0]==1 && e[2]==2 && activeFlowNodes.contains(e[1])) {
                flowIdx.add(ei);
            } else if (e[0]==0 && activeFlowNodes.contains(e[3])) {
                flowIdx.add(ei);
            } else if (e[2]==3 && activeFlowNodes.contains(e[1])
                                && storageSet.contains(e[1])) {
                flowIdx.add(ei);
            }
        }
    }

    // ── layout ────────────────────────────────────────────────────────────────
    private void computeLayout(int W, int H) {
        if (n == 0) return;
        int sourceX = PAD_SIDE + 30;
        int inX     = Math.max(IN_X,  W / 4);
        int outX    = Math.max(OUT_X, W / 2 + 60);
        int sinkX   = W - PAD_SIDE - 30;
        int totalH  = (Math.max(1,n)-1) * ROW_GAP;
        int startY  = Math.max(PAD_TOP, (H-totalH)/2);

        for (int row = 0; row < nodeOrder.size(); row++) {
            int node = nodeOrder.get(row), y = startY + row * ROW_GAP;
            inPts.put(node,  new Point(inX,  y));
            outPts.put(node, new Point(outX, y));
        }
        sPoint = new Point(sourceX, startY + totalH/2);
        tPoint = new Point(sinkX,   startY + totalH/2);

        // reset ctrl points on resize; preserve user drags otherwise
        boolean resized = (W != lastW || H != lastH);
        lastW = W; lastH = H;
        for (int ei = 0; ei < edges.size(); ei++) {
            if (!resized && edgeCtrl.containsKey(ei)) continue;
            Point p1 = rawPx(edges.get(ei)[0], edges.get(ei)[1]);
            Point p2 = rawPx(edges.get(ei)[2], edges.get(ei)[3]);
            if (p1==null || p2==null) continue;
            edgeCtrl.put(ei, new Point((p1.x+p2.x)/2, (p1.y+p2.y)/2));
        }
    }

    private Point rawPx(int type, int id) {
        switch (type) {
            case 0:  return sPoint;
            case 3:  return tPoint;
            case 1:  return inPts.get(id);
            default: return outPts.get(id);
        }
    }
    private Point px(int type, int id) {
        Point b = rawPx(type, id);
        if (b==null) return new Point(0,0);
        return new Point(b.x+offsetX, b.y+offsetY);
    }
    private Point ctrlPx(int ei) {
        Point c = edgeCtrl.get(ei);
        if (c==null) return new Point(0,0);
        return new Point(c.x+offsetX, c.y+offsetY);
    }

    // ── label position — called identically for flow and non-flow ─────────────
    // Rules:
    //   s→i'  : label tracks 40% along the actual edge line, offset perp ±12px
    //            so each d= label sits on its own edge away from s
    //   i'→i" : label above the midpoint
    //   u"→v' : label alternates above/below midpoint per edge index
    //   j"→t  : labels spread vertically near t, one per storage node
    private Point labelPos(int ei, int[] e, Point p1, Point p2, Point ctrl) {
        boolean isS  = e[0]==0;
        boolean isT  = e[2]==3;
        boolean isInt = e[0]==1 && e[2]==2 && e[1]==e[3];
        boolean isInf = "inf".equals(edgeCaps.get(ei));

        if (isInt) {
            // E= above internal edge midpoint
            return new Point(ctrl.x, ctrl.y - 20);
        }

        if (isS) {
            // Each d= label gets a unique position by combining:
            // - a fixed fraction along the edge (avoids clustering near s)
            // - a perpendicular offset scaled by DG index (avoids label overlap)
            // Use DG insertion order in dgSet for stable indexing.
            int dgIdx = new ArrayList<>(dgSet).indexOf(e[3]);
            if (dgIdx < 0) dgIdx = 0;
            // place label at 45% along the edge from s to the in-node
            double t  = 0.45;
            int    lx = (int)(p1.x + t*(p2.x - p1.x));
            int    ly = (int)(p1.y + t*(p2.y - p1.y));
            // perpendicular unit vector
            double dx = p2.x - p1.x, dy = p2.y - p1.y;
            double len = Math.sqrt(dx*dx + dy*dy);
            if (len > 0) { dx /= len; dy /= len; }
            // alternate sides with increasing offset per DG index
            int perpSign = (dgIdx % 2 == 0) ? 1 : -1;
            int perpDist = 20 + dgIdx * 10; // 20, 30, 40... px per DG
            lx += (int)(-dy * perpDist * perpSign);
            ly += (int)( dx * perpDist * perpSign);
            return new Point(lx, ly);
        }

        if (isT) {
            // m= labels: fixed x well left of t, spread 50px apart vertically
            int idx    = storageList.indexOf(e[1]);
            int total  = storageList.size();
            int spread = (idx - (total-1)/2) * 50;
            return new Point(tPoint.x + offsetX - 85, tPoint.y + offsetY + spread);
        }

        if (isInf) {
            // spread inf labels: use ei to pick position along edge AND
            // side, giving each routing edge a unique label location
            double t = 0.25 + (ei % 4) * 0.17; // 0.25, 0.42, 0.59, 0.76
            int lx = (int)(p1.x + t*(p2.x - p1.x));
            int ly = (int)(p1.y + t*(p2.y - p1.y));
            // also offset perpendicular slightly so overlapping edges separate
            double dx = p2.x-p1.x, dy = p2.y-p1.y;
            double len = Math.sqrt(dx*dx+dy*dy);
            if (len>0){dx/=len;dy/=len;}
            int off = (ei % 2 == 0) ? -10 : 10;
            lx += (int)(-dy * off);
            ly += (int)( dx * off);
            return new Point(lx, ly);
        }

        return new Point(ctrl.x, ctrl.y - 14);
    }

    // ── hit testing ───────────────────────────────────────────────────────────
    private static final int HIT_DIST  = 8;
    private static final int HIT_STEPS = 40;

    private int hitEdge(int mx, int my) {
        for (int ei = edges.size()-1; ei >= 0; ei--) {
            int[] e = edges.get(ei);
            Point p1 = px(e[0],e[1]), p2 = px(e[2],e[3]), ctrl = ctrlPx(ei);
            int bx = 2*ctrl.x - (p1.x+p2.x)/2;
            int by = 2*ctrl.y - (p1.y+p2.y)/2;
            for (int s = 0; s <= HIT_STEPS; s++) {
                double t = (double)s/HIT_STEPS, mt = 1-t;
                double qx = mt*mt*p1.x + 2*mt*t*bx + t*t*p2.x;
                double qy = mt*mt*p1.y + 2*mt*t*by + t*t*p2.y;
                if (Math.hypot(mx-qx, my-qy) <= HIT_DIST) return ei;
            }
        }
        return -1;
    }

    // ── paint ─────────────────────────────────────────────────────────────────
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,      RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

        int W = getWidth(), H = getHeight();
        computeLayout(W, H);

        g2.setColor(COL_BG); g2.fillRect(0,0,W,H);
        g2.setColor(new Color(245,242,252));
        g2.fillRoundRect(30,30,W-60,H-60,20,20);
        g2.setColor(new Color(180,160,210));
        g2.setStroke(new BasicStroke(1.2f));
        g2.drawRoundRect(30,30,W-60,H-60,20,20);
        if (n==0) return;

        Stroke def = g2.getStroke();
        List<LabelDraw> labels = new ArrayList<>();

        // ── pass 1: non-flow edges ────────────────────────────────────────────
        for (int ei = 0; ei < edges.size(); ei++) {
            if (flowIdx.contains(ei)) continue;
            int[] e   = edges.get(ei);
            String cap = edgeCaps.get(ei);
            boolean isInt = e[0]==1 && e[2]==2 && e[1]==e[3];
            boolean isInf = "inf".equals(cap);
            Point p1 = px(e[0],e[1]), p2 = px(e[2],e[3]), ctrl = ctrlPx(ei);

            if (isInf) {
                g2.setColor(new Color(COL_EDGE_RT.getRed(), COL_EDGE_RT.getGreen(),
                                      COL_EDGE_RT.getBlue(), 179));
                g2.setStroke(new BasicStroke(1.2f, BasicStroke.CAP_BUTT,
                    BasicStroke.JOIN_MITER, 1f, new float[]{6,4}, 0));
            } else if (isInt) {
                g2.setColor(COL_EDGE_INT);
                g2.setStroke(new BasicStroke(2f));
            } else {
                g2.setColor(COL_EDGE_ST);
                g2.setStroke(new BasicStroke(1.8f));
            }
            drawCurvedArrow(g2, p1, p2, ctrl, true);
            g2.setStroke(def);

            Point lp = labelPos(ei, e, p1, p2, ctrl);
            Color lc  = isInt ? COL_EDGE_INT : isInf ? COL_EDGE_RT : COL_TEXT;
            labels.add(new LabelDraw(cap, lp.x, lp.y, Font.PLAIN, lc));
        }

        // ── pass 2: flow edges on top ─────────────────────────────────────────
        for (int ei = 0; ei < edges.size(); ei++) {
            if (!flowIdx.contains(ei)) continue;
            int[] e    = edges.get(ei);
            String cap = edgeCaps.get(ei);
            Point p1 = px(e[0],e[1]), p2 = px(e[2],e[3]), ctrl = ctrlPx(ei);

            g2.setColor(COL_FLOW_GLOW);
            g2.setStroke(new BasicStroke(14f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            drawCurvedArrow(g2, p1, p2, ctrl, false);
            g2.setColor(COL_FLOW);
            g2.setStroke(new BasicStroke(3.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            drawCurvedArrow(g2, p1, p2, ctrl, true);
            g2.setStroke(def);

            Point lp = labelPos(ei, e, p1, p2, ctrl);
            labels.add(new LabelDraw(cap, lp.x, lp.y, Font.BOLD, COL_FLOW));
        }

        // ── nodes ─────────────────────────────────────────────────────────────
        drawColumnHeaders(g2);
        drawNode(g2, px(0,-1), "s", COL_ST_NODE, COL_ST_NODE.darker());
        drawNode(g2, px(3,-1), "t", COL_ST_NODE, COL_ST_NODE.darker());

        for (int i = 0; i < n; i++) {
            boolean isDG    = dgSet.contains(i);
            boolean isRelay = relayNodes.contains(i);
            boolean isST    = storageSet.contains(i);
            Color fill  = isDG ? COL_DG : isST ? COL_ST : COL_ST_NODE;
            Color light = isDG ? COL_DG_LIGHT : isST ? COL_ST_LIGHT
                                                      : new Color(180,180,195);
            Point pi = px(1,i), po = px(2,i);
            if (isRelay) {
                g2.setColor(new Color(255,170,70,45));
                g2.fillOval(pi.x-NODE_R-7,pi.y-NODE_R-7,NODE_R*2+14,NODE_R*2+14);
                g2.fillOval(po.x-NODE_R-7,po.y-NODE_R-7,NODE_R*2+14,NODE_R*2+14);
                g2.setColor(new Color(230,120,20,150));
                g2.setStroke(new BasicStroke(1.8f));
                g2.drawOval(pi.x-NODE_R-7,pi.y-NODE_R-7,NODE_R*2+14,NODE_R*2+14);
                g2.drawOval(po.x-NODE_R-7,po.y-NODE_R-7,NODE_R*2+14,NODE_R*2+14);
                g2.setStroke(def);
            }
            drawNode(g2, pi, (i+1)+"'",  fill, light);
            drawNode(g2, po, (i+1)+"\"", fill, light);
        }

        // labels drawn last — always on top of edges
        for (LabelDraw lbl : labels) drawEdgeLabel(g2, lbl);

        // drag indicator
        if (dragEdgeIdx >= 0) {
            Point c = ctrlPx(dragEdgeIdx);
            g2.setColor(new Color(220,80,20,60));
            g2.fillOval(c.x-8,c.y-8,16,16);
            g2.setColor(new Color(220,80,20,100));
            g2.setStroke(new BasicStroke(1.5f));
            g2.drawOval(c.x-8,c.y-8,16,16);
            g2.setStroke(def);
        }

        drawLegend(g2, def, W, H);
    }

    // ── drawing helpers ───────────────────────────────────────────────────────
    private void drawCurvedArrow(Graphics2D g2, Point p1, Point p2,
                                  Point ctrl, boolean arrowHead) {
        double dx = p2.x-p1.x, dy = p2.y-p1.y;
        double len = Math.sqrt(dx*dx+dy*dy);
        if (len < 2) return;
        double ux = dx/len, uy = dy/len;
        int x1 = (int)(p1.x+ux*NODE_R),      y1 = (int)(p1.y+uy*NODE_R);
        int x2 = (int)(p2.x-ux*(NODE_R+4)),  y2 = (int)(p2.y-uy*(NODE_R+4));
        if (x1==x2 && y1==y2) return;
        int bx = 2*ctrl.x-(x1+x2)/2, by = 2*ctrl.y-(y1+y2)/2;
        g2.draw(new QuadCurve2D.Double(x1,y1, bx,by, x2,y2));
        if (arrowHead) {
            double tx = x2-bx, ty = y2-by;
            double tl = Math.sqrt(tx*tx+ty*ty);
            if (tl>0) { tx/=tl; ty/=tl; }
            int aw=8, ah=4;
            g2.fillPolygon(
                new int[]{x2,(int)(x2-aw*tx+ah*ty),(int)(x2-aw*tx-ah*ty)},
                new int[]{y2,(int)(y2-aw*ty-ah*tx),(int)(y2-aw*ty+ah*tx)},3);
        }
    }

    private void drawColumnHeaders(Graphics2D g2) {
        g2.setColor(new Color(110,95,145));
        g2.setFont(new Font("SansSerif",Font.BOLD,13));
        FontMetrics fm = g2.getFontMetrics();
        String[] hdrs = {"Super Source","In-Nodes","Out-Nodes","Super Sink"};
        int[] xs = { sPoint.x,
                     inPts.get(nodeOrder.get(0)).x,
                     outPts.get(nodeOrder.get(0)).x,
                     tPoint.x };
        for (int i = 0; i < hdrs.length; i++)
            g2.drawString(hdrs[i], xs[i]-fm.stringWidth(hdrs[i])/2, 60+offsetY);
    }

    private static class LabelDraw {
        String text; int x,y,fs; Color c;
        LabelDraw(String t,int x,int y,int fs,Color c){
            text=t;this.x=x;this.y=y;this.fs=fs;this.c=c;}
    }

    private void drawEdgeLabel(Graphics2D g2, LabelDraw l) {
        g2.setFont(new Font("SansSerif",l.fs,9));
        FontMetrics fm = g2.getFontMetrics();
        String[] lines = l.text.split("\\n");
        int maxW=0;
        for (String ln : lines) maxW=Math.max(maxW,fm.stringWidth(ln));
        int lh=fm.getHeight(), px=5, py=3;
        int bw=maxW+px*2, bh=lines.length*lh+py*2;
        int bx=l.x-bw/2, by=l.y-bh/2;
        g2.setColor(new Color(255,255,255,220));
        g2.fillRoundRect(bx,by,bw,bh,7,7);
        g2.setColor(new Color(185,176,206,160));
        g2.setStroke(new BasicStroke(0.9f));
        g2.drawRoundRect(bx,by,bw,bh,7,7);
        g2.setColor(l.c);
        int y = by+py+fm.getAscent();
        for (String ln : lines) {
            g2.drawString(ln, l.x-fm.stringWidth(ln)/2, y);
            y += lh;
        }
    }

    private void drawNode(Graphics2D g2, Point p, String lbl,
                          Color fill, Color light) {
        g2.setColor(new Color(0,0,0,25));
        g2.fillOval(p.x-NODE_R+2,p.y-NODE_R+3,NODE_R*2,NODE_R*2);
        RadialGradientPaint rgp = new RadialGradientPaint(
            new Point(p.x-NODE_R/3,p.y-NODE_R/3), NODE_R*2,
            new float[]{0f,1f}, new Color[]{light,fill});
        g2.setPaint(rgp);
        g2.fillOval(p.x-NODE_R,p.y-NODE_R,NODE_R*2,NODE_R*2);
        g2.setPaint(null);
        g2.setColor(fill.darker());
        g2.setStroke(new BasicStroke(1.6f));
        g2.drawOval(p.x-NODE_R,p.y-NODE_R,NODE_R*2,NODE_R*2);
        g2.setColor(Color.WHITE);
        g2.setFont(new Font("SansSerif",Font.BOLD,11));
        FontMetrics fm=g2.getFontMetrics();
        g2.drawString(lbl,p.x-fm.stringWidth(lbl)/2,p.y+fm.getAscent()/2-1);
    }

    // ── legend ────────────────────────────────────────────────────────────────
    private void drawLegend(Graphics2D g2, Stroke def, int W, int H) {
        String[] lbl = {
            "DG (source) — i', i\"", "Storage (sink) — i', i\"",
            "Relay node used by flow", "s / t  super source / sink",
            "i' → i\"  energy edge (Eᵢ)", "BSN routing edge (inf cap)",
            "GOA flow path", "Drag any edge line to bend it"
        };
        Color[] col = { COL_DG, COL_ST, new Color(230,120,20), COL_ST_NODE,
                        COL_EDGE_INT, COL_EDGE_RT, COL_FLOW, new Color(100,80,180) };
        g2.setFont(new Font("SansSerif",Font.PLAIN,12));
        FontMetrics fm = g2.getFontMetrics();
        int maxW=0; for (String s:lbl) maxW=Math.max(maxW,fm.stringWidth(s));
        int bw=16,bh=16,gap=9,px=12,py=10;
        int boxW=maxW+bw+px*2+10, boxH=lbl.length*(bh+gap)+py*2;
        int lx=W-boxW-14, ly=H-boxH-14;
        // ensure legend never clips off screen
        if (ly < 10) ly = 10;
        if (lx < 10) lx = 10;
        g2.setColor(new Color(0,0,0,25));
        g2.fillRoundRect(lx+3,ly+3,boxW,boxH,12,12);
        g2.setColor(new Color(255,255,255,240));
        g2.fillRoundRect(lx,ly,boxW,boxH,12,12);
        g2.setColor(new Color(180,160,210));
        g2.setStroke(new BasicStroke(1.2f));
        g2.drawRoundRect(lx,ly,boxW,boxH,12,12);
        for (int i=0; i<lbl.length; i++) {
            int cy=ly+py+i*(bh+gap), ix=lx+px;
            if (i<2||i==3) {
                RadialGradientPaint rgp=new RadialGradientPaint(
                    new Point(ix+bw/2-2,cy+bh/2-2),bw,new float[]{0f,1f},
                    new Color[]{col[i].brighter(),col[i]});
                g2.setPaint(rgp); g2.fillOval(ix,cy,bw,bh); g2.setPaint(null);
                g2.setColor(col[i].darker()); g2.setStroke(new BasicStroke(1f));
                g2.drawOval(ix,cy,bw,bh);
            } else if (i==2) {
                g2.setColor(new Color(255,170,70,80));
                g2.fillOval(ix-2,cy-2,bw+4,bh+4);
                g2.setColor(col[i]); g2.setStroke(new BasicStroke(2f));
                g2.drawOval(ix-2,cy-2,bw+4,bh+4);
            } else if (i==4) {
                g2.setColor(col[i]); g2.setStroke(new BasicStroke(2f));
                g2.drawLine(ix,cy+bh/2,ix+bw,cy+bh/2);
                g2.fillPolygon(new int[]{ix+bw,ix+bw-6,ix+bw-6},
                               new int[]{cy+bh/2,cy+bh/2-3,cy+bh/2+3},3);
                g2.setStroke(def);
            } else if (i==5) {
                g2.setColor(new Color(col[i].getRed(),col[i].getGreen(),
                                      col[i].getBlue(),179));
                g2.setStroke(new BasicStroke(1.3f,BasicStroke.CAP_BUTT,
                    BasicStroke.JOIN_MITER,1f,new float[]{5,4},0));
                g2.drawArc(ix,cy,bw,bh,20,140); g2.setStroke(def);
            } else if (i==6) {
                g2.setColor(COL_FLOW_GLOW);
                g2.setStroke(new BasicStroke(7f,BasicStroke.CAP_ROUND,BasicStroke.JOIN_ROUND));
                g2.drawLine(ix,cy+bh/2,ix+bw,cy+bh/2);
                g2.setColor(col[i]);
                g2.setStroke(new BasicStroke(2.5f,BasicStroke.CAP_ROUND,BasicStroke.JOIN_ROUND));
                g2.drawLine(ix,cy+bh/2,ix+bw,cy+bh/2); g2.setStroke(def);
            } else {
                g2.setColor(new Color(160,140,200,180));
                g2.setStroke(new BasicStroke(1.2f));
                g2.drawOval(ix+bw/2-4,cy+bh/2-4,8,8); g2.setStroke(def);
            }
            g2.setFont(new Font("SansSerif",Font.PLAIN,12));
            g2.setColor(COL_TEXT);
            g2.drawString(lbl[i],ix+bw+8,cy+bh-2);
        }
    }

    private void attachMouseListeners() {
        addMouseListener(new MouseAdapter() {
            @Override public void mousePressed(MouseEvent e) {
                int hit = hitEdge(e.getX(), e.getY());
                if (hit >= 0) {
                    dragEdgeIdx = hit;
                    // record offset from current ctrl point to mouse
                    // so dragging moves ctrl relative to where you grabbed
                    Point c = ctrlPx(hit);
                    dragOffX = e.getX() - c.x;
                    dragOffY = e.getY() - c.y;
                    setCursor(Cursor.getPredefinedCursor(Cursor.CROSSHAIR_CURSOR));
                } else {
                    panning  = true;
                    panDragX = e.getX()-offsetX;
                    panDragY = e.getY()-offsetY;
                    setCursor(Cursor.getPredefinedCursor(Cursor.MOVE_CURSOR));
                }
            }
            @Override public void mouseReleased(MouseEvent e) {
                dragEdgeIdx = -1; panning = false;
                setCursor(Cursor.getDefaultCursor());
            }
        });
        addMouseMotionListener(new MouseMotionAdapter() {
            @Override public void mouseDragged(MouseEvent e) {
                if (dragEdgeIdx >= 0) {
                    // move ctrl by the delta from grab offset
                    // so the line bends smoothly from wherever you grabbed it
                    int rawX = e.getX() - dragOffX - offsetX;
                    int rawY = e.getY() - dragOffY - offsetY;
                    edgeCtrl.put(dragEdgeIdx, new Point(rawX, rawY));
                    repaint();
                } else if (panning) {
                    offsetX = e.getX()-panDragX;
                    offsetY = e.getY()-panDragY;
                    repaint();
                }
            }
            @Override public void mouseMoved(MouseEvent e) {
                setCursor(hitEdge(e.getX(),e.getY()) >= 0
                    ? Cursor.getPredefinedCursor(Cursor.CROSSHAIR_CURSOR)
                    : Cursor.getDefaultCursor());
            }
        });
    }

    @Override
    public void run() {
        attachMouseListeners();
        JFrame frame = new JFrame("BFN Flow Network - " + algoTitle);
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        int prefH = Math.max(900, n*ROW_GAP + PAD_TOP + PAD_BOT + 300);
        frame.setPreferredSize(new Dimension(1700, prefH));
        JLabel banner = new JLabel("BFN: "+algoTitle, SwingConstants.CENTER);
        banner.setOpaque(true);
        banner.setBackground(new Color(116,72,168));
        banner.setForeground(Color.WHITE);
        banner.setFont(new Font("SansSerif",Font.BOLD,18));
        banner.setBorder(BorderFactory.createEmptyBorder(8,10,8,10));
        frame.setLayout(new BorderLayout());
        frame.add(banner, BorderLayout.NORTH);
        frame.add(this,   BorderLayout.CENTER);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}