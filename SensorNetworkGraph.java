import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.util.*;
import java.util.List;
import javax.swing.*;

public class SensorNetworkGraph extends JPanel implements Runnable {
    private static final long serialVersionUID = 1L;

    // ── OR-Tools inspired palette ─────────────────────────────────────────────
    private static final Color COL_BG        = new Color(248, 248, 252);
    private static final Color COL_GRID      = new Color(220, 215, 235);
    private static final Color COL_DG        = new Color(130, 50,  200);
    private static final Color COL_DG_LIGHT  = new Color(200, 165, 240);
    private static final Color COL_ST        = new Color(20,  150, 80);
    private static final Color COL_ST_LIGHT  = new Color(120, 210, 160);
    private static final Color COL_EDGE      = new Color(100, 80,  160);   // darker, more visible
    private static final Color COL_FLOW      = new Color(220, 80,  20);
    private static final Color COL_FLOW_GLOW = new Color(255, 140, 40,  80);
    private static final Color COL_TEXT      = new Color(30,  15,  60);
    private static final Color COL_RELAY     = new Color(245, 185, 35);

    private static final int NODE_R    = 42;  // large enough to fit 4 lines of text cleanly
    private static final int gridCount = 10;
    private static final int legendW   = 300;

    private Map<Integer, Axis>        nodes;
    private double                    graphWidth, graphHeight;
    private int                       scaling = 60;
    private boolean                   connected;
    private Map<Integer,Set<Integer>> adjList;
    private String                    algoTitle = "GOA";

    private Set<Integer>         dgNodeIds      = new HashSet<>();
    private Set<Integer>         storageNodeIds = new HashSet<>();
    private List<int[]>          flowEdges      = new ArrayList<>();
    private Map<Integer,Integer> nodePriority   = new HashMap<>();
    private Map<Integer,Integer> nodePacketSize = new HashMap<>();
    private Map<Integer,Integer> nodePackets    = new HashMap<>();
    private Map<Integer,Integer> nodeStorageCap = new HashMap<>();
    private Map<Integer,Integer> nodeEnergy     = new HashMap<>();

    private Map<Integer,Point> nodePixels = new HashMap<>();
    private int dragNodeId = -1, dragOffsetX, dragOffsetY;
    private boolean pixelsInit = false;

    // ── setters ───────────────────────────────────────────────────────────────
    public void setDgNodeIds(Set<Integer> v)             { dgNodeIds=v;      repaint(); }
    public void setStorageNodeIds(Set<Integer> v)        { storageNodeIds=v; repaint(); }
    public void setFlowEdges(List<int[]> v)              { flowEdges=v;      repaint(); }
    public void setNodePriority(Map<Integer,Integer> v)  { nodePriority=v;   repaint(); }
    public void setNodePacketSize(Map<Integer,Integer> v){ nodePacketSize=v; repaint(); }
    public void setNodePackets(Map<Integer,Integer> v)   { nodePackets=v;    repaint(); }
    public void setNodeStorageCap(Map<Integer,Integer> v){ nodeStorageCap=v; repaint(); }
    public void setNodeEnergy(Map<Integer,Integer> v)    { nodeEnergy=v;     repaint(); }
    public void setAlgoTitle(String t)                   { algoTitle=t; }
    public boolean isConnected()                         { return connected; }
    public void setConnected(boolean c)                  { connected=c; }
    public Map<Integer,Set<Integer>> getAdjList()        { return adjList; }
    public void setAdjList(Map<Integer,Set<Integer>> a)  { adjList=a; }
    public void setNodes(Map<Integer,Axis> n)            { nodes=n; pixelsInit=false; invalidate(); repaint(); }
    public Map<Integer,Axis> getNodes()                  { return nodes; }
    public double getGraphWidth()                        { return graphWidth; }
    public void   setGraphWidth(double w)                { graphWidth=w; }
    public double getGraphHeight()                       { return graphHeight; }
    public void   setGraphHeight(double h)               { graphHeight=h; }

    private int graphAreaWidth()  { return getWidth()  - legendW - scaling; }
    private int graphAreaHeight() { return getHeight() - 2*scaling - 30; }

    // ── pixel init + spread ───────────────────────────────────────────────────
    private void initPixels() {
        if (getWidth()==0||getHeight()==0) return;
        double xs = graphAreaWidth()  / graphWidth;
        double ys = graphAreaHeight() / graphHeight;
        nodePixels.clear();
        for (Integer k : nodes.keySet()) {
            int px = (int)(nodes.get(k).getxAxis()*xs) + scaling;
            int py = (int)((graphHeight-nodes.get(k).getyAxis())*ys) + scaling;
            nodePixels.put(k, new Point(px, py));
        }
        spreadNodes();
        pixelsInit = true;
    }

    private void spreadNodes() {
        // Only clamp nodes to stay within the graph area boundary —
        // no overlap-pushing. Nodes are fixed at their true (x,y) coordinates
        // on first render and only move when the user drags them.
        int xMin=scaling+NODE_R+4, xMax=scaling+graphAreaWidth()-NODE_R-4;
        int yMin=scaling+NODE_R+4, yMax=scaling+graphAreaHeight()-NODE_R-4;
        for (Point p : nodePixels.values()) {
            p.x=Math.max(xMin,Math.min(xMax,p.x));
            p.y=Math.max(yMin,Math.min(yMax,p.y));
        }
    }

    private int hitNode(int mx, int my) {
        for (Map.Entry<Integer,Point> e : nodePixels.entrySet()) {
            Point p=e.getValue();
            if (Math.hypot(mx-p.x,my-p.y)<=NODE_R) return e.getKey();
        }
        return -1;
    }

    private void attachMouseListeners() {
        addMouseListener(new MouseAdapter() {
            @Override public void mousePressed(MouseEvent e) {
                if (!pixelsInit) initPixels();
                dragNodeId=hitNode(e.getX(),e.getY());
                if (dragNodeId!=-1){
                    Point p=nodePixels.get(dragNodeId);
                    dragOffsetX=e.getX()-p.x; dragOffsetY=e.getY()-p.y;
                }
            }
            @Override public void mouseReleased(MouseEvent e){dragNodeId=-1;}
        });
        addMouseMotionListener(new MouseMotionAdapter(){
            @Override public void mouseDragged(MouseEvent e){
                if(dragNodeId!=-1){
                    int xMin=scaling+NODE_R+4,xMax=scaling+graphAreaWidth()-NODE_R-4;
                    int yMin=scaling+NODE_R+4,yMax=scaling+graphAreaHeight()-NODE_R-4;
                    nodePixels.get(dragNodeId).setLocation(
                        Math.max(xMin,Math.min(xMax,e.getX()-dragOffsetX)),
                        Math.max(yMin,Math.min(yMax,e.getY()-dragOffsetY)));
                    repaint();
                }
            }
            @Override public void mouseMoved(MouseEvent e){
                if(!pixelsInit) return;
                setCursor(hitNode(e.getX(),e.getY())!=-1
                    ?Cursor.getPredefinedCursor(Cursor.HAND_CURSOR)
                    :Cursor.getDefaultCursor());
            }
        });
    }

    // ── paint ─────────────────────────────────────────────────────────────────
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        if (!pixelsInit) initPixels();
        if (nodePixels.isEmpty()) return;

        Graphics2D g2=(Graphics2D)g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,      RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

        int W=getWidth(),H=getHeight(),gaw=graphAreaWidth(),gah=graphAreaHeight();

        // background
        g2.setColor(COL_BG); g2.fillRect(0,0,W,H);
        g2.setColor(Color.WHITE); g2.fillRect(scaling,scaling,gaw,gah);
        g2.setColor(new Color(180,160,210));
        g2.setStroke(new BasicStroke(1.2f));
        g2.drawRect(scaling,scaling,gaw,gah);

        // grid + axis tick labels
        g2.setFont(new Font("SansSerif",Font.PLAIN,11));
        for (int i=0;i<=gridCount;i++){
            int y=scaling+i*gah/gridCount;
            g2.setColor(COL_GRID); g2.drawLine(scaling,y,scaling+gaw,y);
            g2.setColor(COL_TEXT);
            String lbl=String.format("%.1f",graphHeight*(1.0-(double)i/gridCount));
            FontMetrics fm=g2.getFontMetrics();
            g2.drawString(lbl,scaling-fm.stringWidth(lbl)-5,y+fm.getAscent()/2);
            int x=scaling+i*gaw/gridCount;
            g2.setColor(COL_GRID); g2.drawLine(x,scaling,x,scaling+gah);
            g2.setColor(COL_TEXT);
            String xl=String.format("%.1f",graphWidth*(double)i/gridCount);
            g2.drawString(xl,x-fm.stringWidth(xl)/2,scaling+gah+fm.getAscent()+4);
        }

        Stroke def=g2.getStroke();
        List<Integer> keyList=new ArrayList<>(nodes.keySet());
        Set<Integer> relayNodes=getRelayNodes();

        // ── BSN edges: use adjacency list keyed on 1-based node IDs ──────────
        // adjList keys and values are 1-based (set in launchGraph via u+1/v+1),
        // matching the nodePixels map which is also 1-based. This ensures the
        // sensor graph edges exactly mirror the BFN routing edges.
        Set<String> drawn=new HashSet<>();
        g2.setStroke(new BasicStroke(2f,BasicStroke.CAP_ROUND,BasicStroke.JOIN_ROUND));
        for (int nd : adjList.keySet()) {
            Set<Integer> nbrs = adjList.get(nd);
            if (nbrs==null) continue;
            for (int adj : nbrs) {
                // deduplicate undirected edges
                String key=Math.min(nd,adj)+"-"+Math.max(nd,adj);
                if (drawn.contains(key)) continue;
                drawn.add(key);
                Point p1=nodePixels.get(nd), p2=nodePixels.get(adj);
                if (p1==null||p2==null) continue;
                g2.setColor(COL_EDGE);
                drawLine(g2,p1,p2,NODE_R);
            }
        }

        // flow edges
        for (int[] fe : flowEdges) {
            // fe[0] and fe[1] are already 1-based (set in launchGraph)
            Point p1=nodePixels.get(fe[0]),p2=nodePixels.get(fe[1]);
            if (p1==null||p2==null) continue;
            g2.setColor(COL_FLOW_GLOW);
            g2.setStroke(new BasicStroke(16f,BasicStroke.CAP_ROUND,BasicStroke.JOIN_ROUND));
            drawLine(g2,p1,p2,NODE_R);
            g2.setColor(COL_FLOW);
            g2.setStroke(new BasicStroke(4f,BasicStroke.CAP_ROUND,BasicStroke.JOIN_ROUND));
            drawArrow(g2,p1,p2,NODE_R,true);
        }
        g2.setStroke(def);

        // nodes
        for (int nodeId : keyList) {
            Point p=nodePixels.get(nodeId);
            if (p==null) continue;
            boolean isDG=dgNodeIds.contains(nodeId);
            boolean isST=storageNodeIds.contains(nodeId);
            boolean isRelay=relayNodes.contains(nodeId);
            Color fill =isDG?COL_DG :isST?COL_ST :COL_EDGE;
            Color light=isDG?COL_DG_LIGHT:isST?COL_ST_LIGHT:COL_EDGE;

            // shadow
            g2.setColor(new Color(0,0,0,30));
            g2.fillOval(p.x-NODE_R+3,p.y-NODE_R+3,NODE_R*2,NODE_R*2);

            // radial gradient fill
            RadialGradientPaint rgp=new RadialGradientPaint(
                new Point(p.x-NODE_R/3,p.y-NODE_R/3),NODE_R*2,
                new float[]{0f,1f},new Color[]{light,fill});
            g2.setPaint(rgp);
            g2.fillOval(p.x-NODE_R,p.y-NODE_R,NODE_R*2,NODE_R*2);
            g2.setPaint(null);

            // border
            g2.setColor(dragNodeId==nodeId?Color.YELLOW:fill.darker());
            g2.setStroke(new BasicStroke(dragNodeId==nodeId?2.5f:1.8f));
            g2.drawOval(p.x-NODE_R,p.y-NODE_R,NODE_R*2,NODE_R*2);
            if (isRelay) {
                g2.setColor(COL_RELAY);
                g2.setStroke(new BasicStroke(2.2f,BasicStroke.CAP_ROUND,
                    BasicStroke.JOIN_ROUND,10f,new float[]{5f,4f},0f));
                g2.drawOval(p.x-NODE_R-4,p.y-NODE_R-4,NODE_R*2+8,NODE_R*2+8);
            }
            g2.setStroke(def);

            // ── text inside node ───────────────────────────────────────────
            g2.setColor(Color.WHITE);

            // header: "DG 9" or "ST 3" — uses 1-based nodeId
            String prefix=isDG?"DG":isST?"ST":"N";
            if(isRelay) prefix += "R";
            g2.setFont(new Font("SansSerif",Font.BOLD,12));
            drawCentered(g2,prefix+" "+nodeId,p.x,p.y-NODE_R+16);

            // divider
            g2.setColor(new Color(255,255,255,80));
            g2.drawLine(p.x-NODE_R+8,p.y-NODE_R+19,p.x+NODE_R-8,p.y-NODE_R+19);
            g2.setColor(Color.WHITE);

            g2.setFont(new Font("SansSerif",Font.BOLD,9));
            if (isDG) {
                int ly=p.y-NODE_R+30;
                if (nodePriority.containsKey(nodeId))
                    { drawCentered(g2,"v="+nodePriority.get(nodeId),p.x,ly); ly+=11; }
                if (nodePacketSize.containsKey(nodeId))
                    { drawCentered(g2,"sz="+nodePacketSize.get(nodeId),p.x,ly); ly+=11; }
                if (nodePackets.containsKey(nodeId))
                    { drawCentered(g2,"d="+nodePackets.get(nodeId),p.x,ly); ly+=11; }
                if (nodeEnergy.containsKey(nodeId))
                    drawCentered(g2,"E="+nodeEnergy.get(nodeId),p.x,ly);
            } else if (isST) {
                int ly=p.y-NODE_R+33;
                if (nodeStorageCap.containsKey(nodeId))
                    { drawCentered(g2,"cap="+nodeStorageCap.get(nodeId),p.x,ly); ly+=12; }
                if (nodeEnergy.containsKey(nodeId))
                    drawCentered(g2,"E="+nodeEnergy.get(nodeId),p.x,ly);
            } else {
                int ly=p.y-NODE_R+36;
                if (nodeEnergy.containsKey(nodeId))
                    { drawCentered(g2,"E="+nodeEnergy.get(nodeId),p.x,ly); ly+=12; }
                if (isRelay) drawCentered(g2,"relay",p.x,ly);
            }

            // (x,y) coords at bottom
            if (nodes.containsKey(nodeId)) {
                g2.setFont(new Font("SansSerif",Font.PLAIN,9));
                g2.setColor(new Color(255,255,255,180));
                drawCentered(g2,
                    String.format("(%.0f,%.0f)",
                        nodes.get(nodeId).getxAxis(),
                        nodes.get(nodeId).getyAxis()),
                    p.x, p.y+NODE_R-5);
            }
        }

        // axis labels
        g2.setColor(COL_TEXT);
        g2.setFont(new Font("SansSerif",Font.BOLD,13));
        FontMetrics fmAx=g2.getFontMetrics();
        String xLbl="X Position (m)";
        g2.drawString(xLbl,scaling+gaw/2-fmAx.stringWidth(xLbl)/2,getHeight()-8);
        Graphics2D g2r=(Graphics2D)g2.create();
        g2r.setFont(new Font("SansSerif",Font.BOLD,13));
        g2r.rotate(-Math.PI/2);
        String yLbl="Y Position (m)";
        FontMetrics fmY=g2r.getFontMetrics();
        g2r.drawString(yLbl,-(scaling+gah/2+fmY.stringWidth(yLbl)/2),16);
        g2r.dispose();

        drawLegend(g2,def,W,H);
    }

    // ── draw helpers ──────────────────────────────────────────────────────────

    /** Straight line between node borders (no arrowhead) — for BSN edges. */
    private void drawLine(Graphics2D g2, Point p1, Point p2, int r) {
        double dx=p2.x-p1.x, dy=p2.y-p1.y;
        double len=Math.sqrt(dx*dx+dy*dy); if(len<2) return;
        double ux=dx/len, uy=dy/len;
        int x1=(int)(p1.x+ux*r), y1=(int)(p1.y+uy*r);
        int x2=(int)(p2.x-ux*r), y2=(int)(p2.y-uy*r);
        g2.drawLine(x1,y1,x2,y2);
    }

    /** Straight arrow — for flow edges. */
    private void drawArrow(Graphics2D g2, Point p1, Point p2, int r, boolean head) {
        double dx=p2.x-p1.x,dy=p2.y-p1.y;
        double len=Math.sqrt(dx*dx+dy*dy); if(len<2) return;
        double ux=dx/len,uy=dy/len;
        int x1=(int)(p1.x+ux*r),y1=(int)(p1.y+uy*r);
        int x2=(int)(p2.x-ux*(r+4)),y2=(int)(p2.y-uy*(r+4));
        g2.drawLine(x1,y1,x2,y2);
        if(head){
            int aw=10,ah=5;
            g2.fillPolygon(
                new int[]{x2,(int)(x2-aw*ux+ah*uy),(int)(x2-aw*ux-ah*uy)},
                new int[]{y2,(int)(y2-aw*uy-ah*ux),(int)(y2-aw*uy+ah*ux)},3);
        }
    }

    private void drawCentered(Graphics2D g2, String s, int cx, int y) {
        FontMetrics fm=g2.getFontMetrics();
        g2.drawString(s,cx-fm.stringWidth(s)/2,y);
    }

    private Set<Integer> getRelayNodes() {
        Map<Integer,Integer> deg=new HashMap<>();
        for(int[] fe:flowEdges){
            if(fe==null||fe.length<2) continue;
            deg.merge(fe[0],1,Integer::sum);
            deg.merge(fe[1],1,Integer::sum);
        }
        Set<Integer> relay=new HashSet<>();
        for(Map.Entry<Integer,Integer> e:deg.entrySet())
            if(e.getValue()>=2) relay.add(e.getKey());
        return relay;
    }

    private void drawLegend(Graphics2D g2, Stroke def, int W, int H) {
        String[] lbls={"DG (source)  - v, sz, d, E","Storage node  - cap, E",
                        "BSN edge","GOA flow path","Relay-capable node ring","Drag to rearrange"};
        Color[] cols={COL_DG,COL_ST,COL_EDGE,COL_FLOW,COL_RELAY,new Color(80,80,80)};
        g2.setFont(new Font("SansSerif",Font.PLAIN,12));
        FontMetrics fm=g2.getFontMetrics();
        int maxW=0; for(String s:lbls) maxW=Math.max(maxW,fm.stringWidth(s));
        int bw=16,bh=16,gap=10,px=14,py=12;
        int boxW=maxW+bw+px*2+10,boxH=lbls.length*(bh+gap)+py*2;
        int rx=W-boxW-12,ry=(H-boxH)/2;
        g2.setColor(new Color(0,0,0,30));       g2.fillRoundRect(rx+3,ry+3,boxW,boxH,12,12);
        g2.setColor(new Color(255,255,255,245)); g2.fillRoundRect(rx,ry,boxW,boxH,12,12);
        g2.setColor(new Color(180,160,210));
        g2.setStroke(new BasicStroke(1.2f));     g2.drawRoundRect(rx,ry,boxW,boxH,12,12);
        for(int i=0;i<lbls.length;i++){
            int cy=ry+py+i*(bh+gap),ix=rx+px;
            if(i<2){
                RadialGradientPaint rgp=new RadialGradientPaint(
                    new Point(ix+bw/2-2,cy+bh/2-2),bw,new float[]{0f,1f},
                    new Color[]{cols[i].brighter(),cols[i]});
                g2.setPaint(rgp); g2.fillOval(ix,cy,bw,bh); g2.setPaint(null);
                g2.setColor(cols[i].darker()); g2.setStroke(new BasicStroke(1f));
                g2.drawOval(ix,cy,bw,bh);
            } else if(i==2){
                g2.setColor(cols[i]); g2.setStroke(new BasicStroke(2f,BasicStroke.CAP_ROUND,BasicStroke.JOIN_ROUND));
                g2.drawLine(ix,cy+bh/2,ix+bw,cy+bh/2); g2.setStroke(def);
            } else if(i==3){
                g2.setColor(new Color(255,140,40,80));
                g2.setStroke(new BasicStroke(8f,BasicStroke.CAP_ROUND,BasicStroke.JOIN_ROUND));
                g2.drawLine(ix,cy+bh/2,ix+bw,cy+bh/2);
                g2.setColor(cols[i]);
                g2.setStroke(new BasicStroke(2.5f,BasicStroke.CAP_ROUND,BasicStroke.JOIN_ROUND));
                g2.drawLine(ix,cy+bh/2,ix+bw,cy+bh/2); g2.setStroke(def);
            } else if(i==4){
                g2.setColor(cols[i]);
                g2.setStroke(new BasicStroke(2f,BasicStroke.CAP_ROUND,BasicStroke.JOIN_ROUND,10f,new float[]{5f,4f},0f));
                g2.drawOval(ix,cy,bw,bh); g2.setStroke(def);
            } else {
                g2.setFont(new Font("SansSerif",Font.PLAIN,15));
                g2.setColor(cols[i]); g2.drawString("\u2725",ix,cy+bh-1);
                g2.setFont(new Font("SansSerif",Font.PLAIN,12));
            }
            g2.setColor(COL_TEXT); g2.setFont(new Font("SansSerif",Font.PLAIN,12));
            g2.drawString(lbls[i],ix+bw+8,cy+bh-2);
        }
    }

    @Override
    public void run(){
        attachMouseListeners();
        JFrame frame=new JFrame("Sensor Network - "+algoTitle);
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        frame.setPreferredSize(new Dimension(1550,900));
        JLabel banner=new JLabel(algoTitle,SwingConstants.CENTER);
        banner.setFont(new Font("SansSerif",Font.BOLD,18));
        banner.setOpaque(true);
        banner.setBackground(new Color(60,20,120));
        banner.setForeground(Color.WHITE);
        banner.setBorder(BorderFactory.createEmptyBorder(10,0,10,0));
        frame.getContentPane().setLayout(new BorderLayout());
        frame.getContentPane().add(banner,BorderLayout.NORTH);
        frame.getContentPane().add(this,BorderLayout.CENTER);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}