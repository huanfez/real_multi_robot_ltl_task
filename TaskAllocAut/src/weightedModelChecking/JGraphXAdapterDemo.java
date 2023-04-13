//package weightedModelChecking;
//
//import com.mxgraph.layout.*;
//import com.mxgraph.swing.*;
//import com.mxgraph.util.mxConstants;
//
//import taskPlanning2.SubtaskAllocationAut;
//
//import org.jgrapht.*;
//import org.jgrapht.ext.*;
//import org.jgrapht.graph.*;
//
//import javax.swing.*;
//import java.awt.*;
//import java.util.ArrayList;
//
///**
// * A demo applet that shows how to use JGraphX to visualize JGraphT graphs. Applet based on
// * JGraphAdapterDemo.
// *
// */
//public class JGraphXAdapterDemo
//    extends
//    JApplet
//{
//    private static final long serialVersionUID = 2202072534703043194L;
//
//    private static final Dimension DEFAULT_SIZE = new Dimension(530, 320);
//
////    private JGraphXAdapter<String, DefaultEdge> jgxAdapter;
//    private JGraphXAdapter<ArrayList<String>, DefaultEdge> jgxAdapter;
//
//    /**
//     * An alternative starting point for this demo, to also allow running this applet as an
//     * application.
//     *
//     * @param args command line arguments
//     */
//    public static void main(String[] args)
//    {
//        JGraphXAdapterDemo applet = new JGraphXAdapterDemo();
//        applet.init();
//        
//        JFrame frame = new JFrame();
//        frame.getContentPane().add(applet);
//        frame.setTitle("JGraphT Adapter to JGraphX Demo");
//        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//        frame.pack();
//        frame.setVisible(true);
//    }
//
//    @Override
//    public void init()
//    {
//        // create a JGraphT graph       
//		Graph<ArrayList<String>, DefaultEdge> parallelTS12 = SubtaskAllocationAut.ParallelComposTS(
//				SubtaskAllocationAut.transitionSystem1, SubtaskAllocationAut.transitionSystem2);
//		Graph<ArrayList<String>, DefaultEdge> parallelTS123 = 
//				SubtaskAllocationAut.ParallelComposTS(parallelTS12, SubtaskAllocationAut.transitionSystem3);
//
//        // create a visualization using JGraph, via an adapter
//		jgxAdapter = new JGraphXAdapter<>(parallelTS123);
//        setPreferredSize(DEFAULT_SIZE);
//        mxGraphComponent component = new mxGraphComponent(jgxAdapter);
//        component.setConnectable(false);
//        component.getGraph().setAllowDanglingEdges(false);
//        component.getGraph().getStylesheet().getDefaultEdgeStyle().put(mxConstants.STYLE_NOLABEL, "1");
//        
//        mxGraphComponent component1 = new mxGraphComponent(jgxAdapter);
//        
//        mxGraphComponent component2 = new mxGraphComponent(jgxAdapter);
//        mxGraphComponent component3 = new mxGraphComponent(jgxAdapter);
//        
//        getContentPane().setLayout(new GridLayout(2,2));
//        getContentPane().add(component);
//        getContentPane().add(component1);
//        getContentPane().add(component2);
//        getContentPane().add(component3);
//        resize(DEFAULT_SIZE);
//        
//        // positioning via jgraphx layouts
//        mxCircleLayout layout = new mxCircleLayout(jgxAdapter);
//        
//        // center the circle
//        int radius = 100;
//        layout.setX0((DEFAULT_SIZE.width / 2.0) - radius);
//        layout.setY0((DEFAULT_SIZE.height / 2.0) - radius);
//        layout.setRadius(radius);
//        layout.setMoveCircle(true);
//        layout.execute(jgxAdapter.getDefaultParent());
//        
////        mxCircleLayout layout1 = new mxCircleLayout(jgxAdapter);
////        // that's all there is to it!...
////        layout1.setX0((DEFAULT_SIZE.width / 2.0));
////        layout1.setY0((DEFAULT_SIZE.height / 2.0));
////        layout1.setRadius(radius);
////        layout1.setMoveCircle(true);
////
////        layout1.execute(jgxAdapter.getDefaultParent());
//    }
//    
//}
