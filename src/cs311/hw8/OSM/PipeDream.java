package cs311.hw8.OSM;

import cs311.hw8.graph.Graph;
import cs311.hw8.graph.IGraph;
import cs311.hw8.graphalgorithms.GraphAlgorithms;
import cs311.hw8.graphalgorithms.IWeight;

import java.util.List;

/**
 * Created by Cory Itzen on 12/5/2016.
 * Class that outputs the weight sum of the MST of the given OSMMap
 */
public class PipeDream {

    OSMMap osmMap;

    PipeDream(OSMMap graph) {
            this.osmMap = graph;
        }

    /**
     * Weight sum of the MST of the OSMMap
     * @return
     */
    public double getMSTWeightSum() {
        List<IGraph.Edge> mstEdges = GraphAlgorithms.Kruscal((Graph) osmMap.osmMap).getEdges();
        double weightSum = 0.0;
        for (IGraph.Edge<IWeight> edge : mstEdges) {
            weightSum += edge.getEdgeData().getWeight();
        }
        return osmMap.metersToMiles(weightSum);
    }
}
