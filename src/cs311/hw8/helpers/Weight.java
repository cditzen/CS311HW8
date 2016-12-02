package cs311.hw8.helpers;

import cs311.hw8.graph.IGraph;
import cs311.hw8.graphalgorithms.IWeight;

/**
 * Created by cditz_000 on 11/6/2016.
 */
public class Weight implements IWeight {

    private double weight;

    public Weight(double weight) {
        this.weight = weight;
    }

    @Override
    public double getWeight() {
        return weight;
    }
}
