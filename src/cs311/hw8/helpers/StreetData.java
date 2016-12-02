package cs311.hw8.helpers;

import cs311.hw8.graphalgorithms.IWeight;

/**
 * Created by cditz_000 on 11/28/2016.
 */
public class StreetData implements IWeight {

    String streetName;
    double distance;

    public StreetData(String streetName, double distance) {
        this.streetName = streetName;
        this.distance = distance;
    }

    public String getStreetName() {
        return streetName;
    }

    public double getDistance() {
        return distance;
    }

    public double getWeight() {
        return distance;
    }
}
