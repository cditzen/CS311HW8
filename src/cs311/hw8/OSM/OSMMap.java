package cs311.hw8.OSM;

import cs311.hw8.graph.IGraph;
import cs311.hw8.graphalgorithms.GraphAlgorithms;
import cs311.hw8.helpers.StreetData;
import org.w3c.dom.*;
import javax.xml.parsers.*;
import cs311.hw8.graph.Graph;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import static cs311.hw8.graphalgorithms.GraphAlgorithms.ShortestPath;

/**
 * Created by Cory Itzen on 11/27/2016.
 */
public class OSMMap {

    Graph<Location, StreetData> osmMap;

    public OSMMap() {
        osmMap = new Graph<>();
        osmMap.setDirectedGraph();
    }

    // TODO Clear old map
    public void LoadMap(String filename) throws ParserConfigurationException, IOException, SAXException {
        File input = new File(filename);
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        DocumentBuilder builder = factory.newDocumentBuilder();

        Document doc = builder.parse(input);
        doc.normalize();

        NodeList nodeList = doc.getElementsByTagName("node");

        int nListLength = nodeList.getLength();

        /** Iterate over entire list and add all nodes */
        for (int i = 0; i < nListLength; i++) {

            /** Get node */
            Element element = (Element) nodeList.item(i);

            /** Get id */
            String id = element.getAttribute("id");

            /** Get lat/long */
            double latitude = Double.parseDouble(element.getAttribute("lat"));
            double longitude = Double.parseDouble(element.getAttribute("lon"));
            Location location = new Location(latitude, longitude);

            /** Store these in a vertex in the graph */
            osmMap.addVertex(id, location);
        }

        NodeList wayList = doc.getElementsByTagName("way");
        int wListLength = wayList.getLength();

        /** Now add all edges */
        for (int i = 0; i < wListLength; i++) {

            /** Get way */
            Element element = (Element) wayList.item(i);

            /** List of vertex references */
            ArrayList<String> refs = new ArrayList<>();
            NodeList ndList = element.getElementsByTagName("nd");
            for (int j = 0; j < ndList.getLength(); j++) {
                refs.add(((Element) ndList.item(j)).getAttribute("ref"));
            }

            String name = "";
            boolean isHighway = false;
            boolean isTwoway = true;
            NodeList tagList = element.getElementsByTagName("tag");
            for (int j = 0; j < tagList.getLength(); j++) {
                Element subElement = (Element) tagList.item(j);
                if (subElement.getAttribute("k").equals("highway")) {
                    isHighway = true;
                } else if (subElement.getAttribute("k").equals("name")) {
                    name = subElement.getAttribute("v");
                } else if (subElement.getAttribute("k").equals("oneway") && subElement.getAttribute("v").equals("yes")) {
                    isTwoway = false;
                }
            }

            if (isHighway && !name.isEmpty()) {
                for (int j = 0; j < refs.size(); j++) {
                    /** Check if there is a next ref available in the list */
                    if (j + 1 < refs.size()) {
                        String v1 = refs.get(j);
                        String v2 = refs.get(j + 1);
                        IGraph.Vertex vertex1 = osmMap.getVertex(v1);
                        IGraph.Vertex vertex2 = osmMap.getVertex(v2);
                        double distance = ((Location) vertex1.getVertexData()).getDistance((Location) vertex2.getVertexData());
                        StreetData streetData = new StreetData(name, distance);
                        try {
                            osmMap.addEdge(v1, v2, streetData);
                        } catch(IGraph.DuplicateEdgeException e) {
                            System.out.println("Edge already exists: " + v1 + ", " + v2);
                        }
                        if (isTwoway) {
                            try {
                                osmMap.addEdge(v2, v1, streetData);
                            } catch(IGraph.DuplicateEdgeException e) {
                                System.out.println("Edge already exists: " + v2 + ", " + v1);
                            }
                        }
                    }
                }
            }
        }
    }

    public double TotalDistance() {
        List<IGraph.Edge<StreetData>> edges = osmMap.getEdges();
        double distance = 0.0;
        for (int i = 0; i < edges.size(); i++) {
            distance += edges.get(i).getEdgeData().getDistance();
        }
        /** Convert meters to miles */
        double metersPerMile = 1609.34;
        distance = distance / metersPerMile;

        /** Because there are few one-way streets in Ames, total distance is approximated by dividing by two */
        return distance / 2;
    }

    public static void main2(String[] args) throws ParserConfigurationException, IOException, SAXException {
        OSMMap osmMap = new OSMMap();
        osmMap.LoadMap("AmesMap.txt");  // TODO Where should this file be stored?
        double totalDistance = osmMap.TotalDistance();
        System.out.println("Total Distance: " + totalDistance);
    }

    /*******  Part 3  *******/

    private static class Location {

        double latitude;
        double longitude;

        Location(double lat, double lon) {
            this.latitude = lat;
            this.longitude = lon;
        }

        double getLatitude() {
            return latitude;
        }

        double getLongitude() {
            return longitude;
        }

        double getDistance(Location other) {

            /** http://stackoverflow.com/questions/3694380/calculating-distance-between-two-points-using-latitude-longitude-what-am-i-doi */

            final int R = 6371; // radius of the earth
            Double latDistance = Math.toRadians(other.getLatitude() - this.latitude);
            Double lonDistance = Math.toRadians(other.getLongitude() - this.longitude);
            Double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2)
                    + Math.cos(Math.toRadians(this.latitude)) * Math.cos(Math.toRadians(other.getLatitude()))
                    * Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);
            Double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
            Double distance = R * c * 1000;

            return distance;
        }
    }

    /**
     * Finds a connected vertex geographically closest to a given latitude and longitude in the OSMMap.
     * If no connected vertices exist in the map, an empty string is returned.
     * @param location Latitude and Longitude
     * @return id of the vertex closest to the given location
     */
    public String ClosestRoad(Location location) {
        List<IGraph.Vertex<Location>> vertices = osmMap.getVertices();
        if (vertices.size() < 1) {
            return "";
        }

        /** Shortest distance and its id */
        double shortestDistance = Double.MAX_VALUE;
        String id = "";

        /** Iterate over all vertices and find the shortest distance from the given location */
        for (int i = 0; i < vertices.size(); i++) {
            if (osmMap.getNeighbors(vertices.get(i).getVertexName()).size() > 0) {
                double distance = (vertices.get(i).getVertexData()).getDistance(location);
                if (distance <= shortestDistance && osmMap.getNeighbors(vertices.get(i).getVertexName()).size() > 0) {
                    shortestDistance = distance;
                    id = vertices.get(i).getVertexName();
                }
            }
        }
        return id;
    }

    /**
     * Returns list of vertex ids along the shortest route connecting two locations
     * @param fromLocation location to start from
     * @param toLocation location to end at
     * @return
     */
    public List<String> ShortestRoute(Location fromLocation, Location toLocation) {
        String v1 = ClosestRoad(fromLocation);
        String v2 = ClosestRoad(toLocation);
        List<IGraph.Edge<StreetData>> edges = GraphAlgorithms.ShortestPath(osmMap, v1, v2);
        List<String> shortestPath = new ArrayList<>();
        if (edges.size() == 0) {
            return shortestPath;
        }
        for (int i = 0; i < edges.size(); i++) {
            shortestPath.add(edges.get(i).getVertexName1());
        }
        shortestPath.add(edges.get(edges.size() - 1).getVertexName2());

        return shortestPath;
    }

    public List<String> StreetRoute(List<String> route) {

        List<String> streetRoute = new ArrayList<>();
        String lastStreet = "";

        for (int i = 0; i < route.size() - 1; i++) {
            String streetName = osmMap.getEdge(route.get(i), route.get(i + 1)).getEdgeData().getStreetName();
            if (!streetName.equals(lastStreet)) {
                streetRoute.add(streetName);
                lastStreet = streetName;
            }
        }
        return streetRoute;
    }

    public static void main(String[] args) throws ParserConfigurationException, IOException, SAXException {
        OSMMap osmMap = new OSMMap();
        osmMap.LoadMap("AmesMap.txt");

        /** List of locations to visit */
        ArrayList<Location> locations = new ArrayList<>();

        /** Scan route.txt for locations */
        File routeFile = new File("route.txt");
        Scanner scanner = new Scanner(routeFile);
        while(scanner.hasNext()) {
            double lat = Double.valueOf(scanner.next());
            double lon = Double.valueOf(scanner.next());
            locations.add(new Location(lat, lon));
        }
        scanner.close();

        /** List of vertex ids */
        List<String> shortestPath = new ArrayList<>();

        for (int i = 0; i < locations.size() - 1; i++) {
            List<String> path = osmMap.ShortestRoute(locations.get(i), locations.get(i + 1));

            /** Remove the last element to avoid duplicates */
            if (shortestPath.size() > 0) {
                shortestPath.remove(shortestPath.size() - 1);
            }
            shortestPath.addAll(path);
        }

        List<String> streetRoute = osmMap.StreetRoute(shortestPath);

        for (String street : streetRoute) {
            System.out.println(street);
        }
    }
}
