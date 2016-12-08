package cs311.hw8.OSM;

import cs311.hw8.graph.IGraph;
import cs311.hw8.graphalgorithms.GraphAlgorithms;
import cs311.hw8.graphalgorithms.IWeight;
import cs311.hw8.helpers.StreetData;
import org.w3c.dom.*;
import javax.xml.parsers.*;
import cs311.hw8.graph.Graph;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Scanner;


/**
 * Created by Cory Itzen on 11/27/2016.
 */
public class OSMMap {

    /****************************  Main Methods  ****************************/

    /**
     * Part 2 Main Method.
     * Parses an input file containing the map file.
     * Outputs the approximate total distance of all streets in the city.
     *
     * @param args
     * @throws ParserConfigurationException
     * @throws IOException
     * @throws SAXException
     */
    public static void main2(String[] args) throws ParserConfigurationException, IOException, SAXException {
        if (args.length < 1) {
            throw new IllegalArgumentException("Must have at least one argument: input map file name.");
        }

        String mapFileString = args[0];

        OSMMap osmMap = new OSMMap();
        osmMap.LoadMap(mapFileString);
        double totalDistance = osmMap.TotalDistance();
        System.out.println("Total Distance: " + totalDistance);
    }

    /**
     * Part 3 Main Method.
     * Parses two input file containing locations; first is the map file, second is the locations to visit.
     * Prints a list of street locations that are traveled to navigate the locations given in the input file.
     *
     * @param args
     * @throws ParserConfigurationException
     * @throws IOException
     * @throws SAXException
     */
    public static void main3(String[] args) throws ParserConfigurationException, IOException, SAXException {
        if (args.length < 2) {
            throw new IllegalArgumentException("Must have two arguments: input map file name, input location file name.");
        }

        String mapFileString = args[0];
        String locationFileString = args[1];

        OSMMap osmMap = new OSMMap();
        osmMap.LoadMap(mapFileString);

        /** List of locations to visit */
        ArrayList<Location> locations = new ArrayList<>();

        /** Scan route file for locations */
        File routeFile = new File(locationFileString);
        Scanner scanner = new Scanner(routeFile);
        while(scanner.hasNextLine()) {
            String line = scanner.nextLine();
            Scanner lineScanner = new Scanner(line);
            double lat = Double.valueOf(lineScanner.next());
            double lon = Double.valueOf(lineScanner.next());
            locations.add(new Location(lat, lon));
            lineScanner.close();
        }
        scanner.close();

        /** List of vertex ids */
        List<String> shortestPath = new ArrayList<>();

        for (int i = 0; i < locations.size() - 1; i++) {
            List<String> path = osmMap.ShortestRoute(locations.get(i), locations.get(i + 1));

            /** Remove the last element to avoid duplicates */
            if (shortestPath.size() > 0 && shortestPath.get(shortestPath.size() - 1).equals(path.get(0))) {
                shortestPath.remove(shortestPath.size() - 1);
            }
            shortestPath.addAll(path);
        }

        List<String> streetRoute = osmMap.StreetRoute(shortestPath);

        for (String street : streetRoute) {
            System.out.println(street);
        }
    }

    /**
     * Part 4 main method
     * Parses two input file containing locations; first is the map file, second is the locations to visit in the TSP
     * Prints the vertex IDs of the appriximate TSP of the given locations.
     * Also prints the "Pipe Dream" Weight Sum of the MST.
     * @param args input text file
     * @throws ParserConfigurationException
     * @throws IOException
     * @throws SAXException
     */
    public static void main4(String[] args) throws ParserConfigurationException, IOException, SAXException {
        if (args.length < 2) {
            throw new IllegalArgumentException("Must have two arguments: input map file name, input location file name.");
        }

        String mapFileString = args[0];
        String locationFileString = args[1];

        OSMMap osmMap = new OSMMap();
        osmMap.LoadMap(mapFileString);

        /** List of locations to visit */
        ArrayList<Location> locations = new ArrayList<>();

        /** Scan route file for locations */
        File routeFile = new File(locationFileString);
        Scanner scanner = new Scanner(routeFile);
        while(scanner.hasNextLine()) {
            String line = scanner.nextLine();
            Scanner lineScanner = new Scanner(line);
            double lat = Double.valueOf(lineScanner.next());
            double lon = Double.valueOf(lineScanner.next());
            locations.add(new Location(lat, lon));
            lineScanner.close();
        }
        scanner.close();

        /** List of vertex ids given in the input file */
        ArrayList<String> vertices = new ArrayList<>();
        for (Location location : locations) {
            vertices.add(osmMap.ClosestRoad(location));
        }

        /** List of Vertex IDs of the approximate TSP tour of the given locations */
        List<String> tsp = osmMap.ApproximateTSP(vertices);
        for (String vertex : tsp) {
            System.out.println("Vertex ID: " + vertex);
        }

        /** Output the MST of the graph */
        PipeDream pipeDream = new PipeDream(osmMap);
        System.out.println("Pipe Dream WeightSum: " + pipeDream.getMSTWeightSum() + " miles");
    }

    /****************************  Part 2  ****************************/

    Graph<Location, StreetData> osmMap;

    /** Creates a new OSMMap that is directed */
    public OSMMap() {
        osmMap = new Graph<>();
        osmMap.setDirectedGraph();
    }

    /**
     * Fills the OSM Map with data from an input XML file.
     *
     * @param filename
     * @throws ParserConfigurationException
     * @throws IOException
     * @throws SAXException
     */
    public void LoadMap(String filename) throws ParserConfigurationException, IOException, SAXException {

        // Clear existing map
        osmMap = new Graph<>();
        osmMap.setDirectedGraph();

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

            /** Search for highways, and check if they are one-ways */
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

            /** Add the edge to the osmMap */
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
                        } catch (IGraph.DuplicateEdgeException e) {
                            System.out.println("Edge already exists: " + v1 + ", " + v2);
                        }
                        if (isTwoway) {
                            try {
                                osmMap.addEdge(v2, v1, streetData);
                            } catch (IGraph.DuplicateEdgeException e) {
                                System.out.println("Edge already exists: " + v2 + ", " + v1);
                            }
                        }
                    }
                }
            }
        }
    }

    /**
     * Returns an approximate total distance of all edges in a map in miles.
     * @return approximate distance in miles
     */
    public double TotalDistance() {
        List<IGraph.Edge<StreetData>> edges = osmMap.getEdges();
        double distance = 0.0;
        for (int i = 0; i < edges.size(); i++) {
            distance += edges.get(i).getEdgeData().getDistance();
        }
        /** Convert meters to miles */
        distance = metersToMiles(distance);

        /** Because there are few one-way streets in Ames, total distance is approximated by dividing by two */
        return distance / 2;
    }

    /**
     * Converts a distance given in meters to units of miles.
     * @param meters distance to convert
     * @return distance in miles
     */
    public double metersToMiles(double meters) {
        return meters / 1609.34;
    }

    /****************************  Part 3  ****************************/

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

            /** Ames is pretty flat, so no need to account for elevation */

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
     * Returns list of vertex IDs that create the shortest route connecting two locations
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

    /**
     * Creates a list of street names from a list of vertices to be traveled
     * Removes duplicate street names
     * @param route List of Vertex IDs. Vertices must be ordered in such a way that an edge exists between all
     *              consecutive vertices in the list.
     * @return
     */
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

    /****************************  Part 4  ****************************/

    /**
     * Uses the Nearest Neighbor Algorithm to find an approximate tour.
     * Nearest Neighbor is a Greedy algorithm that on average yields a tour 25% longer than the length of the optimal
     * solution.
     * The next node in the TSP is chosen by picking the vertex that is closest to the current location.
     * The starting node is added at the end of the algorithm to ensure that it is a tour.
     *
     *  https://en.wikipedia.org/wiki/Travelling_salesman_problem#Constructive_heuristics
     *
     * @param vertices List of Strings containing vertex ids within the graph
     *                  First location is the starting location
     * @return
     */
    public List<String> ApproximateTSP(ArrayList<String> vertices) {

        if (vertices.size() < 1) {
            return Collections.emptyList();
        }

        /** List of vertices in the approximate TSP */
        List<String> approximateTSP = new ArrayList<>();

        /** Initialize the first vertex as the starting location */
        IGraph.Vertex<Location> currentVertex = osmMap.getVertex(vertices.get(0));
        vertices.remove(0);
        approximateTSP.add(currentVertex.getVertexName());

        /** Iterate through all vertices, greedily adding them to the TSP */
        while (vertices.size() > 0) {
            int closestVertexIndex = 0;
            Location currentLocation = new Location(currentVertex.getVertexData().getLatitude(), currentVertex.getVertexData().getLongitude());
            double shortestDistance = Double.MAX_VALUE;

            /** Search through all remaining vertices and find the vertex geographically closest to the current vertex*/
            for (int i = 0; i < vertices.size(); i++) {
                IGraph.Vertex<Location> nextVertex = osmMap.getVertex(vertices.get(i));
                Location nextLocation = new Location(nextVertex.getVertexData().getLatitude(), nextVertex.getVertexData().getLongitude());
                double distance = currentLocation.getDistance(nextLocation);
                if (distance < shortestDistance) {
                    closestVertexIndex = i;
                    currentLocation = nextLocation;
                    shortestDistance = distance;
                }
            }

            /** Set the current vertex and add it to the TSP */
            currentVertex = osmMap.getVertex(vertices.get(closestVertexIndex));
            approximateTSP.add(vertices.get(closestVertexIndex));

            /** Remove the current vertex from the remaining vertices to visit. */
            vertices.remove(closestVertexIndex);
        }

        /** Return to the starting vertex */
        if (approximateTSP.size() > 1) {
            approximateTSP.add(approximateTSP.get(0));
        }

        return approximateTSP;
    }
}
