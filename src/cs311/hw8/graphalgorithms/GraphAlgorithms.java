
package cs311.hw8.graphalgorithms;

import cs311.hw8.graph.Graph;
import cs311.hw8.graph.IGraph;
import cs311.hw8.graph.IGraph.Vertex;
import cs311.hw8.helpers.VertexTree;

import java.util.*;

public class GraphAlgorithms
{
    /**
     * Represents a vertex's state in a Depth First Search
     */
    private enum State {
        UNDISCOVERED, DISCOVERED, COMPLETED
    }

    /**
     * Finds a topological sort of all vertices in a given graph.
     * Performs a Depth First Search and adds vertices
     * in processVertexLate. A DFS is started at every
     * 0-In-Degree vertex, and these searches are combined
     * into one list.
     *
     * If the graph is not a DAG, an empty list is returned.
     *
     * Topological Sort works in O(V + E) time to perform a
     * DFS on all nodes.
     *
     * @param g Graph to
     * @param <V>
     * @param <E>
     * @return
     */
    public static <V, E> List<Vertex<V>> TopologicalSort(IGraph<V, E> g) {

        /** Initialize visited HashMap with each vertex as UNDISCOVERED */
        HashMap<Integer, State> visited = new HashMap<>();
        for (Vertex vertex : g.getVertices()) {
            visited.put(vertex.hashCode(), State.UNDISCOVERED);
        }

        /** Get all 0-In-Degree Vertices */
        List<Vertex> startingVertices = getZeroInDegreeVertices(g);

        /** A list representing the topological sort */
        LinkedList<Vertex<V>> topologicalSort = new LinkedList<>();

        /** Perform a DFS starting at every startingVertex */
        for (Vertex startingVertex : startingVertices) {
            visited.put(startingVertex.hashCode(), State.DISCOVERED);
            TopologicalSort(g, startingVertex, visited, topologicalSort);
        }
        if (topologicalSort.size() != g.getVertices().size()) {
            throw new RuntimeException("Not all vertices were able to be processed. Not a DAG");
        }
        return topologicalSort;
    }

    /**
     * Recursive Depth First Search
     * ProcessVertexLate adds vertices to a list
     *
     * DFS works in O(V + E) time
     *
     * @param g Graph to search through
     * @param startingVertex Vertex to start search from
     * @param visited HashMap storing current state of each Vertex
     * @param path List of topological sort
     * @param <V>
     * @param <E>
     * @return
     */
    public static <V, E> List<Vertex<V>> TopologicalSort( IGraph<V, E> g, Vertex startingVertex, HashMap<Integer, State> visited, List<Vertex<V>> path) {

        /** Iterate over all neighbors of the starting node */
        List<IGraph.Vertex<V>> neighbors = g.getNeighbors(startingVertex.getVertexName());
        for (Vertex neighbor : neighbors) {

            /** If a neighbor is undiscovered, perform DFS on it */
            if (visited.get(neighbor.hashCode()) == State.UNDISCOVERED) {
                visited.put(neighbor.hashCode(), State.DISCOVERED);
                TopologicalSort(g, neighbor, visited, path);
            } else if (visited.get(neighbor.hashCode()) == State.DISCOVERED){
                throw new RuntimeException("Cycle found. Graph is not a DAG");
            }
        }

        /** Once a node has all neighbors processed, add it to the topological sort */
        path.add(0, startingVertex);
        visited.put(startingVertex.hashCode(), State.COMPLETED);

        return path;
    }

    /**
     * Gets all 0-In-Degree vertices in the graph
     * @param g graph to search
     * @param <V>
     * @param <E>
     * @return
     */
    private static <V, E> List<Vertex> getZeroInDegreeVertices(IGraph<V, E> g) {
        return getZeroInDegreeVertices(g, Collections.emptyList());
    }

    /**
     * Gets all 0-In-Degree vertices after removing a set of specified vertices. Edges coming from these
     * vertices will not be counted as In-Degree Edges to any other vertices
     *
     * This runs in O(V + E) time
     *
     * @param g graph to search
     * @param verticesToIgnore list of vertices to ignore
     * @param <V>
     * @param <E>
     * @return
     */
    private static <V, E> List<Vertex> getZeroInDegreeVertices(IGraph<V, E> g, List<Vertex> verticesToIgnore) {
        HashMap<Integer, Vertex> vertexMap = new HashMap<>();
        HashMap<Integer, Boolean> ignoreMap = new HashMap<>();

        /** Create vertexMap */
        for (Vertex vertex : g.getVertices()) {
            vertexMap.put(vertex.hashCode(), vertex);
        }

        /** Set vertices to ignore */
        for (Vertex vertexToIgnore : verticesToIgnore) {
            ignoreMap.put(vertexToIgnore.hashCode(), true);
            vertexMap.remove(vertexToIgnore.hashCode());
        }

        /** Remove vertices that have any incoming edges */
        for (IGraph.Edge edge : g.getEdges()) {

            /** Don't count edged that come from a vertex that is ignored */
            if (ignoreMap.containsKey(g.getVertex(edge.getVertexName1()).hashCode()) == false) {
                vertexMap.remove(g.getVertex(edge.getVertexName2()).hashCode());
            }
        }
        return new ArrayList<Vertex>(vertexMap.values());
    }

    /**
     * Finds every possible TopologicalSort given a directed acyclic graph.
     * A tree keeps track of all topological sorts. A queue keeps track of vertices
     * that will be processed. When a node is processed, all possible vertices that
     * can come after it in a sort will be added to it as children.
     *
     * This runs in O(V(V + E) * n) time, where n is the number of Topological Sort permutations
     *
     * NOTE: This does not handle cyclic graphs.
     *
     * @param g graph to search
     * @param <V>
     * @param <E>
     * @return
     */
    public static <V, E> List<List<Vertex<V>>> AllTopologicalSort( IGraph<V, E> g )
    {
        /** Ensure the graph is directed */
        if (!g.isDirectedGraph()) {
            throw new RuntimeException("Graph is undirected. Must be a directed acyclic graph");
        }

        /** Create a tree of all possible topological sorts */
        VertexTree topologicalTree = new VertexTree();

        /** List of leaves of the topologicalTree */
        List<VertexTree.Node> leaves = new ArrayList<>();

        /** Create a Queue of Vertices to process */
        Queue<VertexTree.Node> queue = new LinkedList<>();

        /** Add all first possible 0-In-Degree Vertices to the tree and the queue */
        List<Vertex> startingVertices = getZeroInDegreeVertices(g);
        for (Vertex startingVertex : startingVertices) {
            VertexTree.Node node = new VertexTree.Node(startingVertex);
            topologicalTree.getRoot().addChild(node);
            queue.add(node);
        }

        /** Process every vertex in every topological sort */
        while (queue.size() > 0) {

            /** Process the next node in the queue */
            VertexTree.Node node = queue.remove();
            VertexTree.Node temp = node;

            /** Get list of selected vertices starting from the current node and then moving to the root */
            List<Vertex> verticesToIgnore = new ArrayList<>();
            while (temp.getParent() != null) {
                verticesToIgnore.add(temp.getVertex());
                temp = temp.getParent();
            }

            /** Get a list of 0-In-Degree vertices after removing the parent vertices */
            List<Vertex> zeroInDegreeVertices = getZeroInDegreeVertices(g, verticesToIgnore);

            /** If the graph has no more vertices, this node is the last element in a topological sort */
            if (zeroInDegreeVertices.size() == 0) {
                leaves.add(node);
            }

            /** After removing this vertex, add all new 0-In-Degree vertices to be processed. */
            for (Vertex zeroInDegreeVertex : zeroInDegreeVertices) {
                VertexTree.Node child = new VertexTree.Node(zeroInDegreeVertex);
                node.addChild(child);
                queue.add(child);
            }
        }

        /** List of all possible topological sorts */
        List<List<Vertex<V>>> allTopologicalSorts = new ArrayList<>();

        /** Add each topological sort by iterating over all leaves of the tree */
        for (VertexTree.Node leaf : leaves) {
            List<Vertex<V>> topologicalSort = new LinkedList<>();
            VertexTree.Node temp = leaf;
            while (temp.getParent() != null) {
                topologicalSort.add(0, temp.getVertex());
                temp = temp.getParent();
            }
            allTopologicalSorts.add(topologicalSort);
        }
        return allTopologicalSorts;
    }

    /**
     * Finds a MST by selecting non-connected edges in order of smallest weighed edges first.
     * All edges must have an edgeData that implements IWeight.
     *
     * It takes O(E logE) time to perform mergeSort, and O(E logE) to create MST of |V| - 1 edges.
     * This algorithm runs in O(E logE) time
     *
     * @param g graph to search
     * @param <V>
     * @param <E>
     * @return
     */
    public static <V, E extends IWeight> IGraph<V, E> Kruscal(IGraph<V, E> g )
    {
        /** Get all vertices and edges */
        List<IGraph.Edge<E>> edges = g.getEdges();
        List<IGraph.Vertex<V>> vertices = g.getVertices();

        /** Sort edges by edge weight */
        mergeSort(edges);

        /** Put vertices into components */
        Components components = new Components(vertices);

        /** Prepare MST graph */
        Graph<V, E> kruskalMST = new Graph();
        if (g.isDirectedGraph()) {
            kruskalMST.setDirectedGraph();
        } else {
            kruskalMST.setUndirectedGraph();
        }
        for (Vertex<V> vertex : vertices) {
            kruskalMST.addVertex(vertex.getVertexName(), vertex.getVertexData());
        }

        int currentIndex = 0;

        /** Add the smallest edge to the MST up to |V| - 1 vertices,
         * as long as the edge doesn't connect two vertices in the same component */
        int graphNumVertices = g.getVertices().size();
        int graphNumEdges = g.getEdges().size();
        int kruskalNumEdges = 0;
        while (kruskalNumEdges < graphNumVertices - 1 && currentIndex < graphNumEdges) {
            IGraph.Edge<E> edge = edges.get(currentIndex);
            Vertex v1 = g.getVertex(edge.getVertexName1());
            Vertex v2 = g.getVertex(edge.getVertexName2());
            if (!components.sameComponent(v1, v2)) {
                components.mergeComponents(v1, v2);
                kruskalMST.addEdge(edge.getVertexName1(), edge.getVertexName2(), edge.getEdgeData());
                kruskalNumEdges++;
            }
            currentIndex++;
        }
        return kruskalMST;
    }

    /**
     * Sorts a list of edges based on Edge Weight
     * All edges must implement IWeight
     * @param edges list of edges to sort
     * @param <E>
     */
    private static <E extends IWeight> void mergeSort(List<IGraph.Edge<E>> edges) {
        mergeSort(edges, 0, edges.size() - 1);
    }

    /**
     * Divides and conquer a list of edges
     * @param edges list of edges to sort
     * @param left left index
     * @param right right index
     * @param <E>
     */
    private static <E extends IWeight> void mergeSort(List<IGraph.Edge<E>> edges, int left, int right) {
        if (left < right) {
            int mid = left + (right - left) / 2;
            mergeSort(edges, left, mid);
            mergeSort(edges, mid + 1, right);
            merge(edges, left, mid, right);
        }
    }

    /**
     * Merges two array subsets into one ordered subset
     * @param edges List of edges to sort
     * @param left left index of the first subset
     * @param mid dividing index between two subsets
     * @param right right index of the second subset
     * @param <E>
     */
    private static <E extends IWeight> void merge(List<IGraph.Edge<E>> edges, int left, int mid, int right) {
        /** Make a copy of the list of edges */
        IGraph.Edge<E>[] copy = new IGraph.Edge[edges.size()];
        for (int i = left; i <= right; i++) {
            copy[i] = edges.get(i);
        }

        /** Indices to iterate over */
        int leftIndex = left;
        int rightIndex = mid + 1;
        int index = left;

        /** Merge two array subsets based in increasing weight */
        while (leftIndex <= mid && rightIndex <= right) {
            if (copy[leftIndex].getEdgeData().getWeight() <= copy[rightIndex].getEdgeData().getWeight()) {
                edges.set(index, copy[leftIndex]);
                leftIndex++;
            } else {
                edges.set(index, copy[rightIndex]);
                rightIndex++;
            }
            index++;
        }

        /** Fill in remaining edges */
        while (leftIndex <= mid) {
            edges.set(index, copy[leftIndex]);
            index++;
            leftIndex++;
        }
    }

    /**
     * Components class assigns a list of vertices a unique component, and allows for them to be compared and merged.
     * @param <V>
     */
    private static class Components<V> {

        /** HashMap for saving current component index of each Vertex */
        HashMap<Integer, Integer> componentMap = new HashMap<>();

        /** ArrayList containing Lists of vertices contained within each component */
        ArrayList<List<Vertex>> componentList = new ArrayList<>();

        public Components(List<Vertex<V>> vertices) {
            for (int i = 0; i < vertices.size(); i++) {
                componentMap.put(vertices.get(i).hashCode(), i);
                List<Vertex> component = new ArrayList();
                component.add(vertices.get(i));
                componentList.add(component);
            }
        }

        /**
         * Returns true if two vertices exist in the same component
         * @param v1
         * @param v2
         * @return
         */
        public boolean sameComponent(Vertex v1, Vertex v2) {
            return componentMap.get(v1.hashCode()).equals(componentMap.get(v2.hashCode()));
        }

        /**
         * Merges all vertices in component v2 to that of component v1
         * @param v1 component that will be merged into
         * @param v2 component to merge into v1
         */
        public void mergeComponents(Vertex v1, Vertex v2) {
            /** Update vertices in component map to all point to the same componentList */
            List v2Neighbors = componentList.get(componentMap.get(v2.hashCode()));
            for (int i = 0; i < v2Neighbors.size(); i++) {
                componentMap.put(v2Neighbors.get(i).hashCode(), componentMap.get(v1.hashCode()));
            }

            /** Update the v1 neighbors to have v2 neighbors*/
            componentList.get(componentMap.get(v1.hashCode())).addAll(v2Neighbors);
        }
    }

    public static <V, E extends IWeight> List<IGraph.Edge<E>> ShortestPath(IGraph<V, E> g, String vertexStart, String vertexEnd) {

        /** Set of Open vertices */
        ArrayList<Vertex<V>> open = new ArrayList<>();
        HashMap<Integer, Boolean> openMap = new HashMap<>();
        open.add(g.getVertex(vertexStart));
        openMap.put(g.getVertex(vertexStart).hashCode(), true);

        /** Set of Closed vertices */
        HashMap<Integer, Boolean> closed = new HashMap<>();

        /** Set distance of all vertices to infinite and pred to empty */
        HashMap<Integer, Double> distance = new HashMap<>();
        HashMap<Integer, Vertex<V>> predecessor = new HashMap<>();
        for (Vertex v : g.getVertices()) {
            distance.put(v.hashCode(), Double.POSITIVE_INFINITY);
            predecessor.put(v.hashCode(), null); // TODO Is this even necessary?
        }

        /** Set distance of vertexStart to 0 */
        distance.put(g.getVertex(vertexStart).hashCode(), 0.0);

        /** While Open Vertices is not empty */
        while(open.size() != 0) {

            /** Set a to be the vertex with smallest cost currently in Open */
            Vertex a = open.get(0);
            int index = 0;
            for (int i = 0; i < open.size(); i++) {
                if (distance.get(open.get(i).hashCode()) < distance.get(a.hashCode())) {
                    a = open.get(i);
                    index = i;
                }
            }

            /** Remove vertex a from Open */
            open.remove(index);
            openMap.remove(a.hashCode());

            /** Add a to Closed */
            closed.put(a.hashCode(), true);

            /** Iterate over all neighbors of A */
            for (Vertex neighbor : g.getNeighbors(a.getVertexName())) {

                /** If X does not exist in Closed */
                if (!closed.containsKey(neighbor.hashCode())) {

                    /** alt = dist(a) + weight(a, x) */
                    double alt = distance.get(a.hashCode()) + g.getEdge(a.getVertexName(), neighbor.getVertexName()).getEdgeData().getWeight();

                    /** if (alt < dist(x)) */
                    if (alt < distance.get(neighbor.hashCode())) {

                        /** dist(x) = alt */
                        distance.put(neighbor.hashCode(), alt);

                        /** pred(x) = a */
                        predecessor.put(neighbor.hashCode(), a);
                    }

                    /** Add neighbor to Open */
                    //if (!Arrays.asList(open).contains(neighbor)) {
                    if (!openMap.containsKey(neighbor.hashCode())) {
                        open.add(neighbor);
                        openMap.put(neighbor.hashCode(), true);
                    }
                }
            }
        }

        /** List of edges containing the shortest path from vertexStart to vertexEnd */
        LinkedList<IGraph.Edge<E>> shortestPath = new LinkedList<>();
        Vertex v = g.getVertex(vertexEnd);

        /** If vertexEnd does not have a predecessor, there is no path. Return the empty list */
        if (predecessor.get(v.hashCode()) == null) {
            return shortestPath;
        }

        /** Add edges to the front of the list up to vertexStart */
        while (!v.getVertexName().equals(vertexStart)) {
            shortestPath.add(0, g.getEdge(predecessor.get(v.hashCode()).getVertexName(), v.getVertexName()));
            v = predecessor.get(v.hashCode());
        }

        return shortestPath;
    }
}