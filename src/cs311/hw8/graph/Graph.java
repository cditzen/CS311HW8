package cs311.hw8.graph;

import cs311.hw8.graphalgorithms.GraphAlgorithms;
import cs311.hw8.helpers.Weight;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;




public class Graph<V, E> implements IGraph<V, E>
{

    public static void main(String args[] )
    {
        Graph<Integer, Weight> g1 = new Graph();
        g1.setDirectedGraph();
        g1.addVertex("A");
        g1.addVertex("B");
        g1.addVertex("C");
        g1.addVertex("D");
        g1.addVertex("E");
        g1.addEdge("A", "B", new Weight(20.0));
        g1.addEdge("A", "C", new Weight(30.3));
        g1.addEdge("B", "E", new Weight(10.4));
        g1.addEdge("B", "D", new Weight(80.4));
        g1.addEdge("C", "D", new Weight(10.2));

        g1.getVertices().forEach(v -> System.out.println(v.getVertexName()));
        g1.getEdges().forEach(e -> System.out.println(e.getVertexName1()+" : "+e.getVertexName2()));

        /** Test topological sort */
        List<Vertex<Integer>> topologicalSort = GraphAlgorithms.TopologicalSort(g1);

        topologicalSort.forEach(v -> System.out.println("Vertex: " + v.getVertexName()));

        /** Test allTopological Sort */
        List<List<Vertex<Integer>>> allTopologicalSort = GraphAlgorithms.AllTopologicalSort(g1);

        int count = 1;

        for (List<Vertex<Integer>> sort : allTopologicalSort) {
            System.out.println("Num: " + count);
            for (Vertex<Integer> v : sort) {
                System.out.println(v.getVertexName());
            }
            System.out.println("----\n");
            count++;
        }

        /** Test Kruscal */
        Graph<Integer, Weight> g2 = new Graph();
        g2.setUndirectedGraph();
        g2.addVertex("A");
        g2.addVertex("B");
        g2.addVertex("C");
        g2.addVertex("D");
        g2.addVertex("E");
        g2.addEdge("A", "B", new Weight(20.0));
        g2.addEdge("A", "C", new Weight(30.3));
        g2.addEdge("B", "E", new Weight(10.4));
        g2.addEdge("B", "D", new Weight(80.4));
        g2.addEdge("C", "D", new Weight(10.2));
        g2.addEdge("E", "D", new Weight(9.0));

        IGraph k = GraphAlgorithms.Kruscal(g2);

        List<Edge> kList = k.getEdges();
        for (Edge e : kList) {
            System.out.println(e.getVertexName1() + " : " + e.getVertexName2() + ", " + ((Weight) e.getEdgeData()).getWeight());
        }

        /** Test Dijkstra's */
        Graph<Integer, Weight> g3 = new Graph();
        g3.setDirectedGraph();
        g3.addVertex("A");
        g3.addVertex("B");
        g3.addVertex("C");
        g3.addVertex("D");
        g3.addVertex("E");
        g3.addVertex("Z");
        g3.addEdge("A", "B", new Weight(2.0));
        g3.addEdge("A", "D", new Weight(5.0));
        g3.addEdge("B", "C", new Weight(6.0));
        g3.addEdge("B", "D", new Weight(4.0));
        g3.addEdge("C", "E", new Weight(1.0));
        g3.addEdge("D", "C", new Weight(2.0));
        g3.addEdge("D", "E", new Weight(5.0));

        System.out.println("Shortest Path Edges: ");
        List<Edge<Weight>> edges = GraphAlgorithms.ShortestPath(g3, "A", "E");
        for (Edge edge : edges) {
            System.out.println("V1: " + edge.getVertexName1() + " V2: " + edge.getVertexName2());
        }


    }




    private boolean directed = false;

    private final HashMap<String, Vertex<V>> vmap = new HashMap<>();
    private final LinkedHashMap<Vertex<V>, LinkedHashMap<Vertex, E>> edges = new LinkedHashMap<>();

    @Override
    public void setDirectedGraph()
    {
        directed = true;
    }

    @Override
    public void setUndirectedGraph()
    {
        if (directed)
        {
            directed = false;
            List<Edge<E>> edgelist = getEdges();
            Iterator<Edge<E>> iter = edgelist.iterator();
            while (iter.hasNext())
            {
                Edge<E> ed = iter.next();
                setEdgeData(ed.getVertexName1(), ed.getVertexName2(), null);
                if (!isEdge(ed.getVertexName2(), ed.getVertexName1() ))
                {
                    addEdge(ed.getVertexName1(), ed.getVertexName2());
                }
            }
        }
    }

    @Override
    public boolean isDirectedGraph()
    {
        return directed;
    }

    @Override
    public void addVertex(String vertexName) throws DuplicateVertexException
    {
        if (vmap.containsKey(vertexName)) throw new DuplicateVertexException();
        vmap.put(vertexName, new Vertex<>(vertexName, null));
        edges.put(vmap.get(vertexName), new LinkedHashMap<>());
    }

    @Override
    public void addVertex(String vertexName, V vertexData) throws DuplicateVertexException
    {
        if (vmap.containsKey(vertexName)) throw new DuplicateVertexException();
        vmap.put(vertexName, new Vertex<>(vertexName, vertexData));
        edges.put(vmap.get(vertexName), new LinkedHashMap<>());
    }

    @Override
    public void addEdge(String vertex1, String vertex2) throws DuplicateEdgeException, NoSuchVertexException
    {
        Vertex<V> v1 = vmap.get(vertex1);
        Vertex<V> v2 = vmap.get(vertex2);
        if (vertex1.equals("391233858") && vertex2.equals("391233859")) {
            System.out.println("Found 1st edge");
        }
        if (v1 == null || v2 == null) throw new NoSuchVertexException();
        if (edges.get(v1).containsKey(v2))  {
            //LinkedHashMap e = edges.get(v1);
            throw new DuplicateEdgeException();
        }



        edges.get(v1).put(v2, null);
    }

    @Override
    public void addEdge(String vertex1, String vertex2, E edgeData) throws DuplicateEdgeException, NoSuchVertexException
    {
        Vertex<V> v1 = vmap.get(vertex1);
        Vertex<V> v2 = vmap.get(vertex2);
        if (v1 == null || v2 == null) throw new NoSuchVertexException();
        if (edges.get(v1).containsKey(v2)){
            //LinkedHashMap e = edges.get(v1);
            throw new DuplicateEdgeException();
        }
        edges.get(v1).put(v2, edgeData);
    }

    @Override
    public V getVertexData(String vertexName) throws NoSuchVertexException
    {
        if (!vmap.containsKey(vertexName)) throw new NoSuchVertexException();
        return vmap.get(vertexName).getVertexData();
    }

    @Override
    public void setVertexData(String vertexName, V vertexData) throws NoSuchVertexException
    {
        if (!vmap.containsKey(vertexName)) throw new NoSuchVertexException();
        vmap.remove(vertexName);
        vmap.put(vertexName, new Vertex<>(vertexName, vertexData));
    }

    @Override
    public E getEdgeData(String vertex1, String vertex2) throws NoSuchVertexException, NoSuchEdgeException
    {
        Vertex<V> v1 = vmap.get(vertex1);
        Vertex<V> v2 = vmap.get(vertex2);
        if (v1 == null || v2 == null) throw new NoSuchVertexException();
        if (!edges.get(v1).containsKey(v2)) throw new NoSuchEdgeException();
        return edges.get(v1).get(v2);
    }

    @Override
    public void setEdgeData(String vertex1, String vertex2, E edgeData) throws NoSuchVertexException, NoSuchEdgeException
    {
        Vertex<V> v1 = vmap.get(vertex1);
        Vertex<V> v2 = vmap.get(vertex2);
        if (v1 == null || v2 == null) throw new NoSuchVertexException();
        if (!edges.get(v1).containsKey(v2)) throw new NoSuchEdgeException();
        edges.get(v1).put(v2, edgeData);
    }

    @Override
    public Vertex<V> getVertex(String vertexName) throws NoSuchVertexException
    {
        if (!vmap.containsKey(vertexName)) throw new NoSuchVertexException();
        return vmap.get(vertexName);
    }

    @Override
    public Edge<E> getEdge(String vertexName1, String vertexName2)
    {
        Vertex<V> v1 = vmap.get(vertexName1);
        Vertex<V> v2 = vmap.get(vertexName2);
        if (v1 == null || v2 == null) throw new NoSuchVertexException();
        if (!edges.get(v1).containsKey(v2))  {
            List<Vertex<V>> neighbors = getNeighbors(vertexName1);
            throw new NoSuchEdgeException();
        }
        return new Edge( vertexName1, vertexName2, edges.get(v1).get(v2));
    }

    @Override
    public List<Vertex<V>> getVertices()
    {
        return new ArrayList<>(vmap.values());
    }

    @Override
    public List<Edge<E>> getEdges()
    {
        ArrayList<Edge<E>> retval = new ArrayList<>();
        vmap.values().forEach( v -> edges.get(v).forEach((nv, ed) -> retval.add( new Edge<>(v.getVertexName(), nv.getVertexName(), ed))));
        return retval;
    }

    @Override
    public List<Vertex<V>> getNeighbors(String vertex)
    {
        Vertex<V> v1 = vmap.get(vertex);
        if (v1 == null) throw new NoSuchVertexException();
        ArrayList<Vertex<V>> retval = new ArrayList<>();
        edges.get(v1).forEach((nv, ed) -> retval.add(nv));
        return retval;
    }

    public boolean isEdge( String v1, String v2)
    {
        Vertex<V> v1v = vmap.get(v1);
        Vertex<V> v2v = vmap.get(v2);
        return isEdge(v1v, v2v );
    }

    public boolean isEdge( Vertex v1, Vertex v2)
    {
        return edges.get(v1).containsKey(v2);
    }

    public boolean isEdge( Edge<E> e)
    {
        return isEdge( e.getVertexName1(), e.getVertexName2());
    }


}