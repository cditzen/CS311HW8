package cs311.hw8.helpers;

import cs311.hw8.graph.IGraph;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by cditz_000 on 11/7/2016.
 */
public class VertexTree<V> {

    private Node<V> root;

    public VertexTree() {
        root = new Node<>(null);
    }

    public Node getRoot() {
        return root;
    }


    public static class Node<V> {
        private IGraph.Vertex<V> vertex;
        private Node parent;
        private List<Node> children;

        public Node( IGraph.Vertex vertex) {
            this.vertex = vertex;
            parent = null;
            children = new ArrayList<>();
        }

        public Node getParent() {
            return parent;
        }

        public List<Node> getChildren() {
            return children;
        }

        public void addChild(Node child) {
            child.setParent(this);
            children.add(child);
        }

        public IGraph.Vertex<V> getVertex () {
            return vertex;
        }

        public void setParent(Node parent) {
            this.parent = parent;
        }
    }
}
