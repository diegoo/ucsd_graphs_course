package roadgraph;

import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

public class GraphVertex {
	private GeographicPoint location;
	private List<GraphEdge> edges = new LinkedList<>();

	public GraphVertex(GeographicPoint location) {
		super();
		this.location = location;
	}

	public GeographicPoint getLocation() {
		return location;
	}

	public void setLocation(GeographicPoint location) {
		this.location = location;
	}

	public List<GraphEdge> getEdges() {
		return edges;
	}

	public void addEdge(GraphEdge edge) {
		this.edges.add(edge);
	}

}
