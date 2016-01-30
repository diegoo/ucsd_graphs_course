package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

public class MapVertex {
	
	private HashSet<MapEdge> edges;
	private GeographicPoint location;
	
	public MapVertex(GeographicPoint location) {
		this.location = location;
		this.edges = new HashSet<MapEdge>();
	}
		
	public void addEdge(MapEdge edge) {
		edges.add(edge);
	}
	
	public Set<MapVertex> getNeighbors() {
		Set<MapVertex> neighbors = new HashSet<MapVertex>();
		for (MapEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}
	
	public GeographicPoint getLocation() {
		return location;
	}
	
	public Set<MapEdge> getEdges() {
		return edges;
	}	
	
	@Override
	public String toString() {
		String toReturn = "(" + location;
		for (MapEdge e: edges) {
			toReturn += ", " + e.getRoadName();
		}
		toReturn += ")";
		return toReturn;
	}
	
}