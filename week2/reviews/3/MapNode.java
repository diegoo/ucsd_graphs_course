package roadgraph;

import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode {
	private GeographicPoint location;
	private List<MapEdge> adjEdges;
	private MapNode prevBfsNode;
	
	public MapNode(GeographicPoint location){
		this.location = location;
		this.adjEdges = new LinkedList<MapEdge>();
		this.prevBfsNode = null;
	}
	/**
	 * @param
	 * @return
	 */
	public GeographicPoint getLocation(){
		return location;
	}
	
	public List<MapNode> getNeighbors(){
		List<MapNode> neighbors = new LinkedList<MapNode>();
		for(MapEdge e: adjEdges)
		{
			neighbors.add(e.getEnd());
		}
		return neighbors;
	}
	
	public void addNeighbor(MapNode n,String roadName,String roadType,double length){
		adjEdges.add(new MapEdge(this, n, roadName,roadType, length));
	}
	public MapNode getPrevNode() {
		return prevBfsNode;
	}
	public void setPrevNode(MapNode prevNode) {
		this.prevBfsNode = prevNode;
	}
}
