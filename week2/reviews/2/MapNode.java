package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

/*
 * Created to store node(intersection) information,i.e, 
 * its location and corresponding road segements(edges)
 * 
 * @author Madhavi
 */
public class MapNode {
	
	private GeographicPoint location;
	private HashSet<MapEdge> edges;
	
	//Constructor
	public MapNode(GeographicPoint locPoint){
		location = locPoint;
		edges = new HashSet<MapEdge>();
	}
	
	public void addEdge(MapEdge edge){
		edges.add(edge);
	}
	
	/** Return the neighbors of this MapNode */
	public Set<MapNode> getNeighbors()
	{
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}
	
	/** get the location of a node */
	GeographicPoint getLocation()
	{
		return location;
	}
	
	/** return the edges out of this node */
	Set<MapEdge> getEdges()
	{
		return edges;
	}
	
	/** Returns whether two nodes are equal.
	 * Nodes are considered equal if their locations are the same, 
	 * even if their street list is different.
	 */
	public boolean equals(Object o)
	{
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}
	
	/** Because we compare nodes using their location, we also 
	 * may use their location for HashCode.
	 * @return The HashCode for this node, which is the HashCode for the 
	 * underlying point
	 */
	public int HashCode()
	{
		return location.hashCode();
	}
	
	/** ToString to print out a MapNode method
	 *  @return the string representation of a MapNode
	 */
	public String toString()
	{
		String toReturn = "[NODE at location (" + location + ")";
		toReturn += " intersects streets: ";
		for (MapEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}

	// For debugging, output roadNames as a String.
	public String roadNamesAsString()
	{
		String toReturn = "(";
		for (MapEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += ")";
		return toReturn;
	}
}
