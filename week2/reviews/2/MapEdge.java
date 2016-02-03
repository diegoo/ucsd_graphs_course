package roadgraph;

import geography.GeographicPoint;


/**
 * Created to store information about a road segment (edge)
 * 
 * @author Madhavi
 *
 */

public class MapEdge {
	private MapNode start;
	private MapNode end;
	
	private String roadName;
	private String roadType;
	
	// Length in km
	private double length;
	
	/** Constructors */
	public MapEdge(MapNode pt1, MapNode pt2, String roadName){
		this(pt1, pt2, roadName, "", 0);
	}
	
	public MapEdge(MapNode pt1, MapNode pt2, String roadName, String roadType){
		this(pt1, pt2, roadName, roadType, 0);
	}
	
	public MapEdge(MapNode pt1, MapNode pt2, String roadName, String roadType, double length){
		start = pt1;
		end = pt2;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	// return the MapNode for the end point
	public MapNode getEndNode() {
		   return end;
	}
		
	// return the location of the start point
	public GeographicPoint getStartPoint()	{
		return start.getLocation();
	}
		
	// return the location of the end point
	public GeographicPoint getEndPoint()	{
		return end.getLocation();
	}
	
	// return the length
	public double getLength()	{
		return length;
	}
		
	// return road name
	public String getRoadName()	{
		return roadName;
	}
		
	// given one node in an edge, return the other node
	public MapNode getOtherNode(MapNode node){
		if (node.equals(start)) 
			return end;
		else if (node.equals(end))
			return start;
		throw new IllegalArgumentException("Looking for " +
			"a point that is not in the edge");
	}
	
	// return String containing details about the edge
	public String toString(){
		String toReturn = "[EDGE between ";
		toReturn += "\n\t" + start.getLocation();
		toReturn += "\n\t" + end.getLocation();
		toReturn += "\nRoad name: " + roadName + " Road type: " + roadType +
				" Segment length: " + String.format("%.3g", length) + "km";
		
		return toReturn;
	}

}