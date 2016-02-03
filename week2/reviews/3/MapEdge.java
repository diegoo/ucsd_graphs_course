package roadgraph;


public class MapEdge {
	private MapNode start;
	private MapNode end;
	private String roadName;
	private String roadType;
	private double length; 
	public MapEdge(MapNode start,MapNode end,String roadName,String roadType,double length) {
		this.start = start;
		this.roadType = roadType;
		this.end = end;
		this.roadName = roadName;
		this.length = length;
	}
	
	public MapNode getStart() {
		return start;
	}
	
	public MapNode getEnd() {
		return end;
	}

	public String getRoadName() {
		return roadName;
	}

	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}
	public double getLength() {
		return length;
	}
	public void setLength(double length) {
		this.length = length;
	}
	
}
