package roadgraph;

import geography.GeographicPoint;

public class GraphEdge {
	private String roadName;
	private String roadType;
	private double length;
	private GeographicPoint from;
	private GeographicPoint to;

	public GraphEdge(String roadName, String roadType, double length, GeographicPoint from, GeographicPoint to) {
		super();
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
		this.from = from;
		this.to = to;
	}

	public GeographicPoint getFrom() {
		return from;
	}

	public void setFrom(GeographicPoint from) {
		this.from = from;
	}

	public GeographicPoint getTo() {
		return to;
	}

	public void setTo(GeographicPoint to) {
		this.to = to;
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
