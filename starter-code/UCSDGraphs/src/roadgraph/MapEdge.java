package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	
	private final MapVertex start;
	private final MapVertex end;
	private String roadName;
	private String roadType;
	private double length = 0.1;

	public MapEdge(MapVertex start, MapVertex end, String roadName, String roadType, double length) {
		this.start = start;
		this.end = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	public static class Builder {
		
		private final MapVertex start;
		private final MapVertex end;
		private String roadName;
		private String roadType;
		private double length = 0.1;
	
		public 	Builder(MapVertex start, MapVertex end) {
			this.start = start;
			this.end = end;
		}
		
		public Builder roadName(String roadName) {
			this.roadName = roadName;
			return this;
		}
		
		public Builder roadType(String roadType) {
			this.roadType = roadType;
			return this;
		}
		
		public Builder length(double length) {
			this.length = length;
			return this;
		}
		
		public MapEdge build() {
			return new MapEdge(this);
		}
	}

	private MapEdge(MapEdge.Builder builder) {
		this.roadName = builder.roadName;
		this.start = builder.start;
		this.end = builder.end;
		this.roadType = builder.roadType;
		this.length = builder.length;
	}

	public MapVertex getEndNode() {
	   return end;
	}
	
	public GeographicPoint getStartPoint() {
		return start.getLocation();
	}
	
	public GeographicPoint getEndPoint() {
		return end.getLocation();
	}
	
	public double getLength() {
		return length;
	}
	
	public String getRoadName() {
		return roadName;
	}
	
	public MapVertex getOtherNode(MapVertex node) {
		if (node.equals(start)) {
			return end;
		} else if (node.equals(end)) {
			return start;
		}
		throw new IllegalArgumentException("wrong point for this edge");
	}
	
	@Override
	public String toString() {
		return "(" + start.getLocation() + ", " + end.getLocation() + ", " + roadName + ", " + roadType + ", " + String.format("%.3g", length) + " km" + ")";
	}

}