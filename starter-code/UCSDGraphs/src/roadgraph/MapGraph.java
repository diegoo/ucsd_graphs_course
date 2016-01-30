/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private HashMap <GeographicPoint, MapVertex> vertices;
	private HashSet<MapEdge> edges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() {
		// TODO: Implement in this constructor in WEEK 2
		this.vertices = new HashMap <GeographicPoint, MapVertex> ();
		this.edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		//TODO: Implement this method in WEEK 2
		return this.vertices.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		//TODO: Implement this method in WEEK 2
		return this.vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		//TODO: Implement this method in WEEK 2
		return this.edges.size();
	}
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		// TODO: Implement this method in WEEK 2
		if (this.vertices.get(location) != null || location == null) {
			return false;
		} 
		this.vertices.put(location, new MapVertex(location));
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, 
			GeographicPoint to, 
			String roadName,
			String roadType, 
			double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		MapVertex start = vertices.get(from);
		MapVertex end = vertices.get(to);

		MapEdge edge = new MapEdge.Builder(start, end)
				.roadName(roadName)
				.roadType(roadType)
				.length(length)
				.build();
		edges.add(edge);
		start.addEdge(edge);
	}	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint startLocation, 
			 					     GeographicPoint goal, 
			 					     Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 2		
		if (startLocation == null || goal == null) {
			throw new NullPointerException("invalid GeographicPoint initial parameters");
		}
		if (this.vertices.get(startLocation) == null || this.vertices.get(goal) == null) {
			return null;
		}

		MapVertex start = this.vertices.get(startLocation);
		MapVertex end = this.vertices.get(goal);

		Queue<MapVertex> agenda = new LinkedList<MapVertex>();
		agenda.add(start);
		
		Ancestors ancestors = new Ancestors();
		HashSet<MapVertex> visited = new HashSet<MapVertex>();
		
		MapVertex next = null;
		boolean foundIt = false;
		
		while (!agenda.isEmpty()) {
			next = agenda.remove();
			nodeSearched.accept(next.getLocation());

			if (next.equals(end)) {
				foundIt = true;
				break;
			}

			for (MapVertex neighbor : next.getNeighbors()) {
				if (!visited.contains(neighbor)) {
					visited.add(neighbor);
					ancestors.set(neighbor, next);
					agenda.add(neighbor);
				}
			}
		}

		if (!foundIt) {
			return null;
		}

		return ancestors.getPath(start, end);
	}
	
	public void showVertices() {
		System.out.println("vertices: " + getNumVertices());
		for (GeographicPoint location : this.getVertices()) {
			System.out.println(this.vertices.get(location));
		}
	}

	public void showEdges() {
		System.out.println("edges: " + getNumEdges());
		for (MapEdge edge : this.edges) {
			System.out.println(edge);
		}
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, 
										  Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, 
											 Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		return null;
	}
	
	public static void main(String[] args) {
		
		// week 2
		MapGraph map = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", map);
		GeographicPoint start = new GeographicPoint(4, -1);
        GeographicPoint end = new GeographicPoint(7, 3);
        List<GeographicPoint> path = map.bfs(start, end);
        System.out.println(path);
        // [Lat: 4.0, Lon: -1.0, Lat: 8.0, Lon: -1.0, Lat: 7.0, Lon: 3.0]
        
        
		/* week 3
		MapGraph theMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		*/		
	}
	
}