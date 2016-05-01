/**
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import com.google.common.collect.Maps;
import com.google.common.collect.Sets;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team
 *
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections of multiple roads.
 * Edges are the roads.
 *
 */
public class MapGraph {

	// Maintain both nodes and edges as you will need to
	// be able to look up nodes by lat/lon or by roads
	// that contain those nodes.
	private HashMap<GeographicPoint,MapNode> pointNodeMap;
	private HashSet<MapEdge> edges;


	/** Create a new empty MapGraph
	 *
	 */
	public MapGraph()
	{
		pointNodeMap = new HashMap<GeographicPoint,MapNode>();
		edges = new HashSet<MapEdge>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return pointNodeMap.values().size();
	}

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
	}

	// For us in DEBUGGING.  Print the Nodes in the graph
	public void printNodes()
	{
		System.out.println("****PRINTING NODES ********");
		System.out.println("There are " + getNumVertices() + " Nodes: \n");
		for (GeographicPoint pt : pointNodeMap.keySet())
		{
			MapNode n = pointNodeMap.get(pt);
			System.out.println(n);
		}
	}

	// For us in DEBUGGING.  Print the Edges in the graph
	public void printEdges()
	{
		System.out.println("******PRINTING EDGES******");
		System.out.println("There are " + getNumEdges() + " Edges:\n");
		for (MapEdge e : edges)
		{
			System.out.println(e);
		}

	}

	/** Add a node corresponding to an intersection
	 *
	 * @param latitude The latitude of the location
	 * @param longitude The longitude of the location
	 * */
	public void addVertex(double latitude, double longitude)
	{
		GeographicPoint pt = new GeographicPoint(latitude, longitude);
		this.addVertex(pt);
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 *
	 * @param location  The location of the intersection
	 */
	public void addVertex(GeographicPoint location)
	{
		MapNode n = pointNodeMap.get(location);
		if (n == null) {
			n = new MapNode(location);
			pointNodeMap.put(location, n);
		}
		else {
			System.out.println("Warning: Node at location " + location +
					" already exists in the graph.");
		}

	}

	/** Add an edge representing a segment of a road.
	 * Precondition: The corresponding Nodes must have already been
	 *     added to the graph.
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 */
	public void addEdge(double lat1, double lon1,
						double lat2, double lon2, String roadName, String roadType)
	{
		// Find the two Nodes associated with this edge.
		GeographicPoint pt1 = new GeographicPoint(lat1, lon1);
		GeographicPoint pt2 = new GeographicPoint(lat2, lon2);

		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, MapEdge.DEFAULT_LENGTH);

	}

	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName,
			String roadType) {

		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, MapEdge.DEFAULT_LENGTH);
	}

	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName,
			String roadType, double length) {
		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, length);
	}

	/** Given a point, return if there is a corresponding MapNode **/
	public boolean isNode(GeographicPoint point)
	{
		return pointNodeMap.containsKey(point);
	}



	// Add an edge when you already know the nodes involved in the edge
	private void addEdge(MapNode n1, MapNode n2, String roadName,
			String roadType,  double length)
	{
		MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
		edges.add(edge);
		n1.addEdge(edge);
	}


	/** Returns the nodes in terms of their geographic locations */
	public Set<GeographicPoint> getVertices() {
		return pointNodeMap.keySet();
	}

	// get a set of neighbor nodes from a mapnode
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}
	
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using Breadth First Search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
									Consumer<GeographicPoint> nodeSearched)
	{
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin BFS
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		toExplore.add(startNode);
		MapNode next = null;

		while (!toExplore.isEmpty()) {
			next = toExplore.remove();
			
			 // hook for visualization
			nodeSearched.accept(next.getLocation());
			
			if (next.equals(endNode)) break;
			Set<MapNode> neighbors = getNeighbors(next);
			for (MapNode neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					visited.add(neighbor);
					parentMap.put(neighbor, next);
					toExplore.add(neighbor);
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}
		
		// Reconstruct the parent path
		List<GeographicPoint> path = reconstructPath(parentMap, startNode, endNode);

		return path;
	}

	/** Reconstruct a path from start to goal using the parentMap
	 *
	 * @param parentMap the HashNode map of children and their parents
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */

	private List<GeographicPoint> reconstructPath(Map<MapNode,MapNode> parentMap, MapNode start, MapNode goal)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
		return path;
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
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null) {
			throw new NullPointerException("invalid geographic points");
		}
		MapNode startNode = pointNodeMap.get(start);
        MapNode goalNode = pointNodeMap.get(goal);
		assert startNode != null;
		assert goalNode != null;
		System.out.println("start: " + startNode + " neighbors: " + startNode.getNeighbors());
		System.out.println("goal: " + goalNode);
		
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>();
		Set<MapNode> visitedNodes = new HashSet<MapNode>();
		Map<MapNode, MapNode> childToParent = new HashMap<MapNode, MapNode>();
		int visitedCount = 0;
		for (MapNode node : pointNodeMap.values()) {
            node.setDistance(Double.POSITIVE_INFINITY);
		}
		
		queue.add(startNode);
		startNode.setDistance(0.0);
		
		MapNode next = null;
		while (!queue.isEmpty()) {
			//System.out.println("\nqueue: " + queue);
			next = queue.remove();
			nodeSearched.accept(next.getLocation());
			visitedCount += 1;
			//System.out.println("visiting: " + next);
			if (next.equals(goalNode)) {
				break;
			}
			if (!visitedNodes.contains(next)) {
				visitedNodes.add(next);				
		        Set<MapEdge> edges = next.getEdges();
		        for (MapEdge edge : edges) {
					//System.out.println("exploring edge: " + edge);
					MapNode neighbor = edge.getEndNode();
					if (!visitedNodes.contains(neighbor)) {
						double currentDist = edge.getLength() + next.getDistance();
						if (currentDist < neighbor.getDistance()) {
							//System.out.println("currentDistance: " + currentDist + " is shorter than neighbor.distance: " + neighbor.getDistance());
							neighbor.setDistance(currentDist);
							//System.out.println("set distance: " + currentDist + " to neighbor: " + neighbor);
							childToParent.put(neighbor, next);
							queue.add(neighbor);
						} else {
							//System.out.println("neighbor.distance " + neighbor.getDistance() + " is shorter than currentDist: " + currentDist);
						}
					} else {
						//System.out.println("already explored " + neighbor);
					}
				}		        
			}			
		}
		System.out.println("total nodes visited: " + visitedCount);
		if (!next.equals(goalNode)) {
			System.out.println("can't go from " + startNode + " to " + goalNode);
			return null;
		}
		return reconstructPath(childToParent, startNode, goalNode);
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
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (start == null || goal == null) {
			throw new NullPointerException("invalid geographic points");
		}
		MapNode startNode = pointNodeMap.get(start);
        MapNode goalNode = pointNodeMap.get(goal);
		assert startNode != null;
		assert goalNode != null;
		System.out.println("start: " + startNode + " neighbors: " + startNode.getNeighbors());
		System.out.println("goal: " + goalNode);
		
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>();
		Set<MapNode> visitedNodes = new HashSet<MapNode>();
		Map<MapNode, MapNode> childToParent = new HashMap<MapNode, MapNode>();
		int visitedCount = 0;
		for (MapNode node : pointNodeMap.values()) {
            node.setDistance(Double.POSITIVE_INFINITY);
            node.setActualDistance(Double.POSITIVE_INFINITY);
		}
		
		queue.add(startNode);
		startNode.setDistance(0.0);
		startNode.setActualDistance(0.0);

		MapNode next = null;
		while (!queue.isEmpty()) {
			//System.out.println("\nqueue: " + queue);
			next = queue.remove();
			nodeSearched.accept(next.getLocation());
			//System.out.println("visiting: " + next);
			visitedCount += 1;
			if (next.equals(goalNode)) {
				break;
			}
			if(!visitedNodes.contains(next)) {
				visitedNodes.add(next);
				Set<MapEdge> edges = next.getEdges();
				for (MapEdge edge : edges) {
					//System.out.println("exploring edge: " + edge);
					MapNode neighbor = edge.getEndNode();
					if (!visitedNodes.contains(neighbor)) {
						Double currentDist = edge.getLength() + next.getActualDistance();
						//System.out.println("currentDist: " + currentDist);
						Double distanceToGoal = currentDist + (neighbor.getLocation()).distance(goal);
						//System.out.println("distanceToGoal: " + distanceToGoal);
						if (distanceToGoal < neighbor.getDistance()) {
							//System.out.println("distanceToGoal: " + distanceToGoal + " is shorter than neighbor.distance: " + neighbor.getDistance());
							neighbor.setActualDistance(currentDist);
							neighbor.setDistance(distanceToGoal);
							//System.out.println("set neighbor distance: " + neighbor.getDistance() + " actualDistance: " + neighbor.getActualDistance());
							childToParent.put(neighbor, next);
							queue.add(neighbor);
						} else {
							//System.out.println("neighbor.distance " + neighbor.getDistance() + " is shorter than distanceToGoal: " + distanceToGoal);
						}
					} else {
						//System.out.println("already explored " + neighbor);
					}
				}
			}
		}
		System.out.println("total nodes visited: " + visitedCount);
		if (!next.equals(goalNode)) {
			System.out.println("can't go from " + startNode + " to " + goalNode);
			return null;
		}
		List<GeographicPoint> path = reconstructPath(childToParent, startNode, goalNode);
		return path;
	}

	private static void runDijkstra() {
		MapGraph map = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", map);
		System.out.print("map: " + map.getNumVertices() + " vertices " + map.getNumEdges() + " edges\n");
		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint end = new GeographicPoint(8.0, -1.0);
		
		/** Dikstra */ 
		List<GeographicPoint> routeDijkstra = map.dijkstra(start, end);
		System.out.println("route Dijkstra: " + routeDijkstra);
		// expected visited count: 9
		// expected: (1.0,1.0), (4.0,1.0), (5.0,1.0), (6.5,0.0), (8.0,-1.0)
		assert routeDijkstra.get(0).getX() == 1.0;
		assert routeDijkstra.get(0).getY() == 1.0;
		assert routeDijkstra.get(1).getX() == 4.0;
		assert routeDijkstra.get(1).getY() == 1.0;
		assert routeDijkstra.get(2).getX() == 5.0;
		assert routeDijkstra.get(2).getY() == 1.0;
		assert routeDijkstra.get(3).getX() == 6.5;
		assert routeDijkstra.get(3).getY() == 0.0;
		assert routeDijkstra.get(4).getX() == 8.0;
		assert routeDijkstra.get(4).getY() == -1.0;
	}
	
	private static void runAStar() {
		MapGraph map = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", map);
		System.out.print("map: " + map.getNumVertices() + " vertices " + map.getNumEdges() + " edges\n");
		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint end = new GeographicPoint(8.0, -1.0);

		/** A* */
		List<GeographicPoint> routeAStar = map.aStarSearch(start, end);
		System.out.println(routeAStar);
		// expected visited count: 5
		// expected: (1.0,1.0), (4.0,1.0), (5.0,1.0), (6.5,0.0), (8.0,-1.0)
		assert routeAStar.get(0).getX() == 1.0;
		assert routeAStar.get(0).getY() == 1.0;
		assert routeAStar.get(1).getX() == 4.0;
		assert routeAStar.get(1).getY() == 1.0;
		assert routeAStar.get(2).getX() == 5.0;
		assert routeAStar.get(2).getY() == 1.0;
		assert routeAStar.get(3).getX() == 6.5;
		assert routeAStar.get(3).getY() == 0.0;
		assert routeAStar.get(4).getX() == 8.0;
		assert routeAStar.get(4).getY() == -1.0;	
	}
	
	private static void runWeek3Quiz() {
		/** Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);

		GeographicPoint startUTC = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint endUTC = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(startUTC,endUTC);
		System.out.println(route);
		List<GeographicPoint> route2 = theMap.aStarSearch(startUTC,endUTC);
		System.out.println(route2);
	}

	private static void runGreedyTSP() {
		MapGraph map = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", map);
		System.out.print("map: " + map.getNumVertices() + " vertices " + map.getNumEdges() + " edges\n");
		GeographicPoint start = new GeographicPoint(1.0, 1.0);		
		List<GeographicPoint> route = map.greedyTSP(start);
		System.out.println("\ntour: " + route);
	}
	
	public List<GeographicPoint> greedyTSP(GeographicPoint start) {
		
		/** validate parameters */
		Set<MapNode> vertices = Sets.newHashSet(pointNodeMap.values());
		if (start == null || vertices.isEmpty()) {
			throw new NullPointerException("missing start or vertices");
		}
		
		MapNode startVertex = pointNodeMap.get(start);
		
		/** initialize collections */
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		path.add(startVertex.getLocation());
		Set<MapNode> visited = new HashSet<MapNode>();
		visited.add(startVertex);
		Set<MapNode> unvisited = new HashSet<MapNode>();
		unvisited.addAll(vertices);
		unvisited.remove(startVertex);
		
		/** iterate over vertices, 
		 *  adding the nearest at each step, until all
		 *  have been visited
		 */
		MapNode current = startVertex;
		while (true) {
			Set<MapNode> visitable = new HashSet<MapNode>();
			visitable.addAll(unvisited);
			visitable.remove(current);
			if (visitable.isEmpty()) {
				path.add(startVertex.getLocation());
				break;
			}
			Entry<MapNode, Double> nearestVertex = findNearest(current, visitable);
			unvisited.remove(current);
			visited.add(nearestVertex.getKey());
			path.add(nearestVertex.getKey().getLocation());
			current = nearestVertex.getKey();
		}

		return path;
	}
	
	private Entry<MapNode, Double> findNearest(MapNode current, Set<MapNode> neighbors) {
		Map<MapNode, Double> vertexDistances = Maps.newHashMap();
		for (MapNode n : neighbors) {
			vertexDistances.put(n, n.distanceFrom(current));
		}
		return findMin(vertexDistances);
	}

	private Entry<MapNode, Double> findMin(Map<MapNode, Double> map) {
		Entry<MapNode, Double> min = null;
		for (Entry<MapNode, Double> entry : map.entrySet()) {
		    if (min == null || min.getValue() > entry.getValue()) {
		        min = entry;
		    }
		}
		return min;
	}

	private void pressAnyKeyToContinue() { 
		System.out.println("Press any key to continue...");
		try {
			System.in.read();
		}  
		catch(Exception e) {}  
	}
	
	/** main method for testing */
	public static void main(String[] args) {
//		runDijkstra();
//		runAStar();
//		runWeek3Quiz();
		runGreedyTSP();
	}

}