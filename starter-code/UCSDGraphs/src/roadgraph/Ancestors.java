package roadgraph;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;

import geography.GeographicPoint;

public class Ancestors {

	private HashMap<MapVertex, MapVertex> child2parent;
	
	public Ancestors() {
		this.child2parent = new HashMap<MapVertex, MapVertex>();
	}
	
	public List<GeographicPoint> getPath(MapVertex start, MapVertex end) {

		Deque<GeographicPoint> stack = new ArrayDeque<GeographicPoint>();
		MapVertex current = end;

		// push stack <= end to start
		while (!current.equals(start)) {
			stack.push(current.getLocation());
			current = child2parent.get(current);
		}
		stack.push(start.getLocation());

		// pop from stack => start to end
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		for (GeographicPoint p : stack) {
			path.add(p);
		}
		return path;
	}
	
	void set(MapVertex child, MapVertex parent) {
		child2parent.put(child, parent);
	}
}