package roadgraph;

import java.util.HashSet;
import java.util.LinkedList;

public class BFSearchStruct {
	private LinkedList<GraphVertex> q = new LinkedList<>();
	private HashSet<GraphVertex> visited = new HashSet<>();

	/**
	 * If the param is not in the visited set adds a new vertex to the end of
	 * the queue and adds it to the visited set
	 * 
	 * @param vertex
	 * @return True if the vertex was added to the queue (didn't exist on the
	 *         visited set), False otherwise
	 */
	public boolean enqueue(GraphVertex vertex) {
		if (!visited.contains(vertex)) {
			visited.add(vertex);
			q.add(vertex);
			return true;
		}
		return false;
	}

	/**
	 * Retrieves and removes the head (first element) of the queue.
	 *
	 * @return the head of this list, or null if the queue is empty
	 */
	public GraphVertex nextVertex() {
		return q.poll();
	}

	/**
	 * @return True if the queue is not empty, False otherwise
	 */
	public boolean hasNext() {
		return !q.isEmpty();
	}
}
