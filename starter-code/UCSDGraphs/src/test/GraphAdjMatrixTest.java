package test;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import basicgraph.Graph;
import basicgraph.GraphAdjMatrix;

public class GraphAdjMatrixTest {

	@Test
	public void test() {
		
		Graph g = new GraphAdjMatrix();
		
		g.addVertex();
		g.addVertex();
		g.addVertex();
		g.addVertex();
		g.addVertex();
		
		g.addEdge(0, 0);
		g.addEdge(0, 0);
		g.addEdge(0, 1);
		g.addEdge(0, 2);
		g.addEdge(0, 3);
		g.addEdge(0, 4);
		g.addEdge(1, 2);
		
		System.out.println(g.getNumVertices());
		System.out.println(g.getNumEdges());
		
		System.out.println(g.getNeighbors(0)); // out neighbors
		System.out.println(g.getInNeighbors(0)); // in neighbors
		
		System.out.println(g.getNeighbors(1)); // out neighbors
		System.out.println(g.getInNeighbors(1)); // in neighbors
		
		System.out.println(g.adjacencyString());
		
		for (int v = 0; v < g.getNumVertices(); v++) {
			int d = g.getNeighbors(v).size() + g.getInNeighbors(v).size();
			System.out.println(v + " " + d);
		}
		
		System.out.println(g.degreeSequence());
		
		assertEquals(1L, 1L);
	}

}
