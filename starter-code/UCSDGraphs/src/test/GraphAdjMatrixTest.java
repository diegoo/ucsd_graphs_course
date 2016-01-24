package test;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import basicgraph.Graph;
import basicgraph.GraphAdjMatrix;

public class GraphAdjMatrixTest {

	private void showMatrix(int[][] values) {
		for (int i = 0; i < values.length; i++) {
			for (int j = 0; j < values.length; j++) {
				System.out.print(values[i][j] + " ");
			}
			System.out.print("\n");
		}
	}

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
				
		int[][] m = { 
				{ 1, 2 }, 
				{ 4, 5 }
		};

		/** para multiplicar matrices */
		int [][] result = new int[m.length][m[0].length];
		for (int i = 0; i < m.length; i++) { 
		    for (int j = 0; j < m[0].length; j++) { 
		        for (int k = 0; k < m[0].length; k++) { 
		            result[i][j] += m[i][k] * m[k][j];
		        }
		    }
		}
		showMatrix(result);
		
		assertEquals(1L, 1L);
	}

}
