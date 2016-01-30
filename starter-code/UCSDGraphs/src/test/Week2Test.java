package test;

import java.util.List;

import org.junit.Test;

import geography.GeographicPoint;
import roadgraph.MapGraph;
import util.GraphLoader;
import static org.junit.Assert.*;

public class Week2Test {

	@Test
	public void test_add_vertex() {
		MapGraph g = new MapGraph();
		GeographicPoint location = new GeographicPoint(10,10);
		
		g.addVertex(location);
		assertEquals(1, g.getNumVertices());
		
		GeographicPoint l = g.getVertices().iterator().next();
		assertEquals(location, l);
	}
		
	@Test
	public void test_successful_search() {
        MapGraph g = new MapGraph();
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", g);
        g.showVertices();
        g.showEdges();
        
        GeographicPoint start = new GeographicPoint(4, -1);
        GeographicPoint end = new GeographicPoint(7, 3);
        
        List<GeographicPoint> path = g.bfs(start, end);
        assertEquals(3, path.size());
	}

	@Test
	public void test_unsuccessful_search() {
        MapGraph g = new MapGraph();
        GraphLoader.loadRoadMap("data/testdata/test_week2.map", g);
        g.showVertices();
        g.showEdges();
        
        GeographicPoint start = new GeographicPoint(1, 1);
        GeographicPoint end = new GeographicPoint(8, -1);
        
        List<GeographicPoint> path = g.bfs(start, end);
        assertNull(path);
	}

}