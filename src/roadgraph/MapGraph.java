/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.*;

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
	
	//Map Vertex to its neighbors
	private Map<Vertex,ArrayList<Vertex>> adjListsMap;
	
	//map GeographicPoint to vertex
	private Map<GeographicPoint, Vertex> easyMap;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		adjListsMap = new HashMap<Vertex,ArrayList<Vertex>>();
		easyMap = new HashMap<GeographicPoint, Vertex>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return adjListsMap.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		HashSet<GeographicPoint> temp = new HashSet<GeographicPoint>();
		for (Vertex v: adjListsMap.keySet()) {
			temp.add(v.getLoc());
		}
		return temp;
		
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		int NumEdges = 0;
		for (Vertex location: adjListsMap.keySet()) {
			NumEdges += adjListsMap.get(location).size();
		}
		return NumEdges;
	}
	

	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		boolean found = adjListsMap.containsKey(location);
		if (location == null || found) {return false;}
		else {
			Vertex tempV = new Vertex(location);
			ArrayList<Vertex> temp = new ArrayList<>();
			
			adjListsMap.put(tempV, temp);
			easyMap.put(location, tempV);
			return true;
		}
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
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		

		if (easyMap.containsKey(from) && easyMap.containsKey(to)) {
			
			Vertex Vfrom = easyMap.get(from);
			Vertex Vto = easyMap.get(to);
			
			adjListsMap.get(Vfrom).add(Vto);//Add Vertex neighbor
			Vto.AddInNeighbor(from);;//Add GeoPoint inNeighbor
			
			//Add properties of the road
			Vfrom.RoadBuilder(roadName, roadType, length, Vto);
			
			
		}		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		
		//Test the legitimacy of inputs
		if (start.equals(goal) || start==null || goal==null ) {return null;}
		
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	
	public void clearAllVisit() {
		for (Vertex v: adjListsMap.keySet()) {
			v.setVisited(false);
		}
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		
		
		Queue<GeographicPoint> q = new LinkedList<>();
		HashMap<GeographicPoint,GeographicPoint> parentMap = new HashMap<>();
		q.add(start);
		ArrayList<GeographicPoint> path = new ArrayList<>();
		
		while (!q.isEmpty()) {
			GeographicPoint cur = q.remove();
			Vertex curV = easyMap.get(cur);
			ArrayList<Vertex> neighbors = adjListsMap.get(curV);
			curV.setVisited(true);
			
			nodeSearched.accept(curV.getLoc());
			
			if (cur.equals(goal)) {
				//If match, then start tracing the path
				path = reconstructPath(cur,parentMap);	
			}
			else {
				for (Vertex v: neighbors) {
					if (!v.getVisited()) {
						q.add(v.getLoc());
						parentMap.put(v.getLoc(), cur);
					}
				}
			}
		}
		
		//CLEAR ALL VISIT RECORD
		clearAllVisit();
		
		if (path.isEmpty()) {return null;}

		return path;
	}
	
	private ArrayList<GeographicPoint> reconstructPath(GeographicPoint curr,
		 HashMap<GeographicPoint,GeographicPoint> parents) {
		
		ArrayList<GeographicPoint> path = new ArrayList<>();
		path.add(curr);
		while (parents.containsKey(curr)) {
			path.add(parents.get(curr));
			curr = parents.get(curr);
		}
		Collections.reverse(path);
		return path;
	}
	private ArrayList<GeographicPoint> reconstructPath1(Vertex curr,
			 HashMap<Vertex,Vertex> parents) {
			
			ArrayList<GeographicPoint> path = new ArrayList<>();
			path.add(curr.getLoc());
			while (parents.containsKey(curr)) {
				path.add(parents.get(curr).getLoc());
				curr = parents.get(curr);
			}
			Collections.reverse(path);
			return path;
		}
	
	
	//Flag! This is a dumb way of tracing a route!!
	private ArrayList<GeographicPoint> bfsTracePath(Vertex curr,
			GeographicPoint start) {
		//Initiate an empty arraylist for the path
		ArrayList<GeographicPoint> path = new ArrayList<>();
		
		//Some pre-loading before initiating the while loop
		ArrayList<GeographicPoint> inNeighbors = curr.getInNeighbors();
		path.add(curr.getLoc());
		Vertex temp = curr;
		
		while (!inNeighbors.contains(start)) {
			for (GeographicPoint neighbor: inNeighbors) {
				Vertex neighborV = easyMap.get(neighbor);
				if (neighborV.getVisited()) {
					path.add(neighbor);
					neighborV.setVisited(false);
					temp = neighborV;break;
				}
			}
			inNeighbors = temp.getInNeighbors();					
		}
		path.add(start);
		Collections.reverse(path);
		return path;
	}
	public ArrayList<GeographicPoint> dfs(
			GeographicPoint start,GeographicPoint goal){
		//Test the legitimacy of inputs
		if (start.equals(goal) || start==null || goal==null ) {return null;}
		
		ArrayList<GeographicPoint> path = new ArrayList<>();
		dfsCall(start, goal, path, false);//Execute recursive call
		
		//CLEAR ALL VISIT RECORD
		clearAllVisit();
		
		if (path.isEmpty()) {return null;}
		Collections.reverse(path);
		return path;
		
	}
	
	private boolean dfsCall( GeographicPoint a, GeographicPoint goal,
			ArrayList<GeographicPoint> path, boolean found){
		
		Vertex curr = easyMap.get(a);
		ArrayList<Vertex> neighbors = adjListsMap.get(curr);
		curr.setVisited(true);
		
		
		if (a.equals(goal)) {path.add(a);return true;}
		else if (neighbors.isEmpty()) { return false;}
		else {
			for (Vertex v: neighbors) {
				if (!v.getVisited()) {
					boolean F = dfsCall(v.getLoc(), goal, path, found);
					if (F) {path.add(a);return F;}
				}
			}
			return found;			
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
	public void setInfinity(){

		for (Vertex v: adjListsMap.keySet()){
			v.setDistance(-1);
		}
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
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		Vertex startV = easyMap.get(start);
		Vertex goalV = easyMap.get(goal);
		
		PriorityQueue<Vertex> PQ = new PriorityQueue<>();
		HashMap<Vertex, Vertex> parentsMap = new HashMap<>();
		ArrayList<GeographicPoint> Path = new ArrayList<>();
		setInfinity();
		
		startV.setDistance(0);
		PQ.offer(startV);
		while (!PQ.isEmpty()) {
			Vertex curr = PQ.remove();
			nodeSearched.accept( curr.getLoc());
			if (curr.getVisited() == false) {
				curr.setVisited(true);
				if (curr.equals(goalV)) {
					Path = reconstructPath1(curr, parentsMap);
					clearAllVisit();
					return Path;}
				ArrayList<Vertex> neighbors = adjListsMap.get(curr);
				for (Vertex v: neighbors) {
					if (v.getVisited() == false) {

						double length = curr.getLengths().get(v);
						double newLength = length + curr.getDistance();
						double oldLength = v.getDistance();
						if (oldLength < 0 || newLength < oldLength){ 
							v.setDistance(newLength);
							PQ.offer(v);
							parentsMap.put(v, curr);
						}						
					}
				}
			}
		}
		
		
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
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		Vertex startV = easyMap.get(start);
		Vertex goalV = easyMap.get(goal);
		
		//Setting up things 
		PriorityQueue<Vertex> Open = new PriorityQueue<>();
		HashMap<Vertex, Vertex> parentsMap = new HashMap<>();
		ArrayList<GeographicPoint> Path = new ArrayList<>();

		
		
		startV.setDistance(startV.h(goal));
		Open.offer(startV);
		
		while (!Open.isEmpty()) {
			Vertex curr = Open.remove();
			curr.setVisited(true);
			nodeSearched.accept(curr.getLoc());

			if (curr.equals(goalV)) {
				Path = reconstructPath1(curr, parentsMap);
				clearAllVisit();
				return Path;
			}
			ArrayList<Vertex> neighbors = adjListsMap.get(curr);
			for (Vertex neighbor: neighbors) {
				double neighborCost = curr.getG()
						+ curr.getLengths().get(neighbor);
					
				if (Open.contains(neighbor)) {
					if (neighbor.getG() >= neighborCost) {
						
						neighbor.setG(neighborCost);
						neighbor.setDistance(neighborCost+ neighbor.h(goal));
						parentsMap.put(neighbor, curr);
					}
				} else if (neighbor.getVisited()) {
					if (neighbor.getG() >= neighborCost) {						
						neighbor.setVisited(false);
						Open.offer(neighbor);
						
						neighbor.setG(neighborCost);
						neighbor.setDistance(neighborCost+ neighbor.h(goal));
						parentsMap.put(neighbor, curr);							
					}
				} else {
					Open.offer(neighbor);
					
					neighbor.setG(neighborCost);
					neighbor.setDistance(neighborCost + neighbor.h(goal));
					parentsMap.put(neighbor, curr);
				}
			}
		}

		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
