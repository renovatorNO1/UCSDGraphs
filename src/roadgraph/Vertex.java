package roadgraph;

import java.util.ArrayList;
import java.util.*;
import geography.GeographicPoint;

public class Vertex implements Comparable {
	private GeographicPoint loc;
	private ArrayList<String> roadNames = new ArrayList<>();
	private ArrayList<String> roadTypes = new ArrayList<>();
	private HashMap<Vertex, Double> lengths = new HashMap<>();
	private ArrayList<GeographicPoint> inNeighbors= new ArrayList<>();
	private boolean visited = false;
	private double distance = 0;
	private double g = 0;
	
		
	
	//This constructor adds a location into the ArrayList
	public Vertex(GeographicPoint a) {
		loc = a;
	}
	
	public void RoadBuilder(String rn, String rt,
			double l, Vertex v){
		roadNames.add(rn); roadTypes.add(rt); lengths.put(v, l);
	}
	
	public void AddInNeighbor(GeographicPoint p) {
		inNeighbors.add(p);
	}
	
	public GeographicPoint getLoc() {
		return loc;
	}
	
	public ArrayList<GeographicPoint> getInNeighbors() {
		return inNeighbors;
	}
	
	public void setVisited(boolean s){
		visited = s;
	}
	
	public boolean getVisited() {
		return visited;
	}
	
	public int compareTo(Object otherV) {
		Vertex other = (Vertex) otherV;
		if (distance == -1) {return 1;}
		else if (other.getDistance() == -1) {return -1;}
		else {
			double difference = (distance - other.distance);
			if (difference < 0) {return -1;}
			else {return 1;}
			}
	}
	public boolean equals(Object otherV) {
		Vertex other = (Vertex)otherV;
		if (loc.equals(other.getLoc())) {return true;}
		else {return false;}
	}
	public void setDistance(double i){
		distance = i;
	}
	public double getDistance() {
		return distance;
	}
	public HashMap<Vertex, Double> getLengths() {
		return lengths;
	}
	public double h(GeographicPoint goal) {
		return loc.distance(goal);
	}
	public void setG(double d) {
		g = d;
	}
	public double getG() {
		return g;
	}
}
