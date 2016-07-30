package roadgraph;
import java.util.*;


public class QueueVertex<Object> extends LinkedList<Object> {
	
	    private LinkedList<Object> data = new LinkedList<Object>();
	    public void enqueue(Object item) {data.addLast(item);}
	    public Object dequeue() {return data.removeFirst();}
	    public Object peek() {return data.getFirst();}
	    public int size() {return data.size();}
	    public boolean isEmpty() {return data.isEmpty();}
}

