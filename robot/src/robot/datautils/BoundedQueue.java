package robot.datautils;

import java.util.LinkedList;

/**
 * Implements a bounded queue, which has a fixed length
 * and removes old elements when the size exceeds maxSize.
 * 
 * WARNING: Not thread-safe.
 * 
 * @author vmayar
 *
 * @param <T>
 */
public class BoundedQueue<T> extends LinkedList<T> {
	
	private static final int DEFAULT_MAX_SIZE = 50;
	
	private static final long serialVersionUID = -5369599259664646166L;

	private final int maxSize;

	/**
	 * Creates an instance of BoundedQueue with the
	 * specified maximum size.
	 * 
	 * @param maxSize The maximum number of data points to
	 * keep track of.
	 */
	public BoundedQueue(int maxSize) {
		this.maxSize = maxSize;
	}
	
	/**
	 * Creates an instance of BoundedQueue with the
	 * default maximum size.
	 */
	public BoundedQueue() {
		this(DEFAULT_MAX_SIZE);
	}

	/**
	 * Adds a new data element to the queue, removing
	 * old elements if necessary.
	 * 
	 * @param data The data to add.
	 */
	@Override
	public boolean add(T data) {
		super.add(data);
		while (size() > maxSize) {
			super.remove();
		}
		return true;
	}
	
	
}
