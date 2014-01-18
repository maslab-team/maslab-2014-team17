package tablet;

import java.util.LinkedList;

/**
 * Keeps track of previous sensor data, with a limit on the number
 * of stored data points.  On add(), removes old elements, if the
 * size exceeds maxSize.
 * 
 * WARNING: Not thread-safe.
 * 
 * @author vmayar
 *
 */
public class SensorDataHistory extends LinkedList<SensorData>{

	private static final int DEFAULT_MAX_SIZE = 50;
	
	private static final long serialVersionUID = -5369599259664646166L;

	private final int maxSize;

	/**
	 * Creates an instance of SensorDataHistory with the
	 * specified maximum size.
	 * 
	 * @param maxSize The maximum number of data points to
	 * keep track of.
	 */
	public SensorDataHistory(int maxSize) {
		this.maxSize = maxSize;
	}
	
	/**
	 * Creates an instance of SensorDataHistory with the
	 * default maximum size.
	 */
	public SensorDataHistory() {
		this(DEFAULT_MAX_SIZE);
	}

	/**
	 * Adds a new data element to the history, removing
	 * old elements if necessary.
	 * 
	 * @param data The data to add.
	 */
	@Override
	public boolean add(SensorData data) {
		super.add(data);
		while (size() > maxSize) {
			super.remove();
		}
		return true;
	}
	
	
}
