package nl.tue.alignment;

public interface Queue {

	/**
	 * Return the algorithm for which this Queue is used.
	 * 
	 * @return
	 */
	public ReplayAlgorithm getAlgorithm();

	/**
	 * Show the number of the marking at the head of the priority queue
	 * 
	 * @return
	 */
	public int peek();

	/**
	 * remove and return the head of the queue
	 * 
	 * @return
	 */
	public int poll();

	/**
	 * add a new marking to the queue. If it exists and the new score is better,
	 * update the score.
	 * 
	 * @param marking
	 * @return true if the marking was added to the priority queue
	 */
	public boolean add(int marking);

	/**
	 * returns true if the queue is empty
	 * 
	 * @return
	 */
	public boolean isEmpty();

	/**
	 * returns the maximum memory use in bytes the queue ever had.
	 * 
	 * @return
	 */
	public int maxBytesUsed();

	/**
	 * returns the maximum memory capacity the queue ever had.
	 * 
	 * @return
	 */
	public int maxCapacity();

	/**
	 * returns maximum number of elements the queue ever contained empty.
	 * 
	 * @return
	 */
	public int maxSize();

	/**
	 * Returns the current number of elements in the queue
	 * 
	 * @return
	 */
	public int size();

	public boolean contains(int marking);
}
