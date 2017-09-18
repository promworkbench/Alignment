package nl.tue.alignment;

public interface VisitedSet {

	/**
	 * Return the algorithm for which this closedSet is used.
	 * 
	 * @return
	 */
	public ReplayAlgorithm getAlgorithm();

	/**
	 * Add a marking to the set of visited markings.
	 * 
	 * If it already exists, return it's number, otherwise add it and return
	 * newIndex
	 * 
	 * @param marking
	 * @return
	 */
	public int add(byte[] marking, int newIndex);

	/**
	 * returns the reserved space in this set. Not all slots may be used, but
	 * memory is consumed.
	 * 
	 * @return
	 */
	public int capacity();

	/**
	 * empties the set and restores it to its original state
	 */
	public void clear();

}
