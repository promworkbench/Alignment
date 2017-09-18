package nl.tue.alignment.util;

import nl.tue.alignment.ReplayAlgorithm;

public class SortedHashBackedPriorityQueue extends HashBackedPriorityQueue {

	/**
	 * Creates a {@code HashBackedPriorityQueue} with the specified initial
	 * capacity that orders its elements according to the specified comparator.
	 * 
	 * @param algorithm
	 *            the algorithm the queue is used in
	 * @param initialCapacity
	 *            the initial capacity for this priority queue
	 * @param maxCost
	 *            the maximum cost for anything to be added to this queue
	 * 
	 * @throws IllegalArgumentException
	 *             if {@code initialCapacity} is less than 1
	 */

	public SortedHashBackedPriorityQueue(ReplayAlgorithm algorithm, int initialCapacity, int maxCost) {
		super(algorithm, initialCapacity, maxCost);
	}

	public SortedHashBackedPriorityQueue(ReplayAlgorithm algorithm, int initialCapacity) {
		super(algorithm, initialCapacity);
	}

	/**
	 * First order sorting is based on F score. Second order sorting on G score,
	 * where higher G score is better.
	 */
	@Override
	public boolean isBetter(int marking1, int marking2) {
		// retrieve stored cost
		int c1 = algorithm.getFScore(marking1);
		int c2 = algorithm.getFScore(marking2);

		if (c1 < c2) {
			return true; //
		} else if (c1 == c2) {
			// (second) order sorting on G score

			if (algorithm.getGScore(marking1) > algorithm.getGScore(marking2)) {
				return true;
			} else if (algorithm.getGScore(marking1) < algorithm.getGScore(marking2)) {
				return false;
			} else {
				return algorithm.getPredecessorTransition(marking1) < algorithm.getPredecessorTransition(marking2);
			}
		}
		return false;
	}

}
