package nl.tue.alignment.algorithms.datastructures;

import nl.tue.alignment.algorithms.ReplayAlgorithm;

public class SortedHashBackedPriorityQueue extends HashBackedPriorityQueue {

	private final boolean preferExact;

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

	public SortedHashBackedPriorityQueue(ReplayAlgorithm algorithm, int initialCapacity, int maxCost,
			boolean preferExact) {
		super(algorithm, initialCapacity, maxCost);
		this.preferExact = preferExact;
	}

	public SortedHashBackedPriorityQueue(ReplayAlgorithm algorithm, int initialCapacity, boolean preferExact) {
		super(algorithm, initialCapacity);
		this.preferExact = preferExact;
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

			// second order sorting on exactness
			if (preferExact) {
				boolean b1 = algorithm.hasExactHeuristic(marking1);
				boolean b2 = algorithm.hasExactHeuristic(marking2);
				if (b1 && !b2) {
					return true;
				} else if (b2 == b1) {
					// when both markings have exact or inexact heuristics, schedule the largest event number
					
					c1 = getLastEventNumber(marking1);
					c2 = getLastEventNumber(marking2);
					if (c1 > c2) {
						// more events explained;
						return true;
					} else if (c2 == c1) {
						// when both markings have exact or inexact heuristics, schedule the largest g score first
						
						if (algorithm.getGScore(marking1) > algorithm.getGScore(marking2)) {
							return true;
						} else if (algorithm.getGScore(marking1) == algorithm.getGScore(marking2)) {
							// when they are equal, prefer the lowest predecessor
							return algorithm.getPredecessorTransition(marking1) < algorithm
									.getPredecessorTransition(marking2);
						}
					}
				}
			}
		}
		return false;
	}

	private int getLastEventNumber(int marking) {
		int eventNumber = -1;
		while (marking > 0 && eventNumber == -1) {
			eventNumber = algorithm.getNet().getEventOf(algorithm.getPredecessorTransition(marking));
			marking = algorithm.getPredecessor(marking);
		}
		return eventNumber;
	}
}
