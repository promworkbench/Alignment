package nl.tue.alignment.algorithms;

import nl.tue.alignment.ReplayAlgorithm;
import nl.tue.alignment.SyncProduct;

/**
 * Implements a variant of Dijkstra's shortest path algorithm for alignments,
 * i.e. the heuristic is always equal to 0.
 * 
 * This implementation can be used for prefix alignments, as it defers the
 * decision if a marking is a final marking to the synchronous product used as
 * input.
 * 
 * @author bfvdonge
 * 
 */
public class Dijkstra extends ReplayAlgorithm {

	public Dijkstra(SyncProduct product) {
		this(product, false, false, Debug.NONE);
	}

	public Dijkstra(SyncProduct product, boolean moveSorting, boolean queueSorting, Debug debug) {
		super(product, moveSorting, queueSorting, true, debug);
	}

	/**
	 * Dijkstra always estimates 0
	 */
	@Override
	public int getExactHeuristic(int marking, byte[] markingArray, int markingBlock, int markingIndex) {
		return 0;
	}

	/**
	 * To allow for prefix versions of this algorithm, ask the net if the given
	 * marking is final.
	 */
	@Override
	protected boolean isFinal(int marking) {
		return net.isFinalMarking(getMarking(marking));
	}

	protected void deriveOrEstimateHValue(int from, int fromBlock, int fromIndex, short transition, int to,
			int toBlock, int toIndex) {
		setHScore(toBlock, toIndex, 0, true);
	}

}