package nl.tue.alignment;

import nl.tue.alignment.Replayer.Algorithm;
import nl.tue.alignment.algorithms.ReplayAlgorithm.Debug;

public abstract class ReplayerParameters {
	final Algorithm algorithm; // which algorithm
	final boolean moveSort; // moveSort on total order
	final boolean queueSort; // queue sorted "depth-first"
	final boolean preferExact; // prefer Exact solution
	final boolean multiThread; // do multithreading
	final boolean useInt; //  use Integer
	final Debug debug; // debug level
	final int timeoutMilliseconds; // timeout for the log
	final int intialBins; // initial bins for LPbased replayer

	private ReplayerParameters(Algorithm algorithm, boolean moveSort, boolean queueSort, boolean preferExact,
			boolean multiThread, boolean useInt, Debug debug, int timeoutMilliseconds, int intialBins) {
		this.algorithm = algorithm;
		this.moveSort = moveSort;
		this.queueSort = queueSort;
		this.preferExact = preferExact;
		this.multiThread = multiThread;
		this.useInt = useInt;
		this.debug = debug;
		this.timeoutMilliseconds = timeoutMilliseconds;
		this.intialBins = intialBins;
	}

	public final static class Default extends ReplayerParameters {
		public Default() {
			super(Algorithm.ASTARWITHMARKINGSPLIT, true, true, true, false, false, Debug.NONE, Integer.MAX_VALUE, 1);
		}

		public Default(Debug debug) {
			super(Algorithm.ASTARWITHMARKINGSPLIT, true, true, true, false, false, debug, Integer.MAX_VALUE, 1);
		}
	}

	public final static class AStar extends ReplayerParameters {
		public AStar() {
			super(Algorithm.ASTAR, true, true, true, true, false, Debug.NONE, Integer.MAX_VALUE, 1);
		}

		public AStar(Debug debug) {
			super(Algorithm.ASTAR, true, true, true, true, false, debug, Integer.MAX_VALUE, 1);
		}

		public AStar(boolean moveSort, boolean queueSort, boolean preferExact, boolean multiThread, boolean useInt,
				Debug debug, int timeoutMilliseconds) {
			super(Algorithm.ASTAR, moveSort, queueSort, preferExact, multiThread, useInt, debug, timeoutMilliseconds, 1);
		}
	}

	public final static class AStarWithMarkingSplit extends ReplayerParameters {
		public AStarWithMarkingSplit() {
			super(Algorithm.ASTARWITHMARKINGSPLIT, false, true, true, false, false, Debug.NONE, Integer.MAX_VALUE, 1);
		}

		public AStarWithMarkingSplit(Debug debug) {
			super(Algorithm.ASTARWITHMARKINGSPLIT, false, true, true, false, false, debug, Integer.MAX_VALUE, 1);
		}

		public AStarWithMarkingSplit(boolean moveSort, boolean useInt, int initialBins, Debug debug,
				int timeoutMilliseconds) {
			super(Algorithm.ASTARWITHMARKINGSPLIT, moveSort, true, true, false, useInt, debug, timeoutMilliseconds,
					initialBins);
		}
	}

	public final static class Dijkstra extends ReplayerParameters {
		public Dijkstra() {
			super(Algorithm.DIJKSTRA, false, true, true, false, false, Debug.NONE, Integer.MAX_VALUE, 1);
		}

		public Dijkstra(Debug debug) {
			super(Algorithm.DIJKSTRA, false, true, true, false, false, debug, Integer.MAX_VALUE, 1);
		}

		public Dijkstra(boolean moveSort, boolean queueSort, Debug debug, int timeoutMilliseconds) {
			super(Algorithm.DIJKSTRA, moveSort, queueSort, true, false, false, debug, timeoutMilliseconds, 1);
		}
	}

}