package nl.tue.alignment.algorithms;

import java.util.Arrays;

import gnu.trove.iterator.TShortIterator;
import gnu.trove.list.TShortList;
import gnu.trove.list.array.TShortArrayList;
import gnu.trove.map.TShortObjectMap;
import gnu.trove.map.hash.TShortObjectHashMap;
import lpsolve.LpSolve;
import lpsolve.LpSolveException;
import nl.tue.alignment.Utils;
import nl.tue.alignment.Utils.Statistic;
import nl.tue.alignment.algorithms.syncproduct.SyncProduct;
import nl.tue.astar.util.ilp.LPMatrixException;

/**
 * Implements a variant of AStar shortest path algorithm for alignments. It uses
 * (I)LP to estimate the remaining distance.
 * 
 * This implementation can NOT be used for prefix alignments. The final marking
 * has to be reached as this is assumed in the underlying (I)LP.
 * 
 * @author bfvdonge
 * 
 */
public class AStarLargeLP extends AbstractLPBasedAlgorithm {

	protected int heuristicsComputedInRun = 0;;

	//	protected int numRows;
	//	protected int numCols;
	private short[] indexMap;

	private final TShortObjectMap<TShortList> rank2LSMove = new TShortObjectHashMap<>();

	private int numRanks;

	private int modelMoves;

	private SyncProduct product;
	private boolean useInteger;
	private short maxRankExact;
	private int maxRankMarking;

	public AStarLargeLP(SyncProduct product) {
		this(product, false, false, 0, Debug.NONE);
	}

	/**
	 * 
	 * @param product
	 * @param moveSorting
	 * @param useInteger
	 * @param debug
	 * @param splitpoints
	 *            provides the initial splitpoints for this sync product. This is an
	 *            array of ranks of log-move transitions. If event at rank 2 is
	 *            problematic, the array should be [2]. In linear traces, the rank
	 *            is the index of the event in the trace.
	 */
	public AStarLargeLP(SyncProduct product, boolean moveSorting, boolean useInteger, Debug debug,
			short[] splitpoints) {
		this(product, moveSorting, useInteger, splitpoints.length, debug);

		if (splitpoints.length > 0) {
			System.arraycopy(splitpoints, 0, this.splitpoints, 1, splitpoints.length);
		}
		this.splitpoints[splitpoints.length + 1] = (short) (numRanks + 1);
		splits = splitpoints.length + 1;

		this.setupTime = (int) ((System.nanoTime() - startConstructor) / 1000);
	}

	/**
	 * 
	 * @param product
	 * @param moveSorting
	 * @param useInteger
	 * @param initialBins
	 * @param debug
	 */
	public AStarLargeLP(SyncProduct product, boolean moveSorting, boolean useInteger, Debug debug) {
		this(product, moveSorting, useInteger, 0, debug);
		this.splitpoints = new short[] { 0, (short) (numRanks + 1) };

		this.setupTime = (int) ((System.nanoTime() - startConstructor) / 1000);
	}

	private AStarLargeLP(SyncProduct product, boolean moveSorting, boolean useInteger, int initialBins, Debug debug) {
		super(product, moveSorting, true, true, debug);
		this.product = product;
		this.useInteger = useInteger;

		rank2LSMove.put(SyncProduct.NORANK, new TShortArrayList(10));
		numRanks = -1;
		TShortList set;
		for (short t = 0; t < product.numTransitions(); t++) {
			short r = product.getRankOf(t);
			set = rank2LSMove.get(r);
			if (set == null) {
				set = new TShortArrayList(3);
				rank2LSMove.put(r, set);
			}
			set.add(t);
			if (product.getRankOf(t) > numRanks) {
				numRanks = product.getRankOf(t);
			}
		}
		modelMoves = rank2LSMove.get(SyncProduct.NORANK).size();
		numRanks++;
		if (numRanks == 0) {
			// ensure model is not empty for empty trace
			numRanks = 1;
		}
		splitpoints = new short[initialBins + 2];
		splits = initialBins + 1;
		restarts = 0;
	}

	private short[] splitpoints;
	private short[] lastSplitpoints;

	private int rows;
	private int coefficients;

	private void init() throws LPMatrixException {
		lpSolutions.clear();
		lpSolutionsSize = 0;
		maxRankExact = SyncProduct.NORANK;
		maxRankMarking = 0;

		rows = (splitpoints.length) * product.numPlaces();

		indexMap = new short[(splitpoints.length - 1) * modelMoves + product.numTransitions()];

		try {
			if (solver != null) {
				solver.deleteAndRemoveLp();
			}
			coefficients = 0;
			synchronized (LpSolve.class) {
				solver = LpSolve.makeLp(rows, 0);
			}
			solver.setAddRowmode(false);

			double[] col = new double[1 + rows];
			int c = 0;

			int start = 1;
			for (int s = 1; s < splitpoints.length; s++) {
				// add model moves in this block (if any)
				c = addModelMovesToSolver(col, c, start);

				//add log and sync moves in this block.
				for (short e = splitpoints[s - 1]; e < splitpoints[s] - 1; e++) {
					c = addLogAndSyncMovesToSolver(col, c, start, e, true);
				}
				c = addLogAndSyncMovesToSolver(col, c, start, (short) (splitpoints[s] - 1), false);
				start += product.numPlaces();
			}

			c = addModelMovesToSolver(col, c, start);

			int r;
			// The first blocks have to result in a marking >= 0 after consumption
			for (r = 1; r <= rows - product.numPlaces(); r++) {
				solver.setConstrType(r, LpSolve.GE);
				solver.setRh(r, -product.getInitialMarking()[(r - 1) % product.numPlaces()]);
				coefficients++;
			}
			for (; r <= rows; r++) {
				solver.setConstrType(r, LpSolve.EQ);
				solver.setRh(r, product.getFinalMarking()[(r - 1) % product.numPlaces()]
						- product.getInitialMarking()[(r - 1) % product.numPlaces()]);
				coefficients++;
			}
			solver.setMinim();
			solver.setVerbose(0);

			solver.setScaling(LpSolve.SCALE_GEOMETRIC | LpSolve.SCALE_EQUILIBRATE | LpSolve.SCALE_INTEGERS);
			solver.setScalelimit(5);
			solver.setPivoting(LpSolve.PRICER_DEVEX | LpSolve.PRICE_ADAPTIVE);
			solver.setMaxpivot(250);
			solver.setBbFloorfirst(LpSolve.BRANCH_AUTOMATIC);
			solver.setBbRule(LpSolve.NODE_PSEUDONONINTSELECT | LpSolve.NODE_GREEDYMODE | LpSolve.NODE_DYNAMICMODE
					| LpSolve.NODE_RCOSTFIXING);
			solver.setBbDepthlimit(-50);
			solver.setAntiDegen(LpSolve.ANTIDEGEN_FIXEDVARS | LpSolve.ANTIDEGEN_STALLING);
			solver.setImprove(LpSolve.IMPROVE_DUALFEAS | LpSolve.IMPROVE_THETAGAP);
			solver.setBasiscrash(LpSolve.CRASH_NOTHING);
			solver.setSimplextype(LpSolve.SIMPLEX_DUAL_PRIMAL);

			//			int res = solver.solve();
			//			double[] vars = new double[indexMap.length];
			//			solver.getVariables(vars);
			//			System.out.println(res + " : " + Arrays.toString(vars));
			//			debug.writeDebugInfo(Debug.NORMAL, "Solver: " + solver.getNrows() + " rows, " + solver.getNcolumns()
			//					+ " columns.");

		} catch (LpSolveException e) {
			solver.deleteAndRemoveLp();
			throw new LPMatrixException(e);
		}
		repeats = 0;
		heuristicsComputedInRun = 0;
		varsMainThread = new double[indexMap.length];

	}

	protected int addLogAndSyncMovesToSolver(double[] col, int c, int start, short rank, boolean full)
			throws LpSolveException {
		short[] input;
		short[] output;
		if (rank2LSMove.get(rank) != null) {
			TShortList list = rank2LSMove.get(rank);
			for (int idx = list.size(); idx-- > 0;) {
				Arrays.fill(col, 0);
				short t = list.get(idx);
				input = product.getInput(t);
				for (int i = 0; i < input.length; i++) {
					for (int p = start + input[i]; p < col.length; p += product.numPlaces()) {
						col[p] -= 1;
						coefficients++;
					}
				}
				output = product.getOutput(t);
				for (int i = 0; i < output.length; i++) {
					//								if (splitpoints[s] - splitpoints[s - 1] > 1) {
					if (full) {
						for (int p = start + output[i]; p < col.length; p += product.numPlaces()) {
							col[p] += 1;
							coefficients++;
						}
					} else {
						for (int p = start + product.numPlaces() + output[i]; p < col.length; p += product
								.numPlaces()) {
							col[p] += 1;
							coefficients++;
						}
					}
				}
				solver.addColumn(col);
				indexMap[c] = t;
				c++;
				solver.setLowbo(c, 0);
				coefficients++;
				solver.setUpbo(c, 1);
				coefficients++;
				solver.setInt(c, useInteger);
				solver.setObj(c, product.getCost(t));
				coefficients++;
			} // for all sync/log moves
		} // if sync/logMoves
		return c;
	}

	protected int addModelMovesToSolver(double[] col, int c, int start) throws LpSolveException {
		short[] input;
		short[] output;
		if (rank2LSMove.get(SyncProduct.NOEVENT) != null) {
			TShortIterator it = rank2LSMove.get(SyncProduct.NOEVENT).iterator();
			// first the model moves in this block
			while (it.hasNext()) {
				Arrays.fill(col, 0);
				short t = it.next();
				input = product.getInput(t);
				for (int i = 0; i < input.length; i++) {
					for (int p = start + input[i]; p < col.length; p += product.numPlaces()) {
						col[p] -= 1;
						coefficients++;
					}
				}
				output = product.getOutput(t);
				for (int i = 0; i < output.length; i++) {
					for (int p = start + output[i]; p < col.length; p += product.numPlaces()) {
						col[p] += 1;
						coefficients++;
					}
				}
				solver.addColumn(col);
				indexMap[c] = t;
				c++;
				solver.setLowbo(c, 0);
				coefficients++;
				solver.setUpbo(c, 255);
				coefficients++;
				solver.setInt(c, useInteger);
				solver.setObj(c, product.getCost(t));
				coefficients++;
			} // for all modelMoves
		} // if modelMoves
		return c;
	}

	@Override
	protected void growArrays() {
		super.growArrays();

	}

	@Override
	protected void initializeIteration() throws LPMatrixException {
		init();
		super.initializeIterationInternal();
	}

	protected double[] varsMainThread;
	protected int splits;
	protected int restarts;
	protected int repeats;

	@Override
	public int getExactHeuristic(int marking, byte[] markingArray, int markingBlock, int markingIndex) {
		// find an available solver and block until one is available.

		short rank = (short) (maxRankExact + 1);// marking == 0 ? SyncProduct.NORANK : getLastRankOf(marking);

		// the current shortest path explains the events up to and including the event at maxRankExact with exact markings.
		// a state must exist with an estimated heuristic for the event maxRankExact+1. But the search cannot continue from there.
		// so, we separate maxRankExact+1 by putting the border at maxRankExact+2.
		// 

		int insert;
		//		do {
		insert = Arrays.binarySearch(splitpoints, ++rank);
		//		} while (insert >= 0);

		if (marking == 0 || insert >= 0 || rank > splitpoints[splitpoints.length - 1]) {
			// No event was explained yet, or the last explained event is already a splitpoint.
			// There's little we can do but continue with the replayer.
			//			debug.writeDebugInfo(Debug.NORMAL, "Solve call started");
			//			solver.printLp();
			int res = getExactHeuristic(solver, marking, markingArray, markingBlock, markingIndex, varsMainThread);
			//			debug.writeDebugInfo(Debug.NORMAL, "End solve: " + (System.currentTimeMillis() - s) / 1000.0 + " ms.");

			assert marking != 0 || (res >= 0 && res < HEURISTICINFINITE)
					|| (alignmentResult & Utils.TIMEOUTREACHED) == Utils.TIMEOUTREACHED;

			heuristicsComputedInRun++;

			short r = getLastRankOf(marking);
			if (r > maxRankExact) {
				maxRankExact = r;
				maxRankMarking = marking;
				//				System.out.println("Explained event at rank " + r + " exactly.");
			}

			return res;
		} else {
			//			System.out.print("Expanding splitpoints " + Arrays.toString(splitpoints));
			lastSplitpoints = splitpoints;
			insert = -insert - 1;
			splitpoints = Arrays.copyOf(splitpoints, splitpoints.length + 1);
			System.arraycopy(splitpoints, insert, splitpoints, insert + 1, splitpoints.length - insert - 1);
			splitpoints[insert] = rank;
			splits++;
			restarts++;
			debug.writeMarkingReached(this, marking);
			debug.writeMarkingReached(this, maxRankMarking, "peripheries=2");
			//			System.out.println(" to " + Arrays.toString(splitpoints));

			return RESTART;
			// Handle this case now.
		}

	}

	private int getExactHeuristic(LpSolve solver, int marking, byte[] markingArray, int markingBlock, int markingIndex,
			double[] vars) {
		repeats++;
		long start = System.nanoTime();
		// start from correct right hand side
		try {

			int e = getLastRankOf(marking);
			int i = 0;
			while (splitpoints[i] < e) {
				i++;
			}
			i--;

			int r;
			for (r = 1; r <= i * product.numPlaces(); r++) {
				solver.setRh(r, 0);
			}
			for (; r <= rows - product.numPlaces(); r++) {
				solver.setRh(r, -markingArray[(r - 1) % product.numPlaces()]);
			}
			for (; r <= rows; r++) {
				solver.setRh(r, product.getFinalMarking()[(r - 1) % product.numPlaces()]
						- markingArray[(r - 1) % product.numPlaces()]);
			}

			solver.defaultBasis();
			// set timeout in seconds;
			long remainingTime = timeoutAtTimeInMillisecond - System.currentTimeMillis();
			// round the remaining time up to the nearest second.
			solver.setTimeout(Math.max(1000, remainingTime + 999) / 1000);
			int solverResult = solver.solve();
			synchronized (this) {
				heuristicsComputed++;
			}

			//			if (solverResult == LpSolve.INFEASIBLE || solverResult == LpSolve.NUMFAILURE) {
			//				// BVD: LpSolve has the tendency to give false infeasible or numfailure answers. 
			//				// It's unclear when or why this happens, but just in case...
			//				solverResult = solver.solve();
			//
			//			}
			if (solverResult == LpSolve.OPTIMAL) {
				// retrieve the solution
				solver.getVariables(vars);
				setNewLpSolution(marking, vars);

				// compute cost estimate
				double c = computeCostForVars(vars);

				if (c >= HEURISTICINFINITE) {
					synchronized (this) {
						alignmentResult |= Utils.HEURISTICFUNCTIONOVERFLOW;
					}
					// continue with maximum heuristic value not equal to infinity.
					return HEURISTICINFINITE - 1;
				}

				// assume precision 1E-7 and round down
				return (int) (c + 1E-7);
			} else if (solverResult == LpSolve.INFEASIBLE) {
				return HEURISTICINFINITE;
			} else if (solverResult == LpSolve.TIMEOUT) {
				//				System.out.println("Remaining was: "+remainingTime);
				remainingTime = timeoutAtTimeInMillisecond - System.currentTimeMillis();
				//				System.out.println("Remaining is:  "+remainingTime);
				assert remainingTime <= 0;
				synchronized (this) {
					alignmentResult |= Utils.TIMEOUTREACHED;
				}
				return HEURISTICINFINITE;
			} else {
				//					lp.writeLp("D:/temp/alignment/debugLP-Alignment.lp");
				System.err.println("Error code from LpSolve solver:" + solverResult);
				return HEURISTICINFINITE;
			}

		} catch (LpSolveException e) {
			e.printStackTrace();
			return HEURISTICINFINITE;
		} finally {
			synchronized (this) {
				solveTime += System.nanoTime() - start;
			}
		}

	}

	protected double computeCostForVars(double[] vars) {
		double c = 0;
		for (int t = vars.length; t-- > 0;) {
			c += vars[t] * net.getCost(indexMap[t]);
		}
		return c;
	}

	@Override
	protected void setNewLpSolution(int marking, double[] solutionDouble) {
		// copy the solution from double array to byte array (rounding down)
		// and compute the maximum.
		Arrays.fill(tempForSettingSolution, 0);
		byte bits = 1;
		for (short i = (short) solutionDouble.length; i-- > 0;) {
			tempForSettingSolution[translate(indexMap[i])] += ((int) (solutionDouble[i] + 1E-7));
			if (tempForSettingSolution[translate(indexMap[i])] > (1 << (bits - 1))) {
				bits++;
			}
		}
		setNewLpSolution(marking, bits, tempForSettingSolution);
	}

	protected void deriveOrEstimateHValue(int from, int fromBlock, int fromIndex, short transition, int to, int toBlock,
			int toIndex) {
		if (hasExactHeuristic(fromBlock, fromIndex) && (getLpSolution(from, transition) >= 1)) {
			// from Marking has exact heuristic
			// we can derive an exact heuristic from it

			setDerivedLpSolution(from, to, transition);
			// set the exact h score
			setHScore(toBlock, toIndex, getHScore(fromBlock, fromIndex) - net.getCost(transition), true);
			heuristicsDerived++;

			short r = getLastRankOf(to);
			if (r > maxRankExact) {
				maxRankExact = r;
				maxRankMarking = to;
				//				System.out.println("Explained event at rank " + r + " exactly.");
			}
		} else {
			if (isFinal(to)) {
				setHScore(toBlock, toIndex, 0, true);
				short r = getLastRankOf(to);
				if (r > maxRankExact) {
					maxRankExact = r;
					maxRankMarking = to;
					//				System.out.println("Explained event at rank " + r + " exactly.");
				}
			} else {
				int h = getHScore(fromBlock, fromIndex) - net.getCost(transition);
				if (h < 0) {
					h = 0;
				}
				if (h > getHScore(toBlock, toIndex)) {
					// estimated heuristic should not decrease.
					setHScore(toBlock, toIndex, h, false);
					heuristicsEstimated++;
				}
			}
		}

	}

	@Override
	protected long getEstimatedMemorySize() {
		long val = super.getEstimatedMemorySize();
		// approximate memory for LpSolve
		val += 8 * coefficients * 2;
		return val;
	}

	@Override
	protected void fillStatistics(short[] alignment) {
		super.fillStatistics(alignment);
		putStatistic(Statistic.HEURISTICTIME, (int) (solveTime / 1000));
		putStatistic(Statistic.SPLITS, splits);
		putStatistic(Statistic.RESTARTS, restarts);
	}

	@Override
	protected void writeEndOfAlignmentDot(short[] alignment, int markingsReachedInRun, int closedActionsInRun) {
		for (int m = 0; m < markingsReachedInRun; m++) {
			if (!isClosed(m)) {
				if (isDerivedLpSolution(m)) {
					debug.writeMarkingReached(this, m, "color=blue,style=bold");
				} else if (hasExactHeuristic(m)) {
					debug.writeMarkingReached(this, m, "style=bold");
				} else {
					debug.writeMarkingReached(this, m, "style=dashed");
				}
			}
		}
		if (alignment != null) {
			lastSplitpoints = splitpoints;
		}
		// close the subgraph
		StringBuilder b = new StringBuilder();
		b.append("info" + iteration + " [shape=plaintext,label=<");
		b.append("Iteration: " + iteration);
		b.append("<br/>");
		b.append("Markings reached: " + markingsReachedInRun);
		b.append("<br/>");
		b.append("Markings closed: " + closedActionsInRun);
		b.append("<br/>");
		b.append("Heuristics computed: " + heuristicsComputedInRun);
		b.append("<br/>");
		b.append("Splitpoints: ");
		b.append(Arrays.toString(lastSplitpoints));
		b.append(">];");
		debug.println(Debug.DOT, b.toString());
		// close the subgraph
		debug.println(Debug.DOT, "}");
		if (alignment != null) {
			b = new StringBuilder();
			b.append("subgraph cluster_info {");
			b.append("label=<Global results>;");
			b.append("info [shape=plaintext,label=<");
			for (Statistic s : Statistic.values()) {
				b.append(s);
				b.append(": ");
				b.append(replayStatistics.get(s));
				b.append("<br/>");
			}
			b.append(">];");
			debug.println(Debug.DOT, b.toString());
			// close the subgraph
			debug.println(Debug.DOT, "}");
			// close the graph
			debug.println(Debug.DOT, "}");
		}
	}

	@Override
	protected void processedMarking(int marking, int blockMarking, int indexInBlock) {
		super.processedMarking(marking, blockMarking, indexInBlock);
		if (isDerivedLpSolution(marking)) {
			debug.writeMarkingReached(this, marking, "color=blue,style=bold");
		} else {
			debug.writeMarkingReached(this, marking, "style=bold");
		}
		lpSolutionsSize -= 12 + 4 + lpSolutions.remove(marking).length; // object size
		lpSolutionsSize -= 1 + 4 + 8; // used flag + key + value pointer
	}
}