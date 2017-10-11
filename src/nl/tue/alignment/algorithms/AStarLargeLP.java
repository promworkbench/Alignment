package nl.tue.alignment.algorithms;

import gnu.trove.iterator.TShortIterator;
import gnu.trove.list.TShortList;
import gnu.trove.list.array.TShortArrayList;
import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.TShortObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TShortObjectHashMap;

import java.util.Arrays;

import lpsolve.LpSolve;
import lpsolve.LpSolveException;
import nl.tue.alignment.ReplayAlgorithm;
import nl.tue.alignment.SyncProduct;
import nl.tue.alignment.Utils;
import nl.tue.alignment.Utils.Statistic;
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
public class AStarLargeLP extends ReplayAlgorithm {

	// for each stored solution, the first byte is used for flagging.
	// the first bit indicates whether the solution is derived
	// The next three bits store the number of bits per transition (0 implies 1 bit per transition, 7 implies 8 bits per transition)
	// The rest of the array  then stores the solution
	protected static final byte COMPUTED = (byte) 0b00000000;
	protected static final byte DERIVED = (byte) 0b10000000;

	protected static final byte BITPERTRANSMASK = (byte) 0b01110000;
	protected static final byte FREEBITSFIRSTBYTE = 4;

	// stores the location of the LP solution plus a flag if it is derived or real
	protected TIntObjectMap<byte[]> lpSolutions = new TIntObjectHashMap<>(16);
	protected long lpSolutionsSize = 4;

	protected long solveTime = 0;
	protected int heuristicsComputedInRun = 0;;

	//	protected int numRows;
	//	protected int numCols;
	private short[] indexMap;

	private final TShortObjectMap<TShortList> trans2LSMove = new TShortObjectHashMap<>();

	private LpSolve solver;

	private int numEvents;

	private int modelMoves;

	private SyncProduct product;
	private boolean useInteger;

	public AStarLargeLP(SyncProduct product) {
		this(product, false, false, Debug.NONE);
	}

	public AStarLargeLP(SyncProduct product, boolean moveSorting, boolean useInteger, Debug debug) {
		super(product, moveSorting, true, true, false, debug);
		this.product = product;
		this.useInteger = useInteger;

		trans2LSMove.put(SyncProduct.NOEVENT, new TShortArrayList(10));
		numEvents = -1;
		TShortList set;
		for (short t = 0; t < product.numTransitions(); t++) {
			short e = product.getEventOf(t);
			set = trans2LSMove.get(e);
			if (set == null) {
				set = new TShortArrayList(3);
				trans2LSMove.put(e, set);
			}
			set.add(t);
			if (product.getEventOf(t) > numEvents) {
				numEvents = product.getEventOf(t);
			}
		}
		modelMoves = trans2LSMove.get(SyncProduct.NOEVENT).size();
		numEvents++;
		if (numEvents == 0) {
			// ensure model is not empty for empty trace
			numEvents = 1;
		}
		//		splitpoints = new short[] { 0, 2, 4, (short) numEvents };
		splitpoints = new short[] { 0, (short) numEvents };

		this.setupTime = (int) ((System.nanoTime() - startConstructor) / 1000);
	}

	private short[] splitpoints;
	private short[] lastSplitpoints;

	private int rows;
	private int coefficients;

	private void init() throws LPMatrixException {
		lpSolutions.clear();
		lpSolutionsSize = 0;

		rows = (splitpoints.length - 1) * product.numPlaces();

		indexMap = new short[(splitpoints.length - 2) * modelMoves + product.numTransitions()];

		try {
			if (solver != null) {
				solver.deleteAndRemoveLp();
			}
			coefficients = 0;
			solver = LpSolve.makeLp(rows, 0);
			solver.setAddRowmode(false);

			double[] col = new double[1 + rows];
			short[] input, output;
			int c = 0;

			int start = 1;
			for (int s = 1; s < splitpoints.length; s++) {
				// add model moves in this block (if any)
				TShortIterator it;
				if (trans2LSMove.get(SyncProduct.NOEVENT) != null) {
					it = trans2LSMove.get(SyncProduct.NOEVENT).iterator();
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

				//add log and sync moves in this block.
				for (short e = splitpoints[s - 1]; e < splitpoints[s]; e++) {
					if (trans2LSMove.get(e) != null) {
						it = trans2LSMove.get(e).iterator();
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
							solver.setUpbo(c, 1);
							coefficients++;
							solver.setInt(c, useInteger);
							solver.setObj(c, product.getCost(t));
							coefficients++;
						} // for all sync/log moves
					} // if sync/logMoves
				}
				start += product.numPlaces();
			}

			int r;
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
		heuristicsComputedInRun = 0;
		varsMainThread = new double[indexMap.length];
		tempForSettingSolutionDouble = new double[net.numTransitions()];

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
	protected int splits = 0;

	@Override
	public int getExactHeuristic(int marking, byte[] markingArray, int markingBlock, int markingIndex) {
		// find an available solver and block until one is available.

		short event = marking == 0 ? SyncProduct.NOEVENT : getLastEventOf(marking);

		// the current shortest path explains the events up to and including event, but cannot continue
		// serializing a previous LP solution at this stage. Hence, around 'marking' should be a 
		// split marking and therefore event+1 needs to be added to the splitpoints.
		int insert;
		do {
			// find the insertion point
			insert = Arrays.binarySearch(splitpoints, ++event);
			// if event already a splitpoint, add the first larger event.
		} while (insert >= 0);

		if (marking == 0 || event > numEvents || event == SyncProduct.NOEVENT) {
			// No event was explained yet, or the last explained event is already a splitpoint.
			// There's little we can do but continue with the replayer.
			//			debug.writeDebugInfo(Debug.NORMAL, "Solve call started");
			int res = getExactHeuristic(solver, marking, markingArray, markingBlock, markingIndex, varsMainThread);
			//			debug.writeDebugInfo(Debug.NORMAL, "End solve: " + (System.currentTimeMillis() - s) / 1000.0 + " ms.");
			heuristicsComputedInRun++;
			return res;
		}
		lastSplitpoints = splitpoints;
		insert = -insert - 1;
		splitpoints = Arrays.copyOf(splitpoints, splitpoints.length + 1);
		System.arraycopy(splitpoints, insert, splitpoints, insert + 1, splitpoints.length - insert - 1);
		splitpoints[insert] = event;
		splits++;
		debug.writeMarkingReached(this, marking,"peripheries=2");

		return RESTART;
		// Handle this case now.

	}

	protected short getLastEventOf(int marking) {
		int m = marking;
		short trans = getPredecessorTransition(m);
		while (net.getEventOf(trans) < 0 && m > 0) {
			m = getPredecessor(m);
			trans = getPredecessorTransition(m);
		}
		short evt = net.getEventOf(trans);
		return evt;
	}

	private int getExactHeuristic(LpSolve solver, int marking, byte[] markingArray, int markingBlock, int markingIndex,
			double[] vars) {

		long start = System.nanoTime();
		// start from correct right hand side
		try {

			int e = getLastEventOf(marking);
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

				// assume precision 1E-9 and round down
				return (int) (c + 1E-9);
			} else if (solverResult == LpSolve.INFEASIBLE) {

				return HEURISTICINFINITE;
			} else {
				//					lp.writeLp("D:/temp/alignment/debugLP-Alignment.lp");
				//System.err.println("Error code from LpSolve solver:" + solverResult);
				return HEURISTICINFINITE;
			}

		} catch (LpSolveException e) {
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

	protected synchronized int getLpSolution(int marking, short transition) {
		byte[] solution = getSolution(marking);
		//		if ((solution[0] & STOREDFULL) == STOREDFULL) {
		// get the bits used per transition
		int bits = 1 + ((solution[0] & BITPERTRANSMASK) >>> FREEBITSFIRSTBYTE);
		// which is the first bit?
		int fromBit = 8 - FREEBITSFIRSTBYTE + transition * bits;
		// that implies the following byte
		int fromByte = fromBit >>> 3;
		// with the following index in byte.
		fromBit &= 7;

		byte currentBit = (byte) (1 << (7 - fromBit));
		int value = 0;
		for (int i = 0; i < bits; i++) {
			// shift value left
			value <<= 1;

			// flip the bit
			if ((solution[fromByte] & currentBit) != 0)
				value++;

			// rotate bit right 
			currentBit = (byte) (((currentBit & 0xFF) >>> 1) | (currentBit << 7));
			// increase byte if needed.
			if (currentBit < 0)
				fromByte++;

		}

		return value;
	}

	protected synchronized boolean isDerivedLpSolution(int marking) {
		return getSolution(marking) != null && (getSolution(marking)[0] & DERIVED) == DERIVED;
	}

	protected synchronized void setDerivedLpSolution(int from, int to, short transition) {
		assert getSolution(to) == null;
		byte[] solutionFrom = getSolution(from);

		byte[] solution = Arrays.copyOf(solutionFrom, solutionFrom.length);

		solution[0] |= DERIVED;

		// get the length of the bits used per transition
		int bits = 1 + ((solution[0] & BITPERTRANSMASK) >>> FREEBITSFIRSTBYTE);
		// which is the least significant bit?
		int fromBit = 8 - FREEBITSFIRSTBYTE + transition * bits + (bits - 1);
		// that implies the following byte
		int fromByte = fromBit >>> 3;
		// with the following index in byte.
		fromBit &= 7;
		// most significant bit in fromBit
		byte lsBit = (byte) (1 << (7 - fromBit));

		// we need to reduce by 1.
		for (int i = 0; i < bits; i++) {
			// flip the bit
			if ((solution[fromByte] & lsBit) != 0) {
				// first bit that is 1. Flip and terminate
				solution[fromByte] ^= lsBit;
				//					assert getLpSolution(to, transition) == getLpSolution(from, transition) - 1;
				addSolution(to, solution);
				return;
			}
			// flip and continue;
			solution[fromByte] ^= lsBit;
			// rotate bit left
			lsBit = (byte) (((lsBit & 0xFF) >>> 7) | (lsBit << 1));
			// decrease byte if needed.
			if (lsBit == 1)
				fromByte--;

		}
		assert false;

		//		}
	}

	private double[] tempForSettingSolutionDouble;

	protected synchronized void setNewLpSolution(int marking, double[] solutionDouble) {
		// copy the solution from double array to byte array (rounding down)
		// and compute the maximum.
		Arrays.fill(tempForSettingSolutionDouble, 0);
		byte bits = 1;
		for (int i = solutionDouble.length; i-- > 0;) {
			// sum double values, compensating for LpSolve precision.
			tempForSettingSolutionDouble[indexMap[i]] += solutionDouble[i] + 1E-10;
			if (tempForSettingSolutionDouble[indexMap[i]] > (1 << bits)) {
				bits++;
			}
		}

		// to store this solution, we need "bits" bits per transition
		// plus a header consisting of 8-FREEBITSFIRSTBYTE bits.
		// this translate to 
		int bytes = 8 - FREEBITSFIRSTBYTE + (tempForSettingSolutionDouble.length * bits + 4) / 8;

		assert getSolution(marking) == null;
		byte[] solution = new byte[bytes];

		// set the computed flag in the first two bits
		solution[0] = COMPUTED;
		// set the number of bits used in the following 3 bits
		bits--;
		solution[0] |= bits << FREEBITSFIRSTBYTE;

		int currentByte = 0;
		byte currentBit = (1 << (FREEBITSFIRSTBYTE - 1));
		for (short t = 0; t < tempForSettingSolutionDouble.length; t++) {
			// tempForSettingSolution[i] can be stored in "bits" bits.
			for (int b = 1 << bits; b > 0; b >>>= 1) {
				// round the sum down.
				int val = (int) tempForSettingSolutionDouble[t];
				// copy the appropriate bit
				if ((val & b) != 0)
					solution[currentByte] |= currentBit;

				// rotate right
				currentBit = (byte) ((((currentBit & 0xFF) >>> 1) | (currentBit << 7)));
				if (currentBit < 0)
					currentByte++;

			}
			//			assert getLpSolution(marking, t) == tempForSettingSolution[t];
		}
		addSolution(marking, solution);
	}

	private byte[] getSolution(int marking) {
		return lpSolutions.get(marking);
	}

	private void addSolution(int marking, byte[] solution) {
		lpSolutions.put(marking, solution);
		lpSolutionsSize += 12 + 4 + solution.length; // object size
		lpSolutionsSize += 1 + 4 + 8; // used flag + key + value pointer
	}

	/**
	 * In ILP version, only one given final marking is the target.
	 */
	protected boolean isFinal(int marking) {
		return equalMarking(marking, net.getFinalMarking());
	}

	protected void deriveOrEstimateHValue(int from, int fromBlock, int fromIndex, short transition, int to,
			int toBlock, int toIndex) {
		if (hasExactHeuristic(fromBlock, fromIndex) && (getLpSolution(from, transition) >= 1)) {
			// from Marking has exact heuristic
			// we can derive an exact heuristic from it

			setDerivedLpSolution(from, to, transition);
			// set the exact h score
			setHScore(toBlock, toIndex, getHScore(fromBlock, fromIndex) - net.getCost(transition), true);
			heuristicsDerived++;

		} else {
			if (isFinal(to)) {
				setHScore(toBlock, toIndex, 0, true);
			}
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

	@Override
	protected long getEstimatedMemorySize() {
		long val = super.getEstimatedMemorySize();
		// count space for all computed solutions
		val += lpSolutionsSize;
		// count size of matrix
		// approximate memory for LpSolve
		val += 8 * coefficients * 2;
		// count size of solver

		return val;
	}

	@Override
	public TObjectIntMap<Utils.Statistic> getStatistics() {
		TObjectIntMap<Statistic> map = super.getStatistics();
		map.put(Statistic.HEURISTICTIME, (int) (solveTime / 1000));
		map.put(Statistic.SPLITS, splits);
		return map;
	}

	@Override
	protected void writeEndOfAlignmentDot(boolean done, int markingsReachedInRun, int closedActionsInRun) {
		TObjectIntMap<Statistic> map = getStatistics();
		for (int m = 0; m < markingsReachedInRun; m++) {
			if (!isClosed(m) && isDerivedLpSolution(m)) {
				debug.writeMarkingReached(this, m, "color=blue");
			} else {
				debug.writeMarkingReached(this, m);
			}
		}
		if (done) {
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
		debug.writeDebugInfo(Debug.DOT, b.toString());
		// close the subgraph
		debug.writeDebugInfo(Debug.DOT, "}");
		if (done) {
			b = new StringBuilder();
			b.append("subgraph cluster_info {");
			b.append("label=<Global results>;");
			b.append("info [shape=plaintext,label=<");
			for (Statistic s : Statistic.values()) {
				b.append(s);
				b.append(": ");
				b.append(map.get(s));
				b.append("<br/>");
			}
			b.append(">];");
			debug.writeDebugInfo(Debug.DOT, b.toString());
			// close the subgraph
			debug.writeDebugInfo(Debug.DOT, "}");
			// close the graph
			debug.writeDebugInfo(Debug.DOT, "}");
		}
	}

	@Override
	protected void processedMarking(int marking, int blockMarking, int indexInBlock) {
		super.processedMarking(marking, blockMarking, indexInBlock);
		synchronized (this) {
			if (isDerivedLpSolution(marking)) {
				debug.writeMarkingReached(this, marking, "color=blue");
			} else {
				debug.writeMarkingReached(this, marking);
			}
			lpSolutionsSize -= 12 + 4 + lpSolutions.remove(marking).length; // object size
			lpSolutionsSize -= 1 + 4 + 8; // used flag + key + value pointer
		}
	}

	@Override
	protected void terminateIteration(boolean done, int markingsReachedInRun, int closedActionsInRun) {
		try {
			super.terminateIteration(done, markingsReachedInRun, closedActionsInRun);
		} finally {
			solver.deleteAndRemoveLp();
		}
	}

	//	@Override
	//	protected short[] handleFinalMarkingReached(long startTime, int marking) {
	//		// Final marking reached.
	//		int s = splitpoints.length - 2;
	//		int n = getPredecessor(marking);
	//		int m2 = marking;
	//		short t;
	//		while (n != NOPREDECESSOR) {
	//			t = getPredecessorTransition(m2);
	//			if (s > 0 && product.getEventOf(t) == splitpoints[s]) {
	//				debug.writeEdgeTraversed(this, n, t, m2, "color=red,fontcolor=red,style=dashed");
	//				s--;
	//			} else {
	//				debug.writeEdgeTraversed(this, n, t, m2, "color=red,fontcolor=red");
	//			}
	//			alignmentLength++;
	//			alignmentCost += net.getCost(t);
	//			m2 = n;
	//			n = getPredecessor(n);
	//		}
	//		short[] alignment = new short[alignmentLength];
	//		n = getPredecessor(marking);
	//		m2 = marking;
	//		int l = alignmentLength;
	//		while (n != NOPREDECESSOR) {
	//			t = getPredecessorTransition(m2);
	//			alignment[--l] = t;
	//			m2 = n;
	//			n = getPredecessor(n);
	//		}
	//
	//		alignmentResult |= Utils.OPTIMALALIGNMENT;
	//		runTime = (int) ((System.nanoTime() - startTime) / 1000);
	//
	//		return alignment;
	//	}

}