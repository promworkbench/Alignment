package nl.tue.alignment.algorithms;

import gnu.trove.list.TIntList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.set.TIntSet;
import gnu.trove.set.hash.TIntHashSet;

import java.util.Arrays;

import lpsolve.LpSolve;
import lpsolve.LpSolveException;
import nl.tue.alignment.ReplayAlgorithm;
import nl.tue.alignment.SyncProduct;
import nl.tue.alignment.Utils;
import nl.tue.alignment.Utils.Statistic;
import nl.tue.astar.util.ilp.LPMatrix;
import nl.tue.astar.util.ilp.LPMatrix.SPARSE.LPSOLVE;
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
public class AStarWithMarkingSplit extends ReplayAlgorithm {

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

	protected final double[] rhf;
	protected int bytesUsed;
	protected long solveTime = 0;

	//	protected int numRows;
	//	protected int numCols;

	private LpSolve solver;

	private LPMatrix.SPARSE.LPSOLVE matrix;
	private final boolean isInteger;
	private final int numTrans;
	TIntList privateSplits = new TIntArrayList();
	TIntSet noSplit = new TIntHashSet(10, 0.5f, -1);
	private int maxEventNumber;

	public AStarWithMarkingSplit(SyncProduct product) throws LPMatrixException {
		this(product, true, false, Debug.NONE);
	}

	public AStarWithMarkingSplit(SyncProduct product, boolean moveSorting, boolean isInteger, Debug debug)
			throws LPMatrixException {
		super(product, moveSorting, true, true, false, debug);
		this.isInteger = isInteger;
		//		this.numRows = net.numPlaces();
		matrix = new LPMatrix.SPARSE.LPSOLVE(net.numPlaces() + 1, net.numTransitions());

		// Set the objective to follow the cost function
		for (short t = net.numTransitions(); t-- > 0;) {
			matrix.setObjective(t, net.getCost(t));
			matrix.setMat(net.numPlaces(), t, net.getCost(t));

			short[] input = net.getInput(t);
			for (int i = input.length; i-- > 0;) {
				// transition t consumes from place p, hence  incidence matrix
				// is -1;
				matrix.adjustMat(input[i], t, -1);
				assert matrix.getMat(input[i], t) <= 1;
			}
			short[] output = net.getOutput(t);
			for (int i = output.length; i-- > 0;) {
				matrix.adjustMat(output[i], t, 1);
				assert matrix.getMat(output[i], t) <= 1;
			}

			// Use integer variables if specified
			matrix.setInt(t, isInteger);
			// Set lower bound of 0
			matrix.setLowbo(t, 0);
			// Set upper bound to 255 (we assume unsigned byte is sufficient to store result)
			matrix.setUpbo(t, 255);

			if (debug != Debug.NONE) {
				matrix.setColName(t, net.getTransitionLabel(t));
			}

		}

		rhf = new double[net.numPlaces() + 1];
		byte[] marking = net.getFinalMarking();
		for (short p = net.numPlaces(); p-- > 0;) {
			if (debug != Debug.NONE) {
				matrix.setRowName(p, net.getPlaceLabel(p));
			}
			// set the constraint to equality
			matrix.setConstrType(p, LPMatrix.EQ);
			rhf[p] = marking[p];
		}
		matrix.setConstrType(net.numPlaces(), LPMatrix.GE);

		matrix.setMinim();
		numTrans = net.numTransitions();
		privateSplits.add(-1);

		maxEventNumber = -1;
		for (short t = 0; t < numTrans; t++) {
			if (net.getEventOf(t) > maxEventNumber) {
				maxEventNumber = net.getEventOf(t);
			}
		}

		// split in max 5 equal parts
		for (int i = 1; i < 5; i++) {
			privateSplits.add((i * maxEventNumber) / 5);
		}

		this.setupTime = (int) ((System.nanoTime() - startConstructor) / 1000);
	}

	private void init() throws LPMatrixException {

		solver = matrix.toSolver();
		// bytes for solver
		bytesUsed = matrix.bytesUsed();
		// bytes for solvers inside monitorthreads
		//		numCols = net.numTransitions() + 1;
		varsMainThread = new double[net.numTransitions()];
		tempForSettingSolution = new int[net.numTransitions()];

	}

	@Override
	public short[] run() throws LPMatrixException {
		long start = System.nanoTime();
		init();
		return super.runReplayAlgorithm(start);
	}

	protected double[] varsMainThread;

	@Override
	public int getExactHeuristic(int marking, byte[] markingArray, int markingBlock, int markingIndex) {

		if (marking == 0)
			return getExactHeuristicInitial();

		//TODO: Compute the heuristic for this marking, but taking into account the current
		// split points greater than the last event in the trace reaching this marking.

		// Compute the heuristic for a marking that is now the head of the queue. All
		// elements in the top of the queue have the same F score, equal to this marking.
		// we have therefore reached a point where the original estimate was not accurate enough.
		int oldFScore = getGScore(marking) + getHScore(marking);
		int estimate = getExactHeuristic(solver, marking, markingArray, varsMainThread, getHScore(marking));
		int newFScore = getGScore(marking) + estimate;

		assert (newFScore >= oldFScore);

		// how many events have been explained so far?
		int evt = getLastEventOf(marking);
		int pos = privateSplits.binarySearch(evt);
		if (pos >= 0 || noSplit.contains(evt)) { //evt <= privateSplits.get(privateSplits.size() - 1)) {
			// we cannot split the trace, hence there's no point to try. Just update this marking
			// push it down the queue and try another one.
			return estimate;
		} else {
			pos = -pos - 1;
			privateSplits.insert(pos, evt);
		}
		//		int split = evt < 0 ? 0 : evt;
		//		privateSplits.add(split);

		// clone the matrix
		LPMatrix.SPARSE.LPSOLVE newMatrix;
		try {
			newMatrix = buildSplitMatrix(privateSplits, oldFScore);
		} catch (LPMatrixException e1) {
			return estimate;
		}
		//			OutputStreamWriter w = new OutputStreamWriter(System.out);
		//			newMatrix.printLp(w, ", ");
		//			w.flush();

		LpSolve largeSolver = null;
		int newFScoreInitialMarking = HEURISTICINFINITE;
		double[] largeVars;
		try {
			largeSolver = newMatrix.toSolver();
			largeVars = new double[newMatrix.getNcolumns()];
			//				largeSolver.setVerbose(4);
			newFScoreInitialMarking = solveLp(largeSolver, 0, largeVars);

		} catch (LPMatrixException | LpSolveException e) {
			return estimate;
		} finally {
			if (largeSolver != null) {
				largeSolver.deleteAndRemoveLp();
			}
		}

		//			double[] debugVars = new double[largeVars.length];
		//			m = marking;
		//			do {
		//				debugVars[getPredecessorTransition(m)] += 1;
		//				m = getPredecessor(m);
		//			} while (m != NOPREDECESSOR);
		//			for (short t = 0; t < numTrans; t++) {
		//				debugVars[numTrans + t] = getLpSolution(marking, t);
		//			}
		//			System.out.println("Witness cost: " + computeCostForVars(debugVars));			

		assert (newFScoreInitialMarking != HEURISTICINFINITE);
		//		assert (newFScoreInitialMarking <= newFScore);
		assert (newFScoreInitialMarking >= oldFScore);
		// remember: newFScore >= oldFScore;

		if (newFScoreInitialMarking == oldFScore) {
			// Too bad. We did not find an improvement on the heuristic from m0
			// using the additional splitpoint.
			noSplit.add(evt);
			privateSplits.removeAt(pos);

		} else {

			// we found an improved estimate for the initial marking!
			// update all open markings (which have equal F score by
			// definition) without changing the queue.
			//			assert queue.checkInv();
			int b, i;
			TIntList toRequeue = new TIntArrayList();
			for (int m = 0; m < markingsReached; m++) {
				b = m >>> blockBit;
				i = m & blockMask;
				if (!isClosed(b, i)) {
					// we are investigating a non-exact heuristic. No other
					// open marking in the queue can have a exact value.
					if (!hasExactHeuristic(b, i)) {
						setHScore(b, i, newFScoreInitialMarking - getGScore(b, i), false);
						//						assert getHScore(b, i) + getGScore(b, i) == newFScore;
					} else {
						// marking m has an exact score, but was not investigated yet
						// we can improve the score
						assert getHScore(b, i) + getGScore(b, i) >= oldFScore;
						if (getHScore(b, i) + getGScore(b, i) < newFScoreInitialMarking) {
							setHScore(b, i, newFScoreInitialMarking - getGScore(b, i), false);
							// exact marking. is in the queue and might have to be requeued later
							assert queue.contains(m);
							toRequeue.add(m);
						} else if (getHScore(b, i) + getGScore(b, i) == newFScoreInitialMarking) {
							// exact marking with equal score. is in the queue and might have to be requeued later
							assert queue.contains(m);
							toRequeue.add(m);
						}
					}
				}
			}
			for (i = toRequeue.size(); i-- > 0;) {
				addToQueue(toRequeue.get(i));
			}

			// set H score of marking to estimate.
			if (estimate >= getHScore(marking)) {
				setHScore(marking, estimate, true);
			}
			assert !queue.contains(marking);
			// requeueing will occur in caller method if applicable.

			writeStatus();

		} // else if result == newFScore

		return estimate;

	}

	protected int getLastEventOf(int marking) {
		int m = marking;
		short trans = getPredecessorTransition(m);
		while (net.getEventOf(trans) < 0 && m > 0) {
			m = getPredecessor(m);
			trans = getPredecessorTransition(m);
		}
		int evt = net.getEventOf(trans);
		return evt;
	}

	private LPSOLVE newMatrix = null;

	private LPMatrix.SPARSE.LPSOLVE buildSplitMatrix(TIntList splits, int lowerBound) throws LPMatrixException {
		if (newMatrix != null) {
			bytesUsed -= newMatrix.bytesUsed();
		}
		int objRow = splits.size() * net.numPlaces();
		newMatrix = new LPMatrix.SPARSE.LPSOLVE(objRow + 1, splits.size() * net.numTransitions());
		int blocks = splits.size() - 1;

		splits.add(maxEventNumber);
		int row;
		for (int b = 0; b <= blocks; b++) {
			for (int p = 0; p < net.numPlaces(); p++) {
				row = b * net.numPlaces() + p;
				if (b == blocks) {
					// right hand side in last block equals mf - mi
					newMatrix.setConstrType(row, LPMatrix.EQ);
					newMatrix.setRh(row, net.getFinalMarking()[p] - net.getInitialMarking()[p]);
				} else {
					// all blocks in between GE -mi
					newMatrix.setConstrType(row, LPMatrix.GE);
					newMatrix.setRh(row, -net.getInitialMarking()[p]);
				}
				for (short t = 0; t < net.numTransitions(); t++) {
					// get value in top-left A
					double val = matrix.getMat(p, t);
					for (int c = t; c < (b + 1) * numTrans; c += numTrans) {
						newMatrix.setInt(c, isInteger);
						newMatrix.setObjective(c, net.getCost(t));
						newMatrix.setMat(objRow, c, net.getCost(t));
						newMatrix.setLowbo(c, 0);
						newMatrix.setUpbo(c, 0);
						if (net.getEventOf(t) < 0) {
							newMatrix.setUpbo(c, 1);
						} else if (net.getEventOf(t) > splits.get(c / numTrans) && //
								net.getEventOf(t) <= splits.get(c / numTrans + 1)) {
							newMatrix.setUpbo(c, 1);
						}
						//						newMatrix.setUpbo(
						//								c,
						//								net.getEventOf(t) < 0
						//										|| (net.getEventOf(t) > splits[c / numTrans] && net.getEventOf(t) <= splits
						//												.get(c / numTrans + 1)) ? 1 : 0);
						if (val != 0) {
							newMatrix.setMat(row, c, val);
						} // if
					} // for c
				} // for col

			} // for row

		} // for b
		newMatrix.setConstrType(objRow, LPMatrix.GE);
		newMatrix.setRh(objRow, lowerBound);
		newMatrix.setMinim();
		bytesUsed += newMatrix.bytesUsed();
		splits.removeAt(splits.size() - 1);
		return newMatrix;
	}

	private int getExactHeuristicInitial() {

		LpSolve largeSolver = null;
		try {
			newMatrix = buildSplitMatrix(privateSplits, 0);

			largeSolver = newMatrix.toSolver();
			double[] largeVars = new double[newMatrix.getNcolumns()];
			//				largeSolver.setVerbose(4);
			return solveLp(largeSolver, 0, largeVars);

		} catch (LPMatrixException | LpSolveException ex) {
			return HEURISTICINFINITE;
		} finally {
			if (largeSolver != null) {
				largeSolver.deleteAndRemoveLp();
			}
		}

	}

	private int getExactHeuristic(LpSolve solver, int marking, byte[] markingArray, double[] vars, int minCost) {

		long start = System.nanoTime();
		// start from correct right hand side
		try {
			setRH(solver, markingArray, minCost);

			return solveLp(solver, marking, vars);

		} catch (LpSolveException e) {
			return HEURISTICINFINITE;
		} finally {
			synchronized (this) {
				solveTime += System.nanoTime() - start;
			}
		}

	}

	private int solveLp(LpSolve solver, int marking, double[] vars) throws LpSolveException {

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
	}

	protected void setRH(LpSolve solver, byte[] markingArray, int minCost) throws LpSolveException {
		for (int p = net.numPlaces(); p-- > 0;) {
			// set right hand side to final marking 
			solver.setRh(p + 1, rhf[p] - markingArray[p]);
		}
		solver.setRh(net.numPlaces() + 1, minCost);
	}

	protected double computeCostForVars(double[] vars) {
		double c = 0;
		for (int t = vars.length; t-- > 0;) {
			c += vars[t] * net.getCost((short) (t % numTrans));
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

	private int[] tempForSettingSolution;

	protected synchronized void setNewLpSolution(int marking, double[] solutionDouble) {
		// copy the solution from double array to byte array (rounding down)
		// and compute the maximum.
		byte bits = 1;
		for (int i = numTrans; i-- > 0;) {
			tempForSettingSolution[i] = ((int) (solutionDouble[i] + 5E-11));
			if (tempForSettingSolution[i] > (1 << bits)) {
				bits++;
			}
		}
		for (int i = solutionDouble.length; i-- > numTrans;) {
			tempForSettingSolution[i % numTrans] += ((int) (solutionDouble[i] + 5E-11));
			if (tempForSettingSolution[i % numTrans] > (1 << bits)) {
				bits++;
			}
		}

		// to store this solution, we need "bits" bits per transition
		// plus a header consisting of 8-FREEBITSFIRSTBYTE bits.
		// this translate to 
		int bytes = 8 - FREEBITSFIRSTBYTE + (tempForSettingSolution.length * bits + 4) / 8;

		assert marking == 0 || getSolution(marking) == null;
		byte[] solution = new byte[bytes];

		// set the computed flag in the first two bits
		solution[0] = COMPUTED;
		// set the number of bits used in the following 3 bits
		bits--;
		solution[0] |= bits << FREEBITSFIRSTBYTE;

		int currentByte = 0;
		byte currentBit = (1 << (FREEBITSFIRSTBYTE - 1));
		for (short t = 0; t < tempForSettingSolution.length; t++) {
			// tempForSettingSolution[i] can be stored in "bits" bits.
			for (int b = 1 << bits; b > 0; b >>>= 1) {
				// copy the appropriate bit
				if ((tempForSettingSolution[t] & b) != 0)
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
			//			if (h < 0) {
			//				h = 0;
			//			}
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
		val += bytesUsed;
		// count size of rhs
		val += rhf.length * 8 + 4;
		return val;
	}

	@Override
	public TObjectIntMap<Utils.Statistic> getStatistics() {
		TObjectIntMap<Statistic> map = super.getStatistics();
		map.put(Statistic.HEURISTICTIME, (int) (solveTime / 1000));
		map.put(Statistic.SPLITS, privateSplits.size() - 1);
		return map;
	}

	@Override
	protected void writeEndOfAlignmentDot() {
		TObjectIntMap<Statistic> map = getStatistics();
		for (int m = 0; m < markingsReached; m++) {
			if (isDerivedLpSolution(m)) {
				debug.writeMarkingReached(this, m, "color=blue");
			} else {
				debug.writeMarkingReached(this, m);
			}
		}
		StringBuilder b = new StringBuilder();
		b.append("info [shape=plaintext,label=<");
		for (Statistic s : Statistic.values()) {
			b.append(s);
			b.append(": ");
			b.append(map.get(s));
			b.append("<br/>");
		}
		b.append("Splits: " + (privateSplits.size() - 1));
		b.append("<br/>");
		b.append(">];");

		debug.writeDebugInfo(Debug.DOT, b.toString());
		debug.writeDebugInfo(Debug.DOT, "}");
	}

	protected void writeEndOfAlignmentNormal() {
		super.writeEndOfAlignmentNormal();
		debug.writeDebugInfo(Debug.NORMAL, "Splits: " + (privateSplits.size() - 1));
	}

	@Override
	protected void processedMarking(int marking, int blockMarking, int indexInBlock) {
		super.processedMarking(marking, blockMarking, indexInBlock);
		synchronized (this) {
			lpSolutionsSize -= 12 + 4 + lpSolutions.remove(marking).length; // object size
			lpSolutionsSize -= 1 + 4 + 8; // used flag + key + value pointer
		}
	}

	protected void terminateRun() {
		try {
			super.terminateRun();
		} finally {
			solver.deleteAndRemoveLp();
		}
	}

	/**
	 * Returns the h score for a stored marking (actually, we store F, hence h
	 * can be <0)
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	@Override
	public int getHScore(int block, int index) {
		return (int) ((e_g_h_pt[block][index] & HMASK) >>> HSHIFT) - getGScore(block, index);
	}

	/**
	 * set the h score for a stored marking (actually, we store F, hence h can
	 * be <0)
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	@Override
	public void setHScore(int block, int index, int score, boolean isExact) {
		long scoreL = ((long) score + getGScore(block, index)) << HSHIFT;
		assert (scoreL & HMASK) == scoreL;
		// overwrite the last three bytes of the score.
		e_g_h_pt[block][index] &= ~HMASK; // reset to 0
		e_g_h_pt[block][index] |= scoreL; // set score
		if (isExact) {
			e_g_h_pt[block][index] |= EXACTMASK; // set exactFlag
		} else {
			e_g_h_pt[block][index] &= ~EXACTMASK; // clear exactFlag

		}
	}

	@Override
	protected void writeStatus() {
		super.writeStatus();
		debug.writeDebugInfo(Debug.NORMAL, "   Split           :" + String.format("%,d", privateSplits.size() - 1));
	}

}