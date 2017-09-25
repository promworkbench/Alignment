package nl.tue.alignment.algorithms;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TIntObjectHashMap;

import java.util.Arrays;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import lpsolve.LpSolve;
import lpsolve.LpSolveException;
import nl.tue.alignment.ReplayAlgorithm;
import nl.tue.alignment.SyncProduct;
import nl.tue.alignment.Utils;
import nl.tue.alignment.Utils.Statistic;
import nl.tue.astar.util.LPProblemProvider;
import nl.tue.astar.util.ilp.LPMatrix;
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
public class AStar extends ReplayAlgorithm {

	private final class BlockMonitor implements Runnable {

		private final int currentBlock;

		private final double[] varsBlockMonitor;

		public BlockMonitor(int currentBlock, int numTrans) {
			this.currentBlock = currentBlock;
			this.varsBlockMonitor = new double[numTrans];
		}

		public void run() {
			int b = currentBlock;
			int i = 0;
			do {
				int m = b * blockSize + i;
				if (m < markingsReached) {
					//					System.out.println("Monitor waiting for " + b + "," + i);
					getLockForComputingEstimate(b, i);
					//					System.out.println("Monitor locking " + b + "," + i);
					try {
						if (!isClosed(b, i) && !hasExactHeuristic(b, i)) {
							// an open marking without an exact solution for the heuristic
							LpSolve solver = provider.firstAvailable();
							int heuristic;
							try {
								heuristic = getExactHeuristic(solver, m, getMarking(m), b, i, varsBlockMonitor);
							} finally {
								provider.finished(solver);
							}
							if (heuristic > getHScore(b, i)) {
								setHScore(b, i, heuristic, true);
								// sort the marking in the queue
								assert queue.contains(m);
								queue.add(m);
								queueActions++;
							} else {
								setHScore(b, i, heuristic, true);
							}
						}
					} finally {
						//						System.out.println("Monitor releasing " + b + "," + (m & blockMask));
						releaseLockForComputingEstimate(b, m & blockMask);
					}
					i++;
				} else {
					synchronized (e_g_h_pt[b]) {
						try {
							e_g_h_pt[b].wait(200);
						} catch (InterruptedException e) {
						}
					}
				}
			} while (!threadpool.isShutdown() && i < blockSize);

			//			System.out.println("Closing monitor for block " + b + ". Threadpool shutdown: " + threadpool.isShutdown()
			//					+ ".");

		}
	}

	private ExecutorService threadpool;

	// for each stored solution, the first byte is used for flagging.
	// the first bit indicates whether the solution is derived
	// the second bit indicates whether the solition is stored in full
	// For fully stored solutions, the next three bits store the number of bits per transition (0 implies 1 bit per transition, 7 implies 8 bits per transition)
	// The rest of the array is then the stores solution
	// For derived solutions, the last 6 bits of the first byte and the second byte store the transition that was fired
	// the remaining bytes store the offset into the predecessor in as few bytes as possible.
	protected static final byte STOREDFULL = (byte) 0b01000000;
	protected static final byte DERIVED = (byte) 0b10000000;
	protected static final byte DERIVEDANDSTOREDFULL = (byte) 0b11000000;;
	protected static final byte BITPERTRANSMASK = (byte) 0b00111000;

	// stores the location of the LP solution plus a flag if it is derived or real
	protected TIntObjectMap<byte[]> lpSolutions = new TIntObjectHashMap<>(16);
	protected long lpSolutionsSize = 4;

	protected final double[] rhf;
	protected int bytesUsed;
	protected long solveTime = 0;
	protected final int numRows;
	protected final int numCols;

	private final LPProblemProvider provider;
	private final LpSolve solver;

	private final boolean doMultiThreading;

	public AStar(SyncProduct product) throws LPMatrixException {
		this(product, true, true, true, false, false, Debug.NONE);
	}

	public AStar(SyncProduct product, boolean moveSorting, boolean queueSorting, boolean preferExact,
			boolean isInteger, boolean doMultiThreading, Debug debug) throws LPMatrixException {
		super(product, moveSorting, queueSorting, preferExact, debug);
		this.doMultiThreading = doMultiThreading;
		this.numRows = net.numPlaces();

		LPMatrix.SPARSE.LPSOLVE matrix = new LPMatrix.SPARSE.LPSOLVE(net.numPlaces(), net.numTransitions());

		// Set the objective to follow the cost function
		for (short t = net.numTransitions(); t-- > 0;) {
			matrix.setObjective(t, net.getCost(t));

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

		rhf = new double[net.numPlaces()];
		byte[] marking = net.getFinalMarking();
		for (short p = net.numPlaces(); p-- > 0;) {
			if (debug != Debug.NONE) {
				matrix.setRowName(p, net.getPlaceLabel(p));
			}
			// set the constraint to equality
			matrix.setConstrType(p, LPMatrix.EQ);
			rhf[p] = marking[p];
		}
		matrix.setMinim();

		int monitorThreads;
		try {
			solver = matrix.toSolver();
			if (doMultiThreading) {
				monitorThreads = Math.max(1, Runtime.getRuntime().availableProcessors() / 2);
				provider = new LPProblemProvider(solver.copyLp(), monitorThreads);
				threadpool = Executors.newFixedThreadPool(monitorThreads);
			} else {
				monitorThreads = 0;
				provider = null;
				threadpool = null;
			}
		} catch (LpSolveException e) {
			throw new LPMatrixException(e);
		}
		// bytes for solver
		bytesUsed = matrix.bytesUsed();
		// bytes for solvers inside monitorthreads
		bytesUsed += matrix.bytesUsed() * monitorThreads;
		numCols = net.numTransitions() + 1;
		varsMainThread = new double[net.numTransitions()];
		tempForSettingSolution = new int[net.numTransitions()];

		this.setupTime = (int) ((System.nanoTime() - startConstructor) / 1000);

	}

	@Override
	protected void growArrays() {
		super.growArrays();
		if (doMultiThreading) {
			threadpool.submit(new BlockMonitor(block, numCols - 1));
		}
	}

	protected final double[] varsMainThread;

	@Override
	public int getExactHeuristic(int marking, byte[] markingArray, int markingBlock, int markingIndex) {
		// find an available solver and block until one is available.

		return getExactHeuristic(solver, marking, markingArray, markingBlock, markingIndex, varsMainThread);
	}

	private int getExactHeuristic(LpSolve solver, int marking, byte[] markingArray, int markingBlock, int markingIndex,
			double[] vars) {

		long start = System.nanoTime();
		// start from correct right hand side
		try {
			for (int p = net.numPlaces(); p-- > 0;) {
				// set right hand side to final marking 
				solver.setRh(p + 1, rhf[p] - markingArray[p]);
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
		for (short t = net.numTransitions(); t-- > 0;) {
			c += vars[t] * net.getCost(t);
		}
		return c;
	}

	protected synchronized int getLpSolution(int marking, short transition) {
		byte[] solution = getSolution(marking);
		if ((solution[0] & STOREDFULL) == STOREDFULL) {
			// get the bits used per transition
			int bits = 1 + ((solution[0] & BITPERTRANSMASK) >>> 3);
			// which is the first bit?
			int fromBit = 5 + transition * bits;
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

			return value;//getSolution(marking)[transition + 1] & 0xFF;
		} else {
			int from = 0;
			for (int i = 2; i < solution.length; i++) {
				from <<= 8;
				from |= solution[i] & 0xFF;
			}

			int fired = ((solution[0] & 0b00111111) << 8);
			fired |= (solution[1] & 0xFF);
			return getLpSolution(from, transition) - (fired == transition ? 1 : 0);
		}

	}

	protected synchronized boolean isDerivedLpSolution(int marking) {
		return getSolution(marking) != null && (getSolution(marking)[0] & DERIVED) == DERIVED;
	}

	protected synchronized void setDerivedLpSolution(int from, int to, short transition) {
		assert getSolution(to) == null;

		int bytes = 6;
		if (from == 0) {
			bytes = 2;
		} else if (from < 256) {
			bytes = 3;
		} else if (from < 65536) {
			bytes = 4;
		} else if (from < 16777216) {
			bytes = 5;
		}
		byte[] solutionFrom = getSolution(from);
		if ((solutionFrom[0] & DERIVED) == DERIVED || solutionFrom.length >= bytes) {
			// only use 6 bytes if this indeed saves memory.

			// transition uses at most 14 bits. We use the 6 bits in the first byte
			// and 8 bits in the second one to store it.
			byte[] solution = new byte[bytes];

			solution[0] = DERIVED;
			solution[0] |= (transition >>> 8) & 0xFF;
			solution[1] = (byte) (transition);

			for (int i = bytes; i-- > 2;) {
				solution[i] = (byte) (from & 0xFF);
				from >>>= 8;
			}
			addSolution(to, solution);

			//			assert (getSolution(to)[0] & DERIVED) == DERIVED;

		} else {
			byte[] solution = Arrays.copyOf(solutionFrom, solutionFrom.length);

			solution[0] |= DERIVEDANDSTOREDFULL;

			// get the length of the bits used per transition
			int bits = 1 + ((solution[0] & BITPERTRANSMASK) >>> 3);
			// which is the least significant bit?
			int fromBit = 5 + transition * bits + (bits - 1);
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

		}
	}

	private final int[] tempForSettingSolution;

	protected synchronized void setNewLpSolution(int marking, double[] solutionDouble) {
		// copy the solution from double array to byte array (rounding down)
		// and compute the maximum.
		byte bits = 1;
		for (int i = solutionDouble.length; i-- > 0;) {
			tempForSettingSolution[i] = ((int) (solutionDouble[i] + 5E-11));
			if (tempForSettingSolution[i] > (1 << bits)) {
				bits++;
			}
		}

		// to store this solution, we need "bits" bits per transition
		// plus a header consisting of 5 bits.
		// this translate to 
		int bytes = 5 + (tempForSettingSolution.length * bits + 4) / 8;

		assert getSolution(marking) == null;
		byte[] solution = new byte[bytes];
		addSolution(marking, solution);

		// set the computed flag in the first two bits
		solution[0] |= STOREDFULL;
		// set the number of bits used in the following 3 bits
		bits--;
		solution[0] |= bits << 3;

		int currentByte = 0;
		byte currentBit = (1 << 2);
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
		lpSolutionsSize += 4 + solution.length;
	}

	private byte[] getSolution(int marking) {
		return lpSolutions.get(marking);
	}

	private void addSolution(int marking, byte[] solution) {
		lpSolutions.put(marking, solution);
		lpSolutionsSize += 4 + solution.length;
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
		val += bytesUsed;
		// count size of rhs
		val += rhf.length * 8 + 4;
		return val;
	}

	@Override
	public TObjectIntMap<Utils.Statistic> getStatistics() {
		TObjectIntMap<Statistic> map = super.getStatistics();
		map.put(Statistic.HEURISTICTIME, (int) (solveTime / 1000));
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
		b.append(">];");

		debug.writeDebugInfo(Debug.DOT, b.toString());
		debug.writeDebugInfo(Debug.DOT, "}");
	}

	@Override
	protected void processedMarking(int marking, int blockMarking, int indexInBlock) {
	}

	protected void terminateRun() {
		super.terminateRun();
		if (doMultiThreading) {
			threadpool.shutdown();
			while (!threadpool.isTerminated()) {
				try {
					threadpool.awaitTermination(100, TimeUnit.MILLISECONDS);
				} catch (InterruptedException e) {
				}
			}
			provider.deleteLps();
		}
		solver.deleteAndRemoveLp();
	}

}