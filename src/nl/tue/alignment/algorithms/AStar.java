package nl.tue.alignment.algorithms;

import gnu.trove.map.TObjectIntMap;

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

		private int currentBlock;

		public BlockMonitor(int currentBlock) {
			this.currentBlock = currentBlock;
		}

		public void run() {
			int b = currentBlock;
			int i = 0;
			do {
				int m = b * blockSize + i;
				if (m < markingsReached) {
					getLockForComputingEstimate(b, i);
					if (!isClosed(b, i)) {
						// the marking was not closed yet

						// obtain a lock
						// there is a marking at index i
						if (!hasExactHeuristic(b, i)) {
							LpSolve solver = provider.firstAvailable();
							int heuristic;
							try {
								heuristic = getExactHeuristic(solver, m, getMarking(m), b, i);
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
					}
					releaseLockForComputingEstimate(b, i);
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
		}
	}

	private ExecutorService threadpool = Executors.newFixedThreadPool(Math.max(1, Runtime.getRuntime()
			.availableProcessors() / 2 - 1));

	protected static final byte COMPUTED = 0;
	protected static final byte DERIVED = 1;
	protected static final byte DERIVEDBUTSTOREDFULL = 2;

	protected static int DEFAULTARRAYSIZE = 16;

	// stores the location of the LP solution plus a flag if it is derived or real
	protected byte[][] lpSolutions = new byte[0][];
	protected long lpSolutionsSize = 4;

	protected final double[] rhf;
	protected final int bytesUsed;
	protected long solveTime = 0;
	protected final int numRows;
	protected final int numCols;

	private final LPProblemProvider provider;

	public AStar(SyncProduct product) throws LPMatrixException {
		this(product, true, true, true, false, Debug.NONE);
	}

	public AStar(SyncProduct product, boolean moveSorting, boolean queueSorting, boolean preferExact,
			boolean isInteger, Debug debug) throws LPMatrixException {
		super(product, moveSorting, queueSorting, preferExact, debug);
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
		try {
			provider = new LPProblemProvider(matrix.toSolver(), (Runtime.getRuntime().availableProcessors() + 1) / 2);
		} catch (LpSolveException e) {
			throw new LPMatrixException(e);
		}
		bytesUsed = matrix.bytesUsed() * (Runtime.getRuntime().availableProcessors() + 1) / 2;
		numCols = net.numTransitions() + 1;
		vars = new double[net.numTransitions()];

		this.setupTime = (int) ((System.nanoTime() - startConstructor) / 1000);

	}

	@Override
	protected void growArrays() {
		super.growArrays();
		threadpool.submit(new BlockMonitor(block));
	}

	protected double[] vars;

	@Override
	public int getExactHeuristic(int marking, byte[] markingArray, int markingBlock, int markingIndex) {
		// find an available solver and block until one is available.

		long start = System.nanoTime();
		LpSolve solver = provider.firstAvailable();
		try {
			return getExactHeuristic(solver, marking, markingArray, markingBlock, markingIndex);
		} finally {
			solveTime += System.nanoTime() - start;
			provider.finished(solver);
		}

	}

	private int getExactHeuristic(LpSolve solver, int marking, byte[] markingArray, int markingBlock, int markingIndex) {

		// start from correct right hand side
		try {
			for (int p = net.numPlaces(); p-- > 0;) {
				// set right hand side to final marking 
				solver.setRh(p + 1, rhf[p] - markingArray[p]);
			}

			solver.defaultBasis();
			int solverResult = solver.solve();
			heuristicsComputed++;

			//			if (solverResult == LpSolve.INFEASIBLE || solverResult == LpSolve.NUMFAILURE) {
			//				// BVD: LpSolve has the tendency to give false infeasible or numfailure answers. 
			//				// It's unclear when or why this happens, but just in case...
			//				solverResult = solver.solve();
			//
			//			}
			if (solverResult == LpSolve.OPTIMAL) {
				// retrieve the solution
				solver.getVariables(vars);
				if (lpSolutions.length <= marking) {
					growArray(marking);
				}
				setNewLpSolution(marking, vars);

				// compute cost estimate
				double c = computeCostForVars();

				if (c >= HEURISTICINFINITE) {
					alignmentResult |= Utils.HEURISTICFUNCTIONOVERFLOW;
					// continue with maximum heuristic value not equal to infinity.
					return HEURISTICINFINITE - 1;
				}

				// assume precision 1E-9 and round down
				return (int) (c + 1E-9);
			} else if (solverResult == LpSolve.INFEASIBLE) {

				return HEURISTICINFINITE;
			} else {
				//					lp.writeLp("D:/temp/alignment/debugLP-Alignment.lp");
				System.err.println("Error code from LpSolve solver:" + solverResult);
				return HEURISTICINFINITE;
			}

		} catch (LpSolveException e) {
			return HEURISTICINFINITE;
		}

	}

	protected double computeCostForVars() {
		double c = 0;
		for (short t = net.numTransitions(); t-- > 0;) {
			c += vars[t] * net.getCost(t);
		}
		return c;
	}

	protected synchronized void growArray(int marking) {
		// grow lpSolutions by a single block at the time.
		lpSolutions = Arrays.copyOf(lpSolutions, marking + DEFAULTARRAYSIZE);
		lpSolutionsSize += DEFAULTARRAYSIZE * 8;
	}

	protected synchronized int getLpSolution(int marking, short transition) {
		if (lpSolutions[marking][0] != DERIVED) {
			return lpSolutions[marking][transition + 1] & 0xFF;
		} else {
			int pre = ((lpSolutions[marking][1] & 0xFF) << 24);
			pre |= ((lpSolutions[marking][2] & 0xFF) << 16);
			pre |= ((lpSolutions[marking][3] & 0xFF) << 8);
			pre |= (lpSolutions[marking][4] & 0xFF);
			int fired = ((lpSolutions[marking][5] & 0xFF) << 8);
			fired |= (lpSolutions[marking][6] & 0xFF);
			return getLpSolution(pre, transition) - (fired == transition ? 1 : 0);
		}

	}

	protected synchronized boolean isDerivedLpSolution(int marking) {
		return lpSolutions[marking] != null && lpSolutions[marking][0] != COMPUTED;
	}

	//	public byte[] getLpSolution(int marking) {
	//		return lpSolutions[marking];
	//	}
	//

	protected synchronized void setDerivedLpSolution(int from, int to, short transition) {
		assert lpSolutions[to] == null;
		if (numRows > 48) {
			// only use 7 bytes if this indeed saves memory, i.e. if there is more than
			// 48 transitions. 48 transitions would correspond to 6 bytes, with 1 flag byte at the beginning.
			lpSolutions[to] = new byte[7];
			lpSolutions[to][0] = DERIVED;
			lpSolutions[to][1] = (byte) (from >>> 24);
			lpSolutions[to][2] = (byte) (from >>> 16);
			lpSolutions[to][3] = (byte) (from >>> 8);
			lpSolutions[to][4] = (byte) (from);
			lpSolutions[to][5] = (byte) (transition >>> 8);
			lpSolutions[to][6] = (byte) (transition);
		} else {
			lpSolutions[to] = Arrays.copyOf(lpSolutions[from], lpSolutions[from].length);
			lpSolutions[to][0] = DERIVEDBUTSTOREDFULL;
			lpSolutions[to][transition + 1] = (byte) ((lpSolutions[from][transition + 1] & 0xFF) - 1);
		}
		lpSolutionsSize += 4 + lpSolutions[to].length;
	}

	protected synchronized void setNewLpSolution(int marking, double[] solution) {
		assert lpSolutions[marking] == null;
		lpSolutions[marking] = new byte[solution.length + 1];
		lpSolutions[marking][0] = COMPUTED;
		for (int i = solution.length; i-- > 0;) {
			// round down into byte, but allow for precision up to 1E-9
			lpSolutions[marking][i + 1] = (byte) ((int) (solution[i] + 1E-9));
		}
		lpSolutionsSize += 4 + lpSolutions[marking].length;
	}

	/**
	 * In ILP version, only one given final marking is the target.
	 */
	protected boolean isFinal(int marking) {
		return equalMarking(marking, net.getFinalMarking());
	}

	//
	//	/**
	//	 * In ILP version, only one given final marking is the target.
	//	 */
	//	protected boolean isFinal(int block, int index) {
	//		return equalMarking(block, index, net.getFinalMarking());
	//	}

	protected void deriveOrEstimateHValue(int from, int fromBlock, int fromIndex, short transition, int to,
			int toBlock, int toIndex) {
		if (hasExactHeuristic(fromBlock, fromIndex) && (getLpSolution(from, transition) >= 1)) {
			// from Marking has exact heuristic
			// we can derive an exact heuristic from it
			if (to >= lpSolutions.length) {
				growArray(to);
			}

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

	protected void terminateRun() {
		super.terminateRun();
		threadpool.shutdown();
		while (!threadpool.isTerminated()) {
			try {
				threadpool.awaitTermination(100, TimeUnit.MILLISECONDS);
			} catch (InterruptedException e) {
			}
		}
		provider.deleteLps();
	}

}