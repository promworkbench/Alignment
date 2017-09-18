package nl.tue.alignment;

import gnu.trove.map.TObjectIntMap;

import java.util.Arrays;

import lpsolve.LpSolve;
import nl.tue.alignment.ReplayAlgorithm.Debug;
import nl.tue.alignment.Utils.Statistic;
import nl.tue.alignment.algorithms.AStar;
import nl.tue.alignment.algorithms.BackTrackingAStar;
import nl.tue.alignment.algorithms.Dijkstra;
import nl.tue.astar.util.ilp.LPMatrixException;

public class Test {

	private static char SEP = '\t';

	public static class SyncProductExampleBook extends SyncProductImpl {

		public SyncProductExampleBook() {
			super("Book Example", new String[] { "As,-", "Aa,-", "Fa,-", "Sso,-", "Ro,-", "Co,-", "t,-", "Da1,-",
					"Do,-", "Da2,-", "Ao,-", "Aaa,-", "As,As", "Aa,Aa", "Sso,Sso", "Ro,Ro", "Ao,Ao", "Aaa,Aaa1",
					"Aaa,Aaa2", "-,As", "-,Aa", "-,Sso", "-,Ro", "-,Ao", "-,Aaa1", "-,Aaa2" }, //
					new String[] { "p0", "p1", "p2", "p3", "p4", "p5", "p6", "p7", "p8", "p9", "p10", "p11", "p12",
							"p13", "p14", "p15", "p16", "p17", "p18" }, //
					new int[] { 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1 }//
			);
			addToInput(0, 0);
			addToInput(1, 1);
			addToInput(2, 2);
			addToInput(3, 3);
			addToInput(4, 4);
			addToInput(5, 6);
			addToInput(6, 5, 6);
			addToInput(7, 1);
			addToInput(8, 7);
			addToInput(9, 8);
			addToInput(10, 7);
			addToInput(11, 9);
			//SyncMoves
			addToInput(12, 0, 11);
			addToInput(13, 1, 12);
			addToInput(14, 3, 13);
			addToInput(15, 4, 14);
			addToInput(16, 7, 15);
			addToInput(17, 9, 16);
			addToInput(18, 9, 17);
			//LogMoves
			addToInput(19, 11);
			addToInput(20, 12);
			addToInput(21, 13);
			addToInput(22, 14);
			addToInput(23, 15);
			addToInput(24, 16);
			addToInput(25, 17);

			addToOutput(0, 1);
			addToOutput(1, 2, 3);
			addToOutput(2, 5);
			addToOutput(3, 4);
			addToOutput(4, 6);
			addToOutput(5, 3);
			addToOutput(6, 7);
			addToOutput(7, 10);
			addToOutput(8, 8);
			addToOutput(9, 10);
			addToOutput(10, 9);
			addToOutput(11, 10);
			addToOutput(12, 1, 12);
			addToOutput(13, 2, 3, 13);
			addToOutput(14, 4, 14);
			addToOutput(15, 6, 15);
			addToOutput(16, 9, 16);
			addToOutput(17, 10, 17);
			addToOutput(18, 10, 18);
			addToOutput(19, 12);
			addToOutput(20, 13);
			addToOutput(21, 14);
			addToOutput(22, 15);
			addToOutput(23, 16);
			addToOutput(24, 17);
			addToOutput(25, 18);

			setInitialMarking(0, 11);
			setFinalMarking(10, 18);
		}

		public boolean isFinalMarking(byte[] marking) {
			// for full alignments:
			return Arrays.equals(marking, finalMarking);

			// for prefix alignments:
			// check only if place 18 marked with a single token
			//			return (marking[18 / 8] & (Utils.FLAG >>> (18 % 8))) != 0
			//					&& (marking[bm + 18 / 8] & (Utils.FLAG >>> (18 % 8))) == 0;
		}

	}

	public static class NastySyncProductExample extends SyncProductImpl {

		public NastySyncProductExample() {
			super("Nasty Example", new String[] { "A,-", "B,-", "C,-", "D,-", "E,-", "F,-", "G,-", "H,-", "I,-", "J,-",
					"K,-", "L,-", "K,K", "L,L", "-,L", "-,K" }, //
					new String[] { "p0", "p1", "p2", "p3", "p4", "p5", "p6", "p7", "p8", "p9", "p10", "p11", "p12",
							"p13", "p14" }, //
					new int[] { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1 }//
			);
			addToInput(0, 0);
			addToOutput(0, 1, 2);

			addToInput(1, 1);
			addToOutput(1, 3);

			addToInput(2, 3);
			addToOutput(2, 7);

			addToInput(3, 1);
			addToOutput(3, 4);

			addToInput(4, 4);
			addToOutput(4, 7);

			addToInput(5, 2);
			addToOutput(5, 5);

			addToInput(6, 5);
			addToOutput(6, 8);

			addToInput(7, 2);
			addToOutput(7, 6);

			addToInput(8, 6);
			addToOutput(8, 8);

			addToInput(9, 7, 8);
			addToOutput(9, 9);

			addToInput(10, 9);
			addToOutput(10, 10);

			addToInput(11, 10);
			addToOutput(11, 11);

			addToInput(12, 9, 13);
			addToOutput(12, 10, 14);

			addToInput(13, 10, 12);
			addToOutput(13, 11, 13);

			addToInput(14, 12);
			addToOutput(14, 13);

			addToInput(15, 13);
			addToOutput(15, 14);

			setInitialMarking(0, 12);
			setFinalMarking(11, 14);
		}
	}

	public static class SmallNastySyncProductExample extends SyncProductImpl {

		public SmallNastySyncProductExample() {
			super("Small Nasty Example", new String[] { "A", "B", "C1", "D1", "C2", "D2", "C3", "D3" }, //
					new String[] { "p0", "p1", "p2", "p3", "p4", "p5", "p6", "p7", }, //
					new int[] { 1, 1, 1, 1, 1, 1, 0, 0 }//
			);
			addToInput(0, 0);
			addToOutput(0, 1);

			addToInput(1, 1);
			addToOutput(1, 2);

			addToInput(2, 2);
			addToOutput(2, 3);

			addToInput(3, 4);
			addToOutput(3, 5);

			addToInput(4, 5);
			addToOutput(4, 6);

			addToInput(5, 6);
			addToOutput(5, 7);

			addToInput(6, 2, 5);
			addToOutput(6, 3, 6);

			addToInput(7, 1, 6);
			addToOutput(7, 2, 7);

			setInitialMarking(0, 4);
			setFinalMarking(3, 7);
		}
	}

	public static void main(String[] args) throws LPMatrixException {

		//		doExperiment(new SyncProductExampleBook());
		testSingleGraph(new NastySyncProductExample(), Debug.DOT);
	}

	public static void doExperiment(SyncProduct net) throws LPMatrixException {

		ReplayAlgorithm algorithm;
		// initialize LpSolve
		LpSolve.lpSolveVersion();
		// initialize relevant classloader
		algorithm = new AStar(net, //
				false, // moveSort on total order
				false, // queue sorted "depth-first"
				false, // prefer Exact solution
				false, //  use Integer
				Debug.NORMAL // debug mode
		);

		System.out.print("Type" + SEP + "Move Sorting" + SEP + "Queue Sorting" + SEP + "Prefer Exact" + SEP
				+ "use Int for LP" + SEP);
		System.out.println(toString(Statistic.values()));
		for (boolean dijkstra : new boolean[] { false }) {
			for (boolean moveSort : new boolean[] { false, true }) {
				for (boolean queueSort : new boolean[] { true, false }) {
					if (!dijkstra) {
						for (boolean preferExact : new boolean[] { true }) {
							for (boolean useInt : new boolean[] { false, true }) {
								System.out.print((dijkstra ? "Dijkstra" : "A star  "));
								System.out.print(SEP);
								System.out.print(moveSort);
								System.out.print(SEP);
								System.out.print(queueSort);
								System.out.print(SEP);
								System.out.print(preferExact);
								System.out.print(SEP);
								System.out.print(useInt);
								System.out.print(SEP);
								algorithm = new AStar(net, //
										moveSort, // moveSort on total order
										queueSort, // queue sorted "depth-first"
										preferExact, // prefer Exact solution
										useInt, //  use Integer
										Debug.NONE // debug mode
								);
								short[] alignment = algorithm.run();
								TObjectIntMap<Statistic> map = algorithm.getStatistics();
								for (Statistic s : Statistic.values()) {
									System.out.print(map.get(s));
									System.out.print(SEP);
								}
								System.out.println();
							}
						}
					} else {
						System.out.print((dijkstra ? "Dijkstra" : "A star"));
						System.out.print(SEP);
						System.out.print(moveSort);
						System.out.print(SEP);
						System.out.print(queueSort);
						System.out.print(SEP);
						System.out.print("-");
						System.out.print(SEP);
						System.out.print("-");
						System.out.print(SEP);
						algorithm = new Dijkstra(net, //
								moveSort, // moveSort on total order
								queueSort, // queue sorted "depth-first"
								Debug.NONE // debug mode
						);
						short[] alignment = algorithm.run();
						TObjectIntMap<Statistic> map = algorithm.getStatistics();
						for (Statistic s : Statistic.values()) {
							System.out.print(map.get(s));
							System.out.print(SEP);
						}
						System.out.println();

					}
				}
			}
		}
	}

	public static void testSingleGraph(SyncProduct net, Debug debug) throws LPMatrixException {

		ReplayAlgorithm algorithm;
		//INITIALIZATION OF CLASSLOADER FOR PROPER RECORDING OF TIMES.
		algorithm = new Dijkstra(net, true, true, Debug.NONE);
		algorithm = new AStar(net, true, true, true, true, Debug.NONE);

		boolean dijkstra = false;
		boolean moveSort = true; // moveSort on total order
		boolean queueSort = true; // queue sorted "depth-first"
		boolean preferExact = true; // prefer Exact solution
		boolean useInt = true; //  use Integer

		if (dijkstra) {
			algorithm = new Dijkstra(net, //
					moveSort, // moveSort on total order
					queueSort, // queue sorted "depth-first"
					debug //
			);
		} else {

			algorithm = new BackTrackingAStar(net, //
					useInt,// use Integers
					debug // debug mode
			);
		}

		short[] alignment = algorithm.run();

		//		for (short t : alignment) {
		//			System.out.println(net.getTransitionLabel(t));
		//		}
	}

	private static String toString(Object[] a) {
		if (a == null)
			return "null";

		int iMax = a.length - 1;
		if (iMax == -1)
			return "";

		StringBuilder b = new StringBuilder();
		for (int i = 0;; i++) {
			b.append(String.valueOf(a[i]));
			if (i == iMax)
				return b.toString();
			b.append(SEP);
		}
	}
}
