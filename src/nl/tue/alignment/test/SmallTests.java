package nl.tue.alignment.test;

import gnu.trove.map.TObjectIntMap;

import java.util.Arrays;

import lpsolve.LpSolve;
import nl.tue.alignment.ReplayAlgorithm;
import nl.tue.alignment.ReplayAlgorithm.Debug;
import nl.tue.alignment.SyncProduct;
import nl.tue.alignment.SyncProductImpl;
import nl.tue.alignment.Utils.Statistic;
import nl.tue.alignment.algorithms.AStar;
import nl.tue.alignment.algorithms.Dijkstra;
import nl.tue.astar.util.ilp.LPMatrixException;

public class SmallTests {

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
			setInput(0, 0);
			setInput(1, 1);
			setInput(2, 2);
			setInput(3, 3);
			setInput(4, 4);
			setInput(5, 6);
			setInput(6, 5, 6);
			setInput(7, 1);
			setInput(8, 7);
			setInput(9, 8);
			setInput(10, 7);
			setInput(11, 9);
			//SyncMoves
			setInput(12, 0, 11);
			setInput(13, 1, 12);
			setInput(14, 3, 13);
			setInput(15, 4, 14);
			setInput(16, 7, 15);
			setInput(17, 9, 16);
			setInput(18, 9, 17);
			//LogMoves
			setInput(19, 11);
			setInput(20, 12);
			setInput(21, 13);
			setInput(22, 14);
			setInput(23, 15);
			setInput(24, 16);
			setInput(25, 17);

			setOutput(0, 1);
			setOutput(1, 2, 3);
			setOutput(2, 5);
			setOutput(3, 4);
			setOutput(4, 6);
			setOutput(5, 3);
			setOutput(6, 7);
			setOutput(7, 10);
			setOutput(8, 8);
			setOutput(9, 10);
			setOutput(10, 9);
			setOutput(11, 10);
			setOutput(12, 1, 12);
			setOutput(13, 2, 3, 13);
			setOutput(14, 4, 14);
			setOutput(15, 6, 15);
			setOutput(16, 9, 16);
			setOutput(17, 10, 17);
			setOutput(18, 10, 18);
			setOutput(19, 12);
			setOutput(20, 13);
			setOutput(21, 14);
			setOutput(22, 15);
			setOutput(23, 16);
			setOutput(24, 17);
			setOutput(25, 18);

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
			setInput(0, 0);
			setOutput(0, 1, 2);

			setInput(1, 1);
			setOutput(1, 3);

			setInput(2, 3);
			setOutput(2, 7);

			setInput(3, 1);
			setOutput(3, 4);

			setInput(4, 4);
			setOutput(4, 7);

			setInput(5, 2);
			setOutput(5, 5);

			setInput(6, 5);
			setOutput(6, 8);

			setInput(7, 2);
			setOutput(7, 6);

			setInput(8, 6);
			setOutput(8, 8);

			setInput(9, 7, 8);
			setOutput(9, 9);

			setInput(10, 9);
			setOutput(10, 10);

			setInput(11, 10);
			setOutput(11, 11);

			setInput(12, 9, 13);
			setOutput(12, 10, 14);

			setInput(13, 10, 12);
			setOutput(13, 11, 13);

			setInput(14, 12);
			setOutput(14, 13);

			setInput(15, 13);
			setOutput(15, 14);

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
			setInput(0, 0);
			setOutput(0, 1);

			setInput(1, 1);
			setOutput(1, 2);

			setInput(2, 2);
			setOutput(2, 3);

			setInput(3, 4);
			setOutput(3, 5);

			setInput(4, 5);
			setOutput(4, 6);

			setInput(5, 6);
			setOutput(5, 7);

			setInput(6, 2, 5);
			setOutput(6, 3, 6);

			setInput(7, 1, 6);
			setOutput(7, 2, 7);

			setInitialMarking(0, 4);
			setFinalMarking(3, 7);
		}
	}

	public static void main(String[] args) throws LPMatrixException {

		//		testSingleGraph(new SyncProductExampleBook(), Debug.DOT);
		//		testSingleGraph(new SmallNastySyncProductExample(), Debug.DOT);
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
				false, // use Integer
				false, // do multithreading
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
										true, // do multithreading
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
		algorithm = new AStar(net, true, true, true, true, true, Debug.NONE);

		boolean dijkstra = false;
		boolean moveSort = true; // moveSort on total order
		boolean queueSort = true; // queue sorted "depth-first"
		boolean preferExact = true; // prefer Exact solution
		boolean multiThread = true;
		boolean useInt = false; //  use Integer

		if (dijkstra) {
			algorithm = new Dijkstra(net, //
					moveSort, // moveSort on total order
					queueSort, // queue sorted "depth-first"
					debug //
			);
		} else {

			algorithm = new AStar(net, //
					moveSort, // moveSort on total order
					queueSort, // queue sorted "depth-first"
					preferExact, // prefer Exact solution
					useInt,// use Integers
					multiThread, // multithreading
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
