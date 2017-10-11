package nl.tue.alignment;

import gnu.trove.iterator.TShortIterator;
import gnu.trove.list.TIntList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.set.TShortSet;
import gnu.trove.set.hash.TShortHashSet;

import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.Random;

public class Utils {

	public static final int BYTEHIGHBIT = 1 << 7;

	public static int OPTIMALALIGNMENT = 1;
	public static int FAILEDALIGNMENT = 2;
	public static int ENABLINGBLOCKEDBYOUTPUT = 4;
	public static int COSTFUNCTIONOVERFLOW = 8;
	public static int HEURISTICFUNCTIONOVERFLOW = 16;

	/**
	 * Default block size determines how many bytes are reserved top store
	 * markings. Whenever a block is full, a new block of this size is
	 * allocated.
	 */
	public static int DEFAULTBLOCKSIZE = 1024;

	/**
	 * Initial size of the priority queue. It grows as needed.
	 */
	public static int DEFAULTQUEUESIZE = 16;

	/**
	 * Initial size of the visited state set. It grows as needed.
	 */
	public static int DEFAULTVISITEDSIZE = 16;

	public enum Statistic {
		EXITCODE("Exit code for alignment"), //
		ALIGNMENTLENGTH("Length of the alignment found"), //
		COST("Cost of the alignment"), //
		EDGESTRAVERSED("Transitions fired"), //
		POLLACTIONS("Markings polled from queue"), //
		CLOSEDACTIONS("Markings added to closed set"), //
		QUEUEACTIONS("Markings queued"), //
		MARKINGSREACHED("Markings reached"), //
		HEURISTICSCOMPUTED("Heuristics computed"), //
		HEURISTICSESTIMATED("Heuristics estimated"), //
		HEURISTICSDERIVED("Heuristics derived"), //
		MAXQUEUELENGTH("Maximum queue length (elts)"), //
		MAXQUEUECAPACITY("Maximum queue capacity (elts)"), //
		VISITEDSETCAPACITY("Maximum capacity visited set (elts)"), //
		MEMORYUSED("Approximate peak memory used (kb)"), //
		RUNTIME("Time to compute alignment (us)"), //
		HEURISTICTIME("Time to compute heuristics (us)"), //
		SETUPTIME("Time to setup algorithm (us)"), //
		TOTALTIME("Total Time including setup (us)"), //
		SPLITS("Number of splits when splitting marking");
		private final String label;

		private Statistic(String label) {
			this.label = label;
		}

		public String toString() {
			return label;
		}
	}

	public static String asVector(byte[] marking, SyncProduct net) {
		StringBuffer buf = new StringBuffer();
		buf.append('[');
		int len = marking.length / 2;
		for (short i = 0; i < net.numPlaces();) {
			buf.append(marking[i]);
			if (++i < net.numPlaces()) {
				buf.append(',');
			}
		}
		buf.append(']');
		return buf.toString();
	}

	public static String asBag(byte[] marking, SyncProduct net) {
		StringBuffer buf = new StringBuffer();
		buf.append('[');
		for (short i = 0; i < net.numPlaces();) {
			if (marking[i] > 0) {
				if (buf.length() > 1) {
					buf.append(',');
				}
				if (marking[i] > 1) {
					buf.append(marking[i]);
				}
				buf.append(net.getPlaceLabel(i));
			}
			i++;
		}
		buf.append(']');
		return buf.toString();
	}

	// Implementing Fisher Yates shuffle
	public static void shuffleArray(int[] ar, Random rnd) {
		for (int i = ar.length - 1; i > 0; i--) {
			int index = rnd.nextInt(i + 1);
			// Simple swap
			int a = ar[index];
			ar[index] = ar[i];
			ar[i] = a;
		}
	}

	// Implementing Fisher Yates shuffle
	public static void shuffleArray(short[] ar, Random rnd) {
		for (int i = ar.length - 1; i > 0; i--) {
			int index = rnd.nextInt(i + 1);
			// Simple swap
			short a = ar[index];
			ar[index] = ar[i];
			ar[i] = a;
		}
	}

	private static int getCostForType(SyncProduct product, short[] alignment, byte type1, byte type2) {
		int cost = 0;
		for (int i = 0; i < alignment.length; i++) {
			if (product.getTypeOf(alignment[i]) == type1 || product.getTypeOf(alignment[i]) == type2) {
				cost += product.getCost(alignment[i]);
			}
		}
		return cost;
	}

	public static int logMoveCost(SyncProduct product, short[] alignment) {
		return getCostForType(product, alignment, SyncProduct.LOG_MOVE, SyncProduct.LOG_MOVE);
	}

	public static int modelMoveCost(SyncProduct product, short[] alignment) {
		return getCostForType(product, alignment, SyncProduct.MODEL_MOVE, SyncProduct.TAU_MOVE);
	}

	public static int syncMoveCost(SyncProduct product, short[] alignment) {
		return getCostForType(product, alignment, SyncProduct.SYNC_MOVE, SyncProduct.SYNC_MOVE);
	}

	public static void toTpn(SyncProduct product, OutputStreamWriter stream) throws IOException {
		for (short p = 0; p < product.numPlaces(); p++) {
			stream.write("place \"place_" + p);
			stream.write("\"");
			if (product.getInitialMarking()[p] > 0) {
				stream.write("init " + product.getInitialMarking()[p]);
			}
			stream.write(";\n");
		}
		for (short t = 0; t < product.numTransitions(); t++) {
			stream.write("trans \"t_" + t);
			stream.write("\"~\"");
			stream.write(product.getTransitionLabel(t));
			stream.write("\" in ");
			for (short p : product.getInput(t)) {
				stream.write(" \"place_" + p);
				stream.write("\"");
			}
			stream.write(" out ");
			for (short p : product.getOutput(t)) {
				stream.write(" \"place_" + p);
				stream.write("\"");
			}
			stream.write(";\n");
		}
		stream.flush();

	}

	public static void toDot(SyncProduct product, OutputStreamWriter stream) throws IOException {
		stream.write("Digraph P { \n rankdir=LR;\n");

		for (short p = 0; p < product.numPlaces(); p++) {
			placeToDot(product, stream, p, p);
		}
		stream.write("{ rank=same;");
		for (short p = 0; p < product.numPlaces(); p++) {
			if (product.getInitialMarking()[p] > 0) {
				stream.write("p" + p + "; ");
			}
		}
		stream.write("}\n");
		stream.write("{ rank=same;");
		for (short p = 0; p < product.numPlaces(); p++) {
			if (product.getFinalMarking()[p] > 0) {
				stream.write("p" + p + "; ");
			}
		}
		stream.write("}\n");

		TShortSet events = new TShortHashSet(product.numTransitions(), 0.5f, (short) -2);
		for (short t = 0; t < product.numTransitions(); t++) {
			events.add(product.getEventOf(t));
			transitionToDot(product, stream, t, t);

			stream.write("\n");
			for (short p : product.getInput(t)) {
				stream.write("p" + p + " -> t" + t);
				if (product.getTypeOf(t) == SyncProduct.SYNC_MOVE) {
					stream.write(" [weight=2]");
				} else {
					stream.write(" [weight=10]");
				}
				stream.write(";\n");
			}
			for (short p : product.getOutput(t)) {
				stream.write("t" + t + " -> p" + p);
				if (product.getTypeOf(t) == SyncProduct.SYNC_MOVE) {
					stream.write(" [weight=2]");
				} else {
					stream.write(" [weight=10]");
				}
				stream.write(";\n");
			}
		}

		events.remove(SyncProduct.NOEVENT);

		short e;
		for (TShortIterator it = events.iterator(); it.hasNext();) {
			e = it.next();
			stream.write("{ rank=same;");
			for (short t = 0; t < product.numTransitions(); t++) {
				if (product.getEventOf(t) == e) {
					stream.write("t" + t + "; ");
				}
			}
			stream.write("}\n");

		}

		stream.write("}");
		stream.flush();

	}

	public static void toDot(SyncProduct product, short[] alignment, OutputStreamWriter stream) throws IOException {
		stream.write("Digraph A { \n rankdir=LR;\n");

		TIntList[] place2index = new TIntList[product.numPlaces()];
		for (short p = 0; p < product.numPlaces(); p++) {
			place2index[p] = new TIntArrayList(3);
			if (product.getInitialMarking()[p] > 0) {
				place2index[p].add(0);
			}
		}

		for (int i = 0; i < alignment.length; i++) {
			short t = alignment[i];
			transitionToDot(product, stream, i, t);

			for (short p : product.getInput(t)) {
				int j = place2index[p].removeAt(place2index[p].size() - 1);
				if (j == 0) {
					placeToDot(product, stream, (j * product.numPlaces() + p), p);
				}
				stream.write("p" + (j * product.numPlaces() + p) + " -> t" + i);
				if (product.getTypeOf(t) == SyncProduct.SYNC_MOVE) {
					stream.write(" [weight=2]");
				} else {
					stream.write(" [weight=10]");
				}
				stream.write(";\n");
			}
			for (short p : product.getOutput(t)) {
				place2index[p].add(i);
				placeToDot(product, stream, i * product.numPlaces() + p, p);
				stream.write("t" + i + " -> p" + (i * product.numPlaces() + p));
				if (product.getTypeOf(t) == SyncProduct.SYNC_MOVE) {
					stream.write(" [weight=2]");
				} else {
					stream.write(" [weight=10]");
				}
				stream.write(";\n");
			}
		}

		stream.write("}");
		stream.flush();

	}

	protected static void placeToDot(SyncProduct product, OutputStreamWriter stream, int i, short p) throws IOException {
		stream.write("p" + i);
		stream.write(" [label=<" + product.getPlaceLabel(p));
		if (product.getInitialMarking()[p] > 0) {
			stream.write("<br/>i:" + product.getInitialMarking()[p]);
		}
		if (product.getFinalMarking()[p] > 0) {
			stream.write("<br/>f:" + product.getFinalMarking()[p]);
		}
		stream.write(">,shape=circle];");
		stream.write("\n");
	}

	protected static void transitionToDot(SyncProduct product, OutputStreamWriter stream, int i, short t)
			throws IOException {
		stream.write("t" + i);
		stream.write(" [label=<" + product.getTransitionLabel(t));
		stream.write("<br/>" + product.getCost(t));
		stream.write(">");

		if (product.getTypeOf(t) == SyncProduct.LOG_MOVE) {
			stream.write(",style=filled,fillcolor=goldenrod2,fontcolor=black");
		} else if (product.getTypeOf(t) == SyncProduct.MODEL_MOVE) {
			stream.write(",style=filled,fillcolor=darkorchid1,fontcolor=white");
		} else if (product.getTypeOf(t) == SyncProduct.SYNC_MOVE) {
			stream.write(",style=filled,fillcolor=forestgreen,fontcolor=white");
		} else if (product.getTypeOf(t) == SyncProduct.TAU_MOVE) {
			stream.write(",style=filled,fillcolor=honeydew4,fontcolor=white");
		}

		stream.write(",shape=box];");
		stream.write("\n");
	}
}
