package nl.tue.alignment;

import gnu.trove.map.TObjectIntMap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import nl.tue.alignment.Utils.Statistic;
import nl.tue.alignment.algorithms.AStar;
import nl.tue.alignment.algorithms.AStarLargeLP;
import nl.tue.alignment.algorithms.Dijkstra;
import nl.tue.alignment.algorithms.ReplayAlgorithm;
import nl.tue.alignment.algorithms.ReplayAlgorithm.Debug;
import nl.tue.alignment.algorithms.datastructures.SyncProduct;
import nl.tue.alignment.algorithms.datastructures.SyncProductFactory;
import nl.tue.astar.util.ilp.LPMatrixException;

import org.deckfour.xes.classification.XEventClass;
import org.deckfour.xes.classification.XEventClasses;
import org.deckfour.xes.classification.XEventClassifier;
import org.deckfour.xes.info.XLogInfo;
import org.deckfour.xes.info.XLogInfoFactory;
import org.deckfour.xes.info.impl.XLogInfoImpl;
import org.deckfour.xes.model.XEvent;
import org.deckfour.xes.model.XLog;
import org.deckfour.xes.model.XTrace;
import org.processmining.models.graphbased.directed.petrinet.Petrinet;
import org.processmining.models.graphbased.directed.petrinet.elements.Transition;
import org.processmining.models.semantics.petrinet.Marking;
import org.processmining.plugins.connectionfactories.logpetrinet.TransEvClassMapping;
import org.processmining.plugins.petrinet.replayresult.PNRepResult;
import org.processmining.plugins.petrinet.replayresult.PNRepResultImpl;
import org.processmining.plugins.replayer.replayresult.SyncReplayResult;

public class Replayer {

	public static enum Algorithm {
		DIJKSTRA, ASTAR, ASTARWITHMARKINGSPLIT;
	}

	public static class Parameters {
		final Algorithm algorithm; // which algorithm
		final boolean moveSort; // moveSort on total order
		final boolean queueSort; // queue sorted "depth-first"
		final boolean preferExact; // prefer Exact solution
		final boolean multiThread; // do multithreading
		final boolean useInt; //  use Integer
		final Debug debug;

		public Parameters(Algorithm algorithm, boolean moveSort, boolean queueSort, boolean preferExact,
				boolean multiThread, boolean useInt, Debug debug) {
			this.algorithm = algorithm;
			this.moveSort = moveSort;
			this.queueSort = queueSort;
			this.preferExact = preferExact;
			this.multiThread = multiThread;
			this.useInt = useInt;
			this.debug = debug;
		}

		public Parameters() {
			this(Algorithm.ASTARWITHMARKINGSPLIT, true, true, true, false, false, Debug.NONE);
		}
	}

	private final Petrinet net;
	private final Marking initialMarking;
	private final Marking finalMarking;
	private final Parameters parameters;

	public Replayer(Petrinet net, Marking initialMarking, Marking finalMarking) {
		this(new Parameters(), net, initialMarking, finalMarking);
	}

	public Replayer(Parameters parameters, Petrinet net, Marking initialMarking, Marking finalMarkings) {
		this.parameters = parameters;
		this.net = net;
		this.initialMarking = initialMarking;
		this.finalMarking = finalMarkings;
	}

	public PNRepResult doAlignmentComputations(XLog log, Map<Transition, Integer> costMOS,
			Map<XEventClass, Integer> costMOT, TransEvClassMapping mapping) throws LPMatrixException {

		//TODO: Detect previously computed cases as duplicates when the traces are equal as sequences of classifiers.

		XEventClassifier eventClassifier = XLogInfoImpl.STANDARD_CLASSIFIER;
		XLogInfo summary = XLogInfoFactory.createLogInfo(log, eventClassifier);
		XEventClasses classes = summary.getEventClasses();

		SyncProductFactory factory = new SyncProductFactory(net, classes, mapping, costMOS, costMOT,
				new HashMap<Transition, Integer>(1), initialMarking, finalMarking);

		List<SyncReplayResult> result = new ArrayList<>();
		SyncProduct product = factory.getSyncProduct();

		int maxModelMoveCost = 0;
		if (product != null) {
			ReplayAlgorithm algorithm = getAlgorithm(product);
			algorithm.run();
			TObjectIntMap<Statistic> stats = algorithm.getStatistics();
			maxModelMoveCost = stats.get(Statistic.COST);
		}

		int t = 0;
		for (XTrace trace : log) {
			product = factory.getSyncProduct(trace);

			if (product != null) {

				ReplayAlgorithm algorithm = getAlgorithm(product);

				short[] alignment = algorithm.run();
				TObjectIntMap<Statistic> stats = algorithm.getStatistics();
				int traceCost = getTraceCost(trace, classes, costMOT);
				SyncReplayResult srr = Utils.toSyncReplayResult(factory, stats, alignment, trace, t);
				srr.addInfo(PNRepResult.TRACEFITNESS,
						1 - (srr.getInfo().get(PNRepResult.RAWFITNESSCOST) / (maxModelMoveCost + traceCost)));
				result.add(srr);
			}
			t++;

		}
		return new PNRepResultImpl(result);
	}

	private ReplayAlgorithm getAlgorithm(SyncProduct product) throws LPMatrixException {
		switch (parameters.algorithm) {
			case ASTAR :
				return new AStar(product, parameters.moveSort, parameters.queueSort, parameters.preferExact,//
						parameters.useInt, parameters.multiThread, parameters.debug);
			case ASTARWITHMARKINGSPLIT :
				return new AStarLargeLP(product, parameters.moveSort, parameters.useInt, parameters.debug);
			case DIJKSTRA :
				return new Dijkstra(product, parameters.moveSort, parameters.queueSort, parameters.debug);
		}
		assert false;
		return null;
	}

	private static int getTraceCost(XTrace trace, XEventClasses classes, Map<XEventClass, Integer> costMOT) {
		int cost = 0;
		for (XEvent e : trace) {
			cost += costMOT.get(classes.getClassOf(e));
		}
		return cost;
	}

}
