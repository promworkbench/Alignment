package nl.tue.alignment;

import gnu.trove.map.TObjectIntMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import nl.tue.alignment.Utils.Statistic;
import nl.tue.alignment.algorithms.AStar;
import nl.tue.alignment.algorithms.AStarLargeLP;
import nl.tue.alignment.algorithms.Dijkstra;
import nl.tue.alignment.algorithms.ReplayAlgorithm;
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

	private final ReplayerParameters parameters;
	private final XLog log;
	private final Map<XEventClass, Integer> costLM;
	private final SyncProductFactory factory;
	private final XEventClasses classes;

	public Replayer(Petrinet net, Marking initialMarking, Marking finalMarking, XLog log,
			Map<Transition, Integer> costMOS, Map<XEventClass, Integer> costMOT, TransEvClassMapping mapping) {
		this(new ReplayerParameters(), net, initialMarking, finalMarking, log, null, costMOS, costMOT, null, mapping);
	}

	public Replayer(Petrinet net, Marking initialMarking, Marking finalMarking, XLog log, TransEvClassMapping mapping) {
		this(new ReplayerParameters(), net, initialMarking, finalMarking, log, null, null, null, null, mapping);
	}

	public Replayer(ReplayerParameters parameters, Petrinet net, Marking initialMarking, Marking finalMarking,
			XLog log, XEventClasses classes, Map<Transition, Integer> costMM, Map<XEventClass, Integer> costLM,
			Map<Transition, Integer> costSM, TransEvClassMapping mapping) {
		this.parameters = parameters;
		this.log = log;
		if (classes == null) {
			XEventClassifier eventClassifier = XLogInfoImpl.STANDARD_CLASSIFIER;
			XLogInfo summary = XLogInfoFactory.createLogInfo(log, eventClassifier);
			this.classes = summary.getEventClasses();
		} else {
			this.classes = classes;
		}
		this.costLM = costLM;
		factory = new SyncProductFactory(net, classes, mapping, costMM, costLM, costSM, initialMarking, finalMarking);
	}

	public PNRepResult computePNRepResult() throws LPMatrixException {

		//TODO: Detect previously computed cases as duplicates when the traces are equal as sequences of classifiers.

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
			SyncReplayResult srr = getSyncReplayResultForTrace(trace, t);

			if (srr != null) {
				int traceCost = getTraceCost(trace);
				srr.addInfo(PNRepResult.TRACEFITNESS,
						1 - (srr.getInfo().get(PNRepResult.RAWFITNESSCOST) / (maxModelMoveCost + traceCost)));
				result.add(srr);
			}
			t++;

		}
		return new PNRepResultImpl(result);
	}

	public SyncReplayResult getSyncReplayResultForTrace(XTrace trace, int traceIndex) throws LPMatrixException {
		SyncProduct product = factory.getSyncProduct(trace);
		if (product != null) {
			ReplayAlgorithm algorithm = getAlgorithm(product);
			short[] alignment = algorithm.run();
			TObjectIntMap<Statistic> stats = algorithm.getStatistics();
			SyncReplayResult srr = Utils.toSyncReplayResult(factory, stats, alignment, trace, traceIndex);
			return srr;
		}
		return null;
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

	private int getTraceCost(XTrace trace) {
		int cost = 0;
		for (XEvent e : trace) {
			cost += costLM.containsKey(classes.getClassOf(e)) ? costLM.get(classes.getClassOf(e)) : 1;
		}
		return cost;
	}

}
