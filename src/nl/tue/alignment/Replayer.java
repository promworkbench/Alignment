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
import org.processmining.plugins.petrinet.replayresult.StepTypes;
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
		this(new ReplayerParameters.Default(), net, initialMarking, finalMarking, log, null, costMOS, costMOT, null,
				mapping);
	}

	public Replayer(Petrinet net, Marking initialMarking, Marking finalMarking, XLog log, TransEvClassMapping mapping) {
		this(new ReplayerParameters.Default(), net, initialMarking, finalMarking, log, null, null, null, null, mapping);
	}

	public Replayer(ReplayerParameters parameters, Petrinet net, Marking initialMarking, Marking finalMarking,
			XLog log, Map<Transition, Integer> costMOS, Map<XEventClass, Integer> costMOT, TransEvClassMapping mapping) {
		this(parameters, net, initialMarking, finalMarking, log, null, costMOS, costMOT, null, mapping);
	}

	public Replayer(ReplayerParameters parameters, Petrinet net, Marking initialMarking, Marking finalMarking,
			XLog log, TransEvClassMapping mapping) {
		this(parameters, net, initialMarking, finalMarking, log, null, null, null, null, mapping);
	}

	public Replayer(ReplayerParameters parameters, Petrinet net, Marking initialMarking, Marking finalMarking,
			XLog log, XEventClasses classes, Map<Transition, Integer> costMM, Map<XEventClass, Integer> costLM,
			Map<Transition, Integer> costSM, TransEvClassMapping mapping) {
		this.parameters = parameters;
		this.log = log;
		if (classes == null) {
			XEventClassifier eventClassifier = XLogInfoImpl.STANDARD_CLASSIFIER;
			XLogInfo summary = XLogInfoFactory.createLogInfo(log, eventClassifier);
			classes = summary.getEventClasses();
		}
		this.classes = classes;
		this.costLM = costLM;
		factory = new SyncProductFactory(net, classes, mapping, costMM, costLM, costSM, initialMarking, finalMarking);
	}

	public PNRepResult computePNRepResult() throws LPMatrixException {

		//TODO: Detect previously computed cases as duplicates when the traces are equal as sequences of classifiers.

		if (parameters.debug == Debug.STATS) {
			parameters.debug.print(Debug.STATS, "SP:name");
			parameters.debug.print(Debug.STATS, ",SP:transitions");
			parameters.debug.print(Debug.STATS, ",SP:places");
			for (Statistic s : Statistic.values()) {
				parameters.debug.print(Debug.STATS, ",ST:" + s);
			}
			parameters.debug.print(Debug.STATS, ",A:length");
			parameters.debug.print(Debug.STATS, ",A:LMcost");
			parameters.debug.print(Debug.STATS, ",A:MMcost");
			parameters.debug.print(Debug.STATS, ",A:SMcost");
			parameters.debug.println(Debug.STATS);

		}

		List<SyncReplayResult> result = new ArrayList<>();
		List<Transition> transitionList = new ArrayList<>();
		SyncProduct product = factory.getSyncProductForEmptyTrace(transitionList);

		int maxModelMoveCost = 0;
		if (product != null) {
			ReplayAlgorithm algorithm = getAlgorithm(product);
			algorithm.run();
			TObjectIntMap<Statistic> stats = algorithm.getStatistics();
			maxModelMoveCost = stats.get(Statistic.COST);
		}

		int t = 0;
		for (XTrace trace : log) {
			SyncReplayResult srr = getSyncReplayResultForTrace(trace, t, transitionList);

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

	public SyncReplayResult getSyncReplayResultForTrace(XTrace trace, int traceIndex, List<Transition> transitionList)
			throws LPMatrixException {
		SyncProduct product = factory.getSyncProduct(trace, transitionList);
		if (product != null) {
			ReplayAlgorithm algorithm = getAlgorithm(product);
			short[] alignment = algorithm.run();
			TObjectIntMap<Statistic> stats = algorithm.getStatistics();
			SyncReplayResult srr = toSyncReplayResult(product, stats, alignment, trace, traceIndex, transitionList);
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
			cost += costLM != null && costLM.containsKey(classes.getClassOf(e)) ? costLM.get(classes.getClassOf(e)) : 1;
		}
		return cost;
	}

	private SyncReplayResult toSyncReplayResult(SyncProduct product, TObjectIntMap<Statistic> statistics,
			short[] alignment, XTrace trace, int traceIndex, List<Transition> transitionList) {
		List<Object> nodeInstance = new ArrayList<>(alignment.length);
		List<StepTypes> stepTypes = new ArrayList<>(alignment.length);
		int mm = 0, lm = 0, sm = 0;
		for (int i = 0; i < alignment.length; i++) {
			short t = alignment[i];
			if (product.getTypeOf(t) == SyncProduct.LOG_MOVE) {
				nodeInstance.add(classes.getClassOf(trace.get(product.getEventOf(t))));
				stepTypes.add(StepTypes.L);
				lm += product.getCost(t);
			} else {
				nodeInstance.add(transitionList.get(t));
				if (product.getTypeOf(t) == SyncProduct.MODEL_MOVE) {
					stepTypes.add(StepTypes.MREAL);
					mm += product.getCost(t);
				} else if (product.getTypeOf(t) == SyncProduct.SYNC_MOVE) {
					stepTypes.add(StepTypes.LMGOOD);
					sm += product.getCost(t);
				} else if (product.getTypeOf(t) == SyncProduct.TAU_MOVE) {
					stepTypes.add(StepTypes.MINVI);
					mm += product.getCost(t);
				}
			}
		}

		SyncReplayResult srr = new SyncReplayResult(nodeInstance, stepTypes, traceIndex);
		srr.addInfo(PNRepResult.RAWFITNESSCOST, 1.0 * statistics.get(Statistic.COST));
		srr.addInfo(PNRepResult.TIME, statistics.get(Statistic.TOTALTIME) / 1000.0);
		srr.addInfo(PNRepResult.QUEUEDSTATE, 1.0 * statistics.get(Statistic.QUEUEACTIONS));
		if (lm + sm == 0) {
			srr.addInfo(PNRepResult.MOVELOGFITNESS, 1.0);
		} else {
			srr.addInfo(PNRepResult.MOVELOGFITNESS, 1.0 - (1.0 * lm) / (lm + sm));
		}
		if (mm + sm == 0) {
			srr.addInfo(PNRepResult.MOVEMODELFITNESS, 1.0);
		} else {
			srr.addInfo(PNRepResult.MOVEMODELFITNESS, 1.0 - (1.0 * mm) / (mm + sm));
		}
		srr.addInfo(PNRepResult.NUMSTATEGENERATED, 1.0 * statistics.get(Statistic.MARKINGSREACHED));
		srr.addInfo(PNRepResult.ORIGTRACELENGTH, 1.0 * trace.size());

		srr.setReliable(statistics.get(Statistic.EXITCODE) == Utils.OPTIMALALIGNMENT);
		return srr;
	}

}
