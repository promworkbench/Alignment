package nl.tue.alignment;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TObjectIntHashMap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import nl.tue.alignment.ReplayerParameters.Algorithm;
import nl.tue.alignment.TraceReplayTask.TraceReplayResult;
import nl.tue.alignment.Utils.Statistic;
import nl.tue.alignment.algorithms.ReplayAlgorithm.Debug;
import nl.tue.alignment.algorithms.syncproduct.SyncProductFactory;
import nl.tue.astar.Trace;

import org.deckfour.xes.classification.XEventClass;
import org.deckfour.xes.classification.XEventClasses;
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

	public static final String MAXMODELMOVECOST = "Model move cost empty trace";

	final TObjectIntMap<Trace> trace2FirstIdenticalTrace;

	private final ReplayerParameters parameters;
	private final XLog log;
	private final Map<XEventClass, Integer> costLM;
	final SyncProductFactory factory;
	final XEventClasses classes;
	private Map<Transition, Integer> costMM;
	Progress progress;

	public Replayer(Petrinet net, Marking initialMarking, Marking finalMarking, XLog log, XEventClasses classes,
			Map<Transition, Integer> costMOS, Map<XEventClass, Integer> costMOT, TransEvClassMapping mapping) {
		this(new ReplayerParameters.Default(), net, initialMarking, finalMarking, log, classes, costMOS, costMOT, null,
				mapping);
	}

	public Replayer(Petrinet net, Marking initialMarking, Marking finalMarking, XLog log, XEventClasses classes,
			TransEvClassMapping mapping) {
		this(new ReplayerParameters.Default(), net, initialMarking, finalMarking, log, classes, null, null, null,
				mapping);
	}

	public Replayer(ReplayerParameters parameters, Petrinet net, Marking initialMarking, Marking finalMarking,
			XLog log, XEventClasses classes, Map<Transition, Integer> costMOS, Map<XEventClass, Integer> costMOT,
			TransEvClassMapping mapping) {
		this(parameters, net, initialMarking, finalMarking, log, classes, costMOS, costMOT, null, mapping);
	}

	public Replayer(ReplayerParameters parameters, Petrinet net, Marking initialMarking, Marking finalMarking,
			XLog log, XEventClasses classes, TransEvClassMapping mapping) {
		this(parameters, net, initialMarking, finalMarking, log, classes, null, null, null, mapping);
	}

	public Replayer(ReplayerParameters parameters, Petrinet net, Marking initialMarking, Marking finalMarking,
			XLog log, XEventClasses classes, Map<Transition, Integer> costMM, Map<XEventClass, Integer> costLM,
			Map<Transition, Integer> costSM, TransEvClassMapping mapping) {
		this.parameters = parameters;
		this.log = log;
		this.classes = classes;
		if (costMM == null) {
			costMM = new HashMap<>();
			for (Transition t : net.getTransitions()) {
				if (t.isInvisible()) {
					costMM.put(t, 0);
				} else {
					costMM.put(t, 1);
				}
			}
		}
		if (costSM == null) {
			costSM = new HashMap<>();
			for (Transition t : net.getTransitions()) {
				costSM.put(t, 0);
			}
		}
		if (costLM == null) {
			costLM = new HashMap<>();
			for (XEventClass clazz : classes.getClasses()) {
				costLM.put(clazz, 1);
			}
		}
		this.costMM = costMM;
		this.costLM = costLM;
		factory = new SyncProductFactory(net, classes, mapping, costMM, costLM, costSM, initialMarking, finalMarking);

		trace2FirstIdenticalTrace = new TObjectIntHashMap<>(log.size() / 2, 0.7f, -1);
	}

	public PNRepResult computePNRepResult(Progress progress) throws InterruptedException, ExecutionException {
		this.progress = progress;
		//TODO: Detect previously computed cases as duplicates when the traces are equal as sequences of classifiers.

		if (parameters.debug == Debug.STATS) {
			parameters.debug.print(Debug.STATS, "SP label");
			for (Statistic s : Statistic.values()) {
				parameters.debug.print(Debug.STATS, ",");
				parameters.debug.print(Debug.STATS, s.toString());
			}
			parameters.debug.println(Debug.STATS);

		}

		// compute timeout per trace
		int timeoutMilliseconds = (int) ((10.0 * parameters.timeoutMilliseconds) / (log.size() + 1));
		timeoutMilliseconds = Math.min(timeoutMilliseconds, parameters.timeoutMilliseconds);

		ExecutorService service;
		if (parameters.algorithm == Algorithm.ASTAR) {
			// multi-threading is handled inside ASTAR
			service = Executors.newFixedThreadPool(1);
		} else {
			service = Executors.newFixedThreadPool(parameters.nThreads);
		}
		progress.setMaximum(log.size() + 1);

		List<Future<TraceReplayTask>> resultList = new ArrayList<>();

		TraceReplayTask tr = new TraceReplayTask(this, parameters, timeoutMilliseconds);
		resultList.add(service.submit(tr));

		int t = 0;
		for (XTrace trace : log) {
			tr = new TraceReplayTask(this, parameters, trace, t, timeoutMilliseconds);
			resultList.add(service.submit(tr));
			t++;
		}

		service.shutdown();

		Iterator<Future<TraceReplayTask>> itResult = resultList.iterator();

		// get the alignment of the empty trace
		int maxModelMoveCost;
		TraceReplayTask traceReplay = itResult.next().get();
		if (traceReplay.getResult() == TraceReplayResult.SUCCESS) {
			maxModelMoveCost = (int) Math.round(traceReplay.getSuccesfulResult().getInfo()
					.get(PNRepResult.RAWFITNESSCOST));
			itResult.remove();
		} else if (traceReplay.getResult() == TraceReplayResult.DUPLICATE) {
			assert false;
			maxModelMoveCost = 0;
		} else {
			maxModelMoveCost = 0;
		}

		TIntObjectMap<SyncReplayResult> result = new TIntObjectHashMap<>();
		// process further changes
		Iterator<XTrace> itTrace = log.iterator();
		while (itResult.hasNext() && !progress.isCancelled()) {
			tr = itResult.next().get();
			int traceCost = getTraceCost(itTrace.next());

			if (tr.getResult() == TraceReplayResult.SUCCESS) {

				SyncReplayResult srr = tr.getSuccesfulResult();
				srr.addInfo(PNRepResult.TRACEFITNESS,
						1 - (srr.getInfo().get(PNRepResult.RAWFITNESSCOST) / (maxModelMoveCost + traceCost)));
				result.put(tr.getTraceIndex(), srr);

			} else if (tr.getResult() == TraceReplayResult.DUPLICATE) {
				SyncReplayResult srr = result.get(tr.getOriginalTraceIndex());
				srr.addNewCase(tr.getTraceIndex());
			} else {
				// FAILURE TO COMPUTE ALIGNMENT
			}

		}

		PNRepResultImpl pnRepResult = new PNRepResultImpl(result.valueCollection());
		pnRepResult.addInfo(MAXMODELMOVECOST, Double.toString(maxModelMoveCost));

		return pnRepResult;

	}

	private int getTraceCost(XTrace trace) {
		int cost = 0;
		for (XEvent e : trace) {
			cost += costLM != null && costLM.containsKey(classes.getClassOf(e)) ? costLM.get(classes.getClassOf(e)) : 1;
		}
		return cost;
	}

	int getCostLM(XEventClass classOf) {
		return costLM != null && costLM.containsKey(classOf) ? costLM.get(classOf) : 1;
	}

	int getCostMM(Transition transition) {
		return costMM != null && costMM.containsKey(transition) ? costMM.get(transition) : 1;
	}

}
