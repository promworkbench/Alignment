package nl.tue.alignment;

import gnu.trove.list.TShortList;
import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TObjectIntHashMap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

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
import org.deckfour.xes.extension.std.XConceptExtension;
import org.deckfour.xes.factory.XFactoryRegistry;
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

	private enum TraceReplayResult {
		FAILED, DUPLICATE, SUCCESS
	};

	private class TraceReplay implements Callable<TraceReplay> {

		private final XTrace trace;
		private final int traceIndex;
		private final int timeoutMilliseconds;
		private SyncReplayResult srr;
		private int original;
		private TraceReplayResult result;

		public TraceReplay(int timeoutMilliseconds) {
			this.trace = XFactoryRegistry.instance().currentDefault().createTrace();
			XConceptExtension.instance().assignName(trace, "Empty");
			this.traceIndex = -1;
			this.timeoutMilliseconds = timeoutMilliseconds;
		}

		public TraceReplay(XTrace trace, int traceIndex, int timeoutMilliseconds) {
			this.trace = trace;
			this.traceIndex = traceIndex;
			this.timeoutMilliseconds = timeoutMilliseconds;
		}

		public TraceReplay call() throws LPMatrixException {
			TShortList traceAsList = factory.getListEventClasses(trace);

			synchronized (trace2FirstIdenticalTrace) {
				original = trace2FirstIdenticalTrace.get(traceAsList);
				if (original < 0) {
					trace2FirstIdenticalTrace.put(traceAsList, traceIndex);
				}
			}
			if (original < 0) {
				SyncProduct product;
				List<Transition> transitionList = new ArrayList<Transition>();
				product = factory.getSyncProduct(trace, transitionList);
				if (product != null) {
					ReplayAlgorithm algorithm = getAlgorithm(product);
					short[] alignment = algorithm.run(timeoutMilliseconds);
					TObjectIntMap<Statistic> stats = algorithm.getStatistics(alignment);
					srr = toSyncReplayResult(product, stats, alignment, trace, traceIndex, transitionList);
					result = TraceReplayResult.SUCCESS;
				} else {
					result = TraceReplayResult.FAILED;
				}
			} else {
				result = TraceReplayResult.DUPLICATE;
			}
			return this;
		}

		public TraceReplayResult getResult() {
			return result;
		}

		public SyncReplayResult getSuccesfulResult() {
			return srr;
		}

		public int getOriginalTraceIndex() {
			return original;
		}

		public int getTraceIndex() {
			return traceIndex;
		}

	}

	public static enum Algorithm {
		DIJKSTRA, ASTAR, ASTARWITHMARKINGSPLIT;
	}

	private final TObjectIntMap<TShortList> trace2FirstIdenticalTrace;

	private final ReplayerParameters parameters;
	private final XLog log;
	private final Map<XEventClass, Integer> costLM;
	private final SyncProductFactory factory;
	private final XEventClasses classes;
	private Map<Transition, Integer> costMM;

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

	public PNRepResult computePNRepResult() throws InterruptedException, ExecutionException {

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
		List<Future<TraceReplay>> resultList = new ArrayList<>();

		TraceReplay tr = new TraceReplay(timeoutMilliseconds);
		resultList.add(service.submit(tr));

		int t = 0;
		for (XTrace trace : log) {
			tr = new TraceReplay(trace, t, timeoutMilliseconds);
			resultList.add(service.submit(tr));
			t++;
		}

		service.shutdown();

		Iterator<Future<TraceReplay>> itResult = resultList.iterator();

		// get the alignment of the empty trace
		int maxModelMoveCost;
		TraceReplay traceReplay = itResult.next().get();
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
		while (itResult.hasNext()) {
			tr = itResult.next().get();
			itResult.remove();
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
		return new PNRepResultImpl(result.valueCollection());
	}

	private ReplayAlgorithm getAlgorithm(SyncProduct product) throws LPMatrixException {
		switch (parameters.algorithm) {
			case ASTAR :
				return new AStar(product, parameters.moveSort, parameters.queueSort, parameters.preferExact,//
						parameters.useInt, parameters.nThreads, parameters.debug);
			case ASTARWITHMARKINGSPLIT :
				return new AStarLargeLP(product, parameters.moveSort, parameters.useInt, parameters.intialBins,
						parameters.debug);
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

	private synchronized SyncReplayResult toSyncReplayResult(SyncProduct product, TObjectIntMap<Statistic> statistics,
			short[] alignment, XTrace trace, int traceIndex, List<Transition> transitionList) {
		List<Object> nodeInstance = new ArrayList<>(alignment.length);
		List<StepTypes> stepTypes = new ArrayList<>(alignment.length);
		int mm = 0, lm = 0, smm = 0, slm = 0;
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
					smm += getCostMM(transitionList.get(t));
					slm += getCostLM(classes.getClassOf(trace.get(product.getEventOf(t))));
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
		if (lm + slm == 0) {
			srr.addInfo(PNRepResult.MOVELOGFITNESS, 1.0);
		} else {
			srr.addInfo(PNRepResult.MOVELOGFITNESS, 1.0 - (1.0 * lm) / (lm + slm));
		}
		if (mm + smm == 0) {
			srr.addInfo(PNRepResult.MOVEMODELFITNESS, 1.0);
		} else {
			srr.addInfo(PNRepResult.MOVEMODELFITNESS, 1.0 - (1.0 * mm) / (mm + smm));
		}
		srr.addInfo(PNRepResult.NUMSTATEGENERATED, 1.0 * statistics.get(Statistic.MARKINGSREACHED));
		srr.addInfo(PNRepResult.ORIGTRACELENGTH, 1.0 * trace.size());

		srr.setReliable(statistics.get(Statistic.EXITCODE) == Utils.OPTIMALALIGNMENT);
		return srr;
	}

	private int getCostLM(XEventClass classOf) {
		return costLM != null && costLM.containsKey(classOf) ? costLM.get(classOf) : 1;
	}

	private int getCostMM(Transition transition) {
		return costMM != null && costMM.containsKey(transition) ? costMM.get(transition) : 1;
	}

}
