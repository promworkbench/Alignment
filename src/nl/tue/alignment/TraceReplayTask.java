package nl.tue.alignment;

import gnu.trove.list.TShortList;
import gnu.trove.map.TObjectIntMap;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;

import nl.tue.alignment.Utils.Statistic;
import nl.tue.alignment.algorithms.AStar;
import nl.tue.alignment.algorithms.AStarLargeLP;
import nl.tue.alignment.algorithms.Dijkstra;
import nl.tue.alignment.algorithms.ReplayAlgorithm;
import nl.tue.alignment.algorithms.datastructures.SyncProduct;
import nl.tue.astar.util.ilp.LPMatrixException;

import org.deckfour.xes.extension.std.XConceptExtension;
import org.deckfour.xes.factory.XFactoryRegistry;
import org.deckfour.xes.model.XTrace;
import org.processmining.models.graphbased.directed.petrinet.elements.Transition;
import org.processmining.plugins.petrinet.replayresult.PNRepResult;
import org.processmining.plugins.petrinet.replayresult.StepTypes;
import org.processmining.plugins.replayer.replayresult.SyncReplayResult;

class TraceReplayTask implements Callable<TraceReplayTask> {

	enum TraceReplayResult {
		FAILED, DUPLICATE, SUCCESS
	};

	private final Replayer replayer;
	private final XTrace trace;
	private final int traceIndex;
	private final int timeoutMilliseconds;
	private SyncReplayResult srr;
	private int original;
	private TraceReplayResult result;
	private ReplayAlgorithm algorithm;
	private final ReplayerParameters parameters;

	public TraceReplayTask(Replayer replayer, ReplayerParameters parameters, int timeoutMilliseconds) {
		this.replayer = replayer;
		this.parameters = parameters;
		this.trace = XFactoryRegistry.instance().currentDefault().createTrace();
		XConceptExtension.instance().assignName(trace, "Empty");
		this.traceIndex = -1;
		this.timeoutMilliseconds = timeoutMilliseconds;
	}

	public TraceReplayTask(Replayer replayer, ReplayerParameters parameters, XTrace trace, int traceIndex,
			int timeoutMilliseconds) {
		this.replayer = replayer;
		this.parameters = parameters;
		this.trace = trace;
		this.traceIndex = traceIndex;
		this.timeoutMilliseconds = timeoutMilliseconds;
	}

	public TraceReplayTask call() throws LPMatrixException {
		TShortList traceAsList = this.replayer.factory.getListEventClasses(trace);

		synchronized (this.replayer.trace2FirstIdenticalTrace) {
			original = this.replayer.trace2FirstIdenticalTrace.get(traceAsList);
			if (original < 0) {
				this.replayer.trace2FirstIdenticalTrace.put(traceAsList, traceIndex);
			}
		}
		if (original < 0) {
			SyncProduct product;
			List<Transition> transitionList = new ArrayList<Transition>();
			product = this.replayer.factory.getSyncProduct(trace, transitionList);
			if (product != null) {
				algorithm = getAlgorithm(product);
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

	private SyncReplayResult toSyncReplayResult(SyncProduct product, TObjectIntMap<Statistic> statistics,
			short[] alignment, XTrace trace, int traceIndex, List<Transition> transitionList) {
		List<Object> nodeInstance = new ArrayList<>(alignment.length);
		List<StepTypes> stepTypes = new ArrayList<>(alignment.length);
		int mm = 0, lm = 0, smm = 0, slm = 0;
		for (int i = 0; i < alignment.length; i++) {
			short t = alignment[i];
			if (product.getTypeOf(t) == SyncProduct.LOG_MOVE) {
				nodeInstance.add(replayer.classes.getClassOf(trace.get(product.getEventOf(t))));
				stepTypes.add(StepTypes.L);
				lm += product.getCost(t);
			} else {
				nodeInstance.add(transitionList.get(t));
				if (product.getTypeOf(t) == SyncProduct.MODEL_MOVE) {
					stepTypes.add(StepTypes.MREAL);
					mm += product.getCost(t);
				} else if (product.getTypeOf(t) == SyncProduct.SYNC_MOVE) {
					stepTypes.add(StepTypes.LMGOOD);
					smm += replayer.getCostMM(transitionList.get(t));
					slm += replayer.getCostLM(replayer.classes.getClassOf(trace.get(product.getEventOf(t))));
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

	public ReplayAlgorithm getUsedAlgorithm() {
		return algorithm;
	}

	ReplayAlgorithm getAlgorithm(SyncProduct product) throws LPMatrixException {
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

}