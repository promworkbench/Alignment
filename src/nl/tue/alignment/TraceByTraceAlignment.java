package nl.tue.alignment;

import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.FutureTask;

import org.deckfour.xes.classification.XEventClass;
import org.deckfour.xes.classification.XEventClasses;
import org.deckfour.xes.classification.XEventClassifier;
import org.deckfour.xes.info.XLogInfo;
import org.deckfour.xes.info.XLogInfoFactory;
import org.deckfour.xes.model.XLog;
import org.processmining.models.graphbased.directed.petrinet.Petrinet;
import org.processmining.models.graphbased.directed.petrinet.PetrinetGraph;
import org.processmining.models.graphbased.directed.petrinet.elements.Transition;
import org.processmining.models.semantics.petrinet.Marking;
import org.processmining.plugins.connectionfactories.logpetrinet.TransEvClassMapping;
import org.processmining.plugins.petrinet.replayresult.PNRepResult;
import org.processmining.plugins.replayer.replayresult.SyncReplayResult;

import nl.tue.alignment.algorithms.ReplayAlgorithm.Debug;

/**
 * To use this class for experiments, the following code snippet can be used. It
 * is required for the provided log that the first trace is an empty trace:
 *
 * <code>
	public static void doLogReplay() {
		TraceByTraceAlignment traceByTraceAlignment = new TraceByTraceAlignment(net, initialMarking, finalMarking, log, classes, mapping);
		
		List<Future<TraceReplayTask>> list = new ArrayList<>(log.size());
		for (int i=0; i< log.size(); i++) {
			list.add(traceByTraceAlignment.doReplay(i, 60*10*000), eventsWithErrors);
		}
		
		PNRepResult repResult = traceByTraceAlignment.merge(list);
	}
 * </code>
 * 
 * @author bfvdonge
 *
 */
public class TraceByTraceAlignment {

	private Petrinet net;
	private Marking initialMarking;
	private Marking finalMarking;
	private XLog log;
	private XEventClasses classes;
	private TransEvClassMapping mapping;
	private ReplayerParameters.AStarWithMarkingSplit parameters;
	private Replayer replayer;

	/**
	 * Setup the trace-by-trace replayer using default parameters for the given net
	 * and log with a default, label-based mapping.
	 * 
	 * @param net
	 * @param initialMarking
	 * @param finalMarking
	 * @param log
	 * @param classes
	 * @param mapping
	 */
	public TraceByTraceAlignment(Petrinet net, Marking initialMarking, Marking finalMarking, XLog log,
			XEventClasses classes, TransEvClassMapping mapping) {
		this.net = net;
		this.initialMarking = initialMarking;
		this.finalMarking = finalMarking;
		this.log = log;
		this.classes = classes;
		this.mapping = mapping;

		setupParameters();
	}

	/**
	 * Setup the trace-by-trace replayer using default parameters for the given net
	 * and log with the mapping provided
	 * 
	 * @param net
	 * @param initialMarking
	 * @param finalMarking
	 * @param log
	 * @param eventClassifier
	 */
	public TraceByTraceAlignment(Petrinet net, Marking initialMarking, Marking finalMarking, XLog log,
			XEventClassifier eventClassifier) {

		this.net = net;
		this.initialMarking = initialMarking;
		this.finalMarking = finalMarking;
		this.log = log;

		XEventClass dummyEvClass = new XEventClass("DUMMY", 99999);
		mapping = constructMapping(net, log, dummyEvClass, eventClassifier);
		XLogInfo summary = XLogInfoFactory.createLogInfo(log, eventClassifier);
		classes = summary.getEventClasses();
		setupParameters();
	}

	private void setupParameters() {

		// number of threads (irrelevant for trace by trace computations)
		int threads = 1;
		// timeout 30 sec per trace minutes
		int timeout = log.size() * 30 * 1000 / 10;
		// no maximum state count
		int maxNumberOfStates = Integer.MAX_VALUE;
		// move sorting (should be false for incremental alignments)
		boolean moveSort = false;
		// use integers in linear programs (false for faster alignments)
		boolean useInt = false;
		// use partial orders (should be false for incremental alignments with pre-set splitpoints)
		boolean partialOrder = false;

		parameters = new ReplayerParameters.AStarWithMarkingSplit(moveSort, threads, useInt, Debug.NONE, timeout,
				maxNumberOfStates, partialOrder, false);
		replayer = new Replayer(parameters, net, initialMarking, finalMarking, log, classes, mapping);

	}

	/**
	 * returns a future to allow for normal merging procedures, but computation is
	 * synchronously. Collect them in a list for later merging. When merging, the
	 * result is already available.
	 * 
	 * @param traceIndex
	 * @param timeoutMilliseconds
	 * @param eventsWithErrors
	 * @return
	 */
	public Future<TraceReplayTask> doReplay(int traceIndex, int timeoutMilliseconds, short... eventsWithErrors) {

		TraceReplayTask task = new TraceReplayTask(replayer, parameters, log.get(traceIndex), traceIndex,
				timeoutMilliseconds, parameters.maximumNumberOfStates, eventsWithErrors);

		FutureTask<TraceReplayTask> futureTask = new FutureTask<>(task);
		futureTask.run();
		return futureTask;
	}

	/**
	 * merge the future's
	 * 
	 * @param resultList
	 * @return
	 * @throws InterruptedException
	 * @throws ExecutionException
	 */
	public PNRepResult merge(List<Future<TraceReplayTask>> resultList) throws InterruptedException, ExecutionException {

		PNRepResult result = replayer.mergeResults(resultList);

		int cost = (int) Double.parseDouble((String) result.getInfo().get(Replayer.MAXMODELMOVECOST));
		int timeout = 0;
		int time = 0;
		int mem = 0;
		for (SyncReplayResult res : result) {
			cost += res.getInfo().get(PNRepResult.RAWFITNESSCOST);
			timeout += res.getInfo().get(Replayer.TRACEEXITCODE).intValue() != 1 ? 1 : 0;
			time += res.getInfo().get(PNRepResult.TIME).intValue();
			mem = Math.max(mem, res.getInfo().get(Replayer.MEMORYUSED).intValue());
		}
		System.out.print(time + ",");
		System.out.print(mem + ",");
		System.out.print(timeout + ",");
		System.out.print(cost + ",");

		System.out.println();
		System.out.flush();

		return result;
	}

	/**
	 * Constructs a default, label-based mapping
	 * 
	 * @param net
	 * @param log
	 * @param dummyEvClass
	 * @param eventClassifier
	 * @return
	 */
	private TransEvClassMapping constructMapping(PetrinetGraph net, XLog log, XEventClass dummyEvClass,
			XEventClassifier eventClassifier) {
		TransEvClassMapping mapping = new TransEvClassMapping(eventClassifier, dummyEvClass);

		XLogInfo summary = XLogInfoFactory.createLogInfo(log, eventClassifier);

		for (Transition t : net.getTransitions()) {
			for (XEventClass evClass : summary.getEventClasses().getClasses()) {
				String id = evClass.getId();

				// map transitions and event classes based on label
				if (t.getLabel().equals(id)) {
					mapping.put(t, evClass);
					break;
				}
			}

			//			if (!mapped && !t.isInvisible()) {
			//				mapping.put(t, dummyEvClass);
			//			}

		}

		return mapping;
	}

}
