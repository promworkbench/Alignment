package nl.tue.alignment.test;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

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
import org.processmining.plugins.replayer.replayresult.SyncReplayResult;

import nl.tue.alignment.Replayer;
import nl.tue.alignment.ReplayerParameters;
import nl.tue.alignment.TraceReplayTask;
import nl.tue.alignment.Utils;
import nl.tue.alignment.algorithms.ReplayAlgorithm.Debug;

public class BasicCodeSnippet {

	public static void doReplay(XLog log, Petrinet net, Marking initialMarking, Marking finalMarking,
			XEventClasses classes, TransEvClassMapping mapping) {

		int nThreads = 2;
		int costUpperBound = 0;
		ReplayerParameters parameters = new ReplayerParameters.Default(nThreads, costUpperBound, Debug.NONE);
		Replayer replayer = new Replayer(parameters, net, initialMarking, finalMarking, classes, mapping, false);

		// timeout per trace in milliseconds
		int timeoutMilliseconds = 10 * 1000;
		// preprocessing time to be added to the statistics if necessary
		long preProcessTimeNanoseconds = 0;

		int fitting = 0;
		int nonfitting = 0;
		ExecutorService service = Executors.newFixedThreadPool(parameters.nThreads);

		@SuppressWarnings("unchecked")
		Future<TraceReplayTask>[] futures = new Future[log.size()];

		for (int i = 0; i < log.size(); i++) {
			// Setup the trace replay task
			TraceReplayTask task = new TraceReplayTask(replayer, parameters, log.get(i), i, timeoutMilliseconds,
					parameters.maximumNumberOfStates, preProcessTimeNanoseconds);

			// submit for execution
			futures[i] = service.submit(task);
		}
		// initiate shutdown and wait for termination of all submitted tasks.
		service.shutdown();

		// obtain the results one by one.

		for (int i = 0; i < log.size(); i++) {

			TraceReplayTask result;
			try {
				result = futures[i].get();
			} catch (Exception e) {
				// execution os the service has terminated.
				assert false;
				throw new RuntimeException("Error while executing replayer in ExecutorService. Interrupted maybe?", e);
			}

			switch (result.getResult()) {
				case DUPLICATE :
					assert false; // cannot happen in this setting
					throw new RuntimeException("Result cannot be a duplicate in per-trace computations.");
				case FAILED :
					// internal error in the construction of synchronous product or other error.
					throw new RuntimeException("Error in alignment computations");
				case SUCCESS :
					// process succcesful execution of the replayer
					SyncReplayResult replayResult = result.getSuccesfulResult();
					int exitCode = replayResult.getInfo().get(Replayer.TRACEEXITCODE).intValue();
					if ((exitCode & Utils.OPTIMALALIGNMENT) == Utils.OPTIMALALIGNMENT) {
						// Optimal alignment found.
						fitting++;
					} else if ((exitCode & Utils.FAILEDALIGNMENT) == Utils.FAILEDALIGNMENT) {
						// failure in the alignment. Error code shows more details.
						nonfitting++;
					}
					if ((exitCode & Utils.ENABLINGBLOCKEDBYOUTPUT) == Utils.ENABLINGBLOCKEDBYOUTPUT) {
						// in some marking, there were too many tokens in a place, blocking the addition of more tokens. Current upper limit is 128
					}
					if ((exitCode & Utils.COSTFUNCTIONOVERFLOW) == Utils.COSTFUNCTIONOVERFLOW) {
						// in some marking, the cost function went through the upper limit of 2^24
					}
					if ((exitCode & Utils.HEURISTICFUNCTIONOVERFLOW) == Utils.HEURISTICFUNCTIONOVERFLOW) {
						// in some marking, the heuristic function went through the upper limit of 2^24
					}
					if ((exitCode & Utils.TIMEOUTREACHED) == Utils.TIMEOUTREACHED) {
						// alignment failed with a timeout
					}
					if ((exitCode & Utils.STATELIMITREACHED) == Utils.STATELIMITREACHED) {
						// alignment failed due to reacing too many states.
					}
					if ((exitCode & Utils.COSTLIMITREACHED) == Utils.COSTLIMITREACHED) {
						// no optimal alignment found with cost less or equal to the given limit.
					}

					break;
			}

		}

	}

	public static TransEvClassMapping constructMappingBasedOnLabelEquality(PetrinetGraph net, XLog log,
			XEventClass dummyEvClass, XEventClassifier eventClassifier) {
		TransEvClassMapping mapping = new TransEvClassMapping(eventClassifier, dummyEvClass);

		XLogInfo summary = XLogInfoFactory.createLogInfo(log, eventClassifier);

		for (Transition t : net.getTransitions()) {
			boolean mapped = false;
			for (XEventClass evClass : summary.getEventClasses().getClasses()) {
				String id = evClass.getId();

				if (t.getLabel().equals(id)) {
					mapping.put(t, evClass);
					mapped = true;
					break;
				}
			}

			if (!mapped && !t.isInvisible()) {
				mapping.put(t, dummyEvClass);
			}

		}

		return mapping;
	}
}
