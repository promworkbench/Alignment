package nl.tue.alignment.algorithms.datastructures;

public interface SyncProduct {

	public static byte LOG_MOVE = 1;
	public static byte MODEL_MOVE = 2;
	public static byte SYNC_MOVE = 3;
	public static byte TAU_MOVE = 4;

	public static final int MAXTRANS = 0b0011111111;
	public static final short NOEVENT = -1;

	/**
	 * Returns the number of transitions. At most MAXTRANS transitions are
	 * allowed for memory reasons. (i.e. valid transition numbers are
	 * 0..MAXTRANS-1)
	 * 
	 * @return
	 */
	public short numTransitions();

	/**
	 * The number of places is in principle bounded Short.MAX_VALUE
	 * 
	 * @return
	 */
	public short numPlaces();

	/**
	 * Returns a sorted array of places serving as input to transition t
	 * 
	 * @param transition
	 * @return
	 */
	public short[] getInput(short transition);

	/**
	 * Returns a sorted array of places serving as output to transition t
	 * 
	 * @param transition
	 * @return
	 */
	public short[] getOutput(short transition);

	/**
	 * Return the initial marking as an array where each byte represents the
	 * marking of that specific place in the interval 0..3
	 * 
	 * @return
	 */
	public byte[] getInitialMarking();

	/**
	 * Return the final marking
	 * 
	 * @return
	 */
	public byte[] getFinalMarking();

	/**
	 * returns the cost of firing t. Note that the maximum cost of an alignment
	 * is 16777216, hence costs should not be excessive.
	 * 
	 * 
	 * @param t
	 * @return
	 */
	public int getCost(short transition);

	/**
	 * Returns the label of transition t
	 * 
	 * @param t
	 * @return
	 */
	public String getTransitionLabel(short t);

	/**
	 * Checks if a given marking is the (a) final marking
	 * 
	 * @param marking
	 * @return
	 */
	public boolean isFinalMarking(byte[] marking);

	/**
	 * Return the label of a place
	 * 
	 * @param p
	 * @return
	 */
	public String getPlaceLabel(short place);

	/**
	 * Returns the label of the synchronous product
	 * 
	 * @return
	 */
	public String getLabel();

	/**
	 * returns the event number associated with this transitions. Events are
	 * assumed numbered 0..n and for model-move transitions, this method returns
	 * NOEVENT (getTypeOf should then return MODEL_MOVE or TAU_MOVE)
	 * 
	 * @param transition
	 * @return
	 */
	public short getEventOf(short transition);

	/**
	 * returns the type of the transion as a byte equal to one of the constants
	 * defined in this class: LOG_MOVE, SYNC_MOVE, MODEL_MOVE, TAU_MOVE
	 * 
	 * @param transition
	 * @return
	 */
	public byte getTypeOf(short transition);

}
