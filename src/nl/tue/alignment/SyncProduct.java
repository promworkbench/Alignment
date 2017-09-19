package nl.tue.alignment;

public interface SyncProduct {

	/**
	 * Returns the number of transitions. At most 32768 transitions are allowed
	 * for memory reasons. (i.e. valid transition numbers are 0..32767)
	 * 
	 * @return
	 */
	public short numTransitions();

	/**
	 * The number of places is in principle bounded by Short.MAX_VALUE
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

}
