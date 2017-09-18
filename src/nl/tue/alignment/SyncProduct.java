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
	 * Returns an array of bytes. The length of the array is equal to
	 * 1+(numPlaces()-1)/8 The least significant bit represents place 0, i.e.
	 * [.....010][00000001] represents an input set consisting of place 0 and
	 * place 9, assuming there are 11 places in the model.
	 * 
	 * @param transition
	 * @return
	 */
	public byte[] getInput(short transition);

	/**
	 * Returns an array of bytes. The length of the array is equal to
	 * 1+(numPlaces()-1)/8 The most significant bit represents place 0, i.e.
	 * [1000 0000][010. ....] represents an output set consisting of place 0 and
	 * place 9, assuming there are 11 places in the model.
	 * 
	 * @param transition
	 * @return
	 */
	public byte[] getOutput(short transition);

	/**
	 * Return the initial marking as an array where the first n bytes represent
	 * the low bits and the second n bytes the high bit, i.e. [1000 1000][000.
	 * ....][1000 0000][010. ....] represents the marking with 3 tokens in place
	 * 0, 1 token in place 4 and 2 tokens in place 9, assuming 11 places in the
	 * model.
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
