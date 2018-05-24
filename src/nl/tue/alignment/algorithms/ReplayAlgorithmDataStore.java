package nl.tue.alignment.algorithms;

import java.util.Arrays;

import nl.tue.alignment.Utils;

class ReplayAlgorithmDataStore {

	// The c_g__pt array is organized as follows
	private static final long CLOSEDMASK /* . 1 */ = 0b1000000000000000000000000000000000000000000000000000000000000000L;
	private static final long GMASK /* ..... 31 */ = 0b0111111111111111111111111111111100000000000000000000000000000000L;
	// Free:                                 17      0b0000000000000000000000000000000011111111111111111000000000000000L
	private static final int GSHIFT /*          */ = 32;
	private static final long PTMASK /* .... 15 */ = 0b0000000000000000000000000000000000000000000000000111111111111111L;

	// The e_h__p array is organized as follows
	private static final long EXACTMASK /* .. 1 */ = 0b1000000000000000000000000000000000000000000000000000000000000000L;
	private static final long HMASK /* ..... 31 */ = 0b0111111111111111111111111111111100000000000000000000000000000000L;
	// Free:                                  1      0b0000000000000000000000000000000010000000000000000000000000000000L
	private static final long PMASK /* ..... 31 */ = 0b0000000000000000000000000000000001111111111111111111111111111111L;
	private static final int HSHIFT /*          */ = 32;

	protected static int HEURISTICINFINITE /*   */ = Integer.MAX_VALUE;
	protected static final int NOPREDECESSOR /* */ = (int) PMASK;

	protected int alignmentResult;

	/**
	 * Stores the blockSize as a power of 2
	 */
	protected final int blockSize;
	/**
	 * Stores the number of trailing 0's in the blockSize.
	 */
	protected final int blockBit;
	/**
	 * equals blockSize-1
	 */
	protected final int blockMask;

	/**
	 * Stores the last block in use
	 */
	protected int block;
	/**
	 * Stores the first new index in current block
	 */
	protected int indexInBlock;

	/**
	 * For each marking stores: 1 bit: whether it is in the closed set 32 bits:
	 * Value of g (unsigned int)
	 * 
	 * 15 bit: Predecessor transition
	 */
	private long[][] c_g__pt;
	/**
	 * Stores: 1 bit: whether it is an estimated heuristic 32 bits: value of h
	 * (signed int) 31 bits: predecessor marking
	 */
	private long[][] e_h__p;

	public ReplayAlgorithmDataStore() {
		super();
		this.blockSize = Utils.DEFAULTBLOCKSIZE;
		this.blockMask = blockSize - 1;
		int bit = 1;
		int i = 0;
		while (bit < blockMask) {
			bit <<= 1;
			i++;
		}
		this.blockBit = i;
	}

	/**
	 * Grow the internal array structure. Method should be considered synchronized
	 * as it should not be executed in parallel.
	 */
	protected void growArrays() {
		if (block + 1 >= c_g__pt.length) {
			int newLength = c_g__pt.length < 64 ? c_g__pt.length * 2 : (c_g__pt.length * 3) / 2;
			if (newLength <= block + 1) {
				newLength = block + 2;
			}
			e_h__p = Arrays.copyOf(e_h__p, newLength);
			c_g__pt = Arrays.copyOf(c_g__pt, newLength);
		}
		// increase the block pointer
		block++;
		// reset the index in block
		indexInBlock = 0;

		// e_g_h_pt holds blocksize values for
		// estimated, g-score, h-score, predecessor transitions
		c_g__pt[block] = new long[blockSize];
		Arrays.fill(c_g__pt[block], GMASK);
		// p holds blocksize predecessors
		e_h__p[block] = new long[blockSize];

	}

	protected void initializeIterationInternal() {
		block = -1;
		indexInBlock = 0;
		e_h__p = new long[0][];
		c_g__pt = new long[0][];
	}

	/**
	 * Returns the g score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public int getGScore(int block, int index) {
		return (int) ((c_g__pt[block][index] & GMASK) >>> GSHIFT);
	}

	/**
	 * Set the g score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public void setGScore(int block, int index, int score) {
		// overwrite the last three bytes of the score.
		c_g__pt[block][index] &= ~GMASK;
		long scoreL = ((long) score) << GSHIFT;
		if ((scoreL & GMASK) != scoreL) {
			alignmentResult |= Utils.COSTFUNCTIONOVERFLOW;
			c_g__pt[block][index] |= scoreL & GMASK;
		} else {
			c_g__pt[block][index] |= scoreL;
		}
	}

	/**
	 * Set the g score for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public void setGScore(int marking, int score) {
		setGScore(marking >>> blockBit, marking & blockMask, score);
	}

	/**
	 * Returns the h score for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public int getHScore(int marking) {
		return getHScore(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * Returns the h score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public int getHScore(int block, int index) {
		return (int) ((e_h__p[block][index] & HMASK) >>> HSHIFT);
	}

	/**
	 * set the h score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public void setHScore(int block, int index, int score, boolean isExact) {
		long scoreL = ((long) score) << HSHIFT;
		assert (scoreL & HMASK) == scoreL;
		// overwrite the last three bytes of the score.
		e_h__p[block][index] &= ~HMASK; // reset to 0
		e_h__p[block][index] |= scoreL; // set score
		if (isExact) {
			e_h__p[block][index] |= EXACTMASK; // set exactFlag
		} else {
			e_h__p[block][index] &= ~EXACTMASK; // clear exactFlag

		}
	}

	/**
	 * Set the h score for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public void setHScore(int marking, int score, boolean isExact) {
		setHScore(marking >>> blockBit, marking & blockMask, score, isExact);
	}

	/**
	 * Returns the predecessor for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public int getPredecessor(int marking) {
		return getPredecessor(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * Returns the predecessor for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public int getPredecessor(int block, int index) {
		return (int) (e_h__p[block][index] & PMASK);
	}

	/**
	 * Sets the predecessor for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public void setPredecessor(int block, int index, int predecessorMarking) {
		e_h__p[block][index] &= ~PMASK;
		e_h__p[block][index] |= predecessorMarking & PMASK;
	}

	/**
	 * Sets the predecessor for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public void setPredecessor(int marking, int predecessorMarking) {
		setPredecessor(marking >>> blockBit, marking & blockMask, predecessorMarking);
	}

	/**
	 * Returns the predecessor transition for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public short getPredecessorTransition(int marking) {
		return getPredecessorTransition(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * Returns the predecessor transition for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public short getPredecessorTransition(int block, int index) {
		return (short) (c_g__pt[block][index] & PTMASK);
	}

	/**
	 * Sets the predecessor transition for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public void setPredecessorTransition(int block, int index, short transition) {
		c_g__pt[block][index] &= ~PTMASK; //clear pt bits
		c_g__pt[block][index] |= transition; //set pt bits
	}

	/**
	 * Sets the predecessor transition for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public void setPredecessorTransition(int marking, short transition) {
		setPredecessorTransition(marking >>> blockBit, marking & blockMask, transition);
	}

	/**
	 * Returns true if marking is in the closed set
	 * 
	 * @param marking
	 * @return
	 */
	public boolean isClosed(int marking) {
		return isClosed(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * Returns the g score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public boolean isClosed(int block, int index) {
		return (c_g__pt[block][index] & CLOSEDMASK) == CLOSEDMASK;
	}

	/**
	 * Set the g score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public void setClosed(int block, int index) {
		c_g__pt[block][index] |= CLOSEDMASK;
	}

	/**
	 * Set the g score for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public void setClosed(int marking) {
		setClosed(marking >>> blockBit, marking & blockMask);
	}

	protected long getEstimatedMemorySize() {
		// e_g_h_pt holds    4 + length * 8 + block * (4 + blockSize * 8) bytes;
		// c_p holds         4 + length * 8 + block * (4 + blockSize * 8) bytes;
		return 2 * 4 + 2 * c_g__pt.length * 8 + block * (8 + blockSize * 16);
	}

	/**
	 * Returns the f score for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public int getFScore(int marking) {
		return getFScore(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * Returns the f score for a stored marking
	 * 
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public int getFScore(int block, int index) {
		return getGScore(block, index) + getHScore(block, index);
	}

	/**
	 * Returns the g score for a stored marking
	 * 
	 * @param marking
	 * @return
	 */
	public int getGScore(int marking) {
		return getGScore(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * returns true if the heuristic stored for the given marking is exact or an
	 * estimate.
	 * 
	 * @param marking
	 * @return
	 */
	public boolean hasExactHeuristic(int marking) {
		return hasExactHeuristic(marking >>> blockBit, marking & blockMask);
	}

	/**
	 * returns true if the heuristic stored for the given marking is exact or an
	 * estimate.
	 * 
	 * @param marking
	 * @param block
	 *            the memory block the marking is stored in
	 * @param index
	 *            the index at which the marking is stored in the memory block
	 * @return
	 */
	public boolean hasExactHeuristic(int block, int index) {
		return (e_h__p[block][index] & EXACTMASK) == EXACTMASK;
	}

	protected boolean isInfinite(int heur) {
		return heur == HEURISTICINFINITE;
	}

}