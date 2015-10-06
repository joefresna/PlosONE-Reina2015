package utils;

import java.util.Random;

/**
 * A subclass of java.util.random that implements the 
 * Xorshift random number generator
 */

public class XORShiftRandom extends Random {
	private static final long serialVersionUID = 4571537475330642112L;
	private long seed;

	public XORShiftRandom(long seed) {
		this.seed = seed;
	}

	protected int next(int nbits) {
		long x = seed;
		x ^= (x << 21);
		x ^= (x >>> 35);
		x ^= (x << 4);
		seed = x;
		x &= ((1L << nbits) - 1);
		return (int) x;
	}
}
