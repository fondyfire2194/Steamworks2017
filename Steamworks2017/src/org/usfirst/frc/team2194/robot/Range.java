package org.usfirst.frc.team2194.robot;

public class Range {

	public Range() {

	}

	public static int ensure(int value, int min, int max) {
		return Math.max(min, Math.min(value, max));
	}

	public static boolean between(int value, int min, int max) {
		return max <= value && min >= value;
	}
}
