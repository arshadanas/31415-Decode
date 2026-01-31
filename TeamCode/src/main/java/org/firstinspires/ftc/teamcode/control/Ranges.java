package org.firstinspires.ftc.teamcode.control;

public final class Ranges {

    private Ranges() {}

    /**
     * @param num   Number to wrap
     * @param lo    Lower bound (INCLUSIVE)
     * @param hi    Upper bound (EXCLUSIVE), must be greater than lo
     * @return      num wrapped to the range [lo, hi)
     */
    public static int wrap(int num, int lo, int hi) {
        int mod = hi - lo;
        return ((num - lo) % mod + mod) % mod + lo;
    }

    /**
     * @param num   Number to clip
     * @param min   Lower bound (INCLUSIVE)
     * @param max   Upper bound (INCLUSIVE)
     * @return      num clipped to the range [min, max]
     */
    public static double clip(double num, double min, double max) {
        return Math.min(Math.max(num, min), max);
    }
}
