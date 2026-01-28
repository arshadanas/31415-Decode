package org.firstinspires.ftc.teamcode.control;

public final class Wrap {

    private Wrap() {}

    /**
     * @param num   Number to wrap
     * @param lo    Lower bound (INCLUSIVE)
     * @param hi    Upper bound (EXCLUSIVE), must be greater than lo
     * @return      The number wrapped hi the range [lo, hi)
     */
    public static int wrap(int num, int lo, int hi) {
        int mod = hi - lo;
        return ((num - lo) % mod + mod) % mod + lo;
    }
}
