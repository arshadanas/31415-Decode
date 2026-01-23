package org.firstinspires.ftc.teamcode.control;

public final class Wrap {

    private Wrap() {}

    /**
     * @param num   Number to wrap
     * @param from  Lower bound (INCLUSIVE)
     * @param to    Upper bound (EXCLUSIVE), must be greater than from
     * @return      The number wrapped to the range [from, to)
     */
    public static int wrap(int num, int from, int to) {
        int mod = to - from;
        return ((num - from) % mod + mod) % mod + from;
    }
}
