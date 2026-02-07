package org.firstinspires.ftc.teamcode.subsystem.utility;

public final class Profiler {

    public static boolean enabled = false;
    private static dev.nullftc.profiler.Profiler profiler = null;

    private Profiler() {}

    public static void setProfiler(dev.nullftc.profiler.Profiler profiler) {
        Profiler.profiler = profiler;
        enabled = profiler != null;
    }

    public static void start(String type){
        if (enabled)
            profiler.start(type);
    }

    public static void end(String type){
        if (enabled)
            profiler.end(type);
    }
}
