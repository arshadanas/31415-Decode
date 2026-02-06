package org.firstinspires.ftc.teamcode.subsystem.utility;

public class Profiler {

    public static Profiler INSTANCE = new Profiler();

    public boolean active;

    dev.nullftc.profiler.Profiler profiler;

    public Profiler(){
        this.profiler = null;
        this.active = false;
    }

    public void setProfiler(dev.nullftc.profiler.Profiler profiler) {
        this.profiler = profiler;
        this.active = profiler != null;
    }

    public void start(String type){
        if (active)
            this.profiler.start(type);
    }

    public void end(String type){
        if (active)
            this.profiler.end(type);
    }
}
