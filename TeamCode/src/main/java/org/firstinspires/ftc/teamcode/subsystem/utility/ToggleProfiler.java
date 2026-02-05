package org.firstinspires.ftc.teamcode.subsystem.utility;

import dev.nullftc.profiler.Profiler;

public class ToggleProfiler {
    boolean active;
    Profiler profiler;
    public ToggleProfiler(Profiler profiler){
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
