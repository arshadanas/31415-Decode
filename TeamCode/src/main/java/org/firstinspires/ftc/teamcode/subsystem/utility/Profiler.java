package org.firstinspires.ftc.teamcode.subsystem.utility;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;

public final class Profiler {

    private Profiler() {}

    private static dev.nullftc.profiler.Profiler profiler;
    private static File profilerOutput;

    public static void start(String label){
        if (profiler != null) profiler.start(label);
    }
    public static void end(String label){
        if (profiler != null) profiler.end(label);
    }

    public static void init(boolean doProfiling) {
        if (!doProfiling) {
            profiler = null;
            profilerOutput = null;
            return;
        }

        // Create profile log folder
        File logsFolder = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logsFolder.exists())
            logsFolder.mkdirs();

        profilerOutput = new File(logsFolder, "profiler-" + System.currentTimeMillis() + ".csv");

        profiler = dev.nullftc.profiler.Profiler.builder()
                .factory(new BasicProfilerEntryFactory())
                .exporter(new CSVProfilerExporter(profilerOutput))
                .build();
    }

    public static void export() {
        // Check if we are actually in profile mode
        if (profiler == null)
            return;

        RobotLog.i("Starting async profiler export to: " + profilerOutput.getAbsolutePath());

        Thread exportThread = new Thread(() -> {
            try {
                profiler.export();
                profiler.shutdown();
            } catch (Exception e) {
                e.printStackTrace();
            }
        });

        exportThread.setDaemon(true);
        exportThread.start();
    }
}
