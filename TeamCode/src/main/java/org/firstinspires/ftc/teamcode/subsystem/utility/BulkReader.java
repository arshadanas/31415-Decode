package org.firstinspires.ftc.teamcode.subsystem.utility;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public final class BulkReader {

//    private final List<LynxModule> revHubs;

    public BulkReader(HardwareMap hardwareMap) {
//        revHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : revHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    public void bulkRead() {

        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
//        for (LynxModule hub : revHubs) hub.clearBulkCache();
    }
}
