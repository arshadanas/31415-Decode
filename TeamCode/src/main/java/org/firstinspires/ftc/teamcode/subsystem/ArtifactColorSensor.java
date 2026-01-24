package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class ArtifactColorSensor {

    private final DigitalChannel pin0, pin1;

    public ArtifactColorSensor(HardwareMap hardwareMap, String pin0Name, String pin1Name) {
        pin0 = hardwareMap.digitalChannel.get(pin0Name);
        pin1 = hardwareMap.digitalChannel.get(pin1Name);
    }

    public Artifact getArtifact() {
        return
                pin0.getState() ?   Artifact.PURPLE :
                pin1.getState() ?   Artifact.GREEN :
                                    Artifact.EMPTY;
    }
}
