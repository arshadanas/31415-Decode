package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.control.Wrap.wrap;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator;

@Config
public enum Artifact {
    PURPLE,
    GREEN,
    EMPTY;

    public static HSV
            minPurple = new HSV(
                    175,
                    0.4,
                    0
            ),
            maxPurple = new HSV(
                    350,
                    1,
                    1
            ),

            minGreen = new HSV(
                    60,
                    0.65,
                    0
            ),
            maxGreen = new HSV(
                    160,
                    1,
                    1
            );

    public static Artifact fromHSV(HSV hsv) {
        return
                hsv.between(minPurple, maxPurple) ? PURPLE :
                hsv.between(minGreen, maxGreen) ?   GREEN :
                EMPTY;
    }

    private final static Artifact[] artifacts = values();

    public Artifact plus(int n) {
        return artifacts[wrap(ordinal() + n, 0, artifacts.length)];
    }

    public Artifact or(Artifact other) {
        return
                this == other ? this :
                this == EMPTY ? other :
                                this;
    }

    public LEDIndicator.LEDColor toLEDColor() {
        switch (this) {
            case PURPLE:    return LEDIndicator.LEDColor.RED;
            case GREEN:     return LEDIndicator.LEDColor.GREEN;
            default:        return LEDIndicator.LEDColor.OFF;
        }
    }

    /**
     * @return The number of times this artifact color appears in the provided array
     */
    public int numOccurrencesIn(Artifact... artifacts) {
        int count = 0;
        for (Artifact artifact : artifacts)
            if (artifact == this)
                count++;
        return count;
    }

    /**
     * @return The index of the first occurrence of this artifact color in the provided array, -1 if no occurrences
     */
    public int firstOccurrenceIn(Artifact... artifacts) {
        for (int i = 0; i < artifacts.length; i++)
            if (artifacts[i] == this)
                return i;
        return -1;
    }
}
