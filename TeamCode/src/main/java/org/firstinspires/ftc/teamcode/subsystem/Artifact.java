package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.control.Wrap.wrap;

import org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator;

public enum Artifact {
    PURPLE,
    GREEN,
    EMPTY;

    private final static Artifact[] artifacts = values();

    public Artifact plus(int n) {
        return artifacts[wrap(ordinal() + n, 0, artifacts.length)];
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
