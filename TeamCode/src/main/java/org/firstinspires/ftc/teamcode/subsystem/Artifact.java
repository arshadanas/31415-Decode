package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public enum Artifact {
    PURPLE,
    GREEN,
    EMPTY;

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
