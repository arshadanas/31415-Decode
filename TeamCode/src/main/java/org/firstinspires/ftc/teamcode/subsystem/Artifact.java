package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;

@Configurable
public enum Artifact {
    PURPLE,
    GREEN,
    EMPTY;

    public static HSV
            minPurple = new HSV(
                    0,
                    0,
                    0
            ),
            maxPurple = new HSV(
                    0,
                    0,
                    0
            ),
            minGreen = new HSV(
                    0,
                    0,
                    0
            ),
            maxGreen = new HSV(
                    0,
                    0,
                    0
            );

    public static Artifact fromHSV(HSV hsv) {
        return
                hsv.between(minPurple, maxPurple) ? PURPLE :
                hsv.between(minGreen, maxGreen) ?   GREEN :
                                                    EMPTY;
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
