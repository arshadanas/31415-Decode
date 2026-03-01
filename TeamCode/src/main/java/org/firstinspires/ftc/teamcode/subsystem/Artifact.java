package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.control.Ranges.wrap;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;

import java.util.OptionalInt;

@Config
public enum Artifact {
    PURPLE,
    GREEN,
    EMPTY;

    public static final Artifact[] EMPTY_ARRAY = {EMPTY, EMPTY, EMPTY};

    public static HSV
            minPurple = new HSV(
                    165,
                    0.4,
                    0
            ),
            maxPurple = new HSV(
                    200,
                    1,
                    1
            ),

            minGreen = new HSV(
                    135,
                    0.7,
                    0
            ),
            maxGreen = new HSV(
                    150,
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
        return this == EMPTY ? other : this;
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
    public OptionalInt firstOccurrenceIn(Artifact... artifacts) {
        for (int i = 0; i < artifacts.length; i++)
            if (artifacts[i] == this)
                return OptionalInt.of(i);
        return OptionalInt.empty();
    }
}
