package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.control.Ranges.wrap;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.PURPLE;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.OptionalInt;

public enum Motif {

    GPP(GREEN, PURPLE, PURPLE),
    PGP(PURPLE, GREEN, PURPLE),
    PPG(PURPLE, PURPLE, GREEN);

    public final Artifact[] artifacts;

    private static final Motif[] motifs = values();

    Motif(Artifact... artifacts) {
        this.artifacts = artifacts;
    }

    /**
     * APRIL TAG IDs (21 22 23) CAN BE PASSED DIRECTLY TO THIS
     * @param greenIndex    Index of the green artifact in the motif GPP = 0, PGP = 1, PPG = 2
     * @return              The {@link Motif} that has a green artifact in the specified position
     */
    public static Motif fromGreenIndex(int greenIndex) {
        return motifs[wrap(greenIndex, 0, motifs.length)];
    }


    /**
     * @param numArtifactsScored Number of artifacts in the classifier ramp
     * @return The motif pattern to score to satisfy this randomization
     */
    public Motif getEffectiveMotif(int numArtifactsScored) {
        return Motif.fromGreenIndex(this.ordinal() - numArtifactsScored);
    }

    /**
     * @param allowOneWrong Allow one wrong artifact color when scoring three artifacts
     * @param numArtifactsScored Number of artifacts in the classifier ramp
     * @param spindexerSlots Artifacts available in the spindexer
     * @return The order to score the artifacts in the spindexer
     */
    public ArrayList<Integer> getScoringOrder(boolean allowOneWrong, int numArtifactsScored, Artifact... spindexerSlots) {

        if (numArtifactsScored == 9)
            return new ArrayList<>();

        assert spindexerSlots.length == 3;

        Motif effectiveMotif = this.getEffectiveMotif(numArtifactsScored);

        allowOneWrong = allowOneWrong && EMPTY.numOccurrencesIn(spindexerSlots) == 0 && numArtifactsScored <= 6;

        int firstIndex, secondIndex;

        OptionalInt indexOfA0 = effectiveMotif.artifacts[0].firstOccurrenceIn(spindexerSlots);
        if (indexOfA0.isPresent()) {
            firstIndex = indexOfA0.getAsInt();
            secondIndex = 1;
        } else {
            if (!allowOneWrong)
                return new ArrayList<>();

            // find occurrence of second motif color in spindexer
            OptionalInt indexOfA1 = effectiveMotif.artifacts[1].firstOccurrenceIn(spindexerSlots);
            if (!indexOfA1.isPresent())
                return new ArrayList<>();

            firstIndex = indexOfA1.getAsInt() - 1;
            secondIndex = 0;
        }

        ArrayList<Integer> scoringOrder = new ArrayList<>(Arrays.asList(0, 1, 2));

        Collections.rotate(scoringOrder, -firstIndex);

        boolean correctAudited = spindexerSlots[scoringOrder.get(secondIndex)] == effectiveMotif.artifacts[secondIndex];

        if (!correctAudited && spindexerSlots[scoringOrder.get(2)] == effectiveMotif.artifacts[secondIndex]) {
            Collections.swap(scoringOrder, secondIndex, 2);
            correctAudited = true;
        }

        boolean correctThird = spindexerSlots[scoringOrder.get(2)] == effectiveMotif.artifacts[2];
        boolean scoringTwoThirds = correctThird && allowOneWrong;

        if ((!correctAudited || numArtifactsScored == 8) && !scoringTwoThirds) {
            scoringOrder.remove(1);
            scoringOrder.remove(1);
        } else if (!correctThird || numArtifactsScored == 7)
            scoringOrder.remove(2);

        return scoringOrder;
    }

    /**
     * @param scoringOrder The order to score the artifacts in the spindexer
     * @param numArtifactsScored Number of artifacts in the classifier ramp
     * @param spindexerSlots Artifacts available in the spindexer
     * @return The number of points scored by following the provided scoringOrder
     */
    public int getScoreValue(ArrayList<Integer> scoringOrder, int numArtifactsScored, Artifact... spindexerSlots) {

        if (numArtifactsScored == 9 || scoringOrder.isEmpty())
            return 0;

        Motif effectiveMotif = getEffectiveMotif(numArtifactsScored);

        return scoringOrder.size() * 3 +
                            (spindexerSlots[scoringOrder.get(0)] == effectiveMotif.artifacts[0] ? 2 : 0) +
                            (scoringOrder.size() > 1 && spindexerSlots[scoringOrder.get(1)] == effectiveMotif.artifacts[1] ? 2 : 0) +
                            (scoringOrder.size() > 2 && spindexerSlots[scoringOrder.get(2)] == effectiveMotif.artifacts[2] ? 2 : 0)
        ;
    }

    /**
     * @param allowOneWrong Allow one wrong artifact color when scoring three artifacts
     * @param numArtifactsScored Number of artifacts in the classifier ramp
     * @param spindexerSlots Artifacts available in the spindexer
     * @return The order to score the artifacts in the spindexer, as a {@link String}
     */
    public String getScoringInstructions(boolean allowOneWrong, int numArtifactsScored, Artifact... spindexerSlots) {
        ArrayList<Integer> scoringOrder = getScoringOrder(allowOneWrong, numArtifactsScored, spindexerSlots);
        if (scoringOrder.isEmpty())
            return "Continue intaking";

        StringBuilder indices = new StringBuilder();
        for (int k : scoringOrder)
            indices.append(k).append(" ");

        StringBuilder colors = new StringBuilder();
        for (int i : scoringOrder)
            colors.append(spindexerSlots[i].name().charAt(0));

        return String.format(
                "Score slot%s %s(%s) for %d pts",

                scoringOrder.size() > 1 ? "s" : "",
                indices,
                colors,
                getScoreValue(scoringOrder, numArtifactsScored, spindexerSlots)
        );
    }

    public static void main(String[] args) {

        // edit these for testing:
        String classifierRamp = "PPG PGG ___";
        boolean allowOneWrong = true;

        System.out.println("Classifier: " + classifierRamp);

        Artifact[] artifacts = Artifact.values();

        int numArtifactsScored = classifierRamp.replace(" ", "").replace("_", "").length();

        for (Motif randomization : Motif.motifs) {
            System.out.printf("Randomization: %s%nEffective: %s%nSpindexer: %n", randomization, randomization.getEffectiveMotif(numArtifactsScored));
            for (Artifact first : artifacts) for (Artifact second : artifacts) for (Artifact third : artifacts) {
                System.out.printf("     %s%s%s --> %s%n",
                        first.name().replace("E", "_").charAt(0),
                        second.name().replace("E", "_").charAt(0),
                        third.name().replace("E", "_").charAt(0),
                        randomization.getScoringInstructions(allowOneWrong, numArtifactsScored, first, second, third)
                );
            }
        }
    }
}
