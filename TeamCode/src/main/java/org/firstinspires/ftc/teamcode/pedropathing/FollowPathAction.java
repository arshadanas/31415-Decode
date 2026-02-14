package org.firstinspires.ftc.teamcode.pedropathing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public final class FollowPathAction implements Action {

    private final Follower follower;
    private final PathChain pathChain;
    private final boolean holdEnd;

    private boolean started = false;

    public FollowPathAction(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
    }

    public FollowPathAction(Follower follower, PathChain pathChain) {
        this(follower, pathChain, true);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket p) {
        if(!started) {
            follower.followPath(pathChain, holdEnd);
            started = true;
        }

        return follower.isBusy();
    }
}