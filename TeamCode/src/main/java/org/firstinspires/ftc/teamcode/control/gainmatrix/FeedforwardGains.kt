package org.firstinspires.ftc.teamcode.control.gainmatrix

import org.firstinspires.ftc.teamcode.control.motion.State

data class FeedforwardGains

@JvmOverloads constructor(
    @JvmField var kV: Double = 0.0,
    @JvmField var kA: Double = 0.0,
    @JvmField var kStatic: Double = 0.0,
) {
    operator fun times(state: State): State {
        return State(
            0.0,
            state.v * kV,
            state.a * kA,
        )
    }
}
