package org.firstinspires.ftc.teamcode.control.gainmatrix

import org.firstinspires.ftc.teamcode.control.motion.State

data class FullStateGains @JvmOverloads constructor(
    @JvmField var pGain: Double = 0.0,
    @JvmField var vGain: Double = 0.0,
    @JvmField var aGain: Double = 0.0,
) {


    operator fun times(state: State): State {
        return State(
            state.x * pGain,
            state.v * vGain,
            state.a * aGain,
        )
    }
}
