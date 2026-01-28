package org.firstinspires.ftc.teamcode.control.gainmatrix

data class KalmanGains @JvmOverloads constructor(
    @JvmField var Q: Double = 0.1,
    @JvmField var R: Double = 0.4,
)
