package org.firstinspires.ftc.teamcode.control.gainmatrix

data class KalmanGains @JvmOverloads constructor(
    /**
     * model covariance
     */
    @JvmField var Q: Double = 0.1,
    /**
     * sensor covariance
     */
    @JvmField var R: Double = 0.4,
)
