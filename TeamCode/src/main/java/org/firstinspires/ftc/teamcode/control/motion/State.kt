package org.firstinspires.ftc.teamcode.control.motion

data class State

@JvmOverloads
constructor(
    @JvmField var x: Double = 0.0,
    @JvmField var v: Double = 0.0,
    @JvmField var a: Double = 0.0,
    @JvmField var j: Double = 0.0,
) {

    operator fun plus(other: State): State {
        return State(
            x + other.x,
            v + other.v,
            a + other.a,
            j + other.j,
        )
    }

    operator fun unaryMinus(): State {
        return State(
            -x,
            -v,
            -a,
            -j,
        )
    }

    operator fun minus(other: State): State {
        return this + -other
    }

    operator fun times(scalar: Double): State {
        return State(
            x * scalar,
            v * scalar,
            a * scalar,
            j * scalar,
        )
    }

    operator fun plusAssign(other: State) {
        x += other.x
        v += other.v
        a += other.a
        j += other.j
    }

    operator fun minusAssign(other: State) {
        x -= other.x
        v -= other.v
        a -= other.a
        j -= other.j
    }

    fun negate() {
        x = -x
        v = -v
        a = -a
        j = -j
    }

    @JvmOverloads
    fun set(
        x: Double = this.x,
        v: Double = this.v,
        a: Double = this.a,
        j: Double = this.j,
    ): State {
        this.x = x
        this.v = v
        this.a = a
        this.j = j
        return this
    }

    fun set(other: State): State = set(other.x, other.v, other.a, other.j)
}
