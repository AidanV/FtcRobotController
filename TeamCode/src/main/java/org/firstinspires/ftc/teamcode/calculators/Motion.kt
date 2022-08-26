package org.firstinspires.ftc.teamcode.calculators

import org.firstinspires.ftc.teamcode.utilities.Vector2D

data class Moving(val vector: Vector2D, val progress: Double)

fun moveTo (progress: Double): Moving {
    var vec = Vector2D()
    fun place(vector: Vector2D){
        vec = vector;
    }
    return Moving(Vector2D(0.0, 0.0), progress)
}

