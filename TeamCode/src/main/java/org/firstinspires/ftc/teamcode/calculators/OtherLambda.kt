package org.firstinspires.ftc.teamcode.calculators

import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData
import org.firstinspires.ftc.teamcode.calculators.Interfaces.OtherCalc

fun createOtherCalc(other: () -> Double): OtherCalc{
    return object : OtherCalc{
        var myProgress = 0.0
        override fun CalcOther(d: MoveData) {
            myProgress = other()
        }

        override fun myProgress(d: MoveData): Double {
            return myProgress
        }
    }
}
