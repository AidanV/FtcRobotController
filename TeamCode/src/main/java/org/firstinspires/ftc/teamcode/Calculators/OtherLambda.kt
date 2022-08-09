package org.firstinspires.ftc.teamcode.Calculators

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Calculators.Interfaces.MoveData
import org.firstinspires.ftc.teamcode.Calculators.Interfaces.OtherCalc
import org.firstinspires.ftc.teamcode.Hardware.FreightRobotName_NA.RobotMap

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