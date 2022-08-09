package org.firstinspires.ftc.teamcode.Op

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Calculators.Interfaces.MoveData.StartData
import org.firstinspires.ftc.teamcode.Calculators.Moving

abstract class Op : LinearOpMode() {



    var progress = 0.0

    fun SMOO(
            movement: (Double) -> Moving) {

        val moving = movement(progress)
        progress = moving.progress
    }


    abstract fun startPositionAndOrientation(): StartData?

    @Throws(InterruptedException::class)
    abstract fun body()


    override fun runOpMode() {

    }
}
