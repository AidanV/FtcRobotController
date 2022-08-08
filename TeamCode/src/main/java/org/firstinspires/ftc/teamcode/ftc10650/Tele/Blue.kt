package org.firstinspires.ftc.teamcode.ftc10650.Tele

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Calculators.Interfaces
import org.firstinspires.ftc.teamcode.Op.ComplexOp
import org.firstinspires.ftc.teamcode.Utilities.Vector2D


@TeleOp(name = "Blue", group = "Blue")
class Blue : ComplexOp(){

    override fun startPositionAndOrientation(): Interfaces.MoveData.StartData
            = Interfaces.MoveData.StartData(Vector2D(0.0, 0.0), 0.0)



    override fun body() {
        ComplexMove(null, null, null)
    }
}