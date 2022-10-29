package org.firstinspires.ftc.teamcode.ftc10650.auto.blue

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.calculators.*
import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData
import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData.StartData
import org.firstinspires.ftc.teamcode.calculators.OrientationCalcs.spinProgress
import org.firstinspires.ftc.teamcode.hardware.FreightRobotName_NA.RobotMap
import org.firstinspires.ftc.teamcode.op.ComplexOp
import org.firstinspires.ftc.teamcode.utilities.Vector2D
import kotlin.jvm.Throws

@Autonomous(name = "Your first auto", group = "Blue")
class ExampleBlueAuto : ComplexOp() {
    override fun startPositionAndOrientation(): StartData =
            StartData(Vector2D(0.0 + .353, 1.0 + .3142), -90.0)


    @Throws(InterruptedException::class)
    override fun body() {

        ComplexMove(
                SpeedCalcs.SetSpeed(0.3),

                MotionCalcs.PointMotion(0.0,
                        Vector2D(1.0, 1.0),
                        Vector2D(2.0, 1.0),
                ),

                OrientationCalcs.spinToProgress(
                        spinProgress(0.1, 0.9, 0.0)
                ),
        )
    }
}
