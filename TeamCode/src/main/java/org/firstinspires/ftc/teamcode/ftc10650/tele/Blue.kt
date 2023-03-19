package org.firstinspires.ftc.teamcode.ftc10650.tele

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.calculators.*
import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData.StartData
import org.firstinspires.ftc.teamcode.hardware.FreightRobotName_NA.RobotMap
import org.firstinspires.ftc.teamcode.op.ComplexOp
import org.firstinspires.ftc.teamcode.utilities.Vector2D
import kotlin.jvm.Throws



@TeleOp(name = "Blue Kotlin", group = "Blue")
class Blue : ComplexOp() {
    override fun startPositionAndOrientation(): StartData {
        return StartData(Vector2D(1.0, 3.0), 0.0)
    }

    @Throws(InterruptedException::class)
    override fun body() {

        RobotMap.IntakeCam.setPipeline(d.robot.intakedPipeline)
        ComplexMove(
                SpeedCalcs.JoystickSpeed(),
                MotionCalcs.FieldCentricJoystick(-90.0),
                OrientationCalcs.GameOrientBlue(),
                OtherCalcs.TeleLift(),
                OtherCalcs.Intake(),
                OtherCalcs.TeleCap(),
                OtherCalcs.TelemetryPosition(),
                createOtherCalc {
                    RobotMap.duck.power = (d.driver.rt() - d.driver.lt()) / 2.0
                    0.0
                }
        )
    }
}

