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

@Autonomous(name = "Blue Duck Storage Auto", group = "Blue")
class BlueAuto : ComplexOp() {
    override fun startPositionAndOrientation(): StartData =
            StartData(Vector2D(0.0 + .353, 1.0 + .3142), -90.0)


    @Throws(InterruptedException::class)
    override fun body() {


        //drive to vision position
        ComplexMove(
                SpeedCalcs.SetSpeed(0.05),
                MotionCalcs.PointMotion(5.0,
                        Vector2D(0.5, 1.3142)
                ),
                OrientationCalcs.spinToProgress(
                        spinProgress(0.0, 1.0, 90.0)
                ),
                OtherCalcs.Lift(
                        d.cameraLiftPos,
                        0.5
                )
        )


        //wait 500 millis
        ComplexMove(null, null, null, OtherCalcs.TimeProgress(500.0))


        //grab duck position
        d.duckPos = d.robot.duckSpotPipeline.duckPos

        //blue side fix
        d.duckPos = (d.duckPos + 1) % 3

        //move tape out of way
        RobotMap.base.position = 0.5

        //change pipeline
        RobotMap.IntakeCam.setPipeline(d.robot.findDuckPipeline)


        //drive to alliance shipping hub
        ComplexMove(
                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.4, 0.3
                ),
                MotionCalcs.PointMotion(
                        0.1,
                        Vector2D(1.9, 1.4),
                        Vector2D(1.9, 2.0)
                ),
                OrientationCalcs.spinToProgress(
                        spinProgress(0.0, 0.3, 180.0)
                ),
                OtherCalcs.Lift(
                        d.cubeLiftPositions[d.duckPos],
                        0.5
                )
        )


        //place cube on shipping hub
        val startTime = System.currentTimeMillis()
        ComplexMove(null, null, null,
                createOtherCalc {
                    val runTime = 1500.0
                    RobotMap.intake.mode = DcMotor.RunMode.RUN_USING_ENCODER
                    RobotMap.bar.position = MoveData.gateOpen

                    val myProgress = (System.currentTimeMillis() - startTime) / runTime

                    if (d.progress >= 1.0) {
                        RobotMap.bar.position = MoveData.gateClose
                        RobotMap.intake.setPower(0.0)
                    } else if (d.progress > 0.1) {
                        RobotMap.intake.setPower(.25)
                    }
                    myProgress
                }
        )
                //OtherCalcs.AutoPlaceCube(1500.0))


        //drive near carousel
        ComplexMove(
                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.6, 0.4
                ),
                MotionCalcs.PointMotion(
                        0.1,
                        Vector2D(1.7, 1.0),
                        Vector2D(0.6, 0.6)
                ),
                OrientationCalcs.spinToProgress(
                        spinProgress(0.1, 0.75, 360.0)
                ),
                OtherCalcs.Lift(
                        d.intakeLiftPos,
                        0.5
                )
        )


        //drive against carousel
        ComplexMove(
                SpeedCalcs.SetSpeed(0.1),
                MotionCalcs.PointMotion(
                        0.1,
                        Vector2D(0.5, 0.5)
                ),
                OrientationCalcs.spinToProgress(
                        spinProgress(0.0, 0.5, 360.0)
                ),
                OtherCalcs.StopAtStall(3.2, RobotMap.brightEx)
        )


        //spin carousel
        ComplexMove(null, null, null, OtherCalcs.AutoDuckBlue(4000.0))
        ComplexMove(
                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.3, 0.3
                ),
                MotionCalcs.PointMotion(
                        0.1,
                        Vector2D(1.0, 0.5)
                ),
                OrientationCalcs.spinToProgress(
                        spinProgress(0.0, 0.8, 270.0)
                )
        )

        //pick up duck
        ComplexMove(
                SpeedCalcs.SetSpeed(0.2),
                MotionCalcs.DriveTowardsDuckBlue(),
                OrientationCalcs.lookToOrientation(270.0),
                OtherCalcs.IntakeDuck()
        )

        //drive to alliance shipping hub
        ComplexMove(
                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.4, 0.3
                ),
                MotionCalcs.PointMotion(
                        0.1,
                        Vector2D(1.85, 1.6),
                        Vector2D(1.85, 2.0)
                ),
                OrientationCalcs.spinToProgress(
                        spinProgress(0.0, 0.4, 180.0)
                ),
                OtherCalcs.HoldIntakePosition(),
                OtherCalcs.Lift(
                        d.topLiftPos,
                        0.5
                )
        )


        //place duck on shipping hub
        ComplexMove(null, null, null, OtherCalcs.AutoPlaceDuck(2000.0))


        //park in storage unit
        ComplexMove(
                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.3, 0.3
                ),
                MotionCalcs.PointMotion(
                        0.1,
                        Vector2D(1.45, 0.4)
                ),
                OrientationCalcs.spinToProgress(
                        spinProgress(0.0, 0.5, 360.0)
                ),
                OtherCalcs.Lift(
                        d.intakeLiftPos,
                        0.5
                )
        )
    }
}
