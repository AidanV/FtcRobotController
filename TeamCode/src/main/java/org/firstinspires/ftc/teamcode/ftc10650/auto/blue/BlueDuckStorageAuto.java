package org.firstinspires.ftc.teamcode.ftc10650.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.calculators.Interfaces;
import org.firstinspires.ftc.teamcode.calculators.MotionCalcs;
import org.firstinspires.ftc.teamcode.calculators.OrientationCalcs;
import org.firstinspires.ftc.teamcode.calculators.OtherCalcs;
import org.firstinspires.ftc.teamcode.calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.hardware.FreightRobotName_NA.RobotMap;
import org.firstinspires.ftc.teamcode.op.ComplexOp;
import org.firstinspires.ftc.teamcode.utilities.Vector2D;

@Autonomous(name = "Blue Duck Storage Auto", group = "Blue")
public class BlueDuckStorageAuto extends ComplexOp {

    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(0.0 + (.353),1.0 + (.3142)), -90);
    }

    @Override
    public void body() throws InterruptedException {


        //drive to vision position
        ComplexMove(

                SpeedCalcs.SetSpeed(0.05),

                MotionCalcs.PointMotion(
                        5,
                        new Vector2D(0.5, 1.3142)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 1.0, 90)
                ),

                OtherCalcs.Lift(
                        d.cameraLiftPos,
                        0.5
                )
        );


        //wait 500 millis
        ComplexMove(null, null, null, OtherCalcs.TimeProgress(500));


        //grab duck position
        d.duckPos = d.robot.duckSpotPipeline.getDuckPos();

        //blue side fix
        d.duckPos = (d.duckPos + 1) % 3;

        //move tape out of way
        d.robot.base.setPosition(0.5);

        //change pipeline
        d.robot.IntakeCam.setPipeline(d.robot.findDuckPipeline);


        //drive to alliance shipping hub
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.4, 0.3
                ),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(1.9, 1.4),
                        new Vector2D(1.9, 2.0)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.3, 180)
                ),

                OtherCalcs.Lift(
                        d.cubeLiftPositions[d.duckPos],
                        0.5
                )
        );


        //place cube on shipping hub
        ComplexMove(null, null, null, OtherCalcs.AutoPlaceCube(1500));


        //drive near carousel
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.6, 0.4
                ),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(1.7, 1.0),
                        new Vector2D(0.6, 0.6)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.1, 0.75, 360)
                ),

                OtherCalcs.Lift(
                        d.intakeLiftPos,
                        0.5
                )
        );


        //drive against carousel
        ComplexMove(

                SpeedCalcs.SetSpeed(0.1),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(0.5, 0.5)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.5, 360)
                ),

                OtherCalcs.StopAtStall(3.2, RobotMap.brightEx)
        );


        //spin carousel
        ComplexMove(null, null, null, OtherCalcs.AutoDuckBlue(4000));


        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.3, 0.3
                ),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(1.0, 0.5)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.8, 270)
                )
        );

        //pick up duck
        ComplexMove(

                SpeedCalcs.SetSpeed(0.2),

                MotionCalcs.DriveTowardsDuckBlue(),

                OrientationCalcs.lookToOrientation(270),

                OtherCalcs.IntakeDuck()
        );

        //drive to alliance shipping hub
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.4, 0.3
                ),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(1.85, 1.6),
                        new Vector2D(1.85, 2.0)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.4, 180)
                ),

                OtherCalcs.HoldIntakePosition(),

                OtherCalcs.Lift(
                        d.topLiftPos,
                        0.5
                )
        );


        //place duck on shipping hub
        ComplexMove(null, null, null, OtherCalcs.AutoPlaceDuck(2000));


        //park in storage unit
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.3, 0.3
                ),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(1.45, 0.4)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.5, 360)
                ),

                OtherCalcs.Lift(
                        d.intakeLiftPos,
                        0.5
                )
        );
    }
}
