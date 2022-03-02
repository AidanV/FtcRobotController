package org.firstinspires.ftc.teamcode.ftc10650.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Calculators.Interfaces;
import org.firstinspires.ftc.teamcode.Calculators.MotionCalcs;
import org.firstinspires.ftc.teamcode.Calculators.OrientationCalcs;
import org.firstinspires.ftc.teamcode.Calculators.OtherCalcs;
import org.firstinspires.ftc.teamcode.Calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.Hardware.FreightRobotName_NA.RobotMap;
import org.firstinspires.ftc.teamcode.Op.ComplexOp;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;

@Autonomous(name = "Red Duck Storage Auto", group = "Red")
public class RedDuckStorageAuto extends ComplexOp {

    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(6.0 - (.353),1.0 + (.3142)), 90);
    }

    @Override
    public void body() throws InterruptedException {


        //drive to vision position
        ComplexMove(

                SpeedCalcs.SetSpeed(0.05),

                MotionCalcs.PointMotion(
                        5,
                        new Vector2D(5.5, 1.3142)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 1.0, -90)
                ),

                OtherCalcs.Lift(
                        d.cameraLiftPos,
                        0.25
                )
        );


        //wait 500 millis
        ComplexMove(null, null, null, OtherCalcs.TimeProgress(500));


        //grab duck position
        d.duckPos = d.robot.duckSpotPipeline.getDuckPos();


        //grab capstone
//        ComplexMove(null, null, null, OtherCalcs.AutoCupGrabBlue(false, 5000));


        //change pipeline
        d.robot.IntakeCam.setPipeline(d.robot.findDuckPipeline);


        //drive to alliance shipping hub
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.4, 0.3
                ),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(4.0, 1.4),
                        new Vector2D(4.0, 1.8)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.3, -180)
                ),

                OtherCalcs.Lift(
                        d.cubeLiftPositions[d.duckPos],
                        0.25
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
                        new Vector2D(4.1, 1.0),
                        new Vector2D(5.4, 0.6)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.1, 0.75, -360)
                ),

                OtherCalcs.Lift(
                        d.intakeLiftPos,
                        0.25
                )
        );


        //drive against carousel
        ComplexMove(

                SpeedCalcs.SetSpeed(0.1),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(5.5, 0.5)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.5, -360)
                ),

                OtherCalcs.StopAtStall(3.2, RobotMap.bleftEx)
        );


        //spin carousel
        ComplexMove(null, null, null, OtherCalcs.AutoDuckRed(4000));


        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.3, 0.3
                ),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(5, 0.5)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.8, -270)
                )
        );

        //pick up duck
        ComplexMove(

                SpeedCalcs.SetSpeed(0.2),

                MotionCalcs.DriveTowardsDuckRed(),

                OrientationCalcs.lookToOrientation(-270),

                OtherCalcs.IntakeDuck()
        );

        //drive to alliance shipping hub
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.4, 0.3
                ),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(4.1, 1.4),
                        new Vector2D(4.1, 1.8)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.6, -180)
                ),

                OtherCalcs.Lift(
                        d.topLiftPos,
                        0.25
                )
        );


        //place duck on shipping hub
        ComplexMove(null, null, null, OtherCalcs.AutoPlaceCube(2000));



        //park in storage unit
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.3, 0.3
                ),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(4.55, 0.4)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.5, -360)
                ),

                OtherCalcs.Lift(
                        d.intakeLiftPos,
                        0.25
                )
        );



    }
}
