package org.firstinspires.ftc.teamcode.ftc10650.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.calculators.Interfaces;
import org.firstinspires.ftc.teamcode.calculators.MotionCalcs;
import org.firstinspires.ftc.teamcode.calculators.OrientationCalcs;
import org.firstinspires.ftc.teamcode.calculators.OtherCalcs;
import org.firstinspires.ftc.teamcode.calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.op.ComplexOp;
import org.firstinspires.ftc.teamcode.utilities.Vector2D;

@Autonomous(name = "Blue Warehouse Auto", group = "Blue")
public class BlueWarehouseAuto extends ComplexOp {

    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(0.0 + (.353),3.0 + (.3142)), -90);
    }

    @Override
    public void body() throws InterruptedException {


        //drive to vision position
        ComplexMove(

                SpeedCalcs.SetSpeed(0.05),

                MotionCalcs.PointMotion(
                        5,
                        new Vector2D(0.5, 3.3142)
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


        //grab capstone
        ComplexMove(null, null, null, OtherCalcs.AutoCupGrabBlue(true, 4500));


        //set vision to cube pickup
        d.robot.IntakeCam.setPipeline(d.robot.intakedPipeline);


        //drive to alliance shipping hub
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.4, 0.3
                ),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(2.0, 3.3)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.6, 0)
                ),

                OtherCalcs.Lift(
                        d.cubeLiftPositions[d.duckPos],
                        0.5
                )
        );


        //place cube on shipping hub
        ComplexMove(null, null, null, OtherCalcs.AutoPlaceCube(2000));


        //drive into warehouse
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.6, 0.2
                ),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(1.3, 3.3),
                        new Vector2D(1.3, 5.2)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.5, 0)
                ),

                OtherCalcs.Lift(
                        d.safeLiftPos,
                        0.25
                )
        );


        //lower lift to reset position and reset y position
        ComplexMove(null, null, null,

                OtherCalcs.Lift(
                        d.intakeLiftPos,
                        0.25
                ),

                OtherCalcs.ResetYPosition(),

                OtherCalcs.TimeProgress(2000)
        );


        //drive until intake
        ComplexMove(

                SpeedCalcs.SetSpeed(0.1),

                MotionCalcs.PointMotion(
                        0.01,
                        new Vector2D(0.25, 5.75)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.25, -45)
                ),

                OtherCalcs.StopAtIntake()
        );


        //drive out of warehouse
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.6, 0.2
                ),

                MotionCalcs.PointMotion(
                        0.01,
                        new Vector2D(1.3, 5.2),
                        new Vector2D(1.3, 3.3)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.3, 0)
                )
        );


        //drive to alliance shipping hub
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.4, 0.4
                ),

                MotionCalcs.PointMotion(
                        0.01,
                        new Vector2D(2.1, 3.1)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.5, 0)
                ),

                OtherCalcs.Lift(
                        d.topLiftPos,
                        0.25
                )
        );


        //place cube on shipping hub
        ComplexMove(null, null, null, OtherCalcs.AutoPlaceCube(2000));


        //drive into warehouse
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 0.6, 0.2
                ),

                MotionCalcs.PointMotion(
                        0.1,
                        new Vector2D(1.3, 3.3),
                        new Vector2D(1.3, 5.2)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.5, 0)
                ),

                OtherCalcs.Lift(
                        d.safeLiftPos,
                        0.25
                )
        );


        //lower lift to reset position
        ComplexMove(null, null, null,

                OtherCalcs.Lift(
                        d.intakeLiftPos,
                        0.25
                ),

                OtherCalcs.TimeProgress(2000)
        );
    }
}