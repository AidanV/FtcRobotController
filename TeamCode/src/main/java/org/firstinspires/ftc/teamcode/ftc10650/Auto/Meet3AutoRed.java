package org.firstinspires.ftc.teamcode.ftc10650.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Calculators.Interfaces;
import org.firstinspires.ftc.teamcode.Calculators.Interfaces.MoveData;
import org.firstinspires.ftc.teamcode.Calculators.MotionCalcs;
import org.firstinspires.ftc.teamcode.Calculators.OrientationCalcs;
import org.firstinspires.ftc.teamcode.Calculators.OtherCalcs;
import org.firstinspires.ftc.teamcode.Calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.Op.ComplexOp;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;

import java.util.Vector;


@Autonomous(name = "Meet 3 Auto Red", group = "Red")
public class Meet3AutoRed extends ComplexOp {

    @Override
    public MoveData.StartData startPositionAndOrientation() {
        return new MoveData.StartData(new Vector2D(150,10), -270);//60 degrees west of south is a good angle to see the barcode
    }

    @Override
    public void initMove() throws InterruptedException {
        //Lift lift to uncover camera
        d.robot.lift.setTargetPosition(d.cameraLiftPos-d.firstLiftPos);
        d.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        d.robot.lift.setPower(0.2);
        //

    }

    @Override
    public void body() throws InterruptedException {

        //Grab Duck Position and save to data
//        ComplexMove(null, null, OrientationCalcs.lookToOrientation(180), OtherCalcs.TimeProgress(15000));
        d.duckPos = d.robot.duckSpotPipeline.getDuckPos();
        d.duckPos = 2;


        //ToDo find the encoder tick to drop in bottom/mid/top container


        if(d.duckPos == 0) {
            ComplexMove(
                    SpeedCalcs.SetProgressSpeed(
                            new SpeedCalcs.ProgressSpeed(
                                    0.2,
                                    0.0,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.5,
                                    0.2,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.5,
                                    0.8,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.2,
                                    1.0,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            )
                    ),

                    MotionCalcs.PointMotion(
                            5,
                            new Vector2D(150, 40) //northwest side of tree
                    ),

                    OrientationCalcs.lookToOrientation(0),

//                    OrientationCalcs.spinToProgress(
//                            new OrientationCalcs.spinProgress(
//                                    0.0,
//                                    1.0,
//                                    0
//                            )
//                    ),

                    OtherCalcs.Lift(
                            d.bottomLiftPos,
                            0.25
                    )
            );
            //Set lift position to bottom / move to tree
                // move includes
                // turn back to west wall
                // move to east side of tree

        }





        else if(d.duckPos == 1) {
            ComplexMove(
                    SpeedCalcs.SetProgressSpeed(
                            new SpeedCalcs.ProgressSpeed(
                                    0.2,
                                    0.0,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.5,
                                    0.2,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.5,
                                    0.8,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.2,
                                    1.0,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            )
                    ),

                    MotionCalcs.PointMotion(
                            5,
                            new Vector2D()
                    ),

                    OrientationCalcs.lookToOrientation(0),

//                    OrientationCalcs.spinToProgress(
//                            new OrientationCalcs.spinProgress(
//                                    0.0,
//                                    1.0,
//                                    90
//                            )
//                    ),

                    OtherCalcs.Lift(
                            d.bottomLiftPos,
                            0.25
                    )
            );
            //Set lift position to middle / move to tree
                // move includes
                // turn back to south wall
                // move to north side of tree

        }



        // IF duck position is for the top goal or if we do not find the correct duck position,
        // we want to place the duck on the top level
        else {
            ComplexMove(
                    SpeedCalcs.SetProgressSpeed(
                            new SpeedCalcs.ProgressSpeed(
                                    0.2,
                                    0.0,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.5,
                                    0.2,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.5,
                                    0.8,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.2,
                                    1.0,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            )
                    ),

                    MotionCalcs.PointMotion(
                            5,
                            new Vector2D(150, 40) // north side of tree
                    ),

                    OrientationCalcs.lookToOrientation(0),
//                    OrientationCalcs.spinToProgress(
//                            new OrientationCalcs.spinProgress(
//                                    0.0,
//                                    1.0,
//                                    90
//                            )
//                    ),

                    OtherCalcs.Lift(
                            d.bottomLiftPos,
                            0.25
                    )
            );
            //Set lift position to top / move to tree
                // move includes
                // turn back to south wall
                // move to north side of tree

        }




        ComplexMove(
                SpeedCalcs.SetProgressSpeed(
                        new SpeedCalcs.ProgressSpeed(
                                0.2,
                                0.0,
                                SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                        ),

                        new SpeedCalcs.ProgressSpeed(
                                0.5,
                                0.2,
                                SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                        ),

                        new SpeedCalcs.ProgressSpeed(
                                0.5,
                                0.8,
                                SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                        ),

                        new SpeedCalcs.ProgressSpeed(
                                0.2,
                                1.0,
                                SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                        )
                ),

                MotionCalcs.PointMotion(
                        5,
                        new Vector2D(150, 10),
                        new Vector2D(180, 10)
                ),

                OrientationCalcs.lookToOrientation(0),
//                OrientationCalcs.spinToProgress(
//                        new OrientationCalcs.spinProgress(
//                                0.0,
//                                1.0,
//                                0
//                        )
//                ),

                OtherCalcs.Lift(
                        d.safeLiftPos,
                        0.25
                )
        );
        // set lift to camera visible position
        // drive into storage area / right side on ground / left side on barrier


        // reset position with distance sensor


        // find cube w/ vision / turn to cube //ignore for now
        // lift lift to pick up position
        // drive forward / intake | until vision detects cube
        // lift lift to safe driving position
        // drive over barrier w/ half over
        // turn robot to back away from driver wall
        // lift lift to top height
        // deposit cube
        // lift lift to safe driving position
        // turn towards barrier
        // drive over center part of barrier into warehouse

        while(opModeIsActive()){
            Thread.sleep(10);
        }
    }
}
