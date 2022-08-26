package org.firstinspires.ftc.teamcode.ftc10650.auto.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData;
import org.firstinspires.ftc.teamcode.calculators.MotionCalcs;
import org.firstinspires.ftc.teamcode.calculators.OrientationCalcs;
import org.firstinspires.ftc.teamcode.calculators.OtherCalcs;
import org.firstinspires.ftc.teamcode.calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.op.ComplexOp;
import org.firstinspires.ftc.teamcode.utilities.Vector2D;

@Disabled
@Autonomous(name = "League Auto Blue", group = "Blue")
public class LeagueAutoBlue extends ComplexOp {

    @Override
    public MoveData.StartData startPositionAndOrientation() {
        return new MoveData.StartData(new Vector2D(200,200), -90);//60 degrees west of south is a good angle to see the barcode
    }

    @Override
    public void initMove() throws InterruptedException {
        //Lift lift to uncover camera

        //

    }

    @Override
    public void body() throws InterruptedException {

        //Grab Duck Position and save to data
//        ComplexMove(null, null, OrientationCalcs.lookToOrientation(180), OtherCalcs.TimeProgress(15000));
//        d.robot.IntakeCam.resumeViewport();
        ComplexMove(
            SpeedCalcs.SetProgressSpeed(
                new SpeedCalcs.ProgressSpeed(
                        0.05,
                        0.0,
                        SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                ),

                new SpeedCalcs.ProgressSpeed(
                        0.2,
                        0.5,
                        SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                ),

                new SpeedCalcs.ProgressSpeed(
                        0.05,
                        1.0,
                        SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                )
                ),


                MotionCalcs.PointMotion(
                        5,
                        new Vector2D(215, 200)
                ),


                OrientationCalcs.lookToOrientation(90),


                OtherCalcs.Lift(
                        d.cameraLiftPos,
                        0.25
                )
        );


        long currTime = System.currentTimeMillis();
//        d.robot.bar.setPosition(0.6);
//        while(System.currentTimeMillis()-currTime < 2000){
//            Thread.sleep(10);
////            d.robot.lift.setTargetPosition(d.cameraLiftPos-d.firstLiftPos);
////            d.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            d.robot.lift.setPower(0.2);
//        }

        d.duckPos = d.robot.duckSpotPipeline.getDuckPos();

        d.duckPos = (d.duckPos + 1) % 3;

        int initialTape = d.robot.tapeEx.getCurrentPosition();
//915 1485
        if(d.duckPos == 0){
            while(opModeIsActive() && d.robot.tapeEx.getCurrentPosition() < 310+initialTape){
                d.robot.base.setPosition(0.345);
                d.robot.height.setPosition(0.1329);
                d.robot.tapeEx.setPower(0.3);
                Thread.sleep(10);
            }
            d.robot.tapeEx.setPower(0.0);
            currTime = System.currentTimeMillis();
            while(opModeIsActive() && System.currentTimeMillis()-currTime < 1000){
                Thread.sleep(10);
            }
            currTime = System.currentTimeMillis();
            while(opModeIsActive() && System.currentTimeMillis()-currTime < 1000){
                d.robot.height.setPosition(0.06);
                Thread.sleep(10);
            }
            while(opModeIsActive() && d.robot.tapeEx.getCurrentPosition() > initialTape + 100){
//            d.robot.tapeEx.setTargetPosition(100+initialTape);
                d.robot.tapeEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                d.robot.tapeEx.setPower(-0.05);
                Thread.sleep(10);
            }
        }

        else if (d.duckPos == 1){
            while(opModeIsActive() && d.robot.tapeEx.getCurrentPosition()<255+initialTape){
                d.robot.base.setPosition(0.500);
                d.robot.height.setPosition(0.1288);
                d.robot.tapeEx.setPower(0.3);
                Thread.sleep(10);
            }
            d.robot.tapeEx.setPower(0.0);
            currTime = System.currentTimeMillis();
            while(opModeIsActive() && System.currentTimeMillis()-currTime < 1000){
                Thread.sleep(10);
            }
            currTime = System.currentTimeMillis();
            while(opModeIsActive() && System.currentTimeMillis()-currTime < 1000){
                d.robot.height.setPosition(0.045);
                Thread.sleep(10);
            }
            while(opModeIsActive() && d.robot.tapeEx.getCurrentPosition() > initialTape + 100){
//            d.robot.tapeEx.setTargetPosition(100+initialTape);
                d.robot.tapeEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                d.robot.tapeEx.setPower(-0.05);
                Thread.sleep(10);
            }
        } else if (d.duckPos == 2) {


            while(opModeIsActive() && d.robot.tapeEx.getCurrentPosition()<400+initialTape){
                d.robot.base.setPosition(0.64);
                d.robot.height.setPosition(0.1817);
                d.robot.tapeEx.setPower(0.3);
                Thread.sleep(10);
            }
            d.robot.tapeEx.setPower(0.0);
            currTime = System.currentTimeMillis();
            while(opModeIsActive() && System.currentTimeMillis()-currTime < 1000){
                Thread.sleep(10);
            }
            currTime = System.currentTimeMillis();
            while(opModeIsActive() && System.currentTimeMillis()-currTime < 1000){
                d.robot.height.setPosition(0.125);
                Thread.sleep(10);
            }
            while(opModeIsActive() && d.robot.tapeEx.getCurrentPosition() > initialTape + 100){
//            d.robot.tapeEx.setTargetPosition(100+initialTape);
                d.robot.tapeEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                d.robot.tapeEx.setPower(-0.05);
                Thread.sleep(10);
            }
//            d.robot.height.setPosition(0.18);
        }

//        d.robot.base.setPosition(0.5);
//        d.robot.height.setPosition(0.5);
//        currTime = System.currentTimeMillis();
//        while(opModeIsActive() && System.currentTimeMillis()-currTime < 500){
//            Thread.sleep(10);
//        }



        d.robot.tapeEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        d.robot.tapeEx.setPower(0.0);


        d.robot.base.setPosition(0.5);
        d.robot.height.setPosition(0.5);

//        d.duckPos = 2;




        if(d.duckPos == 0) {
            ComplexMove(
                    SpeedCalcs.SetProgressSpeed(
                            new SpeedCalcs.ProgressSpeed(
                                    0.1,
                                    0.0,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.3,
                                    0.2,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.3,
                                    0.8,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.1,
                                    1.0,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            )
                    ),


                    MotionCalcs.PointMotion(
                            5,
                            new Vector2D(235, 199.5)
                    ),


                    OrientationCalcs.lookToOrientation(0),


                    OtherCalcs.Lift(
                            d.bottomLiftPos,// CHANGED FROM BOTTOM
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
                                    0.1,
                                    0.0,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.3,
                                    0.2,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.3,
                                    0.8,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.1,
                                    1.0,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            )
                    ),

                    MotionCalcs.PointMotion(
                            5,
                            new Vector2D(235, 197.5)

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
                            d.middleLiftPos,
                            0.25
                    )
            );
            //Set lift position to middle / move to tree
                // move includes
                // turn back to south wall
                // move to north side of tree

        }



//         IF duck position is for the top goal or if we do not find the correct duck position,
//         we want to place the duck on the top level
        else {
            ComplexMove(
                    SpeedCalcs.SetProgressSpeed(
                            new SpeedCalcs.ProgressSpeed(
                                    0.1,
                                    0.0,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.3,
                                    0.2,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.3,
                                    0.8,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            ),

                            new SpeedCalcs.ProgressSpeed(
                                    0.1,
                                    1.0,
                                    SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                            )
                    ),

                    MotionCalcs.PointMotion(
                            5,
                            new Vector2D(235, 203),
                            new Vector2D(235, 198)
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
                            d.topLiftPos,
                            0.25
                    )
            );
            //Set lift position to top / move to tree
                // move includes
                // turn back to south wall
                // move to north side of tree

        }


        d.robot.bar.setPosition(d.gateOpen);

        currTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-currTime < 500){
            Thread.sleep(10);
            d.robot.bright.setPower(0);
            d.robot.fright.setPower(0);
            d.robot.bleft.setPower(0);
            d.robot.fleft.setPower(0);
        }

        d.robot.intake.setPower(0.4);

        currTime = System.currentTimeMillis();

        while(System.currentTimeMillis()-currTime < 2000){
            Thread.sleep(10);
            d.robot.intake.setPower(0.4);
            d.robot.bright.setPower(0);
            d.robot.fright.setPower(0);
            d.robot.bleft.setPower(0);
            d.robot.fleft.setPower(0);
        }

        d.robot.bar.setPosition(d.gateClose);
        d.robot.intake.setPower(0.0);
//
//
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
                        new Vector2D(205, 200)//,
//                        new Vector2D(150, 150)
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
        ComplexMove(
                SpeedCalcs.SetProgressSpeed(
                        new SpeedCalcs.ProgressSpeed(
                                0.4,
                                0.0,
                                SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                        ),

                        new SpeedCalcs.ProgressSpeed(
                                0.7,
                                0.2,
                                SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                        ),

                        new SpeedCalcs.ProgressSpeed(
                                0.7,
                                0.8,
                                SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                        ),

                        new SpeedCalcs.ProgressSpeed(
                                0.4,
                                1.0,
                                SpeedCalcs.ProgressSpeed.timeOrProg.PROG
                        )
                ),

                MotionCalcs.PointMotion(
                        5,
                        new Vector2D(205, 275)//,
//                        new Vector2D(150, 150)
                ),

                OrientationCalcs.lookToOrientation(0)
//                OrientationCalcs.spinToProgress(
//                        new OrientationCalcs.spinProgress(
//                                0.0,
//                                1.0,
//                                0
//                        )
//                ),
        );
//
//
//        // set lift to camera visible position
//        // drive into storage area / right side on ground / left side on barrier
//
//
//        // reset position with distance sensor
//
//
//        // find cube w/ vision / turn to cube //ignore for now
//        // lift lift to pick up position
//        // drive forward / intake | until vision detects cube
//        // lift lift to safe driving position
//        // drive over barrier w/ half over
//        // turn robot to back away from driver wall
//        // lift lift to top height
//        // deposit cube
//        // lift lift to safe driving position
//        // turn towards barrier
//        // drive over center part of barrier into warehouse
//
        d.robot.liftEx.setTargetPosition(5-d.firstLiftPos);

        while(opModeIsActive()){
            Thread.sleep(10);
            d.robot.bright.setPower(0);
            d.robot.fright.setPower(0);
            d.robot.bleft.setPower(0);
            d.robot.fleft.setPower(0);
        }
    }
}
