//package org.firstinspires.ftc.teamcode.ftc10650.Auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.Calculators.Interfaces.MoveData;
//import org.firstinspires.ftc.teamcode.Calculators.MotionCalcs;
//import org.firstinspires.ftc.teamcode.Calculators.OrientationCalcs;
//import org.firstinspires.ftc.teamcode.Calculators.OtherCalcs;
//import org.firstinspires.ftc.teamcode.Calculators.SpeedCalcs;
//import org.firstinspires.ftc.teamcode.Op.ComplexOp;
//import org.firstinspires.ftc.teamcode.Utilities.Vector2D;
//
//import java.util.Vector;
//
//
//@Autonomous(name = "Meet 1 Auto", group = "ftc10650")
//public class Meet1Auto extends ComplexOp {
//
//    @Override
//    public MoveData.StartData startPositionAndOrientation() {
//        return new MoveData.StartData(new Vector2D(150,0), 90);
//    }
//
//    @Override
//    public void initMove() throws InterruptedException {
//        d.robot.grip.setPosition(.5);
////        s.add(new SpeedCalcs.ProgressSpeed(0.05, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG));
////        s.add(new SpeedCalcs.ProgressSpeed(0.2, 1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG));
////        p.add(new Vector3D(0, 0.3f, 0.3f));
//        d.robot.barm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        d.robot.tarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        while(d.robot.bop.getState()) {
//            d.robot.barm.setPower(-0.2);
//            d.robot.tarm.setPower(-0.08);
//        }
//        d.robot.barm.setPower(0.0);
//        d.robot.barm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        d.robot.barm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        d.initBarmPos = d.robot.barm.getCurrentPosition();
////        d.robot.barm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while(d.robot.top.getState()) {
//            d.robot.tarm.setPower(0.2);
//        }
//        d.robot.tarm.setPower(0.0);
//        d.robot.tarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        d.robot.tarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        d.initTarmPos = d.robot.tarm.getCurrentPosition();
//        d.initSarmPos = d.robot.sarm.getCurrentPosition();
//        d.initArmValid = true;
//        d.lastFrameBarmPos = 0.0;
//        d.lastFrameTarmPos = 0.0;
//        d.lastFrameSarmPos = 0.0;
//        d.firstFrameBarmPos = 0.0;
//        d.firstFrameTarmPos = 0.0;
//        d.firstFrameSarmPos = 0.0;
//        while(true){
//            d.arm.update();
//            d.telemetry.update();
//            if(d.arm.moveTowardTarget(-51.04, -0.120, .100, .1)) break;
//            Thread.sleep(10);
//        }
//        d.robot.claw.setPosition(0.55);
//
//
////        d.robot.tarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    }
//    //-51.04 sarm
//    //51.31 barm
//    //203.4 tarm
//    // x -0.1215 y .0951
//    @Override
//    public void body() throws InterruptedException {
//
//        d.duckPos = d.robot.duckSpotPipeline.getDuckPos();
//        d.robot.clawCam.setPipeline(d.robot.cubeFindPipeline);
////
//        if(d.duckPos == 0) {
//            while (opModeIsActive()){
//                if(d.arm.moveTowardTarget(0, -0.10, .40, .15)) break;
//                idle();
//            }
//            d.robot.claw.setPosition(0.8);
//            while (opModeIsActive()){
//                if(d.arm.moveTowardTarget(90, .31, -.03, .2)) break;
//                idle();
//            }
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.1, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2, 0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.1, 1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(150, 40)),
////                        new Vector2D(200, 50),
////                        new Vector2D(150, 0)),
//
//                    OrientationCalcs.lookToOrientation(0),
//                    OtherCalcs.Claw(),
//                    OtherCalcs.moveArm(90, .31, -.03, .2));
////                    OtherCalcs.moveArmOnDuck());
//        }
//        else if(d.duckPos == 1) {
//            while (opModeIsActive()){
//                if(d.arm.moveTowardTarget(0, -0.10, .40, .15)) break;
//                idle();
//            }
//            d.robot.claw.setPosition(0.8);
//            while (opModeIsActive()){
//                if(d.arm.moveTowardTarget(90, .31, .10, .2)) break;
//                idle();
//            }
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.1, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2, 0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.1, 1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(150, 40)),
////                        new Vector2D(200, 50),
////                        new Vector2D(150, 0)),
//
//                    OrientationCalcs.lookToOrientation(0),
//                    OtherCalcs.Claw(),
//                    OtherCalcs.moveArm(90, .31, .10, .2));
////                    OtherCalcs.moveArmOnDuck());
//        }
//        else if(d.duckPos == 2) {
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.1, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.2, 0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.1, 1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(150, 40)),
////                        new Vector2D(200, 50),
////                        new Vector2D(150, 0)),
//
//                    OrientationCalcs.lookToOrientation(0),
//                    OtherCalcs.Claw(),
//                    OtherCalcs.moveArm(-90, -.34, .33, .2));
////                    OtherCalcs.moveArmOnDuck());
//        }
////        ComplexMove(null, null, null, OtherCalcs.TimeProgress(1000));
//        d.arm.setArm2DVelocity(0, 0, 0);
////
//        d.robot.grip.setPosition(1.0);
//        Thread.sleep(1000);
//
////        ComplexMove(null, null, null, OtherCalcs.TimeProgress(1000));
//        ComplexMove(
//                SpeedCalcs.SetProgressSpeed(
//                        new SpeedCalcs.ProgressSpeed(0.1,0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.4,0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.4,0.9, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.1,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                MotionCalcs.PointMotion(5,
//                        new Vector2D(150, 0)),
////                        new Vector2D(200, 0)),
////                        new Vector2D(200, 50),
////                        new Vector2D(150, 0)),
//
//                OrientationCalcs.lookToOrientation(0),
////                OtherCalcs.moveArm(0.0, -0.010, .500, 0.2),
//                OtherCalcs.moveArm(0, .34, .27, .3),
//                OtherCalcs.Claw());
//        ComplexMove(
//                SpeedCalcs.SetProgressSpeed(
//                        new SpeedCalcs.ProgressSpeed(0.1,0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.1,0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.1,0.9, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.15,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                MotionCalcs.PointMotion(5,
////                        new Vector2D(150, 0),
//                        new Vector2D(210, 0)),
////                        new Vector2D(200, 50),
////                        new Vector2D(150, 0)),
//
//                OrientationCalcs.lookToOrientation(0),
////                OtherCalcs.moveArm(0.0, -0.010, .500, 0.2),
//                OtherCalcs.moveArm(0.0, .34, -.15, .4),
//                OtherCalcs.Claw());
////        d.arm.moveTowardTarget(0.0, .34, -.15, .3);
//        d.arm.setArm2DVelocity(0.0, 0.0, 0.0);
//        d.robot.grip.setPosition(1.0);
//        ComplexMove(
//                SpeedCalcs.SetProgressSpeed(
//                        new SpeedCalcs.ProgressSpeed(0.15,0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.15,0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.15,0.9, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.1,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                MotionCalcs.PointMotion(5,
//                        new Vector2D(290, 0)),
////                        new Vector2D(200, 50),
////                        new Vector2D(150, 0)),
//
//                OrientationCalcs.lookToOrientation(0),
//
////                OtherCalcs.Claw(),
////                OtherCalcs.Duck()
//                OtherCalcs.FindGC()
////                OtherCalcs.moveArm(0.0, -0.010, .500, 0.2),
////                OtherCalcs.moveArm(0, .34, -.15, .3),
//        );
//        d.robot.grip.setPosition(0.5);
//        Thread.sleep(500);
//        d.arm.setArm2DVelocity(0, 0, 0);
//        ComplexMove(
//                SpeedCalcs.SetProgressSpeed(
//                        new SpeedCalcs.ProgressSpeed(0.1,0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.4,0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.2,0.9, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.1,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                MotionCalcs.PointMotion(5,
//                        new Vector2D(150, 0),
//                        new Vector2D(150, 48)
//                ),
////                        new Vector2D(200, 50),
////                        new Vector2D(150, 0)),
//
//                OrientationCalcs.lookToOrientation(5),
//                OtherCalcs.moveArm(90, .34, .33, .15),
//                OtherCalcs.Claw()
////                OtherCalcs.moveArm(0.0, -0.010, .500, 0.2),
////                OtherCalcs.moveArm(0, .34, -.15, .3),
//        );
//        d.robot.grip.setPosition(1.0);
//        d.arm.setArm2DVelocity(0, 0, 0);
//        Thread.sleep(1000);
//        ComplexMove(
//                SpeedCalcs.SetProgressSpeed(
//                        new SpeedCalcs.ProgressSpeed(0.1,0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.2,0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.2,0.9, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.1,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                MotionCalcs.PointMotion(5,
//                        new Vector2D(150, 30)
//                ),
//                null);
//        if(d.duckPos == 2) {
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.1, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.1, 0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.1, 0.9, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.15, 1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                    MotionCalcs.PointMotion(5,
////                        new Vector2D(150, 0),
//                            new Vector2D(150, 0)),
////                        new Vector2D(200, 50),
////                        new Vector2D(150, 0)),
//
//                    OrientationCalcs.lookToOrientation(0),
////                OtherCalcs.moveArm(0.0, -0.010, .500, 0.2),
//                    OtherCalcs.moveArm(0.0, .34, -.15, .4),
//                    OtherCalcs.Claw());
////        d.arm.moveTowardTarget(0.0, .34, -.15, .3);
//            d.arm.setArm2DVelocity(0.0, 0.0, 0.0);
//            d.robot.grip.setPosition(1.0);
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.15, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.15, 0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.15, 0.9, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.1, 1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                    MotionCalcs.PointMotion(5,
//                            new Vector2D(290, 0)),
////                        new Vector2D(200, 50),
////                        new Vector2D(150, 0)),
//
//                    OrientationCalcs.lookToOrientation(0),
//
////                OtherCalcs.Claw(),
////                OtherCalcs.Duck()
//                    OtherCalcs.FindGC()
////                OtherCalcs.moveArm(0.0, -0.010, .500, 0.2),
////                OtherCalcs.moveArm(0, .34, -.15, .3),
//            );
//            d.robot.grip.setPosition(0.5);
//            Thread.sleep(500);
//        } else {
//            ComplexMove(
//                    SpeedCalcs.SetProgressSpeed(
//                            new SpeedCalcs.ProgressSpeed(0.1, 0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.4, 0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.3, 0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.7, 0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.7, 0.95, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                            new SpeedCalcs.ProgressSpeed(0.1, 1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                    MotionCalcs.PointMotion(5,
////                        new Vector2D(150, 0),
//                            new Vector2D(150, 0),
//                            new Vector2D(275, 0)),
////                        new Vector2D(200, 50),
////                        new Vector2D(150, 0)),
//
//                    OrientationCalcs.lookToOrientation(0),
////                OtherCalcs.moveArm(0.0, -0.010, .500, 0.2),
//
//                    OtherCalcs.Claw());
//        }
//
//
////        ComplexMove(
////                SpeedCalcs.SetProgressSpeed(
////                        new SpeedCalcs.ProgressSpeed(0.2,0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(0.2,0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(0.1,0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(0.1,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
////                MotionCalcs.PointMotion(5,
////                        new Vector2D(170, 20)
////                ),
//////                        new Vector2D(200, 50),
//////                        new Vector2D(150, 0)),
////
////                OrientationCalcs.spinToProgress(new OrientationCalcs.spinProgress(0.0, 1.0, 135)),
//////                OrientationCalcs.lookToOrientation(135),
//////                OtherCalcs.moveArm(90, .34, .33, .15),
////                OtherCalcs.Claw(),
//////                OtherCalcs.moveArm(0.0, -0.010, .500, 0.2),
////                OtherCalcs.moveArm(0, .34, -.15, .3)
////        );
////        d.arm.setArm2DVelocity(0, 0, 0);
////        ComplexMove(
////                SpeedCalcs.SetProgressSpeed(
////                        new SpeedCalcs.ProgressSpeed(0.1,0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(0.2,0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(0.2,0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(0.1,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
////                MotionCalcs.PointMotion(5,
////                        new Vector2D(50, 50)
////                ),
//////                        new Vector2D(200, 50),
//////                        new Vector2D(150, 0)),
////
//////                OrientationCalcs.spinToProgress(new OrientationCalcs.spinProgress(0.0, 1.0, 135)),
////                OrientationCalcs.lookToOrientation(135),
//////                OtherCalcs.moveArm(90, .34, .33, .15),
////                OtherCalcs.Claw(),
////                OtherCalcs.FindD()
//////                OtherCalcs.moveArm(0.0, -0.010, .500, 0.2),
////        );
////        d.arm.setArm2DVelocity(0, 0, 0);
////        Thread.sleep(500);
////        ComplexMove(
////                SpeedCalcs.SetProgressSpeed(
////                        new SpeedCalcs.ProgressSpeed(0.1,0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(0.4,0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(0.4,0.1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
////                        new SpeedCalcs.ProgressSpeed(0.2,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
////                MotionCalcs.PointMotion(5,
////                        new Vector2D(150, 45)
////                ),
//////                        new Vector2D(200, 50),
//////                        new Vector2D(150, 0)),
////
////                OrientationCalcs.spinToProgress(new OrientationCalcs.spinProgress(0.0, 1.0, 0)),
////                OtherCalcs.moveArm(90, .34, .33, .15),
////                OtherCalcs.Claw()
//////                OtherCalcs.moveArm(0.0, -0.010, .500, 0.2),
//////                OtherCalcs.moveArm(0, .34, -.15, .3),
////        );
////        d.robot.grip.setPosition(1.0);
////        d.arm.setArm2DVelocity(0, 0, 0);
//
//
//
//////        Thread.sleep(1000);
//
//
//    }
//}
