package org.firstinspires.ftc.teamcode.Calculators;

//import org.firstinspires.ftc.teamcode.Hardware.Sensors.GoalPositionPipeline;
//import org.firstinspires.ftc.teamcode.Hardware.Sensors.PowerShotPositionPipeline;
//import org.firstinspires.ftc.teamcode.Hardware.Sensors.StackDeterminationPipeline;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utilities.*;

        import java.util.Vector;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;

public class OtherCalcs {

    public static Interfaces.OtherCalc whileOpMode(){

        return new Interfaces.OtherCalc(){
            double myProgress;
            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }

            @Override
            public void CalcOther(Interfaces.MoveData d){
                myProgress = 0.5;
            }
        };
    }

    public static Interfaces.OtherCalc TimeProgress(final double millis){
        return new Interfaces.OtherCalc() {
            final long initialMillis = System.currentTimeMillis();
            @Override
            public void CalcOther(Interfaces.MoveData d) {

            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return (System.currentTimeMillis() - initialMillis)/(millis);
            }
        };
    }

//    public static Interfaces.OtherCalc moveArmOnDuck(){
//        return new Interfaces.OtherCalc() {
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//
//                d.arm.moveTowardTarget(-90, -.34, .27, .2);
////                switch(d.duckPos){
////                    case 0:
////                        d.arm.moveTowardTarget(-90, -.31, -.04, .2);
////                        break;
////                    case 1:
////                        d.arm.moveTowardTarget(-90, -.31, .10, .2);
////                        break;
////                    case 2:
////                        d.arm.moveTowardTarget(-90, -.34, .27, .2);
////                        break;
////                }
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }
//    public static Interfaces.OtherCalc moveArm(double theta, double x, double y, double speed){
//        return new Interfaces.OtherCalc() {
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                d.arm.moveTowardTarget(theta, x, y, speed);
////                d.arm.moveTowardTarget(-90, -.34, .27, .2);
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }
    public static Interfaces.OtherCalc Lift(final int setPosition, final double setPower){
        return new Interfaces.OtherCalc() {
            @Override
            public void CalcOther(Interfaces.MoveData d) {

                d.robot.lift.setTargetPosition(setPosition - d.firstLiftPos);
                d.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.robot.lift.setPower(setPower);
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }
        };
    }

    public static Interfaces.OtherCalc AutoPlaceCube(double outtakeTimeMillis){
        return new Interfaces.OtherCalc() {
            final long startTime = System.currentTimeMillis();
            double myProgress = 0.0;
            @Override
            public void CalcOther(Interfaces.MoveData d) {

                myProgress = (System.currentTimeMillis()-startTime)/outtakeTimeMillis;

                d.robot.bar.setPosition(d.gateOpen);

                if(myProgress >= 1.0) {
                    d.robot.bar.setPosition(d.gateClose);
                    d.robot.intake.setPower(0.0);
                } else if(myProgress > 0.1){
                    d.robot.intake.setPower(0.4);
                }
             }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }
        };
    }

    public static Interfaces.OtherCalc AutoDuckBlue(double totalTimeMillis){

        return new Interfaces.OtherCalc() {
            final long startTime = System.currentTimeMillis();
            double myProgress = 0.0;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                myProgress = (System.currentTimeMillis()-startTime)/totalTimeMillis;
                d.robot.duck.setPower(0.3);
                if(myProgress >= 1.0){
                    d.robot.duck.setPower(0.0);
                }
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }
        };
    }

    public static Interfaces.OtherCalc StopAtStall(double amps, DcMotorEx motor){
        return new Interfaces.OtherCalc() {
            double myProgress = 0.0;
            double averageCurrent = 0.0;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                averageCurrent = averageCurrent * 0.8 + motor.getCurrent(CurrentUnit.AMPS) * 0.2;
                if(averageCurrent > amps) myProgress = 1.0;
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }
        };
    }

    public static Interfaces.OtherCalc AutoCupGrabBlue(double totalTimeMillis){
        return new Interfaces.OtherCalc() {
            final long startTime = System.currentTimeMillis();
            double myProgress = 0.0;
            boolean hover = false;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                myProgress = (System.currentTimeMillis()-startTime)/totalTimeMillis;
                if(myProgress < 0.2) {
                    if (d.duckPos == 0) {
                        d.robot.base.setPosition(0.375);
                        d.robot.height.setPosition(0.2654);

                        //437 out
                        //base 0.3651
                        //height 0.2654
                    } else if (d.duckPos == 1) {
                        d.robot.base.setPosition(0.4849);
                        d.robot.height.setPosition(0.2830);

                        //372 out
                        //base 0.4849
                        //height 0.2830
                    } else if (d.duckPos == 2) {
                        d.robot.base.setPosition(0.5925);
                        d.robot.height.setPosition(0.2867);

                        //434 out
                        //base 0.6080
                        //height 0.2867
                    }
                } else if (myProgress < 0.4){
                    if (d.duckPos == 0) {
                        if (d.robot.tapeEx.getCurrentPosition() < 440){
                            d.robot.tapeEx.setPower(0.2);
                        } else {
                            d.robot.tapeEx.setPower(0.0);

                        }
                        //437 out
                    } else if (d.duckPos == 1) {
                        if (d.robot.tapeEx.getCurrentPosition() < 400){
                            d.robot.tapeEx.setPower(0.2);
                        } else {
                            d.robot.tapeEx.setPower(0.0);

                        }
                        //372 out
                    } else if (d.duckPos == 2) {
                        if (d.robot.tapeEx.getCurrentPosition() < 480){
                            d.robot.tapeEx.setPower(0.2);
                        } else {
                            d.robot.tapeEx.setPower(0.0);

                        }
                        //434 out
                    }
                } else if (myProgress < 0.6){
                    d.robot.height.setPosition(0.15);
                } else if (myProgress < 0.7){
                    d.robot.height.setPosition(0.5);
                }

                else {
                    d.robot.tapeEx.setPower(-0.1);
                    if(d.robot.tapeEx.getCurrentPosition() < 75){
                        d.robot.tapeEx.setPower(0.0);
                        d.robot.base.setPosition(0.5);
                        myProgress = 1.0;
                    }
                }
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }
        };
    }



    public static Interfaces.OtherCalc TeleLift(){

        return new Interfaces.OtherCalc() {
            boolean canRelease = false;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
//                d.robot.liftEx.setPower(d.manip.ls().y/5.0);
                if(d.manip.u()) {
                    d.robot.lift.setTargetPosition(1585-d.firstLiftPos);//1598 max
                    d.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.robot.lift.setPower(0.75);
                } else if(d.manip.d()){
                    d.robot.lift.setTargetPosition(15-d.firstLiftPos);//0 min
                    d.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.robot.lift.setPower(0.75);

                } else if(d.manip.r()){
                    d.robot.lift.setTargetPosition(150-d.firstLiftPos);
                    d.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.robot.lift.setPower(0.75);

                } else if(d.manip.l()){
                    d.robot.lift.setTargetPosition(700-d.firstLiftPos);
                    d.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.robot.lift.setPower(0.75);
                } else if((d.manip.rt()>0.001 || d.manip.rb()) && d.robot.intakedPipeline.isIntaked()){
                    d.robot.lift.setTargetPosition(700-d.firstLiftPos);
                    d.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.robot.lift.setPower(0.75);
                }

                if(d.manip.y()){
                    d.robot.lift.setPower(-0.1);
                    d.firstLiftPos = d.robot.lift.getCurrentPosition();
                    canRelease = true;
                } else if (canRelease) {
                    d.robot.lift.setPower(0.0);
                    canRelease = false;
                }
                d.telemetry.addData("lift Pos", d.robot.lift.getCurrentPosition() - d.firstLiftPos);
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }
        };
    }

    public static Interfaces.OtherCalc TeleCap(){

        return new Interfaces.OtherCalc() {
            double basePosition = 0.5;
            double heightPosition = 0.5;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                d.telemetry.addData("tape current position", d.robot.tapeEx.getCurrentPosition());
                basePosition += d.manip.rs().x * 0.003;
                heightPosition += d.manip.rs().y * 0.01;
                d.robot.base.setPosition(basePosition);
                d.robot.height.setPosition(heightPosition);

                if(basePosition > 1.0){
                    basePosition = 1.0;
                } else if (basePosition < -1.0) {
                    basePosition = -1.0;
                }

                if(heightPosition > 1.0){
                    heightPosition = 1.0;
                } else if (heightPosition < -1.0) {
                    heightPosition = -1.0;
                }
                d.robot.tapeEx.setPower(Math.signum(d.manip.ls().y)*Math.sqrt(Math.abs(-d.manip.ls().y))/3.0);
//                d.robot.tapeEx.setPower(d.manip.ls().x*0.15);
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }
        };
    }

    public static Interfaces.OtherCalc TestCapstone(){

        return new Interfaces.OtherCalc() {
            double basePosition = 0.5;
            double heightPosition = 0.5;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                d.telemetry.addData("base position", basePosition);
                d.telemetry.addData("height position", heightPosition);
                d.telemetry.addData("tape current position", d.robot.tapeEx.getCurrentPosition());
                basePosition += d.manip.rs().x * 0.003;
                heightPosition += d.manip.rs().y * 0.01;
                d.robot.base.setPosition(basePosition);
                d.robot.height.setPosition(heightPosition);

                if(basePosition > 1.0){
                    basePosition = 1.0;
                } else if (basePosition < -1.0) {
                    basePosition = -1.0;
                }

                if(heightPosition > 1.0){
                    heightPosition = 1.0;
                } else if (heightPosition < -1.0) {
                    heightPosition = -1.0;
                }
                d.robot.tapeEx.setPower(Math.signum(d.manip.ls().y)*Math.sqrt(Math.abs(-d.manip.ls().y))/3.0);
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }
        };
    }


    public static Interfaces.OtherCalc ChangeTapePIDF(){

        return new Interfaces.OtherCalc() {
            int currentMode = 0;
            boolean aReleased = true;
            double pVal = 1;
            double iVal = 0;
            double dVal = -10;
            double fVal = 20;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                double modValue = 0.0;
                if(d.driver.a() && aReleased){
                    currentMode++;
                    aReleased = false;
                } else {
                    aReleased = true;
                }
                if(d.driver.u()){
                    modValue = 0.25;
                } else if (d.driver.d()){
                    modValue = -0.25;
                }

                if(currentMode == 0){
                    pVal += modValue;
                } else if (currentMode == 1){
                    iVal +=modValue;
                } else if (currentMode == 2){
                    dVal +=modValue;
                } else {
                    fVal +=modValue;
                }
                PIDFCoefficients pidTape = new PIDFCoefficients(pVal, iVal, dVal, fVal);
                currentMode %= 4;
                d.robot.tapeEx.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidTape);
                d.telemetry.addData("PIDF val tape", "%f, %f, %f, %f", pVal, iVal, dVal, fVal);
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }
        };
    }

    public static Interfaces.OtherCalc Intake(){
        return new Interfaces.OtherCalc() {
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                if(d.manip.rb() && !d.robot.intakedPipeline.isIntaked() && d.robot.lift.getCurrentPosition() < 50){
                    d.robot.intake.setPower(0.6);
                } else {
                    d.robot.intake.setPower((d.robot.intakedPipeline.isIntaked()?0.0:d.manip.rt()) - d.manip.lt());

                }

                if(d.manip.b()){
                    d.robot.bar.setPosition(d.gateOpen);
                    d.robot.intake.setPower(0.3);
                } else {
                    d.robot.bar.setPosition(d.gateClose);
                }
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }
        };
    }



//    public static Interfaces.OtherCalc moveArmToTele(){
//        return new Interfaces.OtherCalc() {
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                if(d.manip.y()){
//                    d.arm.moveTowardTarget(90, .34, .33, .3);
//                } else if (d.manip.u()){
//                    d.arm.moveTowardTarget(0, .31, -.15, .3);
//                } else if (d.manip.d()){
//                    d.arm.moveTowardTarget(0, .33, -.03, .3);
//                }
//                else {
//                    d.robot.barm.setPower(0.0);
//                    d.robot.tarm.setPower(0.0);
//                    d.robot.sarm.setPower(0.0);
//                }
//
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }
//    public static Interfaces.OtherCalc changeOdometryCovariance(){
//        return new Interfaces.OtherCalc() {
//            private float covariance;
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                if(d.driver.a()){
//                    covariance = 1.0f;
//                    d.robot.setOdometryCovariance(covariance);
//                } else if (d.driver.b()){
//                    covariance = 0.0f;
//                    d.robot.setOdometryCovariance(covariance);
//                }
//                d.telemetry.addData("covariance", covariance);
//
//
//
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }




//    public static Interfaces.OtherCalc Bucket(){
//
//        return new Interfaces.OtherCalc(){
//            double myProgress;
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return myProgress;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                final double SHOOTPOSITION = 0.24;
//                //d.robot.bucket.setPosition((d.MAXBUCKET-d.MINBUCKET)*((d.manip.ls().y+1.0)/2.0)+d.MINBUCKET);
//                if(d.manip.rb()) d.robot.bucket.setPosition(SHOOTPOSITION);
//                else d.robot.bucket.setPosition(0.68);
//
//                //else d.robot.bucket.setPosition((d.MAXBUCKET-d.MINBUCKET)*(1-d.manip.rt()) + d.MINBUCKET);
//
//                if(d.manip.b()) {
//                    d.robot.pusher.setPosition(1.0);
//                } else {
//                    d.robot.pusher.setPosition(0.0);
//                }
//            }
//        };
//    }

//    public static Interfaces.OtherCalc SetBucketPositionWithProgress(final double position){
//
//        return new Interfaces.OtherCalc(){
//            boolean prog = false;
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return prog?1:0;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//
//                d.robot.bucket.setPosition(position);
//                prog = true;
//            }
//        };
//    }

//    public static Interfaces.OtherCalc SetBucketPosition(final double position){
//
//        return new Interfaces.OtherCalc(){
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//
//                d.robot.bucket.setPosition(position);
//
//            }
//        };
//    }

//    public static Interfaces.OtherCalc SetShooterSpeed(final double speed){
//
//        return new Interfaces.OtherCalc(){
//            boolean prog = false;
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return prog?1:0;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//
//                d.robot.shooterEx.setVelocity(speed);
//            }
//        };
//    }

//    public static Interfaces.OtherCalc Shooting(final int loop){
//
//        return new Interfaces.OtherCalc(){
//            boolean prog = false;
//            int i = 0;
//            int l = loop;
//            long startTime = System.currentTimeMillis();
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                //ASSUMING THAT THE BUCKET AND SHOOTER IS ALREADY IN POSITION BECAUSE OF TIME
//                d.robot.bucket.setPosition(0.24);
//                d.robot.shooterEx.setVelocity(1690);
//                long thisTime = System.currentTimeMillis();
//
//                switch(i){
//                    case 0:
//                        d.robot.pusher.setPosition(1.0);
//                        if((thisTime-startTime)>400) {
//                            i = 1;
//                            startTime = System.currentTimeMillis();
//                        }
//                        break;
//                    case 1:
//                        d.robot.pusher.setPosition(0.0);
//                        if((thisTime-startTime)>600) {
//                            i = 0;
//                            startTime = System.currentTimeMillis();
//                            l--;
//                        }
//
//                        break;
//                }
//                if(l<=0) {
//                    prog = true;
//                }
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return prog?1:0;
//            }
//        };
//    }
//
//    public static Interfaces.OtherCalc Shoot(){
//        return new Interfaces.OtherCalc() {
//            //double myProg = 0.0;
//            boolean prog = false;
//            TimeUtil time = new TimeUtil();
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                if(!prog) {
//                    d.robot.shooterEx.setVelocity(1842.0);//1820//1850
//                    d.robot.bucket.setPosition(0.24);
//                    time.startTimer(1000);
//                    while (!time.timerDone()) ;
//                    //d.robot.shooter.setPower(0.547);
//                    time.resetTimer();
//                    time.startTimer(1000);
//                    while (!time.timerDone()) ;
//                    time.resetTimer();
//                    time.startTimer(1000);
//                    while (!time.timerDone()) {
//                        d.robot.pusher.setPosition(1.0);
//                    }
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while (!time.timerDone()) {
////                        d.robot.pusher.setPosition(0.0);
////                        d.robot.bucket.setPosition(0.40);
////                    }
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while(!time.timerDone()){
////                        d.robot.bucket.setPosition(0.24);
////                    }
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while (!time.timerDone()) {
////                        d.robot.pusher.setPosition(1.0);
////                    }
////
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while (!time.timerDone()) {
////                        d.robot.pusher.setPosition(0.0);
////                        d.robot.bucket.setPosition(0.40);
////                    }
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while(!time.timerDone()){
////                        d.robot.bucket.setPosition(0.24);
////                    }
////
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while (!time.timerDone()) {
////                        d.robot.pusher.setPosition(1.0);
////                    }
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while (!time.timerDone()) {
////                        d.robot.pusher.setPosition(0.0);
////                        d.robot.bucket.setPosition(0.40);
////                    }
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while(!time.timerDone()){
////                        d.robot.bucket.setPosition(0.24);
////                    }
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while (!time.timerDone()) {
////                        d.robot.pusher.setPosition(1.0);
////                    }
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while (!time.timerDone()) {
////                        d.robot.pusher.setPosition(0.0);
////                        d.robot.bucket.setPosition(0.40);
////                    }
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while(!time.timerDone()){
////                        d.robot.bucket.setPosition(0.24);
////                    }
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while (!time.timerDone()) {
////                        d.robot.pusher.setPosition(1.0);
////                    }
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while (!time.timerDone()) {
////                        d.robot.pusher.setPosition(0.0);
////                        d.robot.bucket.setPosition(0.40);
////                    }
////                    time.resetTimer();
////                    time.startTimer(1000);
////                    while(!time.timerDone()){
////                        d.robot.bucket.setPosition(0.24);
////                    }
////                    d.robot.shooter.setPower(0.0);
//                    d.robot.shooterEx.setVelocity(0.0);
//                    d.robot.pusher.setPosition(0.0);
//                    d.robot.bucket.setPosition(0.68);
//                    prog = true;
//                }
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                if(prog)return 1.0;
//                return 0.0;
//            }
//        };
//    }
//    public static Interfaces.OtherCalc Intake(){
//        return new Interfaces.OtherCalc() {
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                if(d.manip.a()) d.robot.intakeEx.setVelocity(1000);
//                else if(d.manip.y()) d.robot.intakeEx.setVelocity(-1600);
//                else d.robot.intake.setPower(0.0);
//
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }
//
//    public static Interfaces.OtherCalc Intake(final double speed){
//        return new Interfaces.OtherCalc() {
//            int i = 0;
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                d.robot.intakeEx.setVelocity(speed);
//                i=1;
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return i;
//            }
//        };
//    }
//
//    public static Interfaces.OtherCalc Intake(final boolean on){
//        return new Interfaces.OtherCalc() {
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                d.robot.intakeEx.setVelocity(on?1000:0);
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }

    public static Interfaces.OtherCalc PIDTest(){
        return new Interfaces.OtherCalc() {
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                if(d.manip.u()){

                }
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }
        };
    }

//    public static Interfaces.OtherCalc Lift(){
//        return new Interfaces.OtherCalc() {
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                d.robot.vex.setPower(d.manip.ls().y);
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }
//
//    // Method to yeet
//    //Very cool aidan
//    //wow
//    //i like this one
//    public static Interfaces.OtherCalc Yeetor(){
//        return new Interfaces.OtherCalc() {
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//
////                if(d.manip.lb()) d.robot.shooter.setPower(1.0);
////                else d.robot.shooter.setPower(d.manip.lt());
//                double newVelocity;
//                if(d.manip.ls().y>0.5) newVelocity = 1740.0;
//                else if (d.manip.ls().y<-0.5) newVelocity = 1500;
//                else newVelocity = 2200*d.manip.lt();
//                d.robot.shooterEx.setVelocity(newVelocity);
//                d.shooterCommand = newVelocity;
//            }
//
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }
//
//    public static Interfaces.OtherCalc Wobble(){
//        return new Interfaces.OtherCalc() {
//            boolean dx = true;
//            boolean grab = false;
//            short i = 0;
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                if(dx&&d.manip.x()){
//                    dx=false;
//                    i++;
//                }
//                if(!d.manip.x()) dx = true;
//                if(i == 0){
//                    d.robot.wobble.setTargetPosition(3+d.robot.wobbleOffset);
//                } else if (i == 1) {
//                    d.robot.wobble.setTargetPosition(585+d.robot.wobbleOffset);//575//550//520//170
//                } //else if (i == 2){
////                    d.robot.wobble.setTargetPosition(0);
////                } else {
////                    d.robot.wobble.setTargetPosition(70);
////                }
//                i%=2;
//                if(d.manip.l()) grab = true;
//                if(d.manip.r()) grab = false;
//                if(grab) d.robot.graber.setPosition(0.8);
//                else d.robot.graber.setPosition(0);
//                //                d.robot.wobble.setPower(d.manip.rs().x/20);
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }
//
//    public static Interfaces.OtherCalc SetGrabberPosition(final boolean grab){
//        return new Interfaces.OtherCalc() {
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                if(grab) d.robot.graber.setPosition(0.8);
//                else d.robot.graber.setPosition(0);
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }
//
//    public static Interfaces.OtherCalc ResetWobbleButton(){
//        final float time123 = System.currentTimeMillis();
//
//        return new Interfaces.OtherCalc() {
//
//            double prog = 0;
//            boolean run = false;
//            boolean backUp = true;
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                if(d.manip.back() && backUp) {
//                    run = true;
//                    backUp = false;
//                }
//                if(!d.manip.back()) backUp = true;
//                if(run) {
//                    prog += 0.02;
//                    if (prog > 0.5) {
//                        d.robot.wobbleOffset = d.robot.wobble.getCurrentPosition();
//                        d.robot.wobble.setTargetPosition(3 + d.robot.wobbleOffset);
//                        d.robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        //                    d.robot.wobbleEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        //                    while (d.robot.wobbleEx.getCurrentPosition() == 0) {
//                        //                    }
//                        //                    d.robot.wobbleEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        run = false;
//                        prog = 0;
//                        //
//                    } else {
//                        d.robot.wobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                        d.robot.wobble.setPower(-0.10);
//
//                    }
//                }
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }
//
//    public static Interfaces.OtherCalc ResetWobble(){
//        final float time123 = System.currentTimeMillis();
//
//        return new Interfaces.OtherCalc() {
//
//            double prog = 0;
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                prog += 0.02;
//                if(prog > 0.5) {
//                    d.robot.wobbleOffset = d.robot.wobble.getCurrentPosition();
//                    d.robot.wobble.setTargetPosition(3+d.robot.wobbleOffset);
//                    d.robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                    d.robot.wobbleEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////                    while (d.robot.wobbleEx.getCurrentPosition() == 0) {
////                    }
////                    d.robot.wobbleEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    prog = 1;
////
//                }
//                else {
//                    d.robot.wobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    d.robot.wobble.setPower(-0.10);
//
//                }
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return prog;
//            }
//        };
//    }
//
//    public static Interfaces.OtherCalc SetWobblePosition(final int wobblePos){
//        return new Interfaces.OtherCalc() {
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                d.robot.wobbleEx.setTargetPosition(wobblePos+d.robot.wobbleOffset);
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }
//
//
//    public static Interfaces.OtherCalc ExitAtProgress(final double exitProgress){
//        return new Interfaces.OtherCalc() {
//            double myProgress;
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                if(d.progress>exitProgress) myProgress = 1;
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return myProgress;
//            }
//        };
//    }
//
//
//    public static Interfaces.OtherCalc TapeMeasure(){
//
//        return new Interfaces.OtherCalc(){
//            double myProgress;
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return myProgress;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//                if(d.progress > 0.95){
//                    //d.robot.hooker.setPosition(0.5);
//                }
//            }
//        };
//    }
//
//    public static Interfaces.OtherCalc GetDonutStack(){
//
//
////                    phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//        return new Interfaces.OtherCalc(){
////            final StackDeterminationPipeline pipeline = new StackDeterminationPipeline();
//            double myProgress = 0;
//            boolean first = true;
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return myProgress;
//            }
//
//            @Override
//            public void CalcOther(final Interfaces.MoveData d) {
////                if(first){
////                    d.robot.yeetCam.setPipeline(pipeline);
////                    d.robot.yeetCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
////                    d.robot.yeetCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
////                    {
////                        @Override
////                        public void onOpened() {
////                            d.robot.yeetCam.startStreaming(432, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
////                        }
////                    });
////                    first = false;
////                }
//                d.stackHeight = d.robot.pipeline.getHeight();
//                myProgress = 1.0;
//            }
//        };
//    }
//
//
//    public static Interfaces.OtherCalc GetXOfGoal(){
//        return new Interfaces.OtherCalc() {
//            final GoalPositionPipeline pipeline = new GoalPositionPipeline();
//            boolean first = true;
//            @Override
//            public void CalcOther(final Interfaces.MoveData d) {
//
//
//                d.goalBox = pipeline.getPos();
//                d.hsvValues = pipeline.hsvValues();
//                if(first){
//
//                    d.robot.yeetCam.setPipeline(pipeline);
//                    d.robot.yeetCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//                    d.robot.yeetCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//                    {
//                        @Override
//                        public void onOpened() {
//                            d.robot.yeetCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);//320 240
//                        }
//                    });
//                    first = false;
//                }
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }
//
//    public static Interfaces.OtherCalc GetPowerPositions(){
//        return new Interfaces.OtherCalc() {
//            final PowerShotPositionPipeline pipeline = new PowerShotPositionPipeline();
//            boolean first = true;
//            @Override
//            public void CalcOther(final Interfaces.MoveData d) {
//                d.powerCenter = pipeline.getPowerCenter();
//                if(first){
//                    d.robot.yeetCam.setPipeline(pipeline);
//                    d.robot.yeetCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//                    d.robot.yeetCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//                    {
//                        @Override
//                        public void onOpened() {
//                            d.robot.yeetCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);//320 240 //1280 was 960
//                        }
//                    });
//                    first = false;
//                }
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }
//
//    public static Interfaces.OtherCalc SingleShot(final double delay, final double firstDelay){
//
//        return new Interfaces.OtherCalc() {
////            TimeUtil time = new TimeUtil();
////            TimeUtil time1 = new TimeUtil();
//            double startTime = System.currentTimeMillis(); //+ 1000;
//            boolean prog = false;
//            boolean first = true;
//            int i = 0;
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
////                time.startTimer(delay);
//                d.aimToPowerOverride = true;
////                if(first) {
////                    d.robot.pusher.setPosition(0.0);
////                    d.robot.bucket.setPosition(0.24);
////                    d.robot.shooterEx.setVelocity(1550);//1500
////                    first = false;
////                }
////                if(Math.abs(d.powerError) < 20.0 && (System.currentTimeMillis() - startTime) > delay){
////                    d.robot.pusher.setPosition(1.0);
////                    if((System.currentTimeMillis() - startTime) > (delay + 500)) {
////                        startTime = System.currentTimeMillis();
////                        d.robot.pusher.setPosition(0.0);
////                        i++;
////                    }
////                }
////                if(i > 2){
////                    prog = true;
////                    d.aimToPowerOverride = false;
////                    d.robot.shooterEx.setVelocity(0.0);
////                    d.robot.pusher.setPosition(0.0);
////                }
//                if(i <= 2) {
//                    if (System.currentTimeMillis() - startTime < delay + (first?firstDelay:0.0)) {
//                        d.robot.pusher.setPosition(0.0);
//                        d.robot.bucket.setPosition(0.24);
//                        d.robot.shooterEx.setVelocity(1550);//1500
//                    } else {
//                        d.robot.pusher.setPosition(1.0);
//                    }
//                    if (System.currentTimeMillis() - startTime > delay + 1000.0 + (first?firstDelay:0.0)) {
//                        i++;
//                        startTime = System.currentTimeMillis();
//                        d.aimToPowerOverride = false;
//                        first = false;
//                        d.robot.pusher.setPosition(0.0);
//                    }
//                } else {
//                    prog = true;
//                    d.robot.shooterEx.setVelocity(0.0);
//                }
//            }
//
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return prog?1:0;
//            }
//        };
//    }
////    public static Interfaces.OtherCalc GetDonutStack(final HardwareMap hw, final Telemetry telemetry){
////
////            class MyDonutStackOtherCalc implements Interfaces.OtherCalc{
////                telemetry.addData();
////
////                final StackDeterminationPipeline pipeline = new StackDeterminationPipeline();
////                public MyDonutStackOtherCalc(){
////                    phoneCam.setPipeline(pipeline);
////                    phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
////
////                    phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
////                    {
////                        @Override
////                        public void onOpened() {
////                            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
////                        }
////                    });
////                }
////
////
////                @Override
////                public void CalcOther(Interfaces.MoveData d) {
////
////                    d.stackHeight = pipeline.getHeight();
////                    // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
////                    // out when the RC activity is in portrait. We do our actual image processing assuming
////                    // landscape orientation, though.
////
////                }
////
////                @Override
////                public double myProgress(Interfaces.MoveData d) {
////                    return 0;
////                }
////            }
////            return new MyDonutStackOtherCalc();
////    }
//    public static Interfaces.OtherCalc Arm(){
//
//
//        return new Interfaces.OtherCalc(){
//            private double myProgress = 0;
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return myProgress;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d){
//                d.arm.setArm2DVelocity(d.manip.ls().x, d.manip.ls().y, d.manip.rs().x);
//            }
//        };
//    }

//    public static Interfaces.OtherCalc ArmTele(){
//
//
//        return new Interfaces.OtherCalc(){
//            private double myProgress = 0;
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return myProgress;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d){
//
//
//                if(d.manip.a()) {
//                    try {
//                        double command = -d.robot.cubeFindPipeline.getCubeXValueCommand();
//                        if (command > .1) command = .1;
//                        else if (command < -.1) command = -.1;
//                        if(command > 0.0 && d.sarmAngle > 30.0){
//                            d.robot.sarm.setPower(0.0);
//                        } else if (command < 0.0 && d.sarmAngle < -15.0){
//                            d.robot.sarm.setPower(0.0);
//                        } else {
//                            d.robot.sarm.setPower(command);
//                        }
//                    } catch (Exception e) {
//                        d.robot.sarm.setPower(0.0);
//                        d.telemetry.addData("cube x value", "found nothing");
//                    }
//                } else if(d.manip.y()){
//                    d.arm.moveTowardTarget(90, .34, .36, .4);
//                } else if (d.manip.d()){
//                    d.arm.moveTowardTarget(0, .31, -.15, .3);
//                } else if (d.manip.u()) {
//                    d.arm.moveTowardTarget(0, .33, -.03, .3);
//                } else if (d.manip.lb()){
//                    d.arm.moveTowardTarget(90, .25, .55, .4);
//                } else{
//                    d.arm.setArm2DVelocity(d.manip.ls().x, d.manip.ls().y, -d.manip.rs().x);
//                }
//            }
//        };
//    }



//    public static Interfaces.OtherCalc Claw(){
//
//        return new Interfaces.OtherCalc() {
//
//            double clawPosition = 0.5;
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//
//                if (d.manip.b()) {
//                    d.robot.grip.setPosition(1.0);
//                } else if (d.manip.rb()) {
//                    d.robot.grip.setPosition(0.4);
//                } else if (d.robot.cubeFindPipeline.isCubeClose() || d.robot.cubeFindPipeline.isDuckClose()) {
//                    d.robot.grip.setPosition(0.4);
//                }
//                if (d.tarmAngle > 90.0) {
//                    clawPosition = 0.4 * (180 - d.tarmAngle) / 90.0 + 0.6;
//                } else {
//                    clawPosition = -0.4 * d.tarmAngle / 90.0 + 0.6;
//                }
////                d.robot.grip.setPosition(d.manip.rt()/2.0 +.5);
////                if(d.manip.u()) clawPosition += .05;
////                else if (d.manip.d()) clawPosition -= .05;
//                d.telemetry.addData("claw position", clawPosition);
//                d.robot.claw.setPosition(clawPosition);//min .2 max 1 center .6
//            }
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return 0;
//            }
//        };
//    }

//    public static Interfaces.OtherCalc Arm2D(){
//
//
//        return new Interfaces.OtherCalc(){
//            private double myProgress = 0;
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return myProgress;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d){
//                d.arm.setArm2DVelocity(d.manip.ls().x, d.manip.ls().y, d.manip.rs().x);
////                d.arm.thetaVelocity(d.manip.rs().x);
//            }
//        };
//    }

//    public static Interfaces.OtherCalc FindTurnToCube(){
//
//
//        return new Interfaces.OtherCalc(){
//            private double myProgress = 0;
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return myProgress;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d){
//                d.telemetry.addData("cam fps", d.robot.clawCam.getFps());
//                if(d.manip.a()) {
//                    try {
//                        double command = -d.robot.cubeFindPipeline.getCubeXValueCommand();
//                        if (command > .1) command = .1;
//                        else if (command < -.1) command = -.1;
//                        if(command > 0.0 && d.sarmAngle > 30.0){
//                            d.robot.sarm.setPower(0.0);
//                        } else if (command < 0.0 && d.sarmAngle < -15.0){
//                            d.robot.sarm.setPower(0.0);
//                        } else {
//                            d.robot.sarm.setPower(command);
//                        }
//
////                        if (d.sarmAngle < 30.0 && d.sarmAngle > -30.0) {
////                            double command = -d.robot.cubeFindPipeline.getCubeXValueCommand();
////                            if (command > .1) command = .1;
////                            else if (command < -.1) command = -.1;
////                            d.robot.sarm.setPower(command);
////                            d.telemetry.addData("cube x value", d.robot.cubeFindPipeline.getCubeXValueCommand());
////                        } else {
////                            d.robot.sarm.setPower(0.0);
////                        }
//
//                    } catch (Exception e) {
//                        d.robot.sarm.setPower(0.0);
//                        d.telemetry.addData("cube x value", "found nothing");
//                    }
//                } else {
//                    d.robot.sarm.setPower(0.0);
//                }
//
////                try {
////                    List<Recognition> recogntions = d.robot.tfod.getUpdatedRecognitions();
////                    float greatestBottom = -1.0f;
////                    Recognition bestRecognition = null;
////                    if(recogntions != null) {
////                        for (Recognition r : recogntions) {
////                            if (!r.getLabel().equals("Cube")) continue;
////                            if (r.getBottom() > greatestBottom) {
////                                greatestBottom = r.getBottom();
////                                bestRecognition = r;
////                            }
////                        }
////                    }
////                    if (bestRecognition == null) {
////                        d.telemetry.addData("no cube found", "<-");
////                    } else {
////                        d.telemetry.addData("left", bestRecognition.getLeft());
////                        d.telemetry.addData("right", bestRecognition.getRight());
////                        double command = (bestRecognition.getLeft() + bestRecognition.getRight() / 2.0 / 1920.0) * 2.0 - 0.5;
////                        d.telemetry.addData("command", command);
////                    }
////                } catch (Exception e){
////                    d.telemetry.addData("exception", e);
////                }
////                d.arm.thetaVelocity(d.manip.rs().x);
//            }
//        };
//    }

//    public static Interfaces.OtherCalc FindGC(){
//
//
//        return new Interfaces.OtherCalc(){
//            double clawPosition = 0.5;
//            private double myProgress = 0.0;
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return myProgress;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//
//                d.telemetry.addData("cam fps", d.robot.clawCam.getFps());
//                try {
//                    d.arm.update();
//                    double clawPosition;
//                    if(d.tarmAngle > 90.0){
//                        clawPosition = 0.4* (180-d.tarmAngle)/90.0 + 0.6;
//                    } else {
//                        clawPosition = -0.4* d.tarmAngle/90.0+0.6;
//                    }
//
////                    d.telemetry.addData("claw position", clawPosition);
//                    d.robot.claw.setPosition(clawPosition);//min .2 max 1 center .6
//                    if (!d.robot.cubeFindPipeline.isCubeClose()) {
//                        double command = -d.robot.cubeFindPipeline.getCubeXValueCommand();
//                        if (command > .1) command = .1;
//                        else if (command < -.1) command = -.1;
//                        if(command > 0.0 && d.sarmAngle > 37.0){
//                            d.robot.sarm.setPower(0.0);
////                            d.telemetry.addData("sarm", 0.0);
//                        } else if (command < 0.0 && d.sarmAngle < -15.0){
//                            d.robot.sarm.setPower(0.0);
////                            d.telemetry.addData("sarm", 0.0);
//                        } else {
//                            d.robot.sarm.setPower(command);
////                            d.telemetry.addData("sarm", command);
//                        }
////                        d.telemetry.addData("cube x value", d.robot.cubeFindPipeline.getCubeXValueCommand());
//                    } else {
//                        d.robot.sarm.setPower(0.0);
//                        if(d.wPos.x>200) {
//                            d.robot.sarm.setPower(0.0);
//                            d.robot.grip.setPosition(0.5);
//                            myProgress = 1.0;
//                        }
//                    }
//
//
//                } catch (Exception e) {
//
//                    d.robot.sarm.setPower(0.0);
//                    d.telemetry.addData("cube x value", "found nothing");
//                }
//            }
//        };
//    }


//    public static Interfaces.OtherCalc FindD(){
//
//
//        return new Interfaces.OtherCalc(){
//            double clawPosition = 0.5;
//            private double myProgress = 0.0;
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return myProgress;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d) {
//
//                d.telemetry.addData("cam fps", d.robot.clawCam.getFps());
//                try {
//                    d.arm.update();
//                    double clawPosition;
//                    if(d.tarmAngle > 90.0){
//                        clawPosition = 0.4* (180-d.tarmAngle)/90.0 + 0.6;
//                    } else {
//                        clawPosition = -0.4* d.tarmAngle/90.0+0.6;
//                    }
//
////                    d.telemetry.addData("claw position", clawPosition);
//                    d.robot.claw.setPosition(clawPosition);//min .2 max 1 center .6
//                    if (!d.robot.cubeFindPipeline.isDuckClose()) {
//                        double command = -d.robot.cubeFindPipeline.getCubeXValueCommand();
//                        if (command > .1) command = .1;
//                        else if (command < -.1) command = -.1;
//                        if(command > 0.0 && d.sarmAngle > 37.0){
//                            d.robot.sarm.setPower(0.0);
////                            d.telemetry.addData("sarm", 0.0);
//                        } else if (command < 0.0 && d.sarmAngle < -20.0){
//                            d.robot.sarm.setPower(0.0);
////                            d.telemetry.addData("sarm", 0.0);
//                        } else {
//                            d.robot.sarm.setPower(command);
////                            d.telemetry.addData("sarm", command);
//                        }
////                        d.telemetry.addData("cube x value", d.robot.cubeFindPipeline.getCubeXValueCommand());
//                    } else {
//                        d.robot.sarm.setPower(0.0);
//                        if(d.wPos.x>200) {
//                            d.robot.sarm.setPower(0.0);
//                            d.robot.grip.setPosition(0.4);
//                            myProgress = 1.0;
//                        }
//                    }
//
//
//                } catch (Exception e) {
//
//                    d.robot.sarm.setPower(0.0);
//                    d.telemetry.addData("cube x value", "found nothing");
//                }
//            }
//        };
//    }



//    public static Interfaces.OtherCalc ArmPath(final Vector<SpeedCalcs.ProgressSpeed> speeds, final Vector<Vector3D> points){
//
//        return new Interfaces.OtherCalc(){
//            private double myProgress = 0;
//            private boolean first = true;
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return myProgress;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d){
//                if(first){
//                    d.arm.setArmPath(points);
//                    d.arm.setProgressSpeeds(speeds);
//                }
//                if(d.manip.b()){
////                    d.arm.move();
//                } else {
//                    d.robot.tarmEx.setVelocity(0);
//                    d.robot.barmEx.setVelocity(0);
//                    d.robot.sarmEx.setVelocity(0);
//                }
////                d.robot.barmEx.setVelocity(1500*(-d.manip.ls().x/2.5));
////                d.robot.tarmEx.setVelocity(1500*(-d.manip.ls().y/2.5));
////                d.robot.sarm.setPower(-d.manip.rs().x/2.5);
//            }
//        };
//    }



    public static Interfaces.OtherCalc Duck(){


        return new Interfaces.OtherCalc(){
            private double myProgress = 0;
            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }

            @Override
            public void CalcOther(Interfaces.MoveData d){
                d.robot.duck.setPower((d.driver.rt()-d.driver.lt())/2.0);
            }
        };
    }


    public static Interfaces.OtherCalc TeleOpMatch(){

        final TimeUtil matchTime = new TimeUtil();
        final TimeUtil endGameTime = new TimeUtil();

        return new Interfaces.OtherCalc(){
            private double myProgress;
            private boolean firstLoop = true;
            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }

            @Override
            public void CalcOther(Interfaces.MoveData d){
                if(firstLoop){
                    endGameTime.startTimer(120000);
                    matchTime.startTimer(150000);
                    firstLoop=false;
                }

                d.timeRemainingUntilEndgame = endGameTime.timeRemaining();
                d.timeRemainingUntilMatch = matchTime.timeRemaining();
                myProgress = 1-(d.timeRemainingUntilMatch/150000);

            }
        };
    }


    public static Interfaces.OtherCalc TelemetryPosition(){
        return new Interfaces.OtherCalc() {
            protected int stringFieldWidth = 24;
            protected int stringFieldHeight = 16;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                int realFieldWidth = 366;
                int realFieldHeight = 366;
                int adjustedColumn = (int)Math.round((d.wPos.x/realFieldWidth)* stringFieldWidth);
                int adjustedRow = (int)Math.round((d.wPos.y/realFieldHeight)* stringFieldHeight);
                String rval = "";
                for(int row = stringFieldHeight-1; row>-1; row--){
                    if(row == stringFieldHeight-1) rval += "\u2004______________________________________________\n";
                    rval += "|";

                    for(int col = 0; col< stringFieldWidth; col++){

                        if(row == adjustedRow && col == adjustedColumn){
                            rval += "■";
                        } else {
                            rval += "\u2004\u2002";
                        }

                        if(col == stringFieldWidth-1) {
                            if (row == 0) {
                                rval += "|";
                            } else {
                                rval += "|\n";
                            }
                        }

                    }
                }
                rval += "_______________________________________________\u2004";
                d.field = rval;
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }
        };
    }


    public static Interfaces.OtherCalc TeleSafe(){
        return new Interfaces.OtherCalc() {
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                /*if(!d.manip.isConnected() && !d.driver.isConnected()) {
                    System.out.println("your skrewed");
                } else{
                    if(!d.manip.isConnected()) {
                        System.out.println("yes");
                    }
                    else if(!d.driver.isConnected()) {
                        System.out.println("no");
                    }
                    else {
                        System.out.println("perfect");
                    }
                }*/

            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }
        };
    }


    public static Interfaces.OtherCalc AutoOpMatch(){
        final TimeUtil autoTime = new TimeUtil();

        return new Interfaces.OtherCalc() {
            private double myProgress;
            protected int timeInAuto = 30_000;
            private boolean firstLoop = true;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                if(firstLoop){
                    autoTime.startTimer(timeInAuto);
                    firstLoop = false;
                }
                myProgress = 1-(autoTime.timeRemaining()/timeInAuto);
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }
        };

    }


    public static Interfaces.OtherCalc DistanceStop(final Side side, final double startStopDist, final double stopStopDist, final double startProgress, final double endProgress){

        return new Interfaces.OtherCalc(){
            private double myProgress = 0;
            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }

            @Override
            public void CalcOther(Interfaces.MoveData d){
//                switch(side) {
//                    case FRONT:
//                        if (startStopDist < d.robot.frontRange.getDistance(DistanceUnit.CM)) {
//                            myProgress = 0;
//                        } else if (startStopDist > d.robot.frontRange.getDistance(DistanceUnit.CM)) {
//                            myProgress = startProgress + (endProgress - startProgress) * ((startStopDist - d.robot.frontRange.getDistance(DistanceUnit.CM))
//                                    / (startStopDist - stopStopDist));
//                        } else if (stopStopDist > d.robot.frontRange.getDistance(DistanceUnit.CM)) {
//                            myProgress = endProgress;
//                        }
//                        break;
//                    case BACK:
//                        if (startStopDist < d.robot.backRange.getDistance(DistanceUnit.CM)){
//                            myProgress = 0;
//                        } else if (startStopDist > d.robot.backRange.getDistance(DistanceUnit.CM)) {
//                            myProgress = startProgress + (endProgress - startProgress) * ((startStopDist - d.robot.backRange.getDistance(DistanceUnit.CM))
//                                    / (startStopDist - stopStopDist));
//                        } else if (stopStopDist > d.robot.backRange.getDistance(DistanceUnit.CM)) {
//                            myProgress = endProgress;
//                        }
//                        break;
//                    case RIGHT:
////                        if (startStopDist < d.robot.rightRange.getDistance(DistanceUnit.CM)) {
////                            myProgress = 0;
////                        } else if (startStopDist > d.robot.rightRange.getDistance(DistanceUnit.CM)) {
////                            myProgress = startProgress + (endProgress - startProgress) * ((startStopDist - d.robot.rightRange.getDistance(DistanceUnit.CM))
////                                    / (startStopDist - stopStopDist));
////                        } else if (stopStopDist > d.robot.rightRange.getDistance(DistanceUnit.CM)) {
////                            myProgress = endProgress;
////                        }
//                        break;
//                    case LEFT:
////                        if (startStopDist < d.robot.leftRange.getDistance(DistanceUnit.CM)) {
////                            myProgress = 0;
////                        } else if (startStopDist > d.robot.leftRange.getDistance(DistanceUnit.CM)) {
////                            myProgress = startProgress + (endProgress - startProgress) * ((startStopDist - d.robot.leftRange.getDistance(DistanceUnit.CM))
////                                    / (startStopDist - stopStopDist));
////                        } else if (stopStopDist > d.robot.leftRange.getDistance(DistanceUnit.CM)) {
////                            myProgress = endProgress;
////                        }
//                        break;
//                }
            }
        };
    }


//    public static Interfaces.OtherCalc TimeProgress(){
//
//        TimeUtil matchTime = new TimeUtil();
//        TimeUtil endGameTime = new TimeUtil();
//
//        return new Interfaces.OtherCalc(){
//            @Override
//            public double myProgress(Interfaces.MoveData d) {
//                return true;
//            }
//
//            @Override
//            public void CalcOther(Interfaces.MoveData d){
//                if(d.firstLoop){
//                    endGameTime.startTimer(120000);
//                    matchTime.startTimer(150000);
//                    d.firstLoop=false;
//                }
//
//                d.timeRemainingUntilEndgame = endGameTime.timeRemaining();
//                d.timeRemainingUntilMatch = matchTime.timeRemaining();
//                d.progress = 1-(d.timeRemainingUntilMatch/150000);
//
//            }
//        };
//    }


    public enum Controller{
        DRIVER,
        MANIP
    }

    public enum Side {
        FRONT,
        BACK,
        RIGHT,
        LEFT
    }

}
