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

    public static Interfaces.OtherCalc ResetYPosition(){
        return new Interfaces.OtherCalc() {
            double average = 1.8; // supposed field tiles from wall
            @Override
            public void CalcOther(Interfaces.MoveData d) {

                //totalCm = 18cm from distance sensor to center of robot + cmUltrasonic()
                //totalTiles = totalCm / 59.5
                //offset from wall = 6.0 - totalTiles
                average = average * 0.8 + (6.0 - ((18.0 + d.robot.frontRange.cmUltrasonic()) / 59.5)) * 0.2;
                d.wPos.y = average;
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }
        };
    }


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
                d.robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                myProgress = (System.currentTimeMillis()-startTime)/outtakeTimeMillis;

                d.robot.bar.setPosition(d.gateOpen);

                if(myProgress >= 1.0) {
                    d.robot.bar.setPosition(d.gateClose);
                    d.robot.intake.setPower(0.0);
                } else if(myProgress > 0.1){
                    d.robot.intake.setPower(0.25);
                }
             }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }
        };
    }

    public static Interfaces.OtherCalc AutoPlaceDuck(double outtakeTimeMillis){
        return new Interfaces.OtherCalc() {
            final long startTime = System.currentTimeMillis();
            double myProgress = 0.0;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                d.robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                myProgress = (System.currentTimeMillis()-startTime)/outtakeTimeMillis;

                d.robot.bar.setPosition(d.gateOpen);

                if(myProgress >= 1.0) {
                    d.robot.bar.setPosition(d.gateClose);
                    d.robot.intake.setPower(0.0);
                } else if(myProgress > 0.1){
                    d.robot.intake.setPower(0.1);
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
                d.robot.duck.setPower(0.45);
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

    public static Interfaces.OtherCalc AutoDuckRed(double totalTimeMillis){

        return new Interfaces.OtherCalc() {
            final long startTime = System.currentTimeMillis();
            double myProgress = 0.0;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                myProgress = (System.currentTimeMillis()-startTime)/totalTimeMillis;
                d.robot.duck.setPower(-0.45);
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

    public static Interfaces.OtherCalc StopAtIntake(){
        return new Interfaces.OtherCalc() {
            double myProgress = 0.0;
            @Override
            public void CalcOther(Interfaces.MoveData d) {

                if(d.robot.intakedPipeline.isIntaked()){
                    d.robot.lift.setTargetPosition(d.safeLiftPos);
                    d.robot.intake.setPower(0.0);
                    myProgress = 1.0;
                } else {
                    d.robot.intake.setPower(0.7);
                }
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }
        };
    }


    public static Interfaces.OtherCalc AutoCupGrabBlue(boolean nearPipes, double totalTimeMillis){
        return new Interfaces.OtherCalc() {
            final long startTime = System.currentTimeMillis();
            double myProgress = 0.0;
            boolean hover = false;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                myProgress = (System.currentTimeMillis()-startTime)/totalTimeMillis;
                if(myProgress < 0.2) {
                    if (d.duckPos == 0) {
                        d.robot.base.setPosition(0.385);
                        d.robot.height.setPosition(0.16);

                        //437 out
                        //base 0.3651
                        //height 0.2654
                    } else if (d.duckPos == 1) {
                        d.robot.base.setPosition(0.4849);
                        d.robot.height.setPosition(0.16);

                        //372 out
                        //base 0.4849
                        //height 0.2830
                    } else if (d.duckPos == 2) {
                        d.robot.base.setPosition(0.5925);
                        d.robot.height.setPosition(0.2);

                        //434 out
                        //base 0.6080
                        //height 0.2867
                    }
                } else if (myProgress < 0.4){
                    if (d.duckPos == 0) {
                        if (d.robot.tapeEx.getCurrentPosition() < 480){
                            d.robot.tapeEx.setPower(0.2);
                        } else {
                            d.robot.tapeEx.setPower(0.0);

                        }
                        //437 out
                    } else if (d.duckPos == 1) {
                        if (d.robot.tapeEx.getCurrentPosition() < 440){
                            d.robot.tapeEx.setPower(0.2);
                        } else {
                            d.robot.tapeEx.setPower(0.0);

                        }
                        //372 out
                    } else if (d.duckPos == 2) {
                        if (d.robot.tapeEx.getCurrentPosition() < 500){
                            d.robot.tapeEx.setPower(0.2);
                        } else {
                            d.robot.tapeEx.setPower(0.0);

                        }
                        //434 out
                    }
                } else if (myProgress < 0.65){
                    d.robot.height.setPosition(0.5);
                    d.robot.tapeEx.setPower(0.0);
                }

                else {
                    d.robot.tapeEx.setPower(-0.2);
                    if(d.robot.tapeEx.getCurrentPosition() < 0){
                        myProgress = 1.0;
                    }
                }
                if(myProgress >= 1.0){
                    d.robot.tapeEx.setPower(0.0);
                    d.robot.base.setPosition(0.5);
                }
            }



            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }
        };
    }

    public static Interfaces.OtherCalc AutoCupGrabRed(boolean nearPipes, double totalTimeMillis){
        return new Interfaces.OtherCalc() {
            final long startTime = System.currentTimeMillis();
            double myProgress = 0.0;
            boolean hover = false;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                myProgress = (System.currentTimeMillis()-startTime)/totalTimeMillis;
                if(myProgress < 0.2) {
                    if (d.duckPos == 0) {
                        d.robot.base.setPosition(.4805);
                        d.robot.height.setPosition(.18);

                        //437 out
                        //base 0.3651
                        //height 0.2654
                    } else if (d.duckPos == 1) {
                        d.robot.base.setPosition(.5860);
                        d.robot.height.setPosition(.209);

                        //372 out
                        //base 0.4849
                        //height 0.2830
                    } else if (d.duckPos == 2) {
                        d.robot.base.setPosition(0.6413);
                        d.robot.height.setPosition(0.24);

                        //434 out
                        //base 0.6080
                        //height 0.2867
                    }
                } else if (myProgress < 0.4){
                    if (d.duckPos == 0) {
                        if (d.robot.tapeEx.getCurrentPosition() < 400){
                            d.robot.tapeEx.setPower(0.2);
                        } else {
                            d.robot.tapeEx.setPower(0.0);

                        }
                        //437 out
                    } else if (d.duckPos == 1) {
                        if (d.robot.tapeEx.getCurrentPosition() < 520){
                            d.robot.tapeEx.setPower(0.2);
                        } else {
                            d.robot.tapeEx.setPower(0.0);

                        }
                        //372 out
                    } else if (d.duckPos == 2) {
                        if (d.robot.tapeEx.getCurrentPosition() < 620){
                            d.robot.tapeEx.setPower(0.2);
                        } else {
                            d.robot.tapeEx.setPower(0.0);

                        }
                        //434 out
                    }
                } else if (myProgress < 0.7){
                    d.robot.height.setPosition(0.5);
                    d.robot.tapeEx.setPower(0.0);
                }

                else {
                    d.robot.tapeEx.setPower(-0.2);
                    if(d.robot.tapeEx.getCurrentPosition() < 0){
                        myProgress = 1.0;
                    }
                }
                if(myProgress >= 1.0){
                    d.robot.tapeEx.setPower(0.0);
                    d.robot.base.setPosition(0.5);
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
                } else if(d.manip.y()){
                    d.robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    d.robot.lift.setPower(-0.1);
                    d.firstLiftPos = d.robot.lift.getCurrentPosition();
//                    canRelease = true;
                }
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
//                d.robot.tapeEx.setPower(Math.signum(d.manip.ls().y)*Math.sqrt(Math.abs(-d.manip.ls().y))/3.0);
                d.robot.tapeEx.setPower((d.manip.ls().y)/4.0);

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


    public static Interfaces.OtherCalc IntakeDuck(){
        return new Interfaces.OtherCalc() {
            double myProgress = 0.0;
            @Override
            public void CalcOther(Interfaces.MoveData d) {
                int currentPosition = d.robot.intake.getCurrentPosition();
                d.robot.intake.setPower(0.2);
                if(d.robot.findDuckPipeline.intaked){
                    d.holdPosition = true;
                    d.robot.lift.setTargetPosition(d.safeLiftPos - d.firstLiftPos);
                    d.robot.intake.setPower(0.0);
                    myProgress = 1.0;
                }
            }

            @Override
            public double myProgress(Interfaces.MoveData d) {
                return myProgress;
            }
        };
    }

    public static Interfaces.OtherCalc HoldIntakePosition() {
        return new Interfaces.OtherCalc() {
            double targetHoldPosition;
            boolean firstHoldPosition = true;
            @Override
            public void CalcOther(Interfaces.MoveData d) {

                int currentPosition = d.robot.intake.getCurrentPosition();



                if (firstHoldPosition) {
                    double modVal = (currentPosition * 3) % 145;
                    targetHoldPosition = currentPosition + (145 - modVal) / 3.0;
                    firstHoldPosition = false;
//                    d.robot.intake.setTargetPosition((int) (currentPosition + (145 - modVal) / 3.0));

                }

                d.robot.intake.setPower((targetHoldPosition - currentPosition) / 200.0);
            }
            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0.0;
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
                if(d.manip.lb()) {
                    d.holdPosition = true;
                } else {
                    d.holdPosition = false;
                    if (d.manip.rb() && !d.robot.intakedPipeline.isIntaked() && d.robot.lift.getCurrentPosition() < 50) {
                        d.robot.intake.setPower(0.7);
                    } else {
                        d.robot.intake.setPower((d.robot.intakedPipeline.isIntaked() ? 0.0 : d.manip.rt()) - d.manip.lt());
                    }


                    if (d.manip.b()) {
                        d.robot.bar.setPosition(d.gateOpen);
                        d.robot.intake.setPower(0.3);
                    } else {
                        d.robot.bar.setPosition(d.gateClose);
                    }
                }
            }


            @Override
            public double myProgress(Interfaces.MoveData d) {
                return 0;
            }
        };
    }

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
                int realFieldWidth = 6;
                int realFieldHeight = 6;
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
