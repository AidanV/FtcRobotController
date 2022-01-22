package org.firstinspires.ftc.teamcode.Op;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Calculators.OtherCalcs;
//import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera;
import org.firstinspires.ftc.teamcode.Hardware.FreightRobotName_NA.RobotMap;
import org.firstinspires.ftc.teamcode.Utilities.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.Calculators.Interfaces;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.CompleteController;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;

import java.net.*;

public abstract class ComplexOp extends LinearOpMode{

    private MecanumDrive mecanumDrive;

    double previousHeading = 0;

//    private Vector2D slamraOffset = new Vector2D(-2.0, -16.0);

//    private Pose2d initPose = null;
//    private double initPoseX = 0.0;
//    private double initPoseY = 0.0;

    public void ComplexMove(Interfaces.SpeedCalc speedCalc,
                            Interfaces.MotionCalc motionCalc,
                            Interfaces.OrientationCalc orientationCalc,
                            Interfaces.OtherCalc... otherCalc) throws InterruptedException {

        d.progress = 0;

        Vector2D vector = new Vector2D();

        float endGameTime = 0;

        d.lastCommand = d.currentCommand;
        d.currentCommand = new Interfaces.MoveData.Command(0, vector,0.0);
        DatagramSocket ds = null;
//        try {
//            ds = new DatagramSocket();
//        } catch (SocketException e) {
//            e.printStackTrace();
//        }


        while(d.progress < 1.0) {
            //_______________________
//            if(ds != null) {
//                InetAddress ip = null;
//                try {
//                    ip = InetAddress.getByName("192.168.43.255");
//                    //ip = InetAddress.getLocalHost();
//                } catch (UnknownHostException e) {
//                    e.printStackTrace();
//                }
//                String str = String.valueOf(System.currentTimeMillis())+":"+String.valueOf(Math.abs(d.robot.intakeEx.getVelocity())+":"+Double.toString(d.intakeCommand));
//                byte[] strBytes = str.getBytes();
//                DatagramPacket DpSend =
//                        new DatagramPacket(strBytes, strBytes.length, ip, 10650);
//                try {
//                    ds.send(DpSend);
//                } catch (IOException e) {
//                    e.printStackTrace();
//                }
//            }
            //_______________________


            Orientation orientation = d.robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double heading = orientation.thirdAngle-d.startData.StartNorthOffset;
            double diffHeading = heading - previousHeading;
            if(diffHeading > 180.0) {
                diffHeading -= 360.0;
            } else if (diffHeading <= -180.0){
                diffHeading += 360.0;
            }
            d.heading += diffHeading;
            previousHeading = heading;
//
//            telemetry.addData("gryo", orientation.thirdAngle);
//            telemetry.addData("orientation", d.heading);

            double distanceCorrectionFactorForward = 0.019;
            double distanceCorrectionFactorSide = 0.01864;
            Vector2D encoderPre = d.encoderPos.clone();
            long lastEncoderUpdateTime = d.encodePosUpdateTimeMillis;
            d.encoderPos = mecanumDrive.getVectorDistanceCm();
            telemetry.addData("encoderPos",d.encoderPos);
            d.encodePosUpdateTimeMillis = System.currentTimeMillis();
            Vector2D deltaMove = d.encoderPos.getSubtracted(encoderPre);
            Vector2D moveSpeed = deltaMove.getDivided(Math.max(0.001,(d.encodePosUpdateTimeMillis - lastEncoderUpdateTime)/1000.0));
//            moveSpeed.x *= distanceCorrectionFactorForward;
//            moveSpeed.y *= distanceCorrectionFactorSide;
//            // moveSpeed.y + is moving forward
//            // moveSpeed.x + is moving robot to the right
//            // First value is y value of robot (moving forward == positive)
//            // Second value is x value of robot (positive to left of robot)
//            d.robot.slamra.sendOdometry(-moveSpeed.x,moveSpeed.y);
//
//            telemetry.addData("initPoseX","%.3f", initPoseX);
//            telemetry.addData("initPoseY","%.3f", initPoseY);
//            telemetry.addData("initPoseDeg","%.3f", initPose.getRotation().getDegrees());
//
////            telemetry.addData("encodeMoveSpeed X","%.3f", moveSpeed.x);
////            telemetry.addData("encodeMoveSpeed Y","%.3f", moveSpeed.y);
//
            deltaMove.rotateBy(Math.toRadians(d.heading));//WAS -d.heading !!!!!!!!!!!!!!!!!!!!
            d.preWPos.set(d.wPos);
            d.wPos.add(deltaMove);
//            T265Camera.CameraUpdate cameraUpdate = d.robot.slamra.getLastReceivedCameraUpdate();
//            Pose2d p = cameraUpdate.pose;
//            telemetry.addData("velocity",cameraUpdate.velocity);
//            telemetry.addData("confidence",cameraUpdate.confidence);
//            telemetry.addData("new X",p.getTranslation().getX());
//            telemetry.addData("new Y",p.getTranslation().getY());
//
////            System.out.println("\n init:" + initPoseX + "    :    " + initPoseY);
////            System.out.println(" pose" + p.getTranslation().getX() + "    :    " + p.getTranslation().getY());
//            Vector2D slamraPos = new Vector2D(
//                    (p.getTranslation().getX()-initPoseX)*100.0,
//                    (p.getTranslation().getY()-initPoseY)*100.0);
////            slamraPos.subtract(slamraOffset);
//            slamraPos.rotateBy(-Math.toRadians(startPositionAndOrientation().StartNorthOffset + initPose.getRotation().getDegrees()));
//
//            slamraPos.add(startPositionAndOrientation().StartPos);
////            slamraPos.subtract(slamraOffset);
//
////            slamraPos.subtract(slamraOffset);
//
//
//            d.heading = p.getRotation().getDegrees() - initPose.getRotation().getDegrees();
//
////            slamraPos.add(slamraOffset.getRotatedBy(Math.toRadians(d.heading)));
//
//            d.wPos.set(slamraPos);
            

            if(orientationCalc != null) d.currentCommand.orientationSpeed = orientationCalc.CalcOrientation(d);
            if(motionCalc != null) {
                d.currentCommand.motionSpeed = motionCalc.CalcMotion(d);
                d.currentCommand.motionSpeed.rotateBy(Math.toRadians(90-d.heading));
            }
            if(speedCalc != null) d.currentCommand.speed = speedCalc.CalcSpeed(d);

            for (Interfaces.OtherCalc calc : otherCalc) calc.CalcOther(d);

            if (d.timeRemainingUntilEndgame >= 0) endGameTime = (float)(Math.round(d.timeRemainingUntilEndgame / 100) / 10.0);

            //could add specific telemetry data to show through an implementation of complexOp

            //telemetry.addData("Position: ("+String.valueOf(Math.round(d.wPos.x))+", "+String.valueOf(Math.round(d.wPos.y))+")", "\n"+d.field);
            //telemetry.addData("heading", Math.round(d.heading*10)/10.0);
            //telemetry.addData("position", " "+String.valueOf(Math.round(d.wPos.x))+", "+String.valueOf(Math.round(d.wPos.y)));
            /*
            int z = 0;
            short[]ss = d.robot.vexCrap.readAll();
            for(short s: ss){
                telemetry.addData("Vex Encoder Register: " + z + ": ", s);
                z++;
            }
            */
//            telemetry.addData("manip lx",d.manip.ls().x);
//            telemetry.addData("manip ly", d.manip.ls().y);
//            telemetry.addData("manip rx", d.manip.rs().x);
//            telemetry.addData("manip ry", d.manip.rs().y);
//            telemetry.addData("x pos", d.wPos.x);
//            telemetry.addData("y pos", d.wPos.y);
//            telemetry.addData("right joystick x", d.driver.rs().x);
//            telemetry.addData("wobble position", d.robot.wobbleEx.getCurrentPosition());
//            telemetry.addData("offset", d.robot.wobbleOffset);
//            telemetry.addData("right stick x", d.driver.rs().x);
//            telemetry.addData("right stick x raw", gamepad1.right_stick_x);
//            telemetry.addData("bop", d.robot.barm.getCurrentPosition());
//            telemetry.addData("top", d.robot.tarm.getCurrentPosition());

//            telemetry.addData("barm ticks", d.robot.barm.getCurrentPosition()-d.initBarmPos);
//            telemetry.addData("tarm ticks", d.robot.tarm.getCurrentPosition()-d.initTarmPos);
//            telemetry.addData("sarm ticks", d.robot.sarm.getCurrentPosition()-d.initSarmPos);
//            d.arm.update();
            telemetry.addData("barm Angle", d.barmAngle);
            telemetry.addData("tarm Angle", d.tarmAngle);
//            telemetry.addData("slamra x", d.robot.slamra.getLastReceivedCameraUpdate().pose.getTranslation().getX());
//            telemetry.addData("slamra y", d.robot.slamra.getLastReceivedCameraUpdate().pose.getTranslation().getY());
//            telemetry.addData("slamra just X", d.robot.slamra.getLastReceivedCameraUpdate().pose.getX());
//            telemetry.addData("slamra just Y", d.robot.slamra.getLastReceivedCameraUpdate().pose.getY());

//            telemetry.addData("duck pos", d.duckPos);
//            telemetry.addData("barm velocity", d.debugData1);
//            telemetry.addData("tarm velocity", d.debugData2);
//            telemetry.addData("barmAngle", d.barmAngle);
//            telemetry.addData("tarmAngle", d.tarmAngle);
//            telemetry.addData("x", d.arm.getCartesian().x);
//            telemetry.addData("y", d.arm.getCartesian().y);
//            telemetry.addData("z", d.arm.getCartesian().z);

            //telemetry.addData("goal position", d.goalPosition);

//            telemetry.addData("goal position", d.goalBox);

//            telemetry.addData("H", d.hsvValues[0]);
//            telemetry.addData("S", d.hsvValues[1]);
//            telemetry.addData("V", d.hsvValues[2]);

//            telemetry.addData("height", d.powerCenter.x);


//            telemetry.addData("heading", d.heading);
//            telemetry.addData("bucket", d.robot.bucket.getPosition());
//            telemetry.addData("", d.robot.shooter.getCurrentPosition() - last);
//            last = d.robot.shooter.getCurrentPosition();

//            telemetry.addData("l button", d.manip.l());

//            DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)d.robot.shooter.getController();
//            int motorIndex = ((DcMotorEx)d.robot.shooter).getPortNumber();
//            telemetry.addData("P", motorControllerEx.);
            //PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
            //motorControllerEx.setPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//            DcMotorEx motorExLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "shooter");
//            PIDCoefficients pidModified = motorExLeft.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//            telemetry.addData("P", pidModified.p);
//            telemetry.addData("I", pidModified.i);
//            telemetry.addData("D", pidModified.d);
//            telemetry.addData(d.robot.shooter.)
//            telemetry.addData("bucket pos", d.robot.bucket.getPosition());
//            telemetry.addData("pid orig p", pidOrig.p);
//            telemetry.addData("pid orig i", pidOrig.i);
//            telemetry.addData("pid orig d", pidOrig.d);
//            telemetry.addData("Yeetor speed", d.manip.lt());
//            telemetry.addData("Stack Height", d.stackHeight);
//            telemetry.addData("Motor Velocity", d.robot.shooterEx.getVelocity());
//            telemetry.addData("fright Velocity", d.robot.frightEx.getVelocity());
            telemetry.addData("Robot is here", "\n"+d.field);
            telemetry.addData("position", d.wPos.x + "   " + d.wPos.y);
            telemetry.addData("heading", d.heading);
            telemetry.update();

            //Camera camera = new Camera(hardwareMap,false);
            //camera.cycle();
            //d.robot.yeetCam.cycle();

            mecanumDrive.driveMecanum(
                    d.currentCommand.motionSpeed,
                    d.currentCommand.speed,
                    d.currentCommand.orientationSpeed);



            d.progress = MathUtil.findMaxList(
                    motionCalc == null ? 0 : motionCalc.myProgress(d),
                    orientationCalc == null ? 0 : orientationCalc.myProgress(d),
                    speedCalc == null ? 0 : speedCalc.myProgress(d));


            for (Interfaces.OtherCalc calc : otherCalc) d.progress = Math.max(d.progress,calc.myProgress(d));


            Thread.sleep(10);


            if (!opModeIsActive()) throw new InterruptedException();
        }
    }

    //How data is transferred between calculators and complexOp
    public Interfaces.MoveData d = new Interfaces.MoveData();//if you delete this the world will end


    void initHardware(HardwareMap hwMap) {

        telemetry.addData("ENTERED INIT HARDWARE", "<-");
        d.telemetry = telemetry;
        d.robot = new RobotMap(hwMap);//, startPositionAndOrientation());
//        d.robot.slamra.setPose(new Pose2d(0, 0,
////                startPositionAndOrientation().StartPos.x/100,
////                startPositionAndOrientation().StartPos.y/100,
//
//                new Rotation2d(0)));//Math.toRadians(startPositionAndOrientation().StartHeading))));
//        d.robot.slamra.start();


        mecanumDrive = new MecanumDrive(d);
    }

    public abstract Interfaces.MoveData.StartData startPositionAndOrientation();

    public abstract void body() throws InterruptedException;

    public void initMove() throws InterruptedException{

    }

    void exit(){//so we don't run into a wall at full speed
//        d.lastFrameBarmPos = d.robot.barm.getCurrentPosition();
//        d.lastFrameTarmPos = d.robot.tarm.getCurrentPosition();
//        d.lastFrameSarmPos = d.robot.sarm.getCurrentPosition();
//        d.initBarmPos = d.initBarmPos - d.robot.barm.getCurrentPosition();// - d.firstFrameBarmPos ;
//        d.initTarmPos = d.initTarmPos - d.robot.tarm.getCurrentPosition();// - d.firstFrameTarmPos ;
//        d.initSarmPos = d.initSarmPos - d.robot.sarm.getCurrentPosition();// - d.firstFrameSarmPos ;
        d.robot.bright.setPower(0);
        d.robot.fright.setPower(0);
        d.robot.bleft.setPower(0);
        d.robot.fleft.setPower(0);
//        d.robot.tarm.setPower(0);
//        d.robot.barm.setPower(0);
//        d.robot.sarm.setPower(0);
        d.robot.IntakeCam.stopStreaming();
//        d.robot.slamra.stop();

    }



    @Override
    public void runOpMode() throws InterruptedException{

        //INITIALIZATION
        telemetry.addData("Initializing", "Started");
        telemetry.update();


        d.isFinished = false;
        d.isStarted = false;

        d.driver = new CompleteController();
        d.manip = new CompleteController();

        d.driver.CompleteController(gamepad1);
        d.manip.CompleteController(gamepad2);
        //
        //
        //try {
        initHardware(hardwareMap);
        //} catch (Exception e){
        //    StringWriter sw = new StringWriter();
        //    PrintWriter pw = new PrintWriter(sw);
        //    e.printStackTrace(pw);
        //    telemetry.addData(sw.toString(), "this");
        //}
        //
        //

        //START POSITION
        if (d.startData == null) {
            d.startData = this.startPositionAndOrientation();
            d.preWPos.set(d.startData.StartPos.clone());
            d.wPos.set(d.startData.StartPos.clone());
        }

        final Interfaces.OtherCalc posDisplay = OtherCalcs.TelemetryPosition();
        posDisplay.CalcOther(d);

        telemetry.addData("Place robot here", "\n"+d.field);
        telemetry.addData("heading"," "+d.startData.StartNorthOffset +" | position: ("+String.valueOf(Math.round(d.startData.StartPos.x))+", "+String.valueOf(Math.round(d.startData.StartPos.y))+")");
        telemetry.update();

//        d.firstFrameBarmPos = d.robot.barm.getCurrentPosition();
//        d.firstFrameTarmPos = d.robot.tarm.getCurrentPosition();
//        d.firstFrameSarmPos = d.robot.sarm.getCurrentPosition();
        d.firstLiftPos = d.robot.lift.getCurrentPosition();

        initMove();




        waitForStart();



//        initPose = d.robot.slamra.getLastReceivedCameraUpdate().pose;
//        d.telemetry.addData("confidence", d.robot.slamra.getLastReceivedCameraUpdate().confidence);
//        initPoseX = initPose.getTranslation().getX();
//        initPoseY = initPose.getTranslation().getY();
        d.telemetry.update();

        d.isStarted = true;

        //BODY
        try {
            body();
        } catch (InterruptedException ie) {
            telemetry.addData("Interrupted","Exception");
            telemetry.update();
        }
        telemetry.addData("Body", "Finished");
        telemetry.update();

        //EXIT
        telemetry.addData("Exit", "Started");
        telemetry.update();
        exit();
        d.isFinished = true;
        telemetry.addData("Exit", "Finished");
        telemetry.update();
    }
}
