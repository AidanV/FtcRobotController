package org.firstinspires.ftc.teamcode.ftc10650.Tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Calculators.*;
import org.firstinspires.ftc.teamcode.Op.ComplexOp;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;

@TeleOp(name = "Odom Test", group = "ftc10650")
public class OdometryTest extends ComplexOp {

    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(50, 50), 90);
    }

    @Override
    public void initMove() throws InterruptedException {
        while(d.robot.bop.getState()) {
            d.robot.barm.setPower(-0.03);
            d.robot.tarm.setPower(-0.02);
        }
        d.robot.barm.setPower(0.0);
        d.robot.barm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.initBarmPos = d.robot.barm.getCurrentPosition();
//        d.robot.barm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(d.robot.top.getState()) {
            d.robot.tarm.setPower(0.03);
        }
        d.robot.tarm.setPower(0.0);
        d.robot.tarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.initTarmPos = d.robot.tarm.getCurrentPosition();
//        d.robot.tarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void body() throws InterruptedException {
        ComplexMove(
//                null,
//null,
//null,
//                SpeedCalcs.SetSpeed(1.0),
               SpeedCalcs.JoystickSpeed(),
//                MotionCalcs.ObjectCentricJoystick(),
                MotionCalcs.FieldCentricJoystick(0),
                //MotionCalcs.ConstantDistanceToPoint(100, new Vector2D(100,100)),
                OrientationCalcs.turnWithJoystick(),
//                OrientationCalcs.lookToOrientationUnderJoystick(0),
//                OrientationCalcs.lookToGoal(),
//                OrientationCalcs.lookToPower(),
                OtherCalcs.Arm(),
                OtherCalcs.Duck(),
                OtherCalcs.TelemetryPosition());
                /*OrientationCalcs.lookToPointTurnWithBumperTurnWithJoystick(
                        "a",
                        new OrientationCalcs.lookProgress(new Vector2D(0,0),0.95),
                        new OrientationCalcs.lookProgress(new Vector2D(150,150),1.0)),*/
        /**
         * OtherCalcs.armPath(speeds... , points...
         */
    }
}
