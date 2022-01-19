package org.firstinspires.ftc.teamcode.ftc10650.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.FreightRobotName_NA.RobotMap;

@TeleOp(name = "raw drive", group = "ftc10650")
public class RawDrive  extends OpMode {
    // Declare OpMode members.
    private static DcMotor bleft, fleft, bright, fright;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        bleft = hardwareMap.get(DcMotor.class, "bleft");
        fleft = hardwareMap.get(DcMotor.class, "fleft");
        bright = hardwareMap.get(DcMotor.class, "bright");
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        fright = hardwareMap.get(DcMotor.class, "fright");
        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double fleftSpeed  = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        double frightSpeed = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        double bleftSpeed  = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        double brightSpeed =  gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        bleft.setPower(bleftSpeed);
        fleft.setPower(fleftSpeed);
        bright.setPower(brightSpeed);
        fright.setPower(frightSpeed);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        bleft.setPower(0.0);
        fleft.setPower(0.0);
        bright.setPower(0.0);
        fright.setPower(0.0);
    }

}
