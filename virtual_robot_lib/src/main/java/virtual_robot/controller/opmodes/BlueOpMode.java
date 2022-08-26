package virtual_robot.controller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

import java.util.Random;

import virtual_robot.controller.robots.classes.MechanumBot;

public class BlueOpMode extends LinearOpMode {
  DcMotorImpl bleft, fleft, bright, fright;
  Random r = new Random();
  @Override
  public void runOpMode() throws InterruptedException {
    //TODO fix interruptions

    bleft = (DcMotorImpl) hardwareMap.dcMotor.get("bleft");
    bleft.setId("bleft");
    fleft = (DcMotorImpl) hardwareMap.dcMotor.get("fleft");
    fleft.setId("fleft");
    bright = (DcMotorImpl) hardwareMap.dcMotor.get("bright");
    bright.setId("bright");
    fright = (DcMotorImpl) hardwareMap.dcMotor.get("fright");
    fright.setId("fright");
//    while(!opModeIsActive()) {
//    motorRight = hardwareMap.get(DcMotor.class, "motorRight");
      bleft.setPower(Math.sin(System.currentTimeMillis() / 10000.0));
      fleft.setPower(-Math.sin(System.currentTimeMillis() / 10000.0));
      bright.setPower(Math.sin(System.currentTimeMillis() / 10000.0));
      fright.setPower(-Math.sin(System.currentTimeMillis() / 10000.0));
//    }
  }
}
