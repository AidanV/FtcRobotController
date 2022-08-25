package virtual_robot.controller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

import java.util.Random;

import virtual_robot.controller.robots.classes.MechanumBot;

public class BlueOpMode extends LinearOpMode {
  DcMotorImpl bleft;
  Random r = new Random();
  @Override
  public void runOpMode() throws InterruptedException {

    bleft = (DcMotorImpl) hardwareMap.dcMotor.get("bleft");
    bleft.setId("bleft");
//    while(opModeIsActive()) {
//    motorRight = hardwareMap.get(DcMotor.class, "motorRight");
      bleft.setPower(r.nextDouble());
//    }
  }
}
