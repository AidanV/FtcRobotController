package org.firstinspires.ftc.teamcode.ftc10650.Tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Calculators.*;
import org.firstinspires.ftc.teamcode.Hardware.FreightRobotName_NA.RobotMap;
import org.firstinspires.ftc.teamcode.Op.ComplexOp;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;

@TeleOp(name = "Blue Tele", group = "Blue")
public class BlueTele extends ComplexOp {

    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(50, 50), 0);
    }

    @Override
    public void body() throws InterruptedException {
        d.robot.IntakeCam.setPipeline(d.robot.intakedPipeline);
        ComplexMove(
            SpeedCalcs.JoystickSpeed(),
            MotionCalcs.FieldCentricJoystick(-90),
            OrientationCalcs.turnWithJoystick(),
            OtherCalcs.Duck(),
            OtherCalcs.TeleLift(),
            OtherCalcs.Intake(),
            OtherCalcs.TeleCap(),
            OtherCalcs.TelemetryPosition());
    }
}
