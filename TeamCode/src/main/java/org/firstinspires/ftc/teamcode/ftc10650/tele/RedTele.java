package org.firstinspires.ftc.teamcode.ftc10650.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calculators.*;
import org.firstinspires.ftc.teamcode.op.ComplexOp;
import org.firstinspires.ftc.teamcode.utilities.Vector2D;

@TeleOp(name = "Red Tele", group = "Red")
public class RedTele extends ComplexOp {

    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(5, 3), 0);
    }

    @Override
    public void body() throws InterruptedException {
        d.robot.IntakeCam.setPipeline(d.robot.intakedPipeline);

        ComplexMove(
                SpeedCalcs.JoystickSpeed(),
                MotionCalcs.FieldCentricJoystick(90),
                OrientationCalcs.GameOrientRed(),
                OtherCalcs.Duck(),
                OtherCalcs.TeleLift(),
                OtherCalcs.Intake(),
                OtherCalcs.TeleCap(),
                OtherCalcs.TelemetryPosition());

    }
}