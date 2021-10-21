package org.firstinspires.ftc.teamcode.ftc10650.Tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Calculators.*;
import org.firstinspires.ftc.teamcode.Op.ComplexOp;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;

@TeleOp(name = "Odom Test", group = "ftc10650")
public class OdometryTest extends ComplexOp {

    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(50, 50), 0);
    }

    @Override
    public void body() throws InterruptedException {
        ComplexMove(
//                SpeedCalcs.SetSpeed(1.0),
               SpeedCalcs.JoystickSpeed(),
                MotionCalcs.FieldCentricJoystick(90),
                //MotionCalcs.ConstantDistanceToPoint(100, new Vector2D(100,100)),
                //OrientationCalcs.turnWithJoystick(),
//                OrientationCalcs.lookToOrientationUnderJoystick(0),
//                OrientationCalcs.lookToGoal(),
                OrientationCalcs.lookToPower(),
                OtherCalcs.TelemetryPosition());
                /*OrientationCalcs.lookToPointTurnWithBumperTurnWithJoystick(
                        "a",
                        new OrientationCalcs.lookProgress(new Vector2D(0,0),0.95),
                        new OrientationCalcs.lookProgress(new Vector2D(150,150),1.0)),*/

    }
}
