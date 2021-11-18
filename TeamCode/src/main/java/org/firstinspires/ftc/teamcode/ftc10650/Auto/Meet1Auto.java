package org.firstinspires.ftc.teamcode.ftc10650.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Calculators.Interfaces.MoveData;
import org.firstinspires.ftc.teamcode.Calculators.MotionCalcs;
import org.firstinspires.ftc.teamcode.Calculators.OrientationCalcs;
import org.firstinspires.ftc.teamcode.Calculators.OtherCalcs;
import org.firstinspires.ftc.teamcode.Calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.Op.ComplexOp;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;


@Autonomous(name = "Meet 1 Auto", group = "ftc10650")
public class Meet1Auto extends ComplexOp {

    @Override
    public MoveData.StartData startPositionAndOrientation() {
        return new MoveData.StartData(new Vector2D(150,0), 90);
    }

    @Override
    public void body() throws InterruptedException {

        ComplexMove(
                SpeedCalcs.SetProgressSpeed(
                        new SpeedCalcs.ProgressSpeed(0.4,0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
                        new SpeedCalcs.ProgressSpeed(0.6,0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
                        new SpeedCalcs.ProgressSpeed(0.4,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
                MotionCalcs.PointMotionNoProgress(5,
                        new Vector2D(200, 0)
//                        new Vector2D(-300, 0),
//                        new Vector2D(-150, -150),
//                        new Vector2D(0, 0)
                        /*,
                        new Vector2D(0, 0)*/),
OrientationCalcs.lookToOrientation(0),
                OtherCalcs.TimeProgress(2000)

//                OrientationCalcs.spinToProgress(
//                        new OrientationCalcs.spinProgress(0.15, 0.2, 90),
//                        new OrientationCalcs.spinProgress(0.75, 0.85, 0))
                /*OtherCalcs.DistanceStop(OtherCalcs.Side.LEFT,150,145,0.95,1)*/);

//        ComplexMove(
//                SpeedCalcs.SetProgressSpeed(
//                        new SpeedCalcs.ProgressSpeed(1,0.2, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.1,0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(1,0.4, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.75,0.95, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
//                        new SpeedCalcs.ProgressSpeed(0.1,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
//                MotionCalcs.PointMotion(5,
//                        new Vector2D(-150, 150),
//                        new Vector2D(-300, 0),
//                        new Vector2D(-150, -150),
//                        new Vector2D(0, 0)),
//                OrientationCalcs.spinToProgress(
//                        new OrientationCalcs.spinProgress(0.1, 0.2, 180),
//                        new OrientationCalcs.spinProgress(0.75, 0.85, -90)),
//                OtherCalcs.DistanceStop(OtherCalcs.Side.BACK,1,0,0.95,1));

 
    }
}
