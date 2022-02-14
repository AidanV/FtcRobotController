package org.firstinspires.ftc.teamcode.ftc10650.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Calculators.Interfaces;
import org.firstinspires.ftc.teamcode.Calculators.MotionCalcs;
import org.firstinspires.ftc.teamcode.Calculators.OrientationCalcs;
import org.firstinspires.ftc.teamcode.Calculators.OtherCalcs;
import org.firstinspires.ftc.teamcode.Calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.Op.ComplexOp;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;

@Autonomous(name = "Red Duck Auto", group = "Red")
public class RedDuckAuto extends ComplexOp {

    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(200,200), 90);
    }

    @Override
    public void body() throws InterruptedException {
        ComplexMove(
                SpeedCalcs.SetProgressSpeed(
                        new SpeedCalcs.ProgressSpeed(0.05, 0.0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),

                        new SpeedCalcs.ProgressSpeed(0.2, 0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),

                        new SpeedCalcs.ProgressSpeed(0.05, 1.0, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)
                ),

                MotionCalcs.PointMotion(
                        5,
                        new Vector2D(100, 200)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.5, -180)
                ),
//                OrientationCalcs.lookToOrientation(-90),


                OtherCalcs.Lift(
                        d.cameraLiftPos,
                        0.25
                )
        );

        d.duckPos = d.robot.duckSpotPipeline.getDuckPos();
    }
}
