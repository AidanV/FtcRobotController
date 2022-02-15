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
        return new Interfaces.MoveData.StartData(new Vector2D(6,1), 90);
    }

    @Override
    public void body() throws InterruptedException {
        ComplexMove(

                SpeedCalcs.StandardRampUpDown(
                        0.1, 1.0, 0.3
                ),

                MotionCalcs.PointMotion(
                        5,
                        new Vector2D(2, 1)
                ),

                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.2, 0.8, -180)
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
