package org.firstinspires.ftc.teamcode.ftc10650.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Calculators.Interfaces;
import org.firstinspires.ftc.teamcode.Calculators.Interfaces.MoveData;
import org.firstinspires.ftc.teamcode.Calculators.MotionCalcs;
import org.firstinspires.ftc.teamcode.Calculators.OrientationCalcs;
import org.firstinspires.ftc.teamcode.Calculators.OtherCalcs;
import org.firstinspires.ftc.teamcode.Calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.Op.ComplexOp;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;

import java.util.Vector;


@Autonomous(name = "Meet 1 Auto", group = "ftc10650")
public class Meet1Auto extends ComplexOp {

    @Override
    public MoveData.StartData startPositionAndOrientation() {
        return new MoveData.StartData(new Vector2D(150,0), 90);
    }

    @Override
    public void initMove() throws InterruptedException {
    }

    @Override
    public void body() throws InterruptedException {

        if (true) {
            ComplexMove(
                    null,
                    null,
                     null,
                    OtherCalcs.TimeProgress(1000)//CVD debugging
            );
        }
        if (true)
        {
            ComplexMove(
                    null,
                    null,
                    null,
                    //OtherCalcs.moveArm(90, .34, .33, .15),
                    new Interfaces.OtherCalc() {
                        @Override
                        public void CalcOther(Interfaces.MoveData d) { }
                        @Override
                        public double myProgress(Interfaces.MoveData d) {
                            return 0;
                        }
                    },
                    OtherCalcs.TimeProgress(500000)//CVD debugging
            );
        }
    }
}
