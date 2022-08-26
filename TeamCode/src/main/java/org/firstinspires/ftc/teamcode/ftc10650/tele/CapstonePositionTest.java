package org.firstinspires.ftc.teamcode.ftc10650.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calculators.Interfaces;
import org.firstinspires.ftc.teamcode.calculators.OtherCalcs;
import org.firstinspires.ftc.teamcode.calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.op.ComplexOp;
import org.firstinspires.ftc.teamcode.utilities.Vector2D;
import org.firstinspires.ftc.teamcode.utilities.Vector3D;

import java.util.Vector;



@TeleOp(name = "capstone test")
public class CapstonePositionTest extends ComplexOp {

    Vector<SpeedCalcs.ProgressSpeed> s = new Vector<SpeedCalcs.ProgressSpeed>();
    Vector<Vector3D> p = new Vector<Vector3D>();

    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(0, 0), 0);
    }

    @Override
    public void initMove() throws InterruptedException {

    }

    @Override
    public void body() throws InterruptedException {
//        d.robot.clawCam.setPipeline(d.robot.cubeFindPipeline);
        ComplexMove(
                null,
                null,
                null,
                OtherCalcs.TestCapstone()
        );
        /**
         * OtherCalcs.armPath(speeds... , points...
         */
    }
}

