package org.firstinspires.ftc.teamcode.ftc10650.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.calculators.Interfaces;
import org.firstinspires.ftc.teamcode.calculators.MotionCalcs;
import org.firstinspires.ftc.teamcode.calculators.OrientationCalcs;
import org.firstinspires.ftc.teamcode.calculators.OtherCalcs;
import org.firstinspires.ftc.teamcode.calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.hardware.FreightRobotName_NA.RobotMap;
import org.firstinspires.ftc.teamcode.op.ComplexOp;
import org.firstinspires.ftc.teamcode.utilities.Vector2D;

@Autonomous(name = "Blue Duck Warehouse Auto", group = "Blue")
public class BlueDuckWarehouseAuto extends ComplexOp {

    @Override
    public Interfaces.MoveData.StartData startPositionAndOrientation() {
        return new Interfaces.MoveData.StartData(new Vector2D(0.0 + (.353),1.0 + (.3142)), -90);
    }

    @Override
    public void body() throws InterruptedException {


        //drive to vision position
        ComplexMove(
                SpeedCalcs.SetSpeed(0.05),
                MotionCalcs.PointMotion(5, new Vector2D(0.5, 1.3142)),
                OrientationCalcs.spinToProgress(new OrientationCalcs.spinProgress(0.0, 1.0, 90)),
                OtherCalcs.Lift(d.cameraLiftPos, 0.25));

        //wait 500 millis for vision
        ComplexMove(null, null, null, OtherCalcs.TimeProgress(500));

        //grab duck position
        d.duckPos = d.robot.duckSpotPipeline.getDuckPos();
        //blue side fix
        d.duckPos = (d.duckPos + 1) % 3;

        //grab capstone
        ComplexMove(null, null, null, OtherCalcs.AutoCupGrabBlue(false, 5000));

        //drive to alliance shipping hub
        ComplexMove(
                SpeedCalcs.StandardRampUpDown(0.1, 0.4, 0.3),
                MotionCalcs.PointMotion(0.1, new Vector2D(1.9, 1.7)),
                OrientationCalcs.spinToProgress(new OrientationCalcs.spinProgress(0.0, 0.6, 180)),
                OtherCalcs.Lift(d.cubeLiftPositions[d.duckPos], 0.25));

        //place cube on shipping hub
        ComplexMove(null, null, null, OtherCalcs.AutoPlaceCube(2000));

        //drive near carousel
        ComplexMove(
                SpeedCalcs.StandardRampUpDown(0.1, 0.6, 0.4),
                MotionCalcs.PointMotion(0.1, new Vector2D(1.9, 1.0), new Vector2D(0.75, 0.75)),
                OrientationCalcs.spinToProgress(new OrientationCalcs.spinProgress(0.1, 0.75, 360)),
                OtherCalcs.Lift(d.intakeLiftPos, 0.25));

        //drive against carousel
        ComplexMove(
                SpeedCalcs.SetSpeed(0.05),
                MotionCalcs.PointMotion(0.1, new Vector2D(0.5, 0.5)),
                OrientationCalcs.spinToProgress(new OrientationCalcs.spinProgress(0.0, 0.5, 360)),
                OtherCalcs.StopAtStall(3.0, RobotMap.brightEx));

        //spin carousel
        ComplexMove(null, null, null, OtherCalcs.AutoDuckBlue(5000));

        //park in warehouse
        ComplexMove(
                SpeedCalcs.StandardRampUpDown(0.1, 0.7, 0.2),
                MotionCalcs.PointMotion(0.0, new Vector2D(1.2, 3.0), new Vector2D(1.2, 5.2)),
                OrientationCalcs.spinToProgress(new OrientationCalcs.spinProgress(0.0, 0.5, 360)),
                OtherCalcs.Lift(d.safeLiftPos, 0.25));

        //lower lift to reset position
        ComplexMove(null, null, null,
                OtherCalcs.Lift(d.intakeLiftPos, 0.25),
                OtherCalcs.TimeProgress(5000));
    }
}