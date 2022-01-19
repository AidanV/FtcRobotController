package org.firstinspires.ftc.teamcode.ftc10650.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Calculators.Interfaces.MoveData;
import org.firstinspires.ftc.teamcode.Calculators.MotionCalcs;
import org.firstinspires.ftc.teamcode.Calculators.OrientationCalcs;
import org.firstinspires.ftc.teamcode.Calculators.OtherCalcs;
import org.firstinspires.ftc.teamcode.Calculators.SpeedCalcs;
import org.firstinspires.ftc.teamcode.Op.ComplexOp;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;

import java.util.Vector;


@Autonomous(name = "Meet 3 Auto Red", group = "Red")
public class Meet3Auto extends ComplexOp {

    @Override
    public MoveData.StartData startPositionAndOrientation() {
        return new MoveData.StartData(new Vector2D(150,0), 60);//60 degrees west of south is a good angle to see the barcode
    }

    @Override
    public void initMove() throws InterruptedException {
        //Lift lift to uncover camera
        d.robot.lift.setTargetPosition(d.cameraLiftPos-d.firstLiftPos);
        d.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        d.robot.lift.setPower(0.2);
        //

    }

    @Override
    public void body() throws InterruptedException {

        //Grab Duck Position and save to data
        d.duckPos = d.robot.duckSpotPipeline.getDuckPos();


        //ToDo find the encoder tick to drop in bottom/mid/top container


        if(d.duckPos == 0) {
            d.robot.lift.setTargetPosition(d.bottomLiftPos);
            //Set lift position to bottom / move to tree
                // move includes
                // turn back to west wall
                // move to east side of tree

        }





        else if(d.duckPos == 1) {
            d.robot.lift.setTargetPosition(d.middleLiftPos);
            //Set lift position to middle / move to tree
                // move includes
                // turn back to south wall
                // move to north side of tree

        }





        // IF duck position is for the top goal or if we do not find the correct duck position,
        // we want to place the duck on the top level
        else {
            d.robot.lift.setTargetPosition(d.topLiftPos);
            //Set lift position to top / move to tree
                // move includes
                // turn back to south wall
                // move to north side of tree

        }

        // set lift to camera visible position

        // drive into storage area / right side on ground / left side on barrier
        // reset position with distance sensor
        // find cube w/ vision / turn to cube
        // lift lift to pick up position
        // drive forward / intake | until vision detects cube
        // lift lift to safe driving position
        // drive over barrier w/ half over
        // turn robot to back away from driver wall
        // lift lift to top height
        // deposit cube
        // lift lift to safe driving position
        // turn towards barrier
        // drive over center part of barrier into warehouse

        while(opModeIsActive()){
            Thread.sleep(10);
        }
    }
}
