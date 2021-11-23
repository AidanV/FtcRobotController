package org.firstinspires.ftc.teamcode.Utilities;


import org.firstinspires.ftc.teamcode.Calculators.Interfaces;
import org.firstinspires.ftc.teamcode.Calculators.SpeedCalcs;

import java.util.Vector;

public class Arm {
    Interfaces.MoveData d;

    Vector<Block> blockAvoid = new Vector<Block>();

    int whichLeg = 0;

    Vector<Vector3D> pointPath = new Vector<Vector3D>();
    Vector<XYTheta> xyThetaPath = new Vector<XYTheta>();

    Vector<SpeedCalcs.ProgressSpeed> speedPath = new Vector<SpeedCalcs.ProgressSpeed>();

    Vector3D targetR3 = new Vector3D();
    Vector3D currentR3 = new Vector3D();

    Vector2D currentXY = new Vector2D();

    float speed = 0;

    final float barmLength = (float) 0.29;
    final float tarmLength = (float) 0.33;

    final float deltaTime = 0.1f;

    double iTime = System.currentTimeMillis();


    public Arm(Interfaces.MoveData d) {
        this.d = d;
    }


//    public void setTargetCartesian(float x, float y, float z){
//        targetR3.x = x;
//        targetR3.y = y;
//        targetR3.z = z;
//    }


    public Vector3D getCartesian() {
        return currentR3;
    }

    public void setArmPath(Vector<Vector3D> points) {
        whichLeg = 0;
        pointPath.clear();
        pointPath.add(currentR3);
        pointPath.addAll(points);
    }

//    public void setXYThetaPath(Vector<XYTheta> xytheta){
//        whichLeg = 0;
//        xyThetaPath.clear();
//        xyThetaPath.add();
//        xyThetaPath.addAll(xytheta);
//    }

    public void setProgressSpeeds(Vector<SpeedCalcs.ProgressSpeed> speeds) {
        speedPath.clear();
        speedPath.addAll(speeds);
    }

    public void updateTargetSpeed() {
        float totalDistance = 0;
        for (int i = 1; i < pointPath.size(); i++) {
            totalDistance += pointPath.get(i).to(pointPath.get(i - 1)).length();
        }
        float currDistance = 0;
        for (int i = 1; i < whichLeg + 1; i++) {
            if (i < whichLeg) {
                currDistance += pointPath.get(i).to(pointPath.get(i - 1)).length();
            } else {
                currDistance += pointPath.get(i - 1).to(currentR3).length();
            }
        }
        float progress = currDistance / totalDistance;
        for (int i = 0; i < speedPath.size(); i++) {
            if (speedPath.get(i).atDimension > progress) {

                this.speed =
                        (float) ((speedPath.get(i).rampToSpeed - speedPath.get(i - 1).rampToSpeed)
                                * (progress - speedPath.get(i).atDimension)
                                / (speedPath.get(i).atDimension - speedPath.get(i - 1).atDimension)
                                + speedPath.get(i - 1).rampToSpeed);
                break;
            }
        }
    }


    public void addAvoidanceBlock(Vector3D v1, Vector3D v2) {
        blockAvoid.add(new Block(v1, v2));
    }



    public void setArm2DVelocity(double xCommand, double yCommand) {

        xCommand *= 0.1;
        yCommand *= 0.1;

//        d.telemetry.addData("currentX", currentXY.x);
//        d.telemetry.addData("currentY", currentXY.y);
        float x = (float) (currentXY.x + xCommand);
        float y = (float) (currentXY.y + yCommand);
//        d.telemetry.addData("x", x);
//        d.telemetry.addData("y", y);
        float tarmAng = (float) Math.acos((x * x + y * y - barmLength * barmLength - tarmLength * tarmLength) / (2 * barmLength * tarmLength));


        float barmNegAng = (float) (Math.atan2(y, x) - Math.atan2((tarmLength * Math.sin(-tarmAng)), (barmLength + tarmLength * Math.cos(-tarmAng))));
        float barmAng = (float) (Math.atan2(y, x) - Math.atan2((tarmLength*Math.sin(tarmAng)),(barmLength + tarmLength*Math.cos(tarmAng))));

        if(Math.abs(Math.PI/2.0-barmAng)>=Math.abs(Math.PI/2.0-barmNegAng)){
            barmAng = barmNegAng;
            tarmAng *= -1.0;
        }

//        d.telemetry.addData("tarmAng", tarmAng);
//        d.telemetry.addData("barmAng", barmAng);
        float deltaTarmAng = (float) (Math.toDegrees(tarmAng + barmAng) - d.tarmAngle);
        float deltaBarmAng = (float) (Math.toDegrees(barmAng) - d.barmAngle);
//        d.telemetry.addData("deltaBarmAng", deltaBarmAng);
//        d.telemetry.addData("deltaTarmAng", deltaTarmAng);


//        d.telemetry.addData("speed command barm", d.tickPerDegreeBarm * deltaBarmAng / deltaTime);
//        d.telemetry.addData("speed command tarm", d.tickPerDegreeTarm * deltaTarmAng / deltaTime);
        d.robot.barmEx.setVelocity(d.tickPerDegreeBarm * deltaBarmAng / deltaTime);
        d.robot.tarmEx.setVelocity(d.tickPerDegreeTarm * deltaTarmAng / deltaTime);
//        d.robot.barmEx.setVelocity(Math.abs((d.tickPerDegreeBarm * deltaBarmAng / deltaTime))>1000?1000*Math.signum(d.tickPerDegreeBarm * deltaBarmAng / deltaTime):d.tickPerDegreeBarm * deltaBarmAng / deltaTime);
//        d.robot.tarmEx.setVelocity(Math.abs((d.tickPerDegreeBarm * deltaBarmAng / deltaTime))>1000?1000*Math.signum(d.tickPerDegreeBarm * deltaBarmAng / deltaTime):d.tickPerDegreeBarm * deltaBarmAng / deltaTime);

    }

    public void thetaVelocity(double thetaCommand){
        double theta = thetaCommand + d.sarmAngle;
        d.telemetry.addData("theta", theta);
        if(theta < -170) theta = 190-(theta-(-170));
        if(theta > 190) theta = 190;
        else if(theta < -150) theta = -150;
        d.telemetry.addData("theta 2", theta);
        d.telemetry.addData("speed", speed);
        d.telemetry.addData("theta velocity", speed*(d.tickPerDegreeSarm * (theta) - d.sarmAngle) / deltaTime);
//        d.robot.sarmEx.setVelocity((d.tickPerDegreeSarm * (theta) - d.sarmAngle) / deltaTime);
    }

    public void update(){
        d.barmAngle = (float) ((d.robot.barm.getCurrentPosition() - d.initBarmPos - d.initBarmPosOffsetFromZeroTicsToHorizontal) / d.tickPerDegreeBarm);
        d.tarmAngle = (float) ((d.robot.tarm.getCurrentPosition() - d.initTarmPos - d.initTarmPosOffsetFromZeroTicsToHorizontal) / d.tickPerDegreeTarm);
        d.sarmAngle = (float) ((d.robot.sarm.getCurrentPosition() - d.initSarmPos) / d.tickPerDegreeSarm);


        double groundLength = barmLength * Math.cos(Math.toRadians(d.barmAngle)) + tarmLength * Math.cos(Math.toRadians(d.tarmAngle));
        currentR3.x = (float) (groundLength * Math.cos(Math.toRadians(d.sarmAngle)));
        currentR3.y = (float) (groundLength * Math.sin(Math.toRadians(d.sarmAngle)));
        currentR3.z = (float) (barmLength * Math.sin(Math.toRadians(d.barmAngle)) + tarmLength * Math.sin(Math.toRadians(d.tarmAngle)));

        currentXY.x = barmLength * Math.cos(Math.toRadians(d.barmAngle)) + tarmLength * Math.cos(Math.toRadians(d.tarmAngle));
        currentXY.y = barmLength * Math.sin(Math.toRadians(d.barmAngle)) + tarmLength * Math.sin(Math.toRadians(d.tarmAngle));
    }



    public void move(){


        if(currentR3.to(pointPath.get(whichLeg)).length()<.01) {
            if(whichLeg < pointPath.size()) whichLeg++;
            targetR3 = pointPath.get(whichLeg);
        }
//

        Vector2D xyPlane = new Vector2D(targetR3.x, targetR3.y);

        xyPlane.subtract(currentXY);
        xyPlane.normalize();

//        Vector3D deltaR3 = currentR3.to(targetR3);
        updateTargetSpeed();
        xyPlane.multiply(speed);
        setArm2DVelocity(xyPlane.x, xyPlane.y);
        double theta = targetR3.z;

        if(theta < -170) theta = 190-(theta-(-170));
        if(theta > 190) theta = 190;
        else if(theta < -150) theta = -150;
//        d.robot.sarmEx.setVelocity(speed*(d.tickPerDegreeSarm * (theta) - d.sarmAngle) / deltaTime);
//        double totalTime = deltaR3.length()/metersPerSecond;

//        double deltaTime = (System.currentTimeMillis()-iTime)/1000.0;
        //object avoidance


        //
//        float deltaTime = 0.01f;
//        deltaR3.normalize();
//        deltaR3.scalar((float) (metersPerSecond * (deltaTime)));
//        Vector3D currTarget = currentR3.add(deltaR3);
//        float p = (float)Math.sqrt(currTarget.x*currTarget.x + currTarget.y*currTarget.y + currTarget.z*currTarget.z);
//        float theta = (float) Math.atan2(currTarget.y, currTarget.x);
//        float phi = (float) Math.acos(currTarget.z/p);
//        float x = (float) (p*Math.sin(phi));
//        float y = (float) (p*Math.cos(phi));
//        d.telemetry.addData("currentR3", currentR3);
//        d.telemetry.addData("x", x);
//        d.telemetry.addData("y", y);
//        float tarmAng = (float) Math.acos((x*x + y*y - barmLength*barmLength - tarmLength*tarmLength)/(2*barmLength*tarmLength));
//        float barmAng = (float) (Math.atan2(y, x) - Math.atan2((tarmLength*Math.sin(tarmAng)),(barmLength + tarmLength*Math.cos(tarmAng))));
//
//        d.telemetry.addData("barmTargAng", barmAng);
//        d.telemetry.addData("tarmTargAng", tarmAng);
//
//        float deltaTarmAng = (float) (Math.toDegrees(tarmAng + barmAng) - d.tarmAngle);
//        float deltaBarmAng = (float) (Math.toDegrees(barmAng) - d.barmAngle);

//        d.telemetry.addData("speed command barm", d.tickPerDegreeBarm * deltaBarmAng / deltaTime);
//        d.telemetry.addData("speed command tarm", d.tickPerDegreeTarm * deltaTarmAng / deltaTime);
//        d.telemetry.addData("speed command sarm", d.tickPerDegreeSarm * (Math.toDegrees(theta) - d.sarmAngle) / deltaTime);
//                d.robot.barmEx.setVelocity(d.tickPerDegreeBarm * deltaBarmAng / deltaTime);
//        d.robot.tarmEx.setVelocity(d.tickPerDegreeTarm * deltaTarmAng / deltaTime);
//        if(theta < -170) theta = 190-(theta-(-170));
//        if(theta > 190) theta = 190;
//        else if(theta < -150) theta = -150;
//        d.robot.sarmEx.setVelocity(d.tickPerDegreeSarm * (Math.toDegrees(theta) - d.sarmAngle) / deltaTime);

        /**
         max theta should be like -150 degrees
         min theta should be around 190 degrees
         */

//        d.robot.barm.setTargetPosition((int) barmAng);
//        d.robot.tarm.setTargetPosition((int) (barmAng + tarmAng));
//
//        iTime = System.currentTimeMillis();
    }






//    public PolarBaseCartesianArm getPolarBaseCartesianArm(){
//        float theta = 0;
//        Vector2D v = new Vector2D();
//
//        return new PolarBaseCartesianArm(theta, v);
//    }



}

class XYTheta{
    public Vector2D v;
    public float theta;
    XYTheta (Vector2D v, float theta){
        this.theta = theta;
        this.v = v;
    }
}

class Block {
    public Vector3D top;
    public Vector3D bottom;
    Block (Vector3D top, Vector3D bottom){
        this.top = top;
        this.bottom = bottom;
    }
}