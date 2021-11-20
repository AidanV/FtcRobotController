package org.firstinspires.ftc.teamcode.Utilities;


import org.firstinspires.ftc.teamcode.Calculators.Interfaces;
import org.firstinspires.ftc.teamcode.Calculators.SpeedCalcs;

import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

public class Arm {
    Interfaces.MoveData d;

    Vector<Block> blockAvoid = new Vector<Block>();

    int whichLeg = 0;

    Vector<Vector3D> pointPath = new Vector<Vector3D>();

    Vector<SpeedCalcs.ProgressSpeed> speedPath = new Vector<SpeedCalcs.ProgressSpeed>();

    Vector3D targetR3 = new Vector3D();
    Vector3D currentR3 = new Vector3D();

    float metersPerSecond = 0;

    final float barmLength = (float) 0.3;
    final float tarmLength = (float) 0.3;

    double iTime = System.currentTimeMillis();


    public Arm(Interfaces.MoveData d){
        this.d = d;
    }


//    public void setTargetCartesian(float x, float y, float z){
//        targetR3.x = x;
//        targetR3.y = y;
//        targetR3.z = z;
//    }


    public Vector3D getCartesian(){
        return currentR3;
    }

    public void setArmPath(Vector<Vector3D> points){
        whichLeg = 0;
        pointPath.clear();
        pointPath.add(currentR3);
        pointPath.addAll(points);
    }

    public void setProgressSpeeds(Vector<SpeedCalcs.ProgressSpeed> speeds){
        speedPath.clear();
        speedPath.addAll(speeds);
    }

    public void updateTargetSpeed(){
        float totalDistance = 0;
        for(int i = 1; i < pointPath.size(); i++){
            totalDistance += pointPath.get(i).to(pointPath.get(i-1)).length();
        }
        float currDistance = 0;
        for(int i = 0; i < whichLeg + 1; i++){
            if(i < whichLeg){
                currDistance += pointPath.get(i).to(pointPath.get(i-1)).length();
            } else {
                currDistance += pointPath.get(i-1).to(currentR3).length();
            }
        }
        float progress = currDistance/totalDistance;
        for(int i = 0; i < speedPath.size(); i++){
            if(speedPath.get(i).atDimension > progress){

                this.metersPerSecond =
                        (float) ((speedPath.get(i).rampToSpeed - speedPath.get(i-1).rampToSpeed)
                              * (progress - speedPath.get(i).atDimension)
                              / (speedPath.get(i).atDimension - speedPath.get(i-1).atDimension)
                              + speedPath.get(i-1).rampToSpeed);
                break;
            }
        }
    }


    public void addAvoidanceBlock(Vector3D v1, Vector3D v2){
        blockAvoid.add(new Block(v1, v2));
    }


    public void update(){
        d.barmAngle = (float) ((d.robot.barm.getCurrentPosition() - d.initBarmPos - d.initBarmPosOffsetFromZeroTicsToHorizontal) / d.tickPerDegreeBarm);
        d.tarmAngle = (float) ((d.robot.tarm.getCurrentPosition() - d.initTarmPos - d.initTarmPosOffsetFromZeroTicsToHorizontal) / d.tickPerDegreeTarm);
        d.sarmAngle = (float) ((d.robot.sarm.getCurrentPosition() - d.initSarmPos) / d.tickPerDegreeSarm);


        double groundLength = barmLength * Math.cos(Math.toRadians(d.barmAngle)) + tarmLength * Math.cos(Math.toRadians(d.tarmAngle));
        currentR3.x = (float) (groundLength * Math.cos(Math.toRadians(d.sarmAngle)));
        currentR3.y = (float) (groundLength * Math.sin(Math.toRadians(d.sarmAngle)));
        currentR3.z = (float) (barmLength * Math.sin(Math.toRadians(d.barmAngle)) + tarmLength * Math.sin(Math.toRadians(d.tarmAngle)));
    }



    public void move(){

        if(currentR3.to(pointPath.get(whichLeg)).length()<.01) {
            if(whichLeg < pointPath.size()) whichLeg++;
            targetR3 = pointPath.get(whichLeg);
        }


        Vector3D deltaR3 = currentR3.to(targetR3);
        updateTargetSpeed();
//        double totalTime = deltaR3.length()/metersPerSecond;

        double deltaTime = System.currentTimeMillis()-iTime;
        //object avoidance


        //
        deltaR3.normalize();
        deltaR3.scalar((float) (metersPerSecond * (deltaTime)));
        Vector3D currTarget = currentR3.add(deltaR3);
        float p = (float)Math.sqrt(currTarget.x*currTarget.x + currTarget.y*currTarget.y + currTarget.z*currTarget.z);
        float theta = (float) Math.atan(currTarget.y/currTarget.x);
        float phi = (float) Math.acos(currTarget.z/p);
        float x = (float) (p*Math.sin(phi));
        float y = (float) (p*Math.cos(phi));
        float tarmAng = (float) Math.acos((x*x + y*y - barmLength*barmLength - tarmLength*tarmLength)/(2*barmLength*tarmLength));
        float barmAng = (float) (Math.atan(y/x) - Math.atan((tarmLength*Math.sin(tarmAng))/(barmLength + tarmLength*Math.cos(tarmAng))));

        float deltaTarmAng = (float) (Math.toDegrees(tarmAng + barmAng) - d.tarmAngle);
        float deltaBarmAng = (float) (Math.toDegrees(barmAng) - d.barmAngle);

        d.robot.barmEx.setVelocity(d.tickPerDegreeBarm * deltaBarmAng / deltaTime);
        d.robot.tarmEx.setVelocity(d.tickPerDegreeTarm * deltaTarmAng / deltaTime);
//        if(theta < -170) theta = 190-(theta-(-170));
        if(theta > 190) theta = 190;
        else if(theta < -150) theta = -150;
        d.robot.sarmEx.setVelocity(d.tickPerDegreeSarm * (Math.toDegrees(theta) - d.sarmAngle) / deltaTime);

        /**
         max theta should be like -150 degrees
         min theta should be around 190 degrees
         */

//        d.robot.barm.setTargetPosition((int) barmAng);
//        d.robot.tarm.setTargetPosition((int) (barmAng + tarmAng));
//
        iTime = System.currentTimeMillis();
    }






//    public PolarBaseCartesianArm getPolarBaseCartesianArm(){
//        float theta = 0;
//        Vector2D v = new Vector2D();
//
//        return new PolarBaseCartesianArm(theta, v);
//    }



}

class PolarBaseCartesianArm{
    public Vector2D v;
    public float theta;
    PolarBaseCartesianArm (float theta, Vector2D v){
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