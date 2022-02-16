package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX;

public class IntakedPipeline extends OpenCvPipeline {


    private volatile boolean isIntaked = false;


    @Override
    public Mat processFrame(Mat input) {
        try {

            //create hsv mats
            Mat nonCroppedHsv = new Mat();
            Mat hsv = new Mat();


            //conver the input image to hsv
            Imgproc.cvtColor(input, nonCroppedHsv, Imgproc.COLOR_RGB2HSV);


            //crop the hsv mat
            Rect rectCrop = new Rect(nonCroppedHsv.width()/6, (nonCroppedHsv.height()*7/16), (nonCroppedHsv.width()/3), (nonCroppedHsv.height()/8));
            hsv = new Mat(nonCroppedHsv, rectCrop);


            //read the average hsv values of the image
            Scalar s = Core.mean(hsv);


            //find if the average hsv "value" is above 150
            isIntaked = s.val[2] > 150;


            //cleanup
            nonCroppedHsv.release();
            hsv.release();


            //display input image
            return input;
        } catch (Exception e){
            return input;
        }
    }
    public boolean isIntaked(){
        return isIntaked;
    }

}
