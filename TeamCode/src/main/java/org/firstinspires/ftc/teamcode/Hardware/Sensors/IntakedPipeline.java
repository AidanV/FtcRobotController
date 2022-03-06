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

import static org.opencv.imgproc.Imgproc.COLOR_HSV2RGB;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX;

public class IntakedPipeline extends OpenCvPipeline {


    private volatile boolean isIntaked = false;
    private double lastV = 255.0;

    final Scalar lower = new Scalar(0, 50, 20);//150, 90
    final Scalar upper = new Scalar(15, 255, 255);


    @Override
    public Mat processFrame(Mat input) {
        try {

            //create hsv mats
            Mat nonCroppedHsv = new Mat();
            Mat hsvIntake;
            Mat hsvMarker;
            Mat mask = new Mat();
            Mat hierarchy = new Mat();


            //conver the input image to hsv
            Imgproc.cvtColor(input, nonCroppedHsv, Imgproc.COLOR_RGB2HSV);

            Rect rectCropMarker = new Rect(input.width()*2/3, input.height()/3, input.width()/3, input.height()/3);
            hsvMarker = new Mat(nonCroppedHsv, rectCropMarker);


            Core.inRange(hsvMarker, lower, upper, mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
            Imgproc.erode(mask, mask, kernel);
            Imgproc.dilate(mask, mask, kernel);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxArea = 0;
            MatOfPoint largestContour = new MatOfPoint();
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                } else {
                    contour.release();
                }
            }
            Rect boundingRect = Imgproc.boundingRect(largestContour);
            boundingRect.x += input.width()*2/3;
            boundingRect.y += input.height()/3;


            //crop the hsv mat
            Rect rectCropIntake = new Rect(nonCroppedHsv.width()/8, (boundingRect.y + (boundingRect.width / 2)), (nonCroppedHsv.width()/2), (nonCroppedHsv.height()/64));

//            Rect rectCropIntake = new Rect(nonCroppedHsv.width()/2, (boundingRect.y + (boundingRect.width / 2)), (nonCroppedHsv.width()*3/32), (nonCroppedHsv.height()/64));
            hsvIntake = new Mat(nonCroppedHsv, rectCropIntake);


            //read the average hsv values of the image
            Scalar s = Core.mean(hsvIntake);


            //find if the average hsv "value" is above 150
//            isIntaked = s.val[2] > 150;


            isIntaked = s.val[2] > 110;

//            isIntaked = s.val[2] - lastV > 100;
//            lastV = 0.9*lastV + 0.1*s.val[2];


            //cleanup
            nonCroppedHsv.release();
            hsvMarker.release();
            hsvIntake.release();
            mask.release();


            //draw view box
            Imgproc.rectangle(input, rectCropIntake, new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(input, boundingRect, new Scalar(0, 255, 0), 3);


            //print "v" value
            Imgproc.putText(input, "v: " + (Math.round(s.val[2] * 100.0) / 100.0), new Point(20, input.height()-10), FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(0, 255, 0));


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
