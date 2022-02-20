package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX;

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
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class DuckSpotPipeline extends OpenCvPipeline {
//    Mat grey = new Mat();
    Mat hsv = new Mat();
    Mat mask = new Mat();
    Mat hierarchy = new Mat();
    private volatile int duckPos = -1;
//    final Scalar lower = new Scalar(4, 25, 25);//150, 90
//    final Scalar upper = new Scalar(50, 255, 255);// 230, 255
//    final Scalar lower = new Scalar(4, 50, 20);//150, 90
//    final Scalar upper = new Scalar(36, 255, 255);
//    final Scalar lower = new Scalar(30, 50, 50);//150, 90 //Green of Capstone
//    final Scalar upper = new Scalar(60, 255, 255);// 230, 255 //
    final Scalar lower = new Scalar(140, 10, 10);
    final Scalar upper = new Scalar(180, 255, 255);


    @Override
    public Mat processFrame(Mat input) {
        try {


            //convert to hsv
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);


            //mask image
            Core.inRange(hsv, lower, upper, mask);


            //erode and dilate image to reduce noise
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
            Imgproc.erode(mask, mask, kernel);
            Imgproc.dilate(mask, mask, kernel);


            //find all contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


            //find the largest contour
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


            //set bounding box
            Rect boundingRect = Imgproc.boundingRect(largestContour);

            //display bounding box
            Imgproc.rectangle(mask, boundingRect, new Scalar(50, 50, 50));


            //determine which position
            if (boundingRect.area() > 15_000) {
                if (boundingRect.x < (hsv.width() * 28 / 80)) {
                    duckPos = 0;
                } else {
                    duckPos = 1;
                }
            } else {
                duckPos = 2;
            }


            //display 2 in the center of the screen if the TSE is in the position of camers
            if(duckPos == 2){
                Imgproc.putText(
                        input, Integer.toString(duckPos),
                        new Point(input.width()/2, input.height()/2),
                        FONT_HERSHEY_SIMPLEX,
                        2,
                        new Scalar(0, 255, 0),
                        2);
            } else { //display the position inside of the bounding box
                Imgproc.putText(
                        input, Integer.toString(duckPos),
                        new Point(boundingRect.x + (boundingRect.width / 2), boundingRect.y + (boundingRect.height / 2)),
                        FONT_HERSHEY_SIMPLEX,
                        2,
                        new Scalar(0, 255, 0),
                        2);

                Imgproc.rectangle(input, boundingRect, new Scalar(0, 255, 0), 3);
            }


            //clean-up
            kernel.release();
            largestContour.release();
            hsv.release();


            //display the original image
            return input;
        } catch (Exception e){
            return input;
        }
    }
    public int getDuckPos(){
        return duckPos;
    }

}
