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

public class FindDuckPipeline extends OpenCvPipeline {
    Mat hsv = new Mat();
    Mat hsvIntaked = new Mat();
    final Scalar lower = new Scalar(10, 50, 20);//150, 90
    final Scalar upper = new Scalar(30, 255, 255);
    public volatile int duckOffset = -161;
    public volatile boolean intaked = false;


    @Override
    public Mat processFrame(Mat input) {
        try {
            Mat nonCroppedHsv = new Mat();
            Mat mask = new Mat();
            Mat hierarchy = new Mat();


            //conver the input image to hsv
            Imgproc.cvtColor(input, nonCroppedHsv, Imgproc.COLOR_RGB2HSV);

            Rect rectCrop = new Rect(input.width()/6, input.height()/2, input.width()/2, input.height()/3);
            hsv = new Mat(nonCroppedHsv, rectCrop);
            Rect rectCropIntaked = new Rect(input.width()/6, input.height()*3/8, input.width()/2, input.height()/8);
            hsvIntaked = new Mat(nonCroppedHsv, rectCropIntaked);


            Scalar s = Core.mean(hsvIntaked);

            intaked = s.val[2] > 100;


            //mask image
            Core.inRange(hsv, lower, upper, mask);


            //erode and dilate image to reduce noise
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
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


            //find the center of the bounding rectangle
            Point centerPosition = new Point((boundingRect.tl().x + boundingRect.br().x) / 2, (boundingRect.tl().y + boundingRect.br().y) / 2);


            duckOffset = (int) (centerPosition.x - rectCrop.width/2);
            Imgproc.putText(
                    input, duckOffset + "\n" + s.val[2],
                    new Point(input.width()/2, input.height()/2),
                    FONT_HERSHEY_SIMPLEX,
                    2,
                    new Scalar(0, 255, 0),
                    2);

            Imgproc.rectangle(input, boundingRect, new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(input, rectCrop, new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(input, rectCropIntaked, new Scalar(0, 255, 0), 3);


            //clean-up
            nonCroppedHsv.release();
            kernel.release();
            largestContour.release();
            mask.release();
            hierarchy.release();
            hsv.release();
            hsvIntaked.release();


            //display the original image
            return input;
        } catch (Exception e) {
            return input;
        }
    }


}