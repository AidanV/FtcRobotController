package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CubeFindPipeline extends OpenCvPipeline {
    Mat grey = new Mat();
    Mat hsv = new Mat();
    Mat mask = new Mat();
    Mat hierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {
//        final Scalar lower = new Scalar(0, 0, 20);//188.1, 148.92);
//        final Scalar upper = new Scalar(50, 255, 250);//203.7, 255);
        final Scalar lower = new Scalar(11, 50, 20);//150, 90
        final Scalar upper = new Scalar(18, 255, 250);// 230, 255


        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, lower, upper, mask);


        grey.empty();
        Core.add(grey, new Scalar(255, 0, 0), grey);
//        input.copyTo(grey, mask);
//        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((1) + 1, (1)+1));
//        Imgproc.erode(mask, mask, kernel);
//        Imgproc.dilate(mask, mask, kernel);
//
//        List<MatOfPoint> contours = new ArrayList<>();
//        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//        double maxArea = 0;
//        MatOfPoint largestContour = new MatOfPoint();
//        for (MatOfPoint contour : contours) {
//            double area = Imgproc.contourArea(contour);
//            if (area > maxArea) {
//                maxArea = area;
//                largestContour = contour;
//            }
//        }
//        Rect boundingRect = Imgproc.boundingRect(largestContour);
//        Imgproc.rectangle(mask, boundingRect, new Scalar(255, 0, 0), 3);


//        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB);
//        Core.bitwise_not(input, input, mask);
//        Core.bitwise_not(input, input, mask);
//        Core.bitwise_and(input, mask, input);
//162, 100, 42
//255, 210, 137
//149, 73, 30

//HSV
//37, 46.3%, 100%
//22, 79.9%, 58.4%

//        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        return grey;
    }
}
