//package org.firstinspires.ftc.teamcode.ftc10650.Tele;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.Hardware.Sensors.StackDeterminationPipeline;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//@TeleOp(name="Camera Testing")
//public class CameraTest extends LinearOpMode {
//    OpenCvInternalCamera phoneCam;
//    StackDeterminationPipeline pipeline;
//
//    @Override
//    public void runOpMode()
//    {
//        /**
//         * NOTE: Many comments have been omitted from this sample for the
//         * sake of conciseness. If you're just starting out with EasyOpenCv,
//         * you should take a look at {@link InternalCamera1Example} or its
//         * webcam counterpart, {@link WebcamExample} first.
//         */
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        pipeline = new StackDeterminationPipeline();
//        phoneCam.setPipeline(pipeline);
//
//        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//        // out when the RC activity is in portrait. We do our actual image processing assuming
//        // landscape orientation, though.
//        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
//            }
//        });
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Height Widht Ratio", pipeline.getHeightWidthRatio());
//            telemetry.update();
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(50);
//        }
//    }
//}
