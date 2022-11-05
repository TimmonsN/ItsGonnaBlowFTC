package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous


public class Sensing extends LinearOpMode{

  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime colorWait = new ElapsedTime();

    OpenCvInternalCamera phoneCam;
    FreightDeterminationPipeline pipeline;

    @Override
    public void runOpMode()
    {
        //Hold holdArm = new Hold();


        
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new FreightDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
        int count = 0;
        String x = "";

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (count < 25)
        {
            x = (pipeline.getAnalysis()).name();
            telemetry.addData("Analysis", x);
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            count++;
        }
    }
    public static class FreightDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the Freight position
         */
        public enum ParkingPosition
        {
            DEFAULT,
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar PINK = new Scalar(255, 20, 147);
        static final Scalar lower_yellow_bounds = new Scalar(100, 100, 0, 255);
        static final Scalar upper_yellow_bounds = new Scalar(255, 255, 200, 255);
        static final Scalar lower_cyan_bounds = new Scalar(0, 0, 100, 255);
        static final Scalar upper_cyan_bounds = new Scalar(200, 255, 255, 255);
        static final Scalar lower_magenta_bounds = new Scalar(100, 0, 0, 255);
        static final Scalar upper_magenta_bounds = new Scalar(255, 200, 255, 255);
        static final Scalar YELLOW = new Scalar(255, 255, 0);
        static final Scalar CYAN = new Scalar(0, 255, 255);
        static final Scalar MAGENTA = new Scalar(255, 0, 255);
        static final Scalar WHITE = new Scalar(255, 255, 255);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(150,58);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(150,58);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(260,58);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;
        
        double yelPercent;
        double cyaPercent;
        double magPercent;
        
        Point sleeve_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point sleeve_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + 20, REGION1_TOPLEFT_ANCHOR_POINT.y + 20);
            
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cr, region2_Cr, region3_Cr;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        int avg1, avg2, avg3;
        Mat yelMat = new Mat();
        Mat cyaMat = new Mat();
        Mat magMat = new Mat();
        Mat blurredMat = new Mat();
        Mat kernel = new Mat();
        
        private volatile ParkingPosition position = ParkingPosition.DEFAULT;

        // Volatile since accessed by OpMode thread w/o synchronization

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCr(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {

            inputToCr(firstFrame);
/*
            region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cr = Cr.submat(new Rect(region3_pointA, region3_pointB));
*/
        }

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.blur(input, blurredMat, new Size(5, 5));
            blurredMat = blurredMat.submat(new Rect(region1_pointA, region1_pointB));

            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
            Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

            Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
            Core.inRange(blurredMat, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
            Core.inRange(blurredMat, lower_magenta_bounds, upper_magenta_bounds, magMat);

            yelPercent = Core.countNonZero(yelMat);
            cyaPercent = Core.countNonZero(cyaMat);
            magPercent = Core.countNonZero(magMat);

            double maxPercent = Math.max(yelPercent, Math.max(cyaPercent, magPercent));
            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */

            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if (maxPercent == yelPercent) {
                position = ParkingPosition.LEFT;
                Imgproc.rectangle(
                        input,
                        sleeve_pointA,
                        sleeve_pointB,
                        YELLOW,
                        2
                );
            } else if (maxPercent == cyaPercent) {
                position = ParkingPosition.CENTER;
                Imgproc.rectangle(
                        input,
                        sleeve_pointA,
                        sleeve_pointB,
                        CYAN,
                        2
                );
            } else if (maxPercent == magPercent) {
                position = ParkingPosition.RIGHT;
                Imgproc.rectangle(
                        input,
                        sleeve_pointA,
                        sleeve_pointB,
                        MAGENTA,
                        2
                );
            } else {
                position = ParkingPosition.DEFAULT;
                Imgproc.rectangle(
                        input,
                        sleeve_pointA,
                        sleeve_pointB,
                        WHITE,
                        2
                );
            }
            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public ParkingPosition getAnalysis()
        {
            return position;
        }
    }
}
    
