package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Util;
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
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.FreightDeterminationPipeline;
import org.firstinspires.ftc.teamcode.Utilities;

@Autonomous

public class BenWarehouseAuto extends LinearOpMode{
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor Carousel;
    private DcMotor Intake;
    private DcMotor Slide;
    private Servo Bucket;
    private BNO055IMU RevIMUAsBNO055IMU;
    private ElapsedTime   runtime = new ElapsedTime();
    
    Utilities utility = new Utilities();

    OpenCvInternalCamera phoneCam;
    FreightDeterminationPipeline pipeline;

    @Override
    public void runOpMode()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        Carousel = hardwareMap.get(DcMotor.class, "Duck");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Bucket = hardwareMap.get(Servo.class, "Bucket");
        RevIMUAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imu");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
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
        BNO055IMU.Parameters IMUParameters;

        // Put initialization blocks here.
        IMUParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        IMUParameters.loggingEnabled = false;
        // Initialize IMU.
        RevIMUAsBNO055IMU.initialize(IMUParameters);

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
        Bucket.setPosition(0.63);
        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();

///*
        utility.strafe(.5, 20);
        utility.move(.5, 19.5);
        utility.rotate(90);
        utility.move(.5, -8.85);
        sleep(500);
        if(x.equals("LEFT")){
            telemetry.addData("Analysis", 1);
            telemetry.update();
            utility.place(1);
        }
        else if(x.equals("CENTER")){
            telemetry.addData("Analysis", 2);
            telemetry.update();
            utility.place(2);
        }
        else if(x.equals("RIGHT")){
            telemetry.addData("Analysis", 3);
            telemetry.update();
            utility.place(3);
        }
        utility.move(.5, 16);
        utility.rotate(-90);
        utility.strafe(.5, 59);
        utility.move(.2, -4);
        utility.rotate(-92);
        utility.move(1, -1);
        utility.Duck();
        utility.move(.5, 16.5);
//*/
    }
}
    
