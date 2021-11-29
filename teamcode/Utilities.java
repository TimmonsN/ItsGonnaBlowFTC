package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
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



public class Utilities extends LinearOpMode {
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
      
    OpenCvInternalCamera phoneCam;
    FreightDeterminationPipeline pipeline;
      
    // todo: write your code here
    
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
    }

        
      public void move(double speed, double distance) {
        telemetry.addData("start", "move");
        telemetry.update();
        drive(speed, distance, false);
        telemetry.addData("Stop", "move");
        telemetry.update();
        
        /*
      Positive Case - start at 0, go 50 forward
          Halfway is 25
      Negative Case - start at 0, go 50 backward
          Halfway is 25
      
      speed = 0.5
      end
      // pseudo 
      power = speed * ((distance from end) / (halfway))
      
      power = speed * ((ticCount - (backLeft.getCurrentPosition()) / division)
      power += .1
      
      // What you have
      power = (distance from end) * (speed / (distance from middle))
      */
    }
    
public void rotate(int turn) {
        telemetry.addData("start", "Turn");
        telemetry.update();
        Orientation angles;
        float AngleDif;
        int direction;
        int x = 0;
        double bonus = .3;

      backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      for (int count = 0; count < 250; count++) {
        angles = RevIMUAsBNO055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        AngleDif = angles.firstAngle - (turn);
        if(AngleDif >= 0){
            direction = 1;
        } else{
            direction = -1;
        }
        telemetry.addData("rot about Z", AngleDif);
        telemetry.update();
        frontLeft.setPower(((0.145 * AngleDif) / 10 + (bonus * direction)));
        backRight.setPower(-((0.145 * AngleDif) / 10 + (bonus * direction)));
        backLeft.setPower(((0.145 * AngleDif) / 10 + (bonus * direction)));
        frontRight.setPower(-((0.145 * AngleDif) / 10 + (bonus * direction)));
        if(bonus > 0.09){
          bonus -= 0.01;
        }
        if(Math.abs(AngleDif) <= 1.5){
            x++;
            if(x >= 1){
                count = 100000;
            }
        }
      }
        telemetry.addData("Stop", "");
        telemetry.update();
    }

    public void strafe(double speed, int distance) {
        telemetry.addData("start", "strafe");
        telemetry.update();
        drive(speed, distance, true);
        telemetry.addData("Stop", "strafe");
        telemetry.update();
    }
    
    public void drive(double speed, double distance, boolean isStrafe){
        double Multiplyer;
        double division;
        double start;
        double Multiplyer2;
        double power = 1.0;
        int direction;
        if(distance >= 0){
            direction = 1;
        } else{
            direction = -1;
            distance = distance * -1;
        }
        int ticCount = (int) Math.ceil(((distance / 12.12) * 537));
        // Put initialization blocks here.
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        division = Math.ceil((ticCount / 2.0) * 1);
        start = Math.ceil(ticCount / 4.0);
        //Multiplyer = speed / (ticCount - division);
        //Multiplyer2 = speed / start;
        if(isStrafe){
            backLeft.setTargetPosition(-ticCount * direction);
            backRight.setTargetPosition(ticCount * direction);
            frontRight.setTargetPosition(-ticCount * direction);
            frontLeft.setTargetPosition(ticCount * direction);
        } else{
            backLeft.setTargetPosition(ticCount * direction);
            backRight.setTargetPosition(ticCount * direction);
            frontRight.setTargetPosition(ticCount * direction);
            frontLeft.setTargetPosition(ticCount * direction);
        }
          while (!backLeft.isBusy()) { 
          sleep(1);
          telemetry.addData("Stuck", "");
          telemetry.update();
        }
        while (backLeft.isBusy() && Math.abs(power) > 0.09) {
           if ((backLeft.getCurrentPosition() * direction) <= start) {
                power = (Math.abs((backLeft.getCurrentPosition())) * (speed/start));
                power += .1;
            } else if ((Math.abs(backLeft.getCurrentPosition())) >= division) {
                power = speed * ((ticCount - backLeft.getCurrentPosition()* direction) / division);
                power += .1;
            } else {
                power = (speed);
            }
            frontRight.setPower(power * direction);
            frontLeft.setPower(power * direction);
            backLeft.setPower(power * direction);
            backRight.setPower(power * direction);
            telemetry.addData("power", power);
            telemetry.update();
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        telemetry.addData("Stop", "move");
        telemetry.update();
    }
    
    public void Duck(){
        Carousel.setPower(.3);
        sleep(3000);
        Carousel.setPower(0);
    }
    
    
    
    public void place(int slideCall){
        boolean x = false;
        double time1 = 0;
        double time2 = 0.8;
        double time3 = 1.5;
        int slideCounter1 = 0;
        int slideCounter2 = 0;
        int slideCounter3 = 0;
        
        
        //telemetry.addData("Time Count", runtime.seconds());
        //telemetry.update();
        
       if(slideCall == 1) {            // 1st level (WIP implement call instead of if statement for auto [maybe custom method?])
        runtime.reset();
        
            Slide.setPower(0);
                Bucket.setPosition(0.25);
                sleep(1000);
                Bucket.setPosition(0.63);
                sleep(1000);
       }
     
        if(slideCall == 2) {
            Slide.setPower(1);
            sleep(850);
            Slide.setPower(0);
            Bucket.setPosition(0.25);
            sleep(1000);
            Bucket.setPosition(0.63);
            sleep(1000);
            Slide.setPower(-1);
            sleep(850);
            Slide.setPower(0);
        }
    
        if(slideCall == 3) {
            Slide.setPower(1);
            sleep(1400);
            Slide.setPower(0);
            Bucket.setPosition(0.25);
            sleep(1000);
            Bucket.setPosition(0.73);
            sleep(1000);
            Slide.setPower(-1);
            sleep(1300);
            Slide.setPower(0);
        
       }
    }
    
}