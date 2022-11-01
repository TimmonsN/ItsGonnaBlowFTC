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
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous


public class BlueStorage_Copy extends LinearOpMode{
  private DcMotor frontLeft;
  private DcMotor backRight;
  private DcMotor frontRight;
  private DcMotor backLeft;
  private DcMotor carousel;
  private DcMotor intake;  
  private DcMotor arm;
  private Servo clamp;
  private Servo bucket;
  private BNO055IMU RevIMUAsBNO055IMU;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime colorWait = new ElapsedTime();
  private ColorSensor color;

    OpenCvInternalCamera phoneCam;
    FreightDeterminationPipeline pipeline;

    @Override
    public void runOpMode()
    {
        //Hold holdArm = new Hold();
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        carousel = hardwareMap.get(DcMotor.class, "duck");
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");
        bucket = hardwareMap.get(Servo.class, "intakeArm"); 
        clamp = hardwareMap.get(Servo.class, "clamp");
        RevIMUAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imu");
        color = hardwareMap.get(ColorSensor.class, "color");
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
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
        bucket.setPosition(0.65);
        clamp.setPosition(0);
        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();
        sleep(500);
        

        //ready(1);
        //place();
        /*
        rotate(-165);
        rotate(99);
        rotate(-85);
        //*/
        //rotate(-165);
        //rotate(0);
        //while(opModeIsActive()){
        //}
///*
        strafe(.5, 17);
        move(.75, -17);
        strafe(.5, 27);
        rotate(-165);
        if(x.equals("LEFT")){
            telemetry.addData("Analysis", 1);
            telemetry.update();
            ready(1);
            move(.5,-16.5);
            sleep(500);
            place();
            move(.75, 13.5);
            //sleep(3000);
        }
        else if(x.equals("CENTER")){
            telemetry.addData("Analysis", 2);
            telemetry.update();
            ready(2);
            move(.5,-18.2);
            sleep(500);
            place();
            move(.75, 15.2);
            //sleep(3000);
        }
        else if(x.equals("RIGHT")){
            telemetry.addData("Analysis", 3);
            telemetry.update();
            ready(3);
            move(.5,-26);
            sleep(500);
            place();
            move(.75, 22);
            //sleep(3000);
        }
        //sleep(2000);
        rotate(20);
        strafe(.5, -35);
        strafe(.2, -2);
        Duck();
        strafe(1, 22);
        move(.75,-6);
        rotate(0);
//*/
    }
    
    public void move(double speed, double distance) {
        telemetry.addData("start", "move");
        telemetry.update();
        double Multiplyer;
        double division;
        double start;
        double Multiplyer2;
        double power = 1;
        int direction;
        int x = 0;
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
        backLeft.setTargetPosition(ticCount * direction);
        backRight.setTargetPosition(ticCount * direction);
        frontRight.setTargetPosition(ticCount * direction);
        frontLeft.setTargetPosition(ticCount * direction);
      
          while (!backLeft.isBusy()) { 
          sleep(1);
          telemetry.addData("Stuck", "");
          telemetry.update();
        }
        while (backLeft.isBusy() && x < 150) {
           if ((backLeft.getCurrentPosition() * direction) <= start) {
                power = (Math.abs((backLeft.getCurrentPosition())) * (speed/start));
                power += .1;
            } else if ((Math.abs(backLeft.getCurrentPosition())) >= division) {
                power = speed * ((ticCount - backLeft.getCurrentPosition()* direction) / division);
                power += .1;
            } else {
                power = (speed);
            }
            if (!(ticCount == (Math.abs(backRight.getCurrentPosition()))) && power < 0.1){
                power = 0.1;
            }
            else{
            frontRight.setPower(power * direction);
            frontLeft.setPower(power * direction);
            backLeft.setPower(power * direction);
            backRight.setPower(power * direction);
            }
            telemetry.addData("power", x);
            telemetry.update();
            x++;
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
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
    
/*
private void rotate(int turn) {
        telemetry.addData("start", "turn");
        telemetry.update();
        Orientation angles;
        float angleDiff;
        int direction;
        int x = 0;
        double diff = 0;
        double rotatePower = 0;
        double angle = 0;

      backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      
      for (int count = 0; count < 250; count++) {
        angles = RevIMUAsBNO055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        angle = angles.firstAngle;
       
        angleDiff = (turn) - angles.firstAngle;
        if(angleDiff >= 0){
            direction = -1;
        } else{
            direction = 1;
        }
        
        
        //power = ((-1 * Math.abs((angles.firstAngle)/(turn))) + 1);


        telemetry.addData("rot about Z", angles.firstAngle);
        telemetry.update();
        
        if(angles.firstAngle < 0) {
            angle = angle + 360;
        }
        
        if(!(Math.abs(angle - turn) <= 1.5)){
            
            if(angle <= (turn - 90)){
                rotatePower = -1;
            }
            
            if(angle >= (turn + 90)){
                rotatePower = 1;
            }
            ///*
            if((angle + 90) > turn){ // || (angle - 90) < turn)
                diff = Math.toDegrees(angle - turn);
                 rotatePower = -Math.sin(diff);
                 /*
                 if(rotatePower < 0.1 && rotatePower > 0) {
                    rotatePower = 0.1;
                 }
                 if(rotatePower > -0.1 && rotatePower < 0){
                    rotatePower = -0.1;
                 }
            }
            
            frontLeft.setPower((rotatePower));
            frontRight.setPower(-(rotatePower));
            backLeft.setPower((rotatePower));
            backRight.setPower(-(rotatePower));
        }
        
      /*  if((turn == 180) && (angles.firstAngle >=180 || angles.firstAngle <= -170)){
            frontLeft.setPower(-(-.1));
            backRight.setPower((-.1));
            backLeft.setPower(-(-.1));
            frontRight.setPower((-.1));
        }
        else{
            frontLeft.setPower(-((0.145 * AngleDif) / 15 + (bonus * direction)));
            backRight.setPower(((0.145 * AngleDif) / 15 + (bonus * direction)));
            backLeft.setPower(-((0.145 * AngleDif) / 15 + (bonus * direction)));
            frontRight.setPower(((0.145 * AngleDif) / 15 + (bonus * direction)));
        }
        
        if(bonus > 0.09){
          bonus -= 0.01;
        }
        
        
        if(Math.abs(AngleDif) <= 1.5){
            x++;
            if(x >= 3){
                count = 100000;
            }
        }

      }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        //telemetry.addData("Stop", "");
        telemetry.update();
    }
    */
    
    private void rotate(int turn) {
        telemetry.addData("start", "Turn");
        telemetry.update();
        Orientation angles;
        float AngleDif;
        int direction;
        int x = 0;
        double bonus = -.125;

      backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      for (int count = 0; count < 250; count++) {
        angles = RevIMUAsBNO055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        AngleDif = (turn) - angles.firstAngle;
        if(AngleDif >= 0){
            direction = -1;
        } else{
            direction = 1;
        }
        
        //power = ((-1 * Math.abs((angles.firstAngle)/(turn))) + 1);
        
        telemetry.addData("rot about Z", angles.firstAngle);
        telemetry.update();
        if((turn == 180) && (angles.firstAngle >=180 || angles.firstAngle <= -170)){
            frontLeft.setPower(-(-.1));
            backRight.setPower((-.1));
            backLeft.setPower(-(-.1));
            frontRight.setPower((-.1));
        }
        else{
            frontLeft.setPower(-((0.145 * AngleDif) / 15 + (bonus * direction)));
            backRight.setPower(((0.145 * AngleDif) / 15 + (bonus * direction)));
            backLeft.setPower(-((0.145 * AngleDif) / 15 + (bonus * direction)));
            frontRight.setPower(((0.145 * AngleDif) / 15 + (bonus * direction)));
        }
        /*
        if(bonus > 0.09){
          bonus -= 0.01;
        }
        //*/
        if(Math.abs(AngleDif) <= 1.5){
            x++;
            if(x >= 3){
                count = 100000;
            }
        }
      }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        //telemetry.addData("Stop", "");
        telemetry.update();
    }


    public void strafe(double speed, double distance) {
        telemetry.addData("start", "strafe");
        telemetry.update();
        double Multiplyer;
        double division;
        double start;
        double Multiplyer2;
        double power = 1;
        int direction;
        int x = 0;
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
        backLeft.setTargetPosition(-ticCount * direction);
        backRight.setTargetPosition(ticCount * direction);
        frontRight.setTargetPosition(-ticCount * direction);
        frontLeft.setTargetPosition(ticCount * direction);
      
        while (!backRight.isBusy()) { 
          sleep(1);
          telemetry.addData("Stuck", "");
          telemetry.update();
        }
        while (backRight.isBusy() && x < 150) {
           if ((backRight.getCurrentPosition() * direction) <= start) {
                power = ((backRight.getCurrentPosition() * direction) * (speed/start));
                power += .1;
            } else if ((backRight.getCurrentPosition() * direction) >= division) {
                power = speed * ((ticCount - backRight.getCurrentPosition()* direction) / division);
                power += .1;
            } else {
                power = (speed);
            }
            if (!(ticCount == (Math.abs(backRight.getCurrentPosition()))) && power < 0.1){
                power = 0.1;
            }
            frontRight.setPower(power * direction);
            frontLeft.setPower(power * direction);
            backLeft.setPower(power * direction);
            backRight.setPower(power * direction);
            x++;
            telemetry.addData("power", power);
            telemetry.addData("x", x);
            telemetry.update();
        }
        x = 0;
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        telemetry.addData("strafe", x);
        telemetry.update();
    }
    
    private void Duck(){
        carousel.setPower(-.3);
        sleep(3500);
        carousel.setPower(0);
    }
    
    
    private void ready(int level){
        double tick = 0;
        double diff = 0;
        double pwr = 1;
        boolean run = true;
        int levelDist1 = -3500;
        int levelDist2 = -3000;
        int levelDist3 = -2200;
        double levelBuck1 = .40;
        double levelBuck2 = .42;
        double levelBuck3 = .62;
        int x = 0;
        
        //telemetry.addData("Time Count", runtime.seconds());
        //telemetry.update();
        
       if(level == 1) {
            bucket.setPosition(1);
            sleep(500);
            tick = levelDist1;
            arm.setTargetPosition(levelDist1);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.7);
            while(x <= 15){
                telemetry.addData("Position", arm.getCurrentPosition());
                telemetry.addData("Power", pwr);
                telemetry.update();
                diff = Math.abs(arm.getCurrentPosition() - tick);
                pwr = diff / 100;
                arm.setPower(pwr);
                if(pwr <= 0.05){
                    x++;
                }
            }
            bucket.setPosition(levelBuck1);
       }
     
        if(level == 2) {
            bucket.setPosition(1);
            sleep(500);
            tick = levelDist2;
            arm.setTargetPosition(levelDist2);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.7);
            sleep(500);
            /*
            while(x <= 15){
                telemetry.addData("Position", arm.getCurrentPosition());
                telemetry.addData("Power", pwr);
                telemetry.update();
                diff = Math.abs(arm.getCurrentPosition() - tick);
                pwr = diff / 100;
                arm.setPower(pwr);
                if(pwr <= 0.05){
                    x++;
                }
            }
            */
            bucket.setPosition(levelBuck2);
            sleep(500);
        }
        
        if(level == 3) {
            bucket.setPosition(1);
            sleep(500);
            tick = levelDist3;
            arm.setTargetPosition(levelDist3);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.7);
            sleep(500);
            /*
            while(x <= 15){
                telemetry.addData("Position", arm.getCurrentPosition());
                telemetry.addData("Power", pwr);
                telemetry.update();
                diff = Math.abs(arm.getCurrentPosition() - tick);
                pwr = diff / 100;
                arm.setPower(pwr);
                if(pwr <= 0.05){
                    x++;
                }
            }
            */
            bucket.setPosition(levelBuck3);
       }
       
    }
    
    public void place(){
        clamp.setPosition(.5);
        sleep(500);
        move(.25,5);
        clamp.setPosition(0);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucket.setPosition(1);
        arm.setPower(0.7);
        
    }

    
    
    
    

    public static class FreightDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the Freight position
         */
        public enum FreightPosition
        {
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

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(50,58);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(150,58);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(260,58);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
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

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile FreightPosition position = FreightPosition.LEFT;

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
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCr(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cr = Cr.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bRight yellow and contrast
             * STRONGLY on the Cb channel against everything else, including Freights
             * (because Freights have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The bRightest of the 3 regions
             * is where we assume the Freight to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the Freight.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCr(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cr).val[0];
            avg2 = (int) Core.mean(region2_Cr).val[0];
            avg3 = (int) Core.mean(region3_Cr).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    PINK, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                position = FreightPosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg2) // Was it from region 2?
            {
                position = FreightPosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                position = FreightPosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
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
        public FreightPosition getAnalysis()
        {
            return position;
        }
    }
}
    
