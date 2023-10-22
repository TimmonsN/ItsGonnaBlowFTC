package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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



public class AutoR_NoWall extends LinearOpMode{
  private DcMotor frontLeft;
  private DcMotor backRight;
  private DcMotor frontRight;
  private DcMotor backLeft;
  private DcMotor linearSlide;
  private DcMotor linearSlide1;
  private Servo probe;
  private BNO055IMU RevIMUAsBNO055IMU;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime colorWait = new ElapsedTime();
  
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
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        linearSlide1 = hardwareMap.get(DcMotor.class, "linearSlide1");
        probe = hardwareMap.get(Servo.class, "probe");
        RevIMUAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imu");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
        
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
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (count < 25)
        {
            x = (pipeline.getAnalysis()).name();
            telemetry.addData("Analysis", x);
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            count++;
        }
        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();
        sleep(1000);
        probe.setPosition(0.16);
        sleep(2000);
        linearSlide.setTargetPosition(150);//find number for up
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide1.setTargetPosition(-150);//find number for up
        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.4);
        linearSlide1.setPower(0.4);
        
        strafe(.5, -7.5);
        move(.75, -23);
        linearSlide.setTargetPosition(850);//find number for up
        linearSlide1.setTargetPosition(-850);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.4);
        linearSlide1.setPower(0.4);
        rotate(0);
        strafe(.5, -40.5);
        place(5.5);
        strafe(.5, -14.75);
        linearSlide.setTargetPosition(100);
        linearSlide1.setTargetPosition(-100);
        rotate(0);
        linearSlide.setTargetPosition(420);//find number for up
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide1.setTargetPosition(-420);//find number for up
        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.4);
        linearSlide1.setPower(0.4);
        /*
        move(.75, 46);
        rotate(0);
        grab(420);
        */
        /*
        linearSlide.setTargetPosition(0);
        move(.75, 48);
        grab(300);
        move(.75, -48);
        strafe(.5, 12);
        place(5.3);
        strafe(.5, -12);
        */
        if(x.equals("LEFT")){
            telemetry.addData("Analysis", 1);
            telemetry.update();
            //rotate(0);
            //move(.75,-43);
            /*
            rotate(0);
            strafe(.5, 10);
            place(6.4);
            strafe(.5, -11.5);
            move(.75, 2);
            */
            //One Park
            
        }
        else if(x.equals("CENTER")){
            telemetry.addData("Analysis", 2);
            telemetry.update();
            move(.5, 18.5);
            //move(.75, -18);
            /*
            strafe(.5, 14.5);
            move(.5, -1);
            smallplace(4.3);
            */
            //Two Park
        }
        else if(x.equals("RIGHT")){
            telemetry.addData("Analysis", 3);
            telemetry.update();
            move(.5, 46);
            //Three Park
            //move(.1, 2);
            /*
            rotate(180);
            move(.1, -6);
            strafe(.5, -13.5);
            smallplace(4);
            */
        }
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
        division = Math.ceil(ticCount / 4.0);
        start = Math.ceil(ticCount / 4.0);
        //Multiplyer = speed / (ticCount - division);
        //Multiplyer2 = speed / start;
        backLeft.setTargetPosition(ticCount * direction);
        backRight.setTargetPosition(ticCount * direction);
        frontRight.setTargetPosition(ticCount * direction);
        frontLeft.setTargetPosition(ticCount * direction);
      
        /*  while (!backLeft.isBusy()) { 
          sleep(1);
          telemetry.addData("Stuck", "");
          telemetry.update();
        } */
        while (backLeft.isBusy() && x < 150) {
           if ((backLeft.getCurrentPosition() * direction) <= start) {
                power = (Math.abs((backLeft.getCurrentPosition())) * (speed/start));
                power += .1;
            } else if ((Math.abs(backLeft.getCurrentPosition())) >= division) {
                power = speed * ((ticCount - backLeft.getCurrentPosition()* direction) / (ticCount/4));
                power += .1;
                telemetry.addData("Slowing down", power);
                telemetry.update();
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
        division = Math.ceil((ticCount / 4.0) * 1);
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
                power = speed * ((ticCount - backRight.getCurrentPosition()* direction) / (ticCount / 4));
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
    
    
    /*
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
            
            bucket.setPosition(levelBuck3);
       }
       
    }
    */
    public void WFS(){//WFS wait for slide
        while (linearSlide.isBusy() || linearSlide1.isBusy()) { 
          sleep(1);
          telemetry.addData("Sliding", "");
          telemetry.addData("linear slide 0: ", linearSlide.getCurrentPosition());
          telemetry.addData("linear slide 1: ", linearSlide1.getCurrentPosition());
          telemetry.update();
        }
    }
    
    public void grab(int distance){
        probe.setPosition(0.0);
        sleep(500);
        linearSlide.setPower(0);
        linearSlide.setTargetPosition(distance + 500);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide1.setPower(0);
        linearSlide1.setTargetPosition(-(distance + 500));
        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.85);
        linearSlide1.setPower(0.85);
        WFS();
        move(.1, 8);
        sleep(500);
        linearSlide.setTargetPosition(distance);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide1.setTargetPosition(-distance);
        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.6);
        linearSlide1.setPower(0.6);
        WFS();
        sleep(1000);
        probe.setPosition(.16);
        sleep(500);
        linearSlide.setTargetPosition(distance + 500);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide1.setTargetPosition(-(distance + 500));
        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.85);
        linearSlide1.setPower(0.85);
        WFS();
        move(.1, -8);
        linearSlide.setTargetPosition(100);//find number for down
        linearSlide1.setTargetPosition(-100);
    }
    
    public void smallplace(double distance){
        linearSlide.setPower(0);
        linearSlide.setTargetPosition(1200);//find number for up
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide1.setPower(0);
        linearSlide1.setTargetPosition(-1200);//find number for up
        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.85);
        linearSlide1.setPower(0.85);
        telemetry.addData("ticks", linearSlide.getCurrentPosition());
        telemetry.update();
        WFS();
        move(.25, distance);
        sleep(500);
        probe.setPosition(0);//find number for release
        sleep(500);
        move(.25, -distance);
        telemetry.addData("ticks", linearSlide.getCurrentPosition());
        telemetry.update();
        linearSlide.setTargetPosition(0);//find number for down
        linearSlide1.setTargetPosition(0);
         
    }

    
    public void place(double distance){
        linearSlide.setPower(0);
        linearSlide.setTargetPosition(1600);//find number for up
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide1.setPower(0);
        linearSlide1.setTargetPosition(-1600);//find number for up
        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.85);
        linearSlide1.setPower(0.85);
        telemetry.addData("ticks", linearSlide.getCurrentPosition());
        telemetry.update();
        WFS();
        move(.25, distance);
        sleep(2000);
        probe.setPosition(0);//find number for release
        sleep(500);
        move(.25, -distance);
        telemetry.addData("ticks", linearSlide.getCurrentPosition());
        telemetry.update();
        linearSlide.setTargetPosition(0);//find number for down
        linearSlide1.setTargetPosition(0);
         
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
        static final Scalar lower_magenta_bounds = new Scalar(100, 0, 30, 255);
        static final Scalar upper_magenta_bounds = new Scalar(255, 200, 255, 255);
        static final Scalar YELLOW = new Scalar(255, 255, 0);
        static final Scalar CYAN = new Scalar(0, 255, 255);
        static final Scalar MAGENTA = new Scalar(255, 0, 255);
        static final Scalar WHITE = new Scalar(255, 255, 255);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(120,58);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(150,58);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(260,58);
        static final int REGION_WIDTH = 40;
        static final int REGION_HEIGHT = 40;
        
        double yelPercent;
        double cyaPercent;
        double magPercent;
        
        Point sleeve_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point sleeve_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + 35, REGION1_TOPLEFT_ANCHOR_POINT.y + 35);
            
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
