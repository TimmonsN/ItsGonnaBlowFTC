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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import java.util.List;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class RedBack extends LinearOpMode{
  private DcMotor frontLeft;
  private DcMotor backRight;
  private DcMotor frontRight;
  private DcMotor backLeft;
  private DcMotor slideLeft;
  private DcMotor slideRight;
  private DcMotor fly;
  private Servo bucket;
  private Servo plane;
  private Servo clawOpen;
  private Servo clawLift;
  private BNO055IMU RevIMUAsBNO055IMU;
  private TfodProcessor tfod;
  private VisionPortal visionPortal;
  private int sensedPosition = 2;
  private static final String TFOD_MODEL_FILE = "red2.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
  private static final String[] LABELS = {
    "prop",
  };
  
  private static final double clawClosed = 0.54;

    @Override
    public void runOpMode()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "LeftFront");
        backRight = hardwareMap.get(DcMotor.class, "RightBack");
        frontRight = hardwareMap.get(DcMotor.class, "RightFront");
        backLeft = hardwareMap.get(DcMotor.class, "LeftBack");
        slideLeft = hardwareMap.get(DcMotor.class, "SlideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "SlideRight");
        fly = hardwareMap.get(DcMotor.class, "Airplane");
        clawOpen = hardwareMap.get(Servo.class, "Claw"); 
        clawLift = hardwareMap.get(Servo.class, "ClawUp"); 
        plane = hardwareMap.get(Servo.class, "PlaneArm"); 
        bucket = hardwareMap.get(Servo.class, "Bucket"); 
        RevIMUAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imu");
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        double slideTicks = 0;
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
        
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       //initializing TFOD prior to the waitForStart() call
        initTfod();
        telemetryTfod();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData("Hi :)", sensedPosition);
        telemetry.update();
        
        //find pos
//START
        waitForStart();
        telemetryTfod();
        telemetry.addData("Hi :)", sensedPosition);
        telemetry.update();
        bucket.setPosition(.5);
        clawLift.setPosition(.69);
        clawOpen.setPosition(clawClosed - 0.01);
       // move(0.5, 15);
       
      if (sensedPosition == 3) { //right
            strafe(.5, -5);
            rotate(180);
            rotate(180);
            move(.75, -31);
            strafe(.5, 27);
            rotate(180);
            linePlace();
            strafe(.25, -11);
            boardPlace(-21);
            strafe(.5, -25);
        }
        
        else if (sensedPosition == 2) { //center (drives center, duh)
            strafe(.5, -7.5);
            move(0.25, 2);
            rotate(90);
            move(.75, 18);
            rotate(90);
            linePlace();
            move(.5,-2);
            rotate(180);
            boardPlace(-44);
            move(0.25, 1);
            strafe(.5, -23);
        }
        
        
        else {                      //left
            strafe(.5, -32);
            move(.5, 8);
            rotate(180);
            rotate(180);
            linePlace();
            rotate(180);
            strafe(.25, 4);
            boardPlace(-51);
            move(0.25, 1);
            rotate(180);
            strafe(.5, -31);
        }


           
        //}
//*/
    }

//Game Specific Methods
    public void linePlace(){
        move(0.25, 10);
        move(.25, -5);
        clawLift.setPosition(0.4);
        sleep(750);
        clawOpen.setPosition(clawClosed + 0.2);
        sleep(750);
        //clawOpen.setPosition(clawClosed);
        // telemetry.update();
        // sleep(750);
        // clawLift.setPosition(0.69);
        sleep(1000);
        move(0.25, -5);
        telemetry.update();
    }

    public void boardPlace(int dist){
        //tick count of up pos
        int heightTotal = -1350;
        int height = -900;
        telemetry.addData("right slide", slideRight.getPower());
        telemetry.addData("left slide", slideLeft.getPower());
        telemetry.addData("left pos", slideLeft.getCurrentPosition());
        telemetry.update();
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        move(.25, dist);
        move(.25, 2);
        rotate(180);
        slideLeft.setPower(-.8);
        slideRight.setPower(-.8);
        while (slideLeft.getCurrentPosition() > height) { 
          sleep(1);
          telemetry.update();
        }
        slideLeft.setPower(0);
        slideRight.setPower(0);
        bucket.setPosition(0.35);
        sleep(1000);
        telemetry.update();
        slideLeft.setPower(-.8);
        slideRight.setPower(-.8);
         while (slideLeft.getCurrentPosition() > heightTotal) { 
          sleep(1);
          telemetry.update();
        }
        //find dump pos
        slideLeft.setPower(0);
        slideRight.setPower(0);
        bucket.setPosition(.8);
        sleep(2000);
        //find up pos
        slideLeft.setPower(-.8);
        slideRight.setPower(-.8);
         while (slideLeft.getCurrentPosition() > heightTotal) { 
          sleep(1);
          telemetry.update();
        }
        slideLeft.setPower(0);
        slideRight.setPower(0);
        bucket.setPosition(.53);
        sleep(1500);
        slideLeft.setPower(0.8);
        slideRight.setPower(0.8);
        sleep(450);
        slideLeft.setPower(0);
        slideRight.setPower(0);
        strafe(.5, -3);
        move(.5,3);
    }


//Movement Methods
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
        int count = 0;
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
        while (backLeft.isBusy() && count < 150) {
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
            telemetry.addData("power", count);
            telemetry.update();
            count++;
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        telemetry.addData("Stop", "move");
        telemetry.update();
    }
    
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
    
    //Vision sensing methods:
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

            // With the following lines commented out, the default TfodProcessor Builder
            // will load the default model for the season. To define a custom model to load, 
            // choose one of the following:
            //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
            //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            .setModelFileName(TFOD_MODEL_FILE)

            // The following default settings are available to un-comment and edit as needed to 
            // set parameters for custom models.
            //.setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(BuiltinCameraDirection.BACK);

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
   
   private void telemetryTfod() {
        double bestX = 300;
        double bestY = 0;
        double mostConfident = 0;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            if (recognition.getConfidence() > 0) {
                mostConfident = recognition.getConfidence();
                bestX = x;
                bestY = y;
            }
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        if (bestX < 213) {
            sensedPosition = 3;
        }
        else if (bestX > 440) {
            sensedPosition = 1;
        }
        else {
            sensedPosition = 2;
        }

    }
}  