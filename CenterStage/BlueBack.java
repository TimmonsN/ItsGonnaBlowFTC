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


public class BlueBack extends LinearOpMode{
  private DcMotor frontLeft;
  private DcMotor backRight;
  private DcMotor frontRight;
  private DcMotor backLeft;
  private DcMotor linearSlideLeft;
  private DcMotor linearSlideRight;
 // private DcMotor fourBar;
  private Servo bucket;
  private Servo plane;
  private Servo intakePush;
  private BNO055IMU RevIMUAsBNO055IMU;
  

    OpenCvInternalCamera phoneCam;
    FreightDeterminationPipeline pipeline;

    @Override
    public void runOpMode()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        linearSlideLeft = hardwareMap.get(DcMotor.class, "slideleft");
        linearSlideRight = hardwareMap.get(DcMotor.class, "slideright");
        intake = hardwareMap.get(DcMotor.class, "intake");
        fly = hardwareMap.get(DcMotor.class, "fly");
        intakePush = hardwareMap.get(Servo.class, "intakePush"); 
        plane = hardwareMap.get(Servo.class, "plane"); 
        bucket = hardwareMap.get(Servo.class, "bucket"); 
        RevIMUAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imu");
        linearSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  
        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        double slideTicks = 0;
        double  probePosition = 0;

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

        //find pos
        intakePush.setPosition();
///*
        if(//left){
            strafe(.5, -7.5);
            move(.75, -24);
            strafe(.5,-24);
            linePlace();
            //backup
            //board place
            //strafe to park
        }
        else if(//center){
            //strafe
            //rotate
            //move
            //line place
            //rotate
            //backup
            //board place
            //stafe
        }
        else if(//right){
            //strafe
            //move forward
            //line place
            //backup
            //board place
            //strafe

        }
//*/
    }

//Game Specific Methods
    public void linePlace(){
        intake.setPower(-.25);
        sleep(1000);
        intake.setPower(0);
    }

    public void boardPlace(int dist){
        //tick count of up pos
        int x = 1800;
        linearSlideLeft.setPower(0);
        linearSlideRight.setPower(0);
        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideLeft.setTargetPosition(x);
        linearSlideRight.setTargetPosition(x);
        linearSlideLeft.setPower(.5);
        linearSlideRight.setPower(.5);
        move(.25, dist);
        while (linearSlideLeft.isBusy()) { 
          sleep(1);
        }
        linearSlideLeft.setPower(0);
        linearSlideRight.setPower(0);
        //find dump pos
        bucket.setPosition();
        sleep(2000);
        //find up pos
        bucket.setPosition();
        strafe(.5, 24);

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
}  
