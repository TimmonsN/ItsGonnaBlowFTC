package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleTest extends LinearOpMode {
    
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
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime colorWait = new ElapsedTime();
  
  @Override
  public void runOpMode(){
    
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
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fly.setDirection(DcMotorSimple.Direction.REVERSE);

        // slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  
        // slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
       // int minimum = slideLeft.getCurrentPosition();

//VARIABLES:
    
    double manualBucket = bucket.getPosition();
    double manualGrab = clawOpen.getPosition();
    double manualClaw = clawLift.getPosition();

    waitForStart();

    if (opModeIsActive()) {
      /*//
      if Nathan wants anything to happen at the start of tele op
      //*/

      while (opModeIsActive()) {
//Slow mode (and normal):
        if(gamepad1.left_bumper || gamepad1.right_bumper){
            frontLeft.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) * 0.3);
            frontRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 0.3);
            backRight.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x) * 0.3);
            backLeft.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) * 0.3);
        }
        else {
          frontLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x * 0.75);
          frontRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x * 0.75);
          backRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x * 0.75);
          backLeft.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x * 0.75);
        }
//slides always have same power
        
//manual sliding
  //    if (!gamepad2.x) {
        slideLeft.setPower((.7 * gamepad2.left_stick_y));
        slideRight.setPower((.7 * gamepad2.left_stick_y));
   //   }
   //   else if (gamepad2.x) {
  //      slideLeft.setPower((.9 * (gamepad2.left_stick_y)));
   //     slideRight.setPower((.9 * (gamepad2.left_stick_y)));
  //    }
    // if (slideLeft.getCurrentPosition() < minimum) {
      // slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       //slideLeft.setTargetPosition(minimum);
     //}
     
     
//telemetry
        telemetry.addData("slideLeft pos: ", slideLeft.getCurrentPosition());
      //  telemetry.addData("slideLeft pow: ", slideLeft.getPower());
        telemetry.addData("slide power:", slideLeft.getPower());
        telemetry.addData("bucket position:", bucket.getPosition());
        telemetry.addData("claw (open):", clawOpen.getPosition());
        telemetry.addData("claw (lift):", clawLift.getPosition());
        telemetry.addData("plane arm:", plane.getPosition());
      //  telemetry.addData("right stick:", gamepad2.right_stick_y);
        telemetry.update();

//Intake:
        //manual intake claw up/down
        if(((gamepad2.right_stick_y < 0) && (manualClaw <= 1)) && (clawLift.getPosition() <= 1)) {
          manualClaw += 0.003;
          clawLift.setPosition(manualClaw);
        }
        else if(((gamepad2.right_stick_y > 0) && (manualClaw >= 0)) && (clawLift.getPosition() >= 0)) {
          manualClaw -= 0.003;
          clawLift.setPosition(manualClaw);
        }
        if(gamepad2.dpad_down) {
          manualClaw = 0.3;
          clawLift.setPosition(0.3);
        }
        
        
        if(((gamepad2.right_bumper) && (manualGrab <= 1)) && (clawOpen.getPosition() <= 1)) {
          manualGrab += 0.001;
          clawOpen.setPosition(manualGrab);
        }
        else if(((gamepad2.left_bumper) && (manualGrab >= 0)) && (clawOpen.getPosition() >= 0)) {
          manualGrab -= 0.001;
          clawOpen.setPosition(manualGrab);
        }
        
        
        /*
        //for button intake
        if (gamepad2.a){intake.setPower(1);}
        else if (!gamepad2.a){intake.setPower(0);}
        */
        
//Bucket:
        if(gamepad1.a){
            //pos needs to be found
            bucket.setPosition(0.52);
            manualBucket = 0.52;
        }
        else if(gamepad1.b){
            //pos needs to be found
            manualBucket = 0.8;
            bucket.setPosition(0.8);
        }
        
        if(((gamepad1.right_trigger > 0.5) && (manualBucket <= 0.81)) && (bucket.getPosition() <= 1)) {
          manualBucket += 0.005;
          bucket.setPosition(manualBucket);
        }
        else if(((gamepad1.left_trigger > 0.5) && (manualBucket >= 0.35)) && (bucket.getPosition() >= 0)) {
          manualBucket -= 0.005;
          bucket.setPosition(manualBucket);
        }

//Airplane
        if (gamepad2.y){
          fly.setPower(0.35);
        }
        if (gamepad2.b){
          plane.setPosition(0.15);
        }
        if (gamepad2.x) {
          fly.setPower(0);
        }
        if (gamepad2.a) { 
          plane.setPosition(0.35);
        }
      

        
       /* deccel relic? if(inPos){
          diff = Math.abs(arm.getCurrentPosition() - tick);
          pwr = diff / 100;
          telemetry.addData("Power", pwr);
          telemetry.update();
          slideLeft.setPower(pwr);
        } */
          
      } // while OpModeActive
    } // if OpModeActive
  } // runOpMode
} // LinearOpMode