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
public class Tele extends LinearOpMode {
    
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
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime colorWait = new ElapsedTime();
  
  @Override
  public void runOpMode(){
    
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
        
       // int minimum = linearSlideLeft.getCurrentPosition();

//VARIABLES:
    
    /* int armCounter1 = 0;
    int armCounter2 = 0;
    int armCounter3 = 0;
    int armCounter4 = 0;
    // No boolean inPos0 = false;
    boolean inPos1 = false;
    boolean inPos2 = false;
    boolean inPos3 = false;
    double tick = 0;
    double diff = 0;
    double pwr = 0; */
    int low = 1040;   // Testing encoder heights: 1040 ish, approx. 80 per inch
    int mid = 1840;   //                          1840 ish
    int high = 2640;  //                          2640 ish
    int bottom = 0;   //                           240 ish
   
   
    int front = 0;    // Encoder Position over guide
    int back = -700;     // Encoder Position over gears
    int top = -400;      // Encoder Position when straight up
    int count1 = 0;
    int count2 = 0;
    boolean forward = false;
    boolean backward = false;
    double manualServo = 0;
    boolean controlled = false;

    waitForStart();

    if (opModeIsActive()) {
      /*//
      if Nathan wants anything to happen at the start of tele op
      //*/

      while (opModeIsActive()) {
//Slow mode (and normal):
        if(!gamepad1.left_bumper && !gamepad1.right_bumper){
            frontLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x * 0.75);
            frontRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x * 0.75);
            backRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x * 0.75);
            backLeft.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x * 0.75);
        }
        else if(gamepad1.left_bumper || gamepad1.right_bumper){
            frontLeft.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) * 0.3);
            frontRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 0.3);
            backRight.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x) * 0.3);
            backLeft.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) * 0.3);
        }
//slides always have same power
      linearSlideRight.setPower(linearSlideLeft.getPower());
        
//manual sliding
      if (!gamepad2.x) {
        linearSlideLeft.setPower((1 * gamepad2.left_stick_y));
      }
      else if (gamepad2.x) {
        linearSlideLeft.setPower((0.33 * (gamepad2.left_stick_y)));
      }
    // if (linearSlideLeft.getCurrentPosition() < minimum) {
      // linearSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       //linearSlideLeft.setTargetPosition(minimum);
     //}
     
     
//telemetry
        telemetry.addData("linearSlideLeft pos: ", linearSlideLeft.getCurrentPosition());
        telemetry.addData("linearSlideLeft pow: ", linearSlideLeft.getPower());
        telemetry.addData("bucket position:", bucket.getPosition());
        telemetry.addData("right stick:", gamepad2.right_stick_y);
        telemetry.update();

//Intake:
        //manual intake
        intake.setPower(gamepad2.right_stick_y * 1);

        /*
        //for button intake
        if (gamepad2.a){intake.setPower(1);}
        else if (!gamepad2.a){intake.setPower(0);}
        */
        
//Bucket:
        if(gamepad2.right_bumper){
            //pos needs to be found
            //bucket.setPosition();
        }
        else if(gamepad2.left_bumper){
            //pos needs to be found
            //bucket.setPosition();
        }
        
        if(((gamepad2.right_trigger > 0.5) && (manualServo <= 1)) && (bucket.getPosition() <= 1)) {
          manualServo += 0.01;
          bucket.setPosition(manualServo);
        }
        if(((gamepad2.left_trigger > 0.5) && (manualServo >= 0)) && (bucket.getPosition() >= 0)) {
          manualServo -= 0.01;
          bucket.setPosition(manualServo);
        }

//Airplane
        if (gamepad2.y){fly.setPower(1);}
        if (gamepad2.b){
            //plane.setPosition();
            fly.setPower(0);
        }

        
//AutoLevel (tm)
        
       /* if(gamepad2.dpad_left && (controlled == false)){
          armCounter1++;
          if(armCounter1 == 1){
            linearSlideLeft.setTargetPosition(low);
            linearSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlideLeft.setPower(0.3);
          }
        }
        else if(!gamepad2.dpad_left) {
          armCounter1 = 0;
        }
        
        if(gamepad2.dpad_up && (controlled == false)){
          armCounter2++;
          if(armCounter2 == 1){ 
            linearSlideLeft.setTargetPosition(mid);
            linearSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlideLeft.setPower(0.3);
          }
        }
        else if(!gamepad2.dpad_up) {
          armCounter2 = 0;
        }
        
        if(gamepad2.dpad_right && (controlled == false)){
          armCounter3++;
          if(armCounter3 == 1){ 
            linearSlideLeft.setTargetPosition(low);
            linearSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlideLeft.setPower(0.3);
          }
        }
        else if(!gamepad2.dpad_right) {
          armCounter3 = 0;
        }
        
        if(gamepad2.dpad_down && (controlled == false)) {
          armCounter4++;
          if(armCounter4 == 1){ 
            linearSlideLeft.setTargetPosition(bottom);
            linearSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlideLeft.setPower(0.3);
          }
        }
        else if(!gamepad2.dpad_down) {
          armCounter4 = 0; 
        }
        
       /* deccel relic? if(inPos){
          diff = Math.abs(arm.getCurrentPosition() - tick);
          pwr = diff / 100;
          telemetry.addData("Power", pwr);
          telemetry.update();
          linearSlideLeft.setPower(pwr);
        } */
          
      } // while OpModeActive
    } // if OpModeActive
  } // runOpMode
} // LinearOpMode
