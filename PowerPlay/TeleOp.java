package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
  private DcMotor linearSlide;
  private Servo probe;
  private BNO055IMU RevIMUAsBNO055IMU;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime colorWait = new ElapsedTime();
  
  @Override
  public void runOpMode(){
    
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        probe = hardwareMap.get(Servo.class, "probe"); 
        RevIMUAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imu");
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // int minimum = linearSlide.getCurrentPosition();

//VARIABLES:
    
    int armCounter1 = 0;
    int armCounter2 = 0;
    int armCounter3 = 0;
    int armCounter4 = 0;
    /* No boolean inPos0 = false;
    boolean inPos1 = false;
    boolean inPos2 = false;
    boolean inPos3 = false;
    double tick = 0;
    double diff = 0;
    double pwr = 0; */
    int low = 1040;   // Testing encoder heights: 1040 ish, approx. 80 per inch
    int mid = 1840;   //                          1840 ish
    int high = 2640;  //                          2640 ish
    int bottom = 0;   //                          240 ish
    boolean controlled = false;

    waitForStart();

    if (opModeIsActive()) {
      /*//
      if Nathan wants anything to happen at the start of tele op
      //*/

      while (opModeIsActive()) {
//Slow mode (and normal):
        if(!gamepad1.left_bumper){
            frontLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x);
            frontRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            backRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x);
            backLeft.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x);
        }
        else if(gamepad1.left_bumper){
            frontLeft.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) / 2);
            frontRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / 2);
            backRight.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x) / 2);
            backLeft.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) / 2);
        }
          
        
//manual sliding
   
      if (!gamepad2.b) {
        linearSlide.setPower(-0.75 * gamepad2.left_stick_y);
      }
      else if (gamepad2.b) {
        linearSlide.setPower(-0.33 * (gamepad2.left_stick_y));
      }
    // if (linearSlide.getCurrentPosition() < minimum) {
      // linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       //linearSlide.setTargetPosition(minimum);
     //}
      
        telemetry.addData("linearSlide pos: ", linearSlide.getCurrentPosition());
        telemetry.addData("probe position:", probe.getPosition());
        telemetry.addData("left stick value:", gamepad2.left_stick_y);
        telemetry.update();
        
       if(gamepad2.right_stick_y != 0) {
         controlled = true;
       }
        
//probe:
        if(gamepad2.right_bumper){
          probe.setPosition(0.5);
        }
        else if(gamepad2.left_bumper){
          probe.setPosition(0);
        }
        if(gamepad2.a){
          if(probe.getPosition() > 0.1) {
            probe.setPosition(0);
          }
          else {
            probe.setPosition(0.5);
          }
        }
        
//AutoLevel (tm)
        
       /* if(gamepad2.dpad_left && (controlled == false)){
          armCounter1++;
          if(armCounter1 == 1){
            linearSlide.setTargetPosition(low);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(0.3);
          }
        }
        else if(!gamepad2.dpad_left) {
          armCounter1 = 0;
        }
        
        if(gamepad2.dpad_up && (controlled == false)){
          armCounter2++;
          if(armCounter2 == 1){ 
            linearSlide.setTargetPosition(mid);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(0.3);
          }
        }
        else if(!gamepad2.dpad_up) {
          armCounter2 = 0;
        }
        
        if(gamepad2.dpad_right && (controlled == false)){
          armCounter3++;
          if(armCounter3 == 1){ 
            linearSlide.setTargetPosition(low);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(0.3);
          }
        }
        else if(!gamepad2.dpad_right) {
          armCounter3 = 0;
        }
        
        if(gamepad2.dpad_down && (controlled == false)) {
          armCounter4++;
          if(armCounter4 == 1){ 
            linearSlide.setTargetPosition(bottom);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(0.3);
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
          linearSlide.setPower(pwr);
        } */
          
        } // while OpModeActive
      } // if OpModeActive
  } // runOpMode
} // LinearOpMode
