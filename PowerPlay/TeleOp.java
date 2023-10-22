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
  private DcMotor linearSlide;
  private DcMotor linearSlide1;
 // private DcMotor fourBar;
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
        linearSlide1 = hardwareMap.get(DcMotor.class, "linearSlide1");
//        fourBar = hardwareMap.get(DcMotor.class, "fourBar");
        probe = hardwareMap.get(Servo.class, "probe"); 
        RevIMUAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imu");
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  //      fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
 //       fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 //       fourBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // int minimum = linearSlide.getCurrentPosition();

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
//slides going opposite ways
      linearSlide1.setPower(-1 * linearSlide.getPower());
        
//manual sliding
      if (!gamepad2.x) {
        linearSlide.setPower((1 * gamepad2.left_stick_y));
      }
      else if (gamepad2.x) {
        linearSlide.setPower((0.33 * (gamepad2.left_stick_y)));
      }
    // if (linearSlide.getCurrentPosition() < minimum) {
      // linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       //linearSlide.setTargetPosition(minimum);
     //}
     
     
//virtual four bar
    
  //  if(gamepad2.a) {
  //  fourBar.setPower(0.5);
  //  }

  //  if (gamepad2.x) {
        // forward = true;
      //  fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      // fourBar.setTargetPosition(front);
  //    fourBar.setPower(0);
        
//    }
 //   if (gamepad2.y) {
   //    //backward = true;
   //    fourBar.setTargetPosition(-512);
   //    fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
   // }

   // if (backward) {
    //    if (fourBar.getCurrentPosition() < top) {
    //      fourBar.setPower(1 - (fourBar.getCurrentPosition() / top));
    //    }
        /*
        else if(fourBar.getCurrentPosition() > top) {
          fourBar.setPower(-0.3);
        }
        
        if (!fourBar.isBusy()) {
          fourBar.setPower(0);
          forward = false;
        }
        */
   // }
      /*
    else if (forward) {}
    else if (!forward && !backward) {
      fourBar.setPower(0);
      
    } 
      */
//telemetry
        telemetry.addData("linearSlide pos: ", linearSlide.getCurrentPosition());
        telemetry.addData("linearSlide pow: ", linearSlide.getPower());
        telemetry.addData("probe position:", probe.getPosition());
        telemetry.addData("right stick:", gamepad2.right_stick_y);
        telemetry.update();
        
//probe:
        if(gamepad2.right_bumper){
          probe.setPosition(0.16);
        }
        else if(gamepad2.left_bumper){
          probe.setPosition(0);
        }
        
        if(((gamepad2.right_trigger > 0.5) && (manualServo <= 1)) && (probe.getPosition() <= 1)) {
          manualServo += 0.01;
          probe.setPosition(manualServo);
        }
        if(((gamepad2.left_trigger > 0.5) && (manualServo >= 0)) && (probe.getPosition() >= 0)) {
          manualServo -= 0.01;
          probe.setPosition(manualServo);
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
