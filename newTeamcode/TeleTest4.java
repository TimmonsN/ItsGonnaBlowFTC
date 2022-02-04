package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled

public class TeleTest_Copy extends LinearOpMode {
    
  private DcMotor FrontLeft;
  private DcMotor BackRight;
  private DcMotor FrontRight;
  private DcMotor BackLeft;
  private DcMotor Carousel;
  private DcMotor Intake;
  private DcMotor Slide;
  private Servo Bucket;
  private ElapsedTime    runtime = new ElapsedTime(); 

  @Override
  public void runOpMode(){
    
    FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    BackRight = hardwareMap.get(DcMotor.class, "BackRight");
    FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
    BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
    Carousel = hardwareMap.get(DcMotor.class, "Duck");     //double check for duck/carousel discrepencies
    Intake = hardwareMap.get(DcMotor.class, "Intake");
    Slide = hardwareMap.get(DcMotor.class, "Slide");
    Bucket = hardwareMap.get(Servo.class, "Bucket");

//VARIABLES:
    int counter = 0;
    int counter2 = 0;
    boolean counter3 = false;
    int tracker = 0;
    int tracker2 = 0;
    double speed = 0;
    double speed2 = 0;
    int slideCounter1 = 0;
    int slideCounter2 = 0;
    int slideCounter3 = 0;
    int slideCounter4 = 0;
        waitForStart();

    if (opModeIsActive()) {
        Bucket.setPosition(0.63);
           Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      while (opModeIsActive()) {
          FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
          BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      
             if(gamepad1.left_bumper){
          FrontLeft.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) / 2);
          FrontRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / 2);
          BackRight.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x) / 2);
          BackLeft.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) / 2);
        }
        
        else{
          FrontLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x);
          FrontRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
          BackRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x);
          BackLeft.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x);
        }
        
          Intake.setPower(-gamepad2.right_stick_y);
        
          if(!gamepad2.dpad_left && !gamepad2.dpad_down && !gamepad2.dpad_right) {
          Slide.setPower(gamepad2.left_stick_y);
        }

          telemetry.addData("Time Count", runtime.seconds());
          telemetry.update();

//CAROUSEL:
        if(gamepad2.b){
            speed = 0.45;
        }
        
        if(!gamepad2.b){
            speed = 0;
        }   
        
        Carousel.setPower(speed);
        
//BUCKET:
        if(gamepad2.right_bumper){
            Bucket.setPosition(0.63);
        }
        
        if(gamepad2.left_bumper){
            Bucket.setPosition(0.25);
        }
        
//LINEAR SLIDE:
        
        
        if(gamepad2.dpad_left){
          slideCounter1++;
          if(slideCounter1 == 1) {
            setSlidePosition(0);
          }
        }
        
        else if(!gamepad2.dpad_left) {
          slideCounter1 = 0;
        }
        
        if(gamepad2.dpad_up){
          slideCounter2++;
          if(slideCounter2 == 1) {
            setSlidePosition(1);
          }
        }
        
        else if(!gamepad2.dpad_up) {
          slideCounter2 = 0;
        }
        
        if(gamepad2.dpad_right){
          slideCounter3++;
          if(slideCounter3 == 1) {
            setSlidePosition(2);
          }
        }
          
        else if(!gamepad2.dpad_right) {
          slideCounter3 = 0;
        }
        
        if(gamepad2.dpad_down) {
          slideCounter4++;
          if(slideCounter4 == 1) {
            runtime.reset(); 
            if(counter3 = false) {
              while (runtime.seconds() <= time3) {
                Slide.setPower(1);
              }
            }
            else if(counter3 = true) {
              while (runtime.seconds() <= time3) {
               Slide.setPower(-1);
              }
            }
            if(counter3 == true) {
              counter3 = false;
            }
            else if(counter3 == false) {
              counter3 = true;
            }
          }
        }
        
        
        else if(!gamepad2.dpad_down) {
          slideCounter4 = 0;
        }
        
        
        } // while OpModeActive
      } // if OpModeActive
  } // runOpMode
  
  int levelWanted = 0;
  int lastLevelWanted = 0;
  double time1 = 0;
  double time2 = 1.1;
  double time3 = 0.5;
  private void setSlidePosition(int levelWanted) {
          
          if(lastLevelWanted == 0){
            if(levelWanted == 0){
              lastLevelWanted = 0;
            }
            else if(levelWanted == 1){
              runtime.reset();
              while ((runtime.seconds() <= time2)){
                Slide.setPower(1);
              }
              Slide.setPower(0);
              lastLevelWanted = 1;
            }
            else if(levelWanted == 2){
              runtime.reset();
              while ((runtime.seconds() <= time3)){
                Slide.setPower(1);
              }
              Slide.setPower(0);
              lastLevelWanted = 2;
            }
          }
          
          else if(lastLevelWanted == 1){
            if(levelWanted == 0){
              runtime.reset();
              while ((runtime.seconds() <= time2)){
                Slide.setPower(-1);
              }
              Slide.setPower(0);
              lastLevelWanted = 0;
            }
            else if(levelWanted == 1){
              lastLevelWanted = 1;
            }
            else if(levelWanted == 2){
              runtime.reset();
              while ((runtime.seconds() <= (time3 - time2))){
                Slide.setPower(1);
              }
              Slide.setPower(0);
              lastLevelWanted = 2;
            }
          }
          
          else if(lastLevelWanted == 2){
            if(levelWanted == 0){
              runtime.reset();
              while ((runtime.seconds() <= time3)){
                Slide.setPower(-1);
              }
              Slide.setPower(0);
              lastLevelWanted = 0;
            }
            else if(levelWanted == 1){
              runtime.reset();
              while ((runtime.seconds() <= (time3 - time2))){
                Slide.setPower(-1);
              }
              Slide.setPower(0);
              lastLevelWanted = 1;
            }
            else if(levelWanted == 2){ 
              lastLevelWanted = 2;
            }
          }
        }//setPosition method
} // LinearOpMode
