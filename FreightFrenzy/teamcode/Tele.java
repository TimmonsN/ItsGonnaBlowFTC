package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Tele extends LinearOpMode {
    
  private DcMotor FrontLeft;
  private DcMotor BackRight;
  private DcMotor FrontRight;
  private DcMotor BackLeft;
  private DcMotor Carousel;
  private DcMotor Intake;
  private DcMotor Slide;
  private Servo Bucket;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime colorWait = new ElapsedTime();
  private ColorSensor color;
  
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
    color = hardwareMap.get(ColorSensor.class, "color");

//VARIABLES:
    int counter = 0;
    int counter2 = 0;
    int tracker = 0;
    int tracker2 = 0;
    double speed = 0;
    double speed2 = 0;
    double time1 = 0;
    double time2 = 0.8;
    double time3 = 1.5;
    int slideCounter1 = 0;
    int slideCounter2 = 0;
    int slideCounter3 = 0;
    boolean isBlock = false;

        waitForStart();

    if (opModeIsActive()) {
        Bucket.setPosition(0.65);
           Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      while (opModeIsActive()) {
          FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
          BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
          FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      
        if(!gamepad1.left_bumper){
            FrontLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x);
            FrontRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            BackRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x);
            BackLeft.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x);
        }
        else if(gamepad1.left_bumper){
            FrontLeft.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) / 2);
            FrontRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / 2);
            BackRight.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x) / 2);
            BackLeft.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) / 2);
        }
          
          
        Intake.setPower(-(gamepad2.right_stick_y/1.01));
          
        if(!gamepad2.dpad_down && !gamepad2.dpad_right && !gamepad2.dpad_left) {
            Slide.setPower(gamepad2.left_stick_y);
        }

          telemetry.addData("Time Count", runtime.seconds());
          telemetry.update();
          
//color
        if(color.alpha() > 175){
          if(!isBlock){
            colorWait.reset();
          }
          isBlock = true;
        }
        else{
          if(isBlock){
            colorWait.reset();
          }
          isBlock = false;
        }
        if(isBlock && !gamepad2.left_bumper && (Bucket.getPosition() != 0.25)){
          if(colorWait.seconds() >= .25){
            Bucket.setPosition(0.45);
          }
        }
        else{
          if(colorWait.seconds() >= 1){
          Bucket.setPosition(0.65);
          }
        }

//CAROUSEL:
        if(gamepad2.y){
            speed = 0.45;
        }
        
        if(!gamepad2.y && !gamepad2.x){
            speed = 0;
        }  
        if(gamepad2.x){
          speed = -0.45;
        }
        
        Carousel.setPower(speed);
        
//BUCKET:
        if(gamepad2.right_bumper){
            Bucket.setPosition(0.65);
        }
        
        if(gamepad2.left_bumper){
            Bucket.setPosition(0.25);
        }
        
//LINEAR SLIDE:
        /*if(gamepad2.dpad_left) {
          slideCounter1++;

          if(slideCounter1 == 1) {
            runtime.reset();
          }

          if (runtime.seconds() <= (time1 + 1)) {
            Slide.setPower(0);
            Bucket.setPosition(0.25);
          }

          else if ((time1 + 1) < runtime.seconds()) {
            Bucket.setPosition(0.63);
          }
        }

        else if(!gamepad2.dpad_left) {
            slideCounter1 = 0;
        }
                  
        if(gamepad2.dpad_down) {
          slideCounter2++;

          if(slideCounter2 == 1) {
            runtime.reset();
          }

          if (runtime.seconds() <= time2) {
            Slide.setPower(1);
          }

          else if (time2 < runtime.seconds() && runtime.seconds() <= (time2 + 1)) {
            Slide.setPower(0);
            Bucket.setPosition(0.25);
          }

          else if ((time2 + 1) < runtime.seconds() && runtime.seconds() <= (time2 + time2 + 1)) {
            Bucket.setPosition(0.63);
            Slide.setPower(-1);
          }
          else if(runtime.seconds() > (time2 + time2 + 1)) {
            Slide.setPower(0);
          }
        }

        else if(!gamepad2.dpad_down) {
          slideCounter2 = 0;
        }              

        if(gamepad2.dpad_right) {
          slideCounter3++;
            if(slideCounter3 == 1) {
              runtime.reset();
          }

          if (runtime.seconds() <= time3) {
            Slide.setPower(1);
          }

          else if (time3 < runtime.seconds() && runtime.seconds() <= (time3 + 1)) {
              Slide.setPower(0);
              Bucket.setPosition(0.25);
          }

          else if ((time3 + 1) < runtime.seconds() && runtime.seconds() <= (time3 + time3 + 1)) {
            Bucket.setPosition(0.63);
              Slide.setPower(-1);
          }
          
          else if(runtime.seconds() > (time3 + time3 + 1)) {
            Slide.setPower(0);
          }
        } 

        else if(!gamepad2.dpad_right) {
            slideCounter3 = 0;
        }
        */

        } // while OpModeActive
      } // if OpModeActive
  } // runOpMode
} // LinearOpMode
