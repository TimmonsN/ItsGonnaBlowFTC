package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp

public class Test extends LinearOpMode {

  private DcMotor FrontLeft;
  private DcMotor BackRight;
  private DcMotor FrontRight;
  private DcMotor BackLeft;
  private DcMotor Carousel;
  private DcMotor Intake;
  private DcMotor Slide;
  private Servo Bucket;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    BackRight = hardwareMap.get(DcMotor.class, "BackRight");
    FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
    BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
    Carousel = hardwareMap.get(DcMotor.class, "Duck");
    Intake = hardwareMap.get(DcMotor.class, "Intake");
    Slide = hardwareMap.get(DcMotor.class, "Slide");
    Bucket = hardwareMap.get(Servo.class, "Bucket");
    int counter = 0;
    int tracker = 0;
    double speed = 0;
    double speed2 = 0;
    int counter2 = 0;
    int tracker2 = 0;
    

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      Bucket.setPosition(.63);
      Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      
      while (opModeIsActive()) {
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x);
        FrontRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        BackRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x);
        BackLeft.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x);
        
        Slide.setPower(gamepad2.left_stick_y);
        Intake.setPower(-gamepad2.right_stick_y);
        
        
        if(gamepad2.b){
          speed = .45;
        }
        if(!gamepad2.b){
          speed = 0;
        }   
        Carousel.setPower(speed);
          
        if(gamepad2.right_bumper){
          Bucket.setPosition(.63);
        }
        if(gamepad2.left_bumper){
          Bucket.setPosition(.25);
      }
    }
  }
}}