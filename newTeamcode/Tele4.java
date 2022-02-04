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
public class Tele_Copy extends LinearOpMode {
    
  private DcMotor frontLeft;
  private DcMotor backRight;
  private DcMotor frontRight;
  private DcMotor backLeft;
  private DcMotor carousel;
  private DcMotor intake;  
  private DcMotor arm;
  private Servo clamp;
  private Servo bucket;
  private BNO055IMU RevIMUAsBNO055IMU;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime colorWait = new ElapsedTime();
  private ColorSensor color;
  
  @Override
  public void runOpMode(){
    
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        carousel = hardwareMap.get(DcMotor.class, "duck");
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");
        bucket = hardwareMap.get(Servo.class, "intakeArm"); 
        clamp = hardwareMap.get(Servo.class, "clamp");
        RevIMUAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imu");
        color = hardwareMap.get(ColorSensor.class, "color");
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);

//VARIABLES:
    int counter = 0;
    int counter2 = 0;
    int tracker = 0;
    int tracker2 = 0;
    double speed = 0;
    double speeds = 0;
    double speed2 = 0;
    double time1 = 0;
    double time2 = 0.8;
    double time3 = 1.5;
    int armCounter1 = 0;
    int armCounter2 = 0;
    int armCounter3 = 0;
    int armCounter4 = 0;
    boolean isBlock = false;
    double pos = 0.00;
    double rest = .67;
    boolean inPos0 = false;
    boolean inPos1 = false;
    boolean inPos2 = false;
    boolean inPos3 = false;
    boolean control = false;
    double tick = 0;
    double diff = 0;
    double pwr = 0;
    int levelDist1 = -3500;
    int levelDist2 = -3000;
    int levelDist3 = -2200;
    double levelBuck1 = .40;
    double levelBuck2 = .42;
    double levelBuck3 = .62;

        waitForStart();

    if (opModeIsActive()) {
      pos = rest; 
      bucket.setPosition(pos);
      arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          

      while (opModeIsActive()) {
        
      
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
          
        if(gamepad2.a){
            speeds = 1;
        }
        
        if(!gamepad2.a && !gamepad2.b){
            speeds = 0;
        }  
        if(gamepad2.b){
          speeds = -.8;
        }
        
        intake.setPower(speeds);
        
        
        if(control){
          arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          arm.setPower(gamepad2.left_stick_y);
        }
        
          
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
        /*
        if(isBlock && !gamepad2.left_bumper && (bucket.getPosition() != 0.25)){
          if(colorWait.seconds() >= .25){
            bucket.setPosition(0.45);
          }
        }
        else{
          if(colorWait.seconds() >= 1){
          bucket.setPosition(0.65);
          }
        }
        */

//carousel:
        if(gamepad2.y){
        carousel.setPower(0.45);
        }
        
        if(!gamepad2.y && !gamepad2.x){
        carousel.setPower(0);
        }  
        if(gamepad2.x){
        carousel.setPower(-0.45);
        }
        
        
//bucket:
        if(gamepad2.right_bumper){
          clamp.setPosition(0.24);
        }
        if(gamepad2.left_stick_button){
          clamp.setPosition(0.5);
        }
        if(gamepad2.left_bumper){
          clamp.setPosition(0.0);
        }
        
        if(gamepad2.left_trigger >= .9){ 
          if(gamepad2.right_stick_y < 0){
              pos = pos + 0.01;
              bucket.setPosition(pos);
              if(pos >= 1){
                pos = 1;
              }
          }
          if(gamepad2.right_stick_y > 0){
              pos = pos - 0.01;
              bucket.setPosition(pos);
              if(pos <= 0){
                pos = 0;
              }
          }
        }
        else{
            if(gamepad2.right_stick_y > 0){
              pos = pos - 0.05;
              bucket.setPosition(pos);
              if(pos <= 0){
                pos = 0;
              }
          }
          if(gamepad2.right_stick_y < 0){
              pos = pos + 0.05;
              bucket.setPosition(pos);
              if(pos >= 1){
                pos = 1;
              }
          }
        }
        
  //arm
        if(gamepad2.right_trigger >= .9){
          inPos0 = false;
          inPos1 = false;
          inPos2 = false;
          inPos3 = false;
          control = true;
        }
        
        /*
        if(!arm.isBusy() && gamepad2.left_stick_y == 0){
          arm.setPower(0);
        }  
        */
        
        if(gamepad2.dpad_left){
          armCounter1++;
          if(armCounter1 == 1){ 
            inPos0 = false;
            inPos2 = false;
            inPos3 = false;
            control = false;
            bucket.setPosition(1);
            tick = levelDist1;
            arm.setTargetPosition(levelDist1);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.7);
            inPos1 = true;
          }
        }
       
        else if(!gamepad2.dpad_left) {
          armCounter1 = 0;
        }
        
        if((arm.getCurrentPosition()) >= (levelDist1 - 10) && (arm.getCurrentPosition()) <= (levelDist1 + 10) && inPos1 == true) {
         
         bucket.setPosition(levelBuck1);
         pos = levelBuck1;
        }
        
        if(gamepad2.dpad_up){
          armCounter2++;
          if(armCounter2 == 1){ 
            inPos0 = false;
            inPos1 = false;
            inPos3 = false;
            control = false;
            bucket.setPosition(1);
            tick = levelDist2;
            arm.setTargetPosition(levelDist2);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.7);
            inPos2 = true;
          }
        }
       
        else if(!gamepad2.dpad_up) {
          armCounter2 = 0;
        }
        
        if((arm.getCurrentPosition()) >= (levelDist2 - 10) && (arm.getCurrentPosition()) <= (levelDist2 + 10) && inPos2 == true) {
         
         bucket.setPosition(levelBuck2);
         pos = levelBuck2;
        }
        
        if(gamepad2.dpad_right){
          armCounter3++;
          if(armCounter3 == 1){ 
            inPos0 = false;
            inPos1 = false;
            inPos2 = false;
            control = false;
            bucket.setPosition(1);
            tick = levelDist3;
            arm.setTargetPosition(levelDist3);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.7);
            inPos3 = true;
          }
        }
       
        else if(!gamepad2.dpad_right) {
          armCounter3 = 0;
        }
        
        if((arm.getCurrentPosition()) >= (levelDist3 - 10) && (arm.getCurrentPosition()) <= (levelDist3 + 10) && inPos3 == true) {
         
         bucket.setPosition(levelBuck3);
         pos = levelBuck3;
        }
        
        if(gamepad2.dpad_down) {
          armCounter4++;
          if(armCounter4 == 1){ 
            inPos1 = false;
            inPos2 = false;
            inPos3 = false;
            control = false;
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bucket.setPosition(1);
            arm.setPower(0.7);
            inPos0 = true;
          }
        }
        
        else if(!gamepad2.dpad_down) {
          armCounter4 = 0; 
        }
        
        if((arm.getCurrentPosition()) >= -10 && (arm.getCurrentPosition()) <= 10 && inPos0 == true) {
        
         bucket.setPosition(rest);
         pos = rest;
        }
        
        if(inPos1 || inPos2 || inPos3){
          diff = Math.abs(arm.getCurrentPosition() - tick);
          pwr = diff / 100;
          telemetry.addData("Power", pwr);
          telemetry.update();
          arm.setPower(pwr);
        }

        } // while OpModeActive
      } // if OpModeActive
  } // runOpMode
} // LinearOpMode

