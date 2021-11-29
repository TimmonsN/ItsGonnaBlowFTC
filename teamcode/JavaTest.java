package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class JavaTest extends LinearOpMode {

  private DcMotor FrontLeftAsDcMotor;
  private DcMotor BackRightAsDcMotor;
  private DcMotor FrontRightAsDcMotor;
  private DcMotor BackLeftAsDcMotor;
  private DcMotor WheelAsDcMotor; //carousel
  private ElapsedTime     runtime = new ElapsedTime();  

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    FrontLeftAsDcMotor = hardwareMap.get(DcMotor.class, "Front Left");
    BackRightAsDcMotor = hardwareMap.get(DcMotor.class, "Back Right");
    FrontRightAsDcMotor = hardwareMap.get(DcMotor.class, "Front Right");
    BackLeftAsDcMotor = hardwareMap.get(DcMotor.class, "Back Left");
    WheelAsDcMotor = hardwareMap.get(DcMotor.class, "Wheel");
    

    // Put initialization blocks here.
    waitForStart();
    if(opModeIsActive()) {
      // Put run blocks here.
      double speed = 0;
      int counter = 0;
      
      while (opModeIsActive()) {
        FrontLeftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeftAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeftAsDcMotor.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x);
        FrontRightAsDcMotor.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        BackRightAsDcMotor.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x);
        BackLeftAsDcMotor.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x);
         
        if(gamepad1.dpad_up){
          counter++;
           
          if(counter == 1){
              speed = 1;
          }
        }
        else if(gamepad1.dpad_down){
          counter++;
           
          if(counter == 1){
              speed = 0;
          }
        }

        if(!gamepad1.dpad_up && !gamepad1.dpad_down){
          counter = 0;
        }
        
        
        WheelAsDcMotor.setPower(speed); //problem line
        telemetry.addData("Wheel Speed", speed);
        telemetry.update();
        
        
        
      }
    }
  }
}