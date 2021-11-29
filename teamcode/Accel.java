package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Accel extends LinearOpMode {

  private DcMotor FrontLeft;
  private DcMotor FrontRight;
  private DcMotor BackRight;
  private DcMotor BackLeft;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double Multiplyer;
    int TicCount;
    double Speed;
    int Division;
    int Start;
    double Multiplyer2;
    double Power;
    boolean Forward;
    int Direction;

    FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
    BackRight = hardwareMap.get(DcMotor.class, "BackRight");
    BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
    }
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    BackRight.setPower(0);
    BackLeft.setPower(0);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Forward = false;
    Speed = 0.5;
    TicCount = 2500;
    Division = (TicCount / 2) * 1;
    Start = TicCount / 4;
    Multiplyer = Speed / (TicCount - Division);
    Multiplyer2 = Speed / Start;
    if(Forward){
      Direction = 1;
    } else{
      Direction = -1;
    }
    BackLeft.setTargetPosition(TicCount * Direction);
    BackRight.setTargetPosition(TicCount * Direction);
    FrontRight.setTargetPosition(TicCount * Direction);
    FrontLeft.setTargetPosition(TicCount * Direction);
    while (BackLeft.isBusy()) {
      if ((BackLeft.getCurrentPosition() * Direction) >= Division) {
        Power = ((TicCount - (BackLeft.getCurrentPosition() * Direction)) * Multiplyer + 0.05);
      } else if ((BackLeft.getCurrentPosition()* Direction) <= Start) {
        Power = ((BackLeft.getCurrentPosition() * Direction) * Multiplyer2 + 0.05);
      } else {
        Power = (Speed);
      }
      FrontRight.setPower(Power * Direction);
      FrontLeft.setPower(Power * Direction);
      BackLeft.setPower(Power * Direction);
      BackRight.setPower(Power * Direction);
      telemetry.addData("Division", BackLeft.getCurrentPosition());
      telemetry.update();
    }
    FrontRight.setPower(0);
    FrontLeft.setPower(0);
    BackLeft.setPower(0);
    BackRight.setPower(0);
  }
}