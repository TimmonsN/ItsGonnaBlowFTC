package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class GyroTest extends LinearOpMode {

  private DcMotor frontLeft;
  private DcMotor backRight;
  private DcMotor frontRight;
  private DcMotor backLeft;
  private DcMotor Wheel; //carousel
  private ElapsedTime     runtime = new ElapsedTime();  
  private BNO055IMU RevIMUAsBNO055IMU;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    backRight = hardwareMap.get(DcMotor.class, "BackRight");
    frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
    backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
    Wheel = hardwareMap.get(DcMotor.class, "Duck");
    RevIMUAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imu");
    frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    

      // Put initialization blocks here.
      BNO055IMU.Parameters IMUParameters;

        // Put initialization blocks here.
      IMUParameters = new BNO055IMU.Parameters();
      // Use degrees as angle unit
      IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
      // Express acceleration as m/s^2.
      IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
      // Disable logging.
      IMUParameters.loggingEnabled = false;
        // Initialize IMU.
      RevIMUAsBNO055IMU.initialize(IMUParameters);
    waitForStart();
    if(opModeIsActive()) {
      rotate(90);
      rotate(-90);
      rotate(-179);
      
      //rotate(174);
    }
  }
  
private void rotate(int turn) {
        telemetry.addData("start", "Turn");
        telemetry.update();
        Orientation angles;
        float AngleDif;
        int direction;
        int x = 0;
        double bonus = .3;

      backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      for (int count = 0; count < 250; count++) {
        angles = RevIMUAsBNO055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        AngleDif = angles.firstAngle - (turn);
        if(AngleDif >= 0){
            direction = 1;
        } else{
            direction = -1;
        }
        telemetry.addData("rot about Z", AngleDif);
        telemetry.update();
        frontLeft.setPower(((0.145 * AngleDif) / 10 + (bonus * direction)));
        backRight.setPower(-((0.145 * AngleDif) / 10 + (bonus * direction)));
        backLeft.setPower(((0.145 * AngleDif) / 10 + (bonus * direction)));
        frontRight.setPower(-((0.145 * AngleDif) / 10 + (bonus * direction)));
        if(bonus > 0.09){
          bonus -= 0.01;
        }
        if(Math.abs(AngleDif) <= 1.5){
            x++;
            if(x >= 5){
                count = 100000;
            }
        }
      }
        telemetry.addData("Stop", "");
        telemetry.update();
    }
}