/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class Pilot {
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private Servo bucket;
    private DcMotor duck;
    private Blinker expansion_Hub_2;
    private Blinker expansion_Hub_4;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor intake;
    private DcMotor slide;
    private Gyroscope imu;
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            Move(0.25, 48, true);
        }
    }
    
    private void Move(double Speed, int Distance, boolean Forward) {
        double Multiplyer;
        double Division;
        double Start;
        double Multiplyer2;
        double Power;
        int Direction;
        int TicCount = (Distance / 12) * 538;
        // Put initialization blocks here.
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            //telemetry.addData("Division", BackLeft.getCurrentPosition());
            //telemetry.update();
        }
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

}
*/