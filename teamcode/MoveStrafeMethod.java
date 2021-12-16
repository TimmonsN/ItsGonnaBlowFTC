   
   public void move(double speed, double distance) {
        telemetry.addData("start", "move");
        telemetry.update();
        double Multiplyer;
        double end;
        double start;
        double Multiplyer2;
        double power = 1;
        int direction;
        if(distance >= 0){
            direction = 1;
        } else{
            direction = -1;
            distance = distance * -1;
        }
        int ticCount = (int) Math.ceil(((distance / 12.12) * 537));
        // Put initialization blocks here.
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        end = Math.ceil(ticCount / 2.0);
        start = Math.ceil(ticCount / 4.0);
        //Multiplier = speed / (ticCount - end);
        //Multiplier2 = speed / start;
        backLeft.setTargetPosition(ticCount * direction);
        backRight.setTargetPosition(ticCount * direction);
        frontRight.setTargetPosition(ticCount * direction);
        frontLeft.setTargetPosition(ticCount * direction);
      
          while (!backRight.isBusy()) { 
          sleep(1);
          telemetry.addData("Stuck", "");
          telemetry.update();
        }
        while (backRight.isBusy()) {
           if ((backRight.getCurrentPosition() * direction) <= start) {
                power = (Math.abs((backRight.getCurrentPosition())) * (speed/start));
                power += .1;
            } else if ((Math.abs(backRight.getCurrentPosition())) >= end) {
                power = speed * ((ticCount - (backRight.getCurrentPosition()* direction)) / end);
                power += .1;
            } else {
                power = speed;
            }

           //boom?
            if (!ticCount - (Math.abs(backRight.getCurrentPosition())) == 0 && power < 0.1){
                power = 0.1;
            }
            frontRight.setPower(power * direction);
            frontLeft.setPower(power * direction);
            backLeft.setPower(power * direction);
            backRight.setPower(power * direction);
            telemetry.addData("power", power);
            telemetry.addLine;
            telemetry.addData("TickCount", ticCount);
            telemetry.update();
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        telemetry.addData("Stop", "move");
        telemetry.update();
        
              /*
      
      Positive Case - start at 0, go 50 forward
          Halfway is 25
      Negative Case - start at 0, go 50 backward
          Halfway is 25
      
      speed = 0.5
      end
      // pseudo 
      power = speed * ((distance from end) / (halfway))
      
      power = speed * ((ticCount - (backLeft.getCurrentPosition()) / end)
      power += .1
      
      // What you have
      power = (distance from end) * (speed / (distance from middle))
      
      
      */
        
    }
    
private void rotate(int turn) {
        telemetry.addData("start", "Turn");
        telemetry.update();
        Orientation angles;
        float AngleDif;
        int direction;
        int x = 0;
        double bonus = .2;

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
            if(x >= 3){
                count = 100000;
            }
        }
      }
        telemetry.addData("Stop", "");
        telemetry.update();
    }

    public void strafe(double speed, double distance) {
        telemetry.addData("start", "strafe");
        telemetry.update();
        double Multiplyer;
        double end;
        double start;
        double Multiplyer2;
        double power = 1;
        int direction;
        if(distance >= 0){
            direction = 1;
        } else{
            direction = -1;
            distance = distance * -1;
        }
        int ticCount = (int) Math.ceil(((distance / 12.12) * 537));
        // Put initialization blocks here.
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        end = Math.ceil(ticCount / 2.0);
        start = Math.ceil(ticCount / 4.0);
        //Multiplier = speed / (ticCount - end);
        //Multiplier2 = speed / start;
        backLeft.setTargetPosition(-ticCount * direction);
        backRight.setTargetPosition(ticCount * direction);
        frontRight.setTargetPosition(-ticCount * direction);
        frontLeft.setTargetPosition(ticCount * direction);
      
          while (!backRight.isBusy()) { 
          sleep(1);
          telemetry.addData("Stuck", "");
          telemetry.update();
        }
        while (backRight.isBusy() && Math.abs(power) >= 0.1) {
           if ((backRight.getCurrentPosition() * direction) <= start) {
                power = (Math.abs((backRight.getCurrentPosition())) * (speed/start));
                power += .1;
            } else if ((Math.abs(backRight.getCurrentPosition())) >= end) {
                power = speed * ((ticCount - (backRight.getCurrentPosition()* direction)) / end);
                power += .1;
            } else {
                power = speed;
            }
            frontRight.setPower(power * direction);
            frontLeft.setPower(power * direction);
            backLeft.setPower(power * direction);
            backRight.setPower(power * direction);
            telemetry.addData("power", power);
            telemetry.addLine;
            telemetry.addData("TickCount", ticCount);
            telemetry.update();
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        telemetry.addData("Stop", "strafe");
        telemetry.update();
    }