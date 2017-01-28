package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Drew on 1/16/2017.
 */

// 1440 Tcks Per Revolution

@TeleOp(name = "Encoder Test", group = "Tests and Calibration")
public class Encoder_Test extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    int StartPositionL;
    int StartPositionR;
    int CurrentPositionL;
    int CurrentPositionR;
    int target = 2000;

    @Override
    public void init() {


        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setDirection(DcMotor.Direction.REVERSE);
    }

    private void Forward(double power) {

        StartPositionL = leftWheel.getCurrentPosition();
        StartPositionR = rightWheel.getCurrentPosition();

        leftWheel.setPower(power);
        rightWheel.setPower(power);

        leftWheel.setTargetPosition(StartPositionL + target);
        rightWheel.setTargetPosition(StartPositionR + target);

        if (leftWheel.isBusy() && rightWheel.isBusy()){
            telemetry.addData("Status", "Busy");
        }else{
            telemetry.addData("Status", "Free");
        }

    }

    private void Left (double power) {

        StartPositionL = leftWheel.getCurrentPosition();
        StartPositionR = rightWheel.getCurrentPosition();

        leftWheel.setPower(power);
        rightWheel.setPower(power);
        leftWheel.setTargetPosition(StartPositionL - target);
        rightWheel.setTargetPosition(StartPositionR + target);

        if (leftWheel.isBusy() && rightWheel.isBusy()){
            telemetry.addData("Status", "Busy");
        }else{
            telemetry.addData("Status", "Free");
        }

    }

    private void Right (double power) {

        StartPositionL = leftWheel.getCurrentPosition();
        StartPositionR = rightWheel.getCurrentPosition();

        leftWheel.setPower(power);
        rightWheel.setPower(power);
        leftWheel.setTargetPosition(StartPositionL + target);
        rightWheel.setTargetPosition(StartPositionR - target);

        if (leftWheel.isBusy() && rightWheel.isBusy()){
            telemetry.addData("Status", "Busy");
        }else{
            telemetry.addData("Status", "Free");
        }

    }

    private void Back (double power) {

        StartPositionL = leftWheel.getCurrentPosition();
        StartPositionR = rightWheel.getCurrentPosition();

        leftWheel.setPower(power);
        rightWheel.setPower(power);
        leftWheel.setTargetPosition(StartPositionL - target);
        rightWheel.setTargetPosition(StartPositionR - target);

        if (leftWheel.isBusy() && rightWheel.isBusy()){
            telemetry.addData("Status", "Busy");
        }else{
            telemetry.addData("Status", "Free");
        }

    }

    @Override
    public void loop() {

        if (gamepad1.y){
            Forward(.5);
        }
        if (gamepad1.x){
            Left(.5);
        }
        if (gamepad1.b){
            Right(.5);
        }
        if (gamepad1.a){
            Back(.5);
        }

        if (leftWheel.isBusy() && rightWheel.isBusy()){
            telemetry.addData("Status", "Busy");
        }else{
            telemetry.addData("Status", "Free");
        }

        telemetry.addData("Left Wheel", leftWheel.getCurrentPosition());
        telemetry.addData("Right Wheel", rightWheel.getCurrentPosition());
    }


}





