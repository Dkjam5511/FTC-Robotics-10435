package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by Drew on 1/23/2017.
 */
@TeleOp(name = "Gyro Test", group = "Tests and Calibration")
public class Gyro_Test extends LinearOpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    ModernRoboticsI2cGyro gyro;
    int StartPositionL;
    int StartPositionR;
    double WheelPowerBase = .7;
    double inches_to_travel = 120;
    double ticks_to_travel;
    double PowerAdjustment;
    double RelativeAngle;
    double StartingAngle;
    double degrees_off;
    boolean adjustleft;
    boolean straight;

    public void turn_to_heading(double target_heading) {
        boolean go_right;
        double current_heading;
        double degrees_to_turn;
        double wheel_power;

        current_heading = gyro.getHeading();

        degrees_to_turn = (Math.abs(target_heading - current_heading));

        go_right = target_heading > current_heading;

        if (degrees_to_turn > 180) {
            go_right = !go_right;
            degrees_to_turn = 360 - degrees_to_turn;
        }

        while (degrees_to_turn > .5 && opModeIsActive()) {
            wheel_power = (10 * Math.pow((degrees_to_turn + 15) / 40, 3) + 10 )/ 100;

            if (go_right) {
                rightWheel.setPower(-wheel_power);
                leftWheel.setPower(wheel_power);
            } else {
                rightWheel.setPower(wheel_power);
                leftWheel.setPower(-wheel_power);
            }

            current_heading = gyro.getHeading();

            degrees_to_turn = (Math.abs(target_heading - current_heading));
            go_right = target_heading > current_heading;

            if (degrees_to_turn > 180) {
                go_right = !go_right;
                degrees_to_turn = 360 - degrees_to_turn;
            }

            telemetry.addData("Wheel Power", wheel_power);
            telemetry.addData("Degrees to Turn", degrees_to_turn);
            telemetry.addData("Current Heading", current_heading);
            telemetry.update();

        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);

    }


    @Override
    public void runOpMode() throws InterruptedException {

        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftWheel.setDirection(DcMotor.Direction.REVERSE);


        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating() && opModeIsActive()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();

        turn_to_heading(180);
        sleep(1000);

        turn_to_heading(0);
        sleep(1000);

        turn_to_heading(270);
        sleep(1000);

        StartingAngle = gyro.getIntegratedZValue();

        ticks_to_travel = inches_to_travel / 12.5 * 1440;

        StartPositionL = leftWheel.getCurrentPosition();
        StartPositionR = rightWheel.getCurrentPosition();

        rightWheel.setPower(WheelPowerBase);
        leftWheel.setPower(WheelPowerBase);

        while (opModeIsActive() && (leftWheel.getCurrentPosition() < ticks_to_travel + StartPositionL)) {
            RelativeAngle = gyro.getIntegratedZValue();
            degrees_off = (RelativeAngle - StartingAngle);
            adjustleft = (degrees_off < 0);
            degrees_off = Math.abs(degrees_off);
            straight = (degrees_off < 1);

            if (straight) {
                    rightWheel.setPower(WheelPowerBase);
                    leftWheel.setPower(WheelPowerBase);
                    telemetry.addData("Going straight", degrees_off);
                } else {
                    PowerAdjustment = (Math.pow((degrees_off + 2) / 3, 2) + 2) / 100;
                    if (PowerAdjustment + WheelPowerBase > 1) {
                        PowerAdjustment = 1 - WheelPowerBase;
                    }
                    if (adjustleft) {
                        rightWheel.setPower(WheelPowerBase + PowerAdjustment);
                        leftWheel.setPower(WheelPowerBase - PowerAdjustment);
                    } else {
                        rightWheel.setPower(WheelPowerBase - PowerAdjustment);
                        leftWheel.setPower(WheelPowerBase + PowerAdjustment);
                    }
            }
            telemetry.addData("Power Adjustment", PowerAdjustment);
            telemetry.addData("Adjust Left", adjustleft);
            telemetry.addData("Relative Angle", RelativeAngle);
            telemetry.addData("Starting Angle", StartingAngle);
            telemetry.update();


        }


    }
}

