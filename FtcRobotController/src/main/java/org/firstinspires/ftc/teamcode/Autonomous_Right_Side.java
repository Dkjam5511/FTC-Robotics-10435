package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Thread.sleep;


/**
 * Created by Drew on 10/16/2016.
 */
// If you want to see how this program works go to the Autonomous_Left_Corner class which is pretty much the same class with a few adjustments :P
@Autonomous(name="Autonomous Right Side", group="Ball and Park")
public class Autonomous_Right_Side extends LinearOpMode {
    //Defining Varriables
    DcMotor leftWheel;
    DcMotor rightWheel;
    double leftWheelPower = 0;
    double rightWheelPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");

        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheelPower = -1;
        rightWheelPower = -1;
        waitForStart();

        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);
        sleep(1700);

        leftWheel.setPower(-1);
        rightWheel.setPower(1);
        sleep(720);

        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);
        sleep(3300);

        leftWheel.setPower(0);
        rightWheel.setPower(0);


    }

}
