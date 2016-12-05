package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Thread.sleep;


/**
 * Created by Drew on 10/16/2016.
 */
@Autonomous(name="Autonomous Left Corner", group="Ball and Park")
public class Autonomous_Left_Corner extends LinearOpMode {
    //Defining Varriables
    DcMotor leftWheel;
    DcMotor rightWheel;
    double leftWheelPower = 0;
    double rightWheelPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Defining the wheels
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        //Reversing one wheel that didn't work
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        //Setting Wheel Power
        leftWheelPower = -1;
        rightWheelPower = -1;

        waitForStart();
        //Makes the robot go forward for 2.1 seconds
        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);
        sleep(2100);
        //Turns the robot for 0.4 seconds
        leftWheel.setPower(1);
        rightWheel.setPower(-1);
        sleep(400);
        //Makes the Robot go forward for 3 seconds
        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);
        sleep(3000);
        //Stops the Robot
        leftWheel.setPower(0);
        rightWheel.setPower(0);


    }

}
