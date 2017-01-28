package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Drew on 1/21/2017.
 */

// 1440 Tcks Per Revolution
@TeleOp(name = "Ball Shooter Test", group = "Tests and Calibration")
public class Ball_Shooter_Test extends LinearOpMode {

    Servo ball_gate_servo;
    DcMotor ShootMotor;
    int ShootTarget = 2880;
    int StartPositionS;

    public void Shoot() throws InterruptedException {
        StartPositionS = ShootMotor.getCurrentPosition();
        ShootMotor.setPower(1);
        ShootMotor.setTargetPosition(ShootTarget + StartPositionS);
        sleep(500);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ball_gate_servo = hardwareMap.servo.get("ball_gate");
        ShootMotor = hardwareMap.dcMotor.get("shoot_motor");

        ShootMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        Shoot();

        ball_gate_servo.setPosition(0);

        sleep(1000);

        Shoot();

        sleep(300);

        ball_gate_servo.setPosition(1);

    }
}
