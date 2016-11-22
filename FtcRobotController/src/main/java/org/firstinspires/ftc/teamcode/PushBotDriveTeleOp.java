package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Drew on 10/16/2016.
 */

public class PushBotDriveTeleOp extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    double leftWheelPower = 0;
    double rightWheelPower = 0;
    double Speed = 1;
    boolean SlowMode = false;
    boolean NormalMode = true;

    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");

        leftWheel.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {
    }


    @Override
    public void loop() {
        NormalMode = (gamepad1.right_trigger == 0);
        if (NormalMode = true){
            Speed = 1;
        }else {
            SlowMode = true;
        }

        SlowMode = (gamepad1.right_trigger == 1);
        if (SlowMode) {
            Speed = 0.25;
        }else {
            NormalMode = true;
        }

        leftWheelPower = gamepad1.left_stick_y * Speed;
        rightWheelPower = gamepad1.right_stick_y * Speed;

        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);


    }

    @Override
    public void stop() {

    }
}
