package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Drew on 11/29/2016.
 */
@TeleOp (name="Servo Calibration", group="Tests and Calibration")
public class Servo_Calibration extends OpMode{
    Servo servo;
    double servo_position;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("button_servo");
        servo_position = .63;
        servo.setPosition(servo_position);

    }

    @Override
    public void loop() {

    }


}
