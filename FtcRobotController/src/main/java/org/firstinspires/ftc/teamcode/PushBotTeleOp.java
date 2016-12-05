package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
        import com.qualcomm.robotcore.hardware.Servo;

        import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Drew on 10/16/2016.
 */
@TeleOp(name="Drive", group="Drive")
public class PushBotTeleOp extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    Servo btn_servo;
    DeviceInterfaceModule CDI;
    double init_btn_servo_position = .45;
    double btn_servo_position;
    double btn_servo_degrees = .2;
    double leftWheelPower = 0;
    double rightWheelPower = 0;
    double Speed = 1;
    boolean SlowMode = false;
    boolean NormalMode = true;

    @Override
    public void init() {
        //Setting Up Devices in the Hardware Map
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        btn_servo = hardwareMap.servo.get("button_servo");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        // Reversing One Wheel That Was Going Backwards For Some Reason
        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        btn_servo.setPosition(init_btn_servo_position);
    }

    @Override
    public void start() {
    }


    @Override
    public void loop() {
        //Checking Normal Mode
        NormalMode = (gamepad1.right_trigger == 0);
        if (NormalMode = true){
            Speed = 1;
            SlowMode = false;
        }else {
            SlowMode = true;
        }
        //Turning on SlowMode
        SlowMode = (gamepad1.right_trigger == 1);
        if (SlowMode) {
            Speed = 0.25;
            NormalMode = false;
        }else {
            NormalMode = true;
        }
        //Setting the Wheel powers to the controller Input
        leftWheelPower = gamepad1.left_stick_y * Speed;
        rightWheelPower = gamepad1.right_stick_y * Speed;
        //Sending Those Wheel Powers to the Actual Wheels
        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);

        //Setting the btn_servo position to the bumpers
        if (gamepad1.right_bumper){
            btn_servo_position = init_btn_servo_position - btn_servo_degrees;
            btn_servo.setPosition(btn_servo_position);
        }

        if (gamepad1.left_bumper){
            btn_servo_position = init_btn_servo_position + btn_servo_degrees;
            btn_servo.setPosition(btn_servo_position);
        }

        if (gamepad1.y){
            btn_servo_position = init_btn_servo_position;
            btn_servo.setPosition(btn_servo_position);
        }
        //Changing light based on speed
        CDI.setLED(0, SlowMode);           //Blue light
        CDI.setLED(1, NormalMode);           //Red light

        //Adding Screen Display
        telemetry.addData("Speed", (leftWheelPower+rightWheelPower)/-2);
        telemetry.addData("SlowMode", SlowMode);
        telemetry.addData("Servo Position", btn_servo_position);

    }

    @Override
    public void stop() {

    }
}
