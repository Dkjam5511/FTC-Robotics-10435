package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
        import com.qualcomm.robotcore.hardware.Servo;

        import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Drew on 10/16/2016.
 * :P
 */
@TeleOp(name="Drive", group="Drive")
public class PushBotTeleOp extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor LiftMotor;
    Servo btn_servo;
    Servo left_lift_servo;
    Servo right_lift_servo;
    DeviceInterfaceModule CDI;
    double init_btn_servo_position = .45;
    double btn_servo_position;
    double btn_servo_degrees = .2;
    double init_lift_servo_position = 0;
    double lift_servo_position;
    double lift_servo_degrees = 50;
    double leftWheelPower = 0;
    double rightWheelPower = 0;
    double Speed = 1;
    double LiftSpeed = 1;
    double LeftSpeedInput;
    double RightSpeedInput;
    double ServoPower = 0;
    double LiftPower;
    boolean SlowMode = false;
    boolean BlueOn;
    boolean RedOn;
    boolean Reverse = false;

    @Override
    public void init() {
        //Setting Up Devices in the Hardware Map
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        LiftMotor = hardwareMap.dcMotor.get("lift_motor");
        btn_servo = hardwareMap.servo.get("button_servo");
        left_lift_servo = hardwareMap.servo.get("left_fork");
        right_lift_servo = hardwareMap.servo.get("right_fork");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        btn_servo.setPosition(init_btn_servo_position);
        right_lift_servo.setDirection(Servo.Direction.REVERSE);

        ServoPower = 0;
    }

    @Override
    public void start() {

    }


    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            SlowMode = false;
            BlueOn = false;
            RedOn = true;
        } else if (gamepad1.right_bumper) {
            SlowMode = true;
            BlueOn = true;
            RedOn = false;
        }

        if (SlowMode) {
            Speed = .25;
        } else {
            Speed = 1;
        }

        if (gamepad1.a) {
            Reverse = true;
        }

        if (gamepad1.b) {
            Reverse = false;
        }

        LeftSpeedInput = gamepad1.left_stick_y * Speed;
        RightSpeedInput = gamepad1.right_stick_y * Speed;

        if(gamepad1.dpad_up){
            LeftSpeedInput = -.09;
            RightSpeedInput = -.09;
        } else if (gamepad1.dpad_down){
            LeftSpeedInput = 0.09;
            RightSpeedInput = 0.09;
        } else if (gamepad1.dpad_left){
            LeftSpeedInput = 0.09;
            RightSpeedInput = -0.09;
        } else if (gamepad1.dpad_right){
            LeftSpeedInput = -0.09;
            RightSpeedInput = 0.09;
        }

        if (Reverse) {
            leftWheel.setDirection(DcMotor.Direction.REVERSE);
            rightWheel.setDirection(DcMotor.Direction.FORWARD);
            leftWheelPower = RightSpeedInput;
            rightWheelPower = LeftSpeedInput;
        } else {
            leftWheel.setDirection(DcMotor.Direction.FORWARD);
            rightWheel.setDirection(DcMotor.Direction.REVERSE);
            leftWheelPower = LeftSpeedInput;
            rightWheelPower = RightSpeedInput;
        }

        //Sending Those Wheel Powers to the Actual Wheels
        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);

        //Setting the btn_servo position to the triggers
        if (gamepad1.right_trigger == 1) {
            btn_servo_position = init_btn_servo_position - btn_servo_degrees;
            btn_servo.setPosition(btn_servo_position);
        }

        if (gamepad1.left_trigger == 1) {
            btn_servo_position = init_btn_servo_position + btn_servo_degrees;
            btn_servo.setPosition(btn_servo_position);
        }

        if (gamepad1.y) {
            btn_servo_position = init_btn_servo_position;
            btn_servo.setPosition(btn_servo_position);
        }

        /*
        if (gamepad2.y) {

            lift_servo_position = init_lift_servo_position - lift_servo_degrees;
            right_lift_servo.setPosition(btn_servo_position);
            left_lift_servo.setPosition(btn_servo_position);
        }

        if (gamepad2.a) {
            lift_servo_position = init_lift_servo_position + lift_servo_degrees;
            right_lift_servo.setPosition(btn_servo_position);
            left_lift_servo.setPosition(btn_servo_position);
        }
        */
        if (gamepad2.right_bumper){
            LiftSpeed = .5;
        }

        if (gamepad2.left_bumper){
            LiftSpeed = 1;
        }


        ServoPower = gamepad2.right_stick_y / 2 + .5;
        right_lift_servo.setPosition(ServoPower);
        left_lift_servo.setPosition(ServoPower);

        LiftPower = gamepad2.left_stick_y * LiftSpeed;
        LiftMotor.setPower(LiftPower / 2);



        //Changing light based on speed
        CDI.setLED(0, BlueOn);           //Blue light
        CDI.setLED(1, RedOn);           //Red light

        telemetry.addData("Reverse", Reverse);
        telemetry.addData("SlowMode", SlowMode);

    }

    @Override
    public void stop () {

    }
}
