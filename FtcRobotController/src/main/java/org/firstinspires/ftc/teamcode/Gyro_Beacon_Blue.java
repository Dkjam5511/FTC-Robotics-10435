/*
Gyro Beacon Autonomous Blue
by Drew Kinneer 1/25/2017
Team 10435
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODERS;

@Autonomous(name = "Gyro_Beacon_Blue", group = "Beacon")

public class Gyro_Beacon_Blue extends LinearOpMode {

    //hardware
    Servo ball_gate_servo;
    DcMotor ShootMotor;
    DcMotor leftWheel;
    DcMotor rightWheel;
    I2cDevice ColorRight;
    I2cDeviceSynch ColorRightreader;  // right beacon sensor
    I2cDevice ColorLeft;
    I2cDeviceSynch ColorLeftreader;   // left beacon sensor
    DeviceInterfaceModule CDI;
    Servo btn_servo;
    OpticalDistanceSensor ODS;
    TouchSensor touchSensor;
    ModernRoboticsI2cGyro gyro;

    // Output of the go_straight_adjustment function
    double power_adjustment_L;
    double power_adjustment_R;
    boolean found_white = false;

    //Btn_Servo Variables
    double init_btn_servo_position = .45;
    double btn_servo_position;
    double btn_servo_degrees = .4;

    //Other
    int Passive = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        btn_servo = hardwareMap.servo.get("button_servo");
        touchSensor = hardwareMap.touchSensor.get("TouchSensor");
        ball_gate_servo = hardwareMap.servo.get("ball_gate");
        ShootMotor = hardwareMap.dcMotor.get("shoot_motor");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        CDI.setLED(0, true);           //Blue light On
        CDI.setLED(1, false);           //Red light OFF

        // Set up right beacon sensor
        ColorRight = hardwareMap.i2cDevice.get("cs_right");
        ColorRightreader = new I2cDeviceSynchImpl(ColorRight, I2cAddr.create8bit(0x3c), false);
        ColorRightreader.engage();
        ColorRightreader.write8(3, Passive);    //Set the mode of the color sensor to passive

        // Set up left beacon sensor
        ColorLeft = hardwareMap.i2cDevice.get("cs_left");
        ColorLeftreader = new I2cDeviceSynchImpl(ColorLeft, I2cAddr.create8bit(0x3a), false);
        ColorLeftreader.engage();
        ColorLeftreader.write8(3, Passive);    //Set the mode of the color sensor to passive

        btn_servo.setPosition(init_btn_servo_position);

        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ball_gate_servo.setPosition(1);

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();

        // get lined up
        go_forward(3, 0, .5, false);
        turn_to_heading(30);

        // go to first white line
        go_forward(72, 30, 1, true);
        if (!found_white) {
            turn_to_heading(0);  // If we missed the line, try to change angle before backing up.
            go_forward(14, 0, -.3, true);
        } else {
            go_forward(6, 30, -.3, true);
        }
        turn_to_heading(90);
        go_forward(14, 90, .5, false);

        // shoot balls
        Shoot();
        ball_gate_servo.setPosition(0);
        sleep(2000);
        Shoot();
        sleep(300);
        ball_gate_servo.setPosition(1);


        // hit first beacon
        button_push("blue");

        // back up, get lined up
        go_forward(8, 90, -1, false);
        turn_to_heading(0);

        // go to second white line
        go_forward(54, 0, 1, true);
        go_forward(8, 0, -.3, true);
        turn_to_heading(90);
        go_forward(14, 90, .5, false);

        // hit first beacon
        button_push("blue");

        // back up
        go_forward(16, 90, -1, false);

        sleep(1000);
        requestOpModeStop();

    } // end of RunOpMode


    public void Shoot() throws InterruptedException {
        //Shooting Variables
        int shoot_target = 2880;
        int start_position_S;

        start_position_S = ShootMotor.getCurrentPosition();
        ShootMotor.setPower(1);
        ShootMotor.setTargetPosition(shoot_target + start_position_S);
        sleep(500);
        ShootMotor.setPower(0);
    } // end of Shoot


    public void turn_to_heading(double target_heading) {
        boolean go_right;
        double current_heading;
        double degrees_to_turn;
        double wheel_power;

        current_heading = gyro.getHeading();
        go_right = target_heading > current_heading;
        degrees_to_turn = Math.abs(target_heading - current_heading);

        if (degrees_to_turn > 180) {
            go_right = !go_right;
            degrees_to_turn = 360 - degrees_to_turn;
        }

        while (degrees_to_turn > .5 && opModeIsActive()) {
            wheel_power = (10 * Math.pow((degrees_to_turn + 15) / 40, 3) + 10) / 100;

            if (go_right) {
                rightWheel.setPower(-wheel_power);
                leftWheel.setPower(wheel_power);
            } else {
                rightWheel.setPower(wheel_power);
                leftWheel.setPower(-wheel_power);
            }

            current_heading = gyro.getHeading();

            degrees_to_turn = Math.abs(target_heading - current_heading);
            go_right = target_heading > current_heading;

            if (degrees_to_turn > 180) {
                go_right = !go_right;
                degrees_to_turn = 360 - degrees_to_turn;
            }

            //telemetry.addData("Wheel Power", wheel_power);
            //telemetry.addData("Degrees to Turn", degrees_to_turn);
            //telemetry.addData("Current Heading", current_heading);
            //telemetry.update();

        }

        leftWheel.setPower(0);
        rightWheel.setPower(0);
        sleep(100);  // do we really need these?  Try without

    } // end of turn_to_heading


    public void go_straight_adjustment(int target_heading) {

        //  This function outputs global variables power_adjustment_R and power_adjustment_L

        double power_adjustment;
        double current_heading;
        double degrees_off;
        boolean go_right;

        current_heading = gyro.getHeading();
        go_right = target_heading > current_heading;
        degrees_off = Math.abs(target_heading - current_heading);

        if (degrees_off > 180) {
            go_right = !go_right;
            degrees_off = 360 - degrees_off;
        }

        if (degrees_off < 1) {
            power_adjustment = 0;
        } else {
            power_adjustment = (Math.pow((degrees_off + 2) / 3, 2) + 2) / 100;
        }

        if (go_right) {
            power_adjustment_R = -power_adjustment;
            power_adjustment_L = power_adjustment;
        } else {
            power_adjustment_R = power_adjustment;
            power_adjustment_L = -power_adjustment;
        }
    }


    public void go_forward(double inches_to_travel, int starting_angle, double speed, boolean find_white) {

        double current_speed = .05;
        double ticks_to_travel;
        boolean touch_sensor_pressed = false;
        boolean destination_reached = false;
        double white_value = .28;
        double speed_increase = .05;
        int start_position_L;
        int start_position_R;

        ticks_to_travel = inches_to_travel / 11.39 * 1440; // 11.39 is for matrix wheels which are 3.625 in diameter

        start_position_L = leftWheel.getCurrentPosition();
        start_position_R = rightWheel.getCurrentPosition();

        found_white = false;

        if (speed < 0) {
            speed_increase = -speed_increase;
        }

        while (opModeIsActive() && !destination_reached && !found_white && !touch_sensor_pressed) {

            current_speed = current_speed + speed_increase;  // this is to slowly ramp up the speed so we don't slip
            if (Math.abs(current_speed) > Math.abs(speed)) {
                current_speed = speed;
            }

            go_straight_adjustment(starting_angle);
            rightWheel.setPower(current_speed + power_adjustment_R);
            leftWheel.setPower(current_speed + power_adjustment_L);

            if (find_white) {
                found_white = ODS.getLightDetected() > white_value;
            }

            touch_sensor_pressed = touchSensor.isPressed();

            if (speed > 0) {
                destination_reached =
                        ((leftWheel.getCurrentPosition() >= start_position_L + ticks_to_travel) ||
                                (rightWheel.getCurrentPosition() >= start_position_R + ticks_to_travel));
            } else {
                destination_reached =
                        ((leftWheel.getCurrentPosition() <= start_position_L - ticks_to_travel) ||
                                (rightWheel.getCurrentPosition() <= start_position_R - ticks_to_travel));

            }
        }

        rightWheel.setPower(0);
        leftWheel.setPower(0);

        sleep(100);  // do we really need these?  Try without

    } // end of go_forward
    
    public void button_push(String BorR) {

        int colorlevelRight;
        int colorlevelLeft;
        byte[] TempByte;
        boolean button_pressed = false;
        double color_good = 8;
        int current_color = 0x07;
        
        if (BorR == "blue"){
            current_color = 0x07;
            color_good = 8;
        }else if (BorR == "red"){
            current_color = 0x05;
            color_good = 4;
        }
        
        

        // Do the first color reads
        TempByte = ColorRightreader.read(current_color, 1);
        colorlevelRight = TempByte[0];
        TempByte = ColorLeftreader.read(current_color, 1);
        colorlevelLeft = TempByte[0];

        // Button pressing section
        button_pressed = false;
        while (!button_pressed && opModeIsActive()) {

            if (colorlevelRight > colorlevelLeft && colorlevelRight >= color_good) {
                btn_servo_position = init_btn_servo_position - btn_servo_degrees;
                btn_servo.setPosition(btn_servo_position);
            } else if (colorlevelLeft > colorlevelRight && colorlevelLeft >= color_good) {
                btn_servo_position = init_btn_servo_position + btn_servo_degrees;
                btn_servo.setPosition(btn_servo_position);
            }
            sleep(1000);
            btn_servo.setPosition(init_btn_servo_position);

            //Read color sensosrs
            TempByte = ColorRightreader.read(current_color, 1);
            colorlevelRight = TempByte[0];
            TempByte = ColorLeftreader.read(current_color, 1);
            colorlevelLeft = TempByte[0];

            button_pressed = colorlevelLeft >= color_good && colorlevelRight >= color_good;
        }
    }
}

