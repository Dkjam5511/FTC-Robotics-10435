/*
Modern Robotics ODS Wall Follow Example
Updated 11/4/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
Optical Distance sensor named "ods"
Left drive train motor named "ml"  (two letters)
Right drive train motor named "mr"
Both motors need encoders

For more information, go to http://modernroboticsedu.com/course/view.php?id=5
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "Beacon_Autonomous_Blue", group = "Beacon")

public class Beacon_Autonomous_Blue extends LinearOpMode {

    //Instance of OpticalDistanceSensor
    OpticalDistanceSensor ODS;

    //Motors
    DcMotor leftWheel;
    DcMotor rightWheel;
    I2cDevice ColorRight;
    I2cDeviceSynch ColorRightreader;  // right beacon sensor
    I2cDevice ColorLeft;
    I2cDeviceSynch ColorLeftreader;   // left beacon sensor
    TouchSensor touchSensor;
    DeviceInterfaceModule CDI;
    Servo btn_servo;
    double white_level;
    double inside_correction = .07;
    double outside_correction = .1;
    double straight_speed = .13;
    double light_reading;
    double perfect_value = .34;
    double wheelpower_base = .15;
    double fuzz_factor = .03;
    double init_btn_servo_position = .45;
    double btn_servo_position;
    double btn_servo_degrees = .25;

    /* color sensor variables */
    int Passive = 1;
    int bluelevelRight;
    int redlevelRight;
    int bluelevelLeft;
    int redlevelLeft;
    boolean found_white = false;
    double rightwheelpower;
    double leftwheelpower;
    byte[] TempByte;
    double red_good = 14;
    double blue_good = 30;
    boolean too_far_away = true;
    boolean BluePressed = false;

    /* telemetry */
    boolean do_telemetry = false;

    @Override
    public void runOpMode() throws InterruptedException {

        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        btn_servo = hardwareMap.servo.get("button_servo");
        touchSensor = hardwareMap.touchSensor.get("TouchSensor");
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

        waitForStart();

            /*
            // Go forward fast for a bit
            leftWheel.setPower(1);
            rightWheel.setPower(1);
            sleep(400);
            */

        while (too_far_away && opModeIsActive()) {

            while (!found_white && opModeIsActive()) {
                leftWheel.setPower(wheelpower_base + .01);
                rightWheel.setPower(wheelpower_base);
                light_reading = ODS.getLightDetected();
                found_white = light_reading >= perfect_value - fuzz_factor;
                if (found_white) {
                    leftWheel.setPower(wheelpower_base);
                    rightWheel.setPower(wheelpower_base);
                    sleep(280);  // Go over the line until back wheels over the line
                    leftWheel.setPower(0);
                    rightWheel.setPower(0);
                    sleep(200);
                }
                if (do_telemetry) {telemetry.addData("Light reading", light_reading);}
                if (do_telemetry) {telemetry.update();}
            }

            found_white = false;  // Turn right until it gets back on the line
            while (!found_white && opModeIsActive()) {
                leftWheel.setPower(.1);
                rightWheel.setPower(-.1);
                light_reading = ODS.getLightDetected();
                found_white = light_reading >= perfect_value - fuzz_factor;
                if (do_telemetry) {telemetry.addData("Light reading", light_reading);}
                if (do_telemetry) {telemetry.update();}
            }

            leftWheel.setPower(0);
            rightWheel.setPower(0);

            light_reading = ODS.getLightDetected();

            white_level = light_reading - perfect_value;

            if (do_telemetry) {telemetry.addData("Light reading", light_reading);}

            if (white_level < -fuzz_factor) {                    // Gray - turn right
                rightwheelpower = -inside_correction;
                leftwheelpower = outside_correction;
                if (do_telemetry) {telemetry.addData("turning right lw", leftwheelpower);}
                if (do_telemetry) {telemetry.addData("turning right rw", rightwheelpower);}
            } else if (white_level > fuzz_factor){               // White - turn left
                leftwheelpower = -inside_correction;
                rightwheelpower = outside_correction;
                if (do_telemetry) {telemetry.addData("turning left lw", leftwheelpower);}
                if (do_telemetry) {telemetry.addData("turning left rw", rightwheelpower);}
            } else {                                            // go straight
                leftwheelpower = straight_speed;
                rightwheelpower = straight_speed;
                if (do_telemetry) {telemetry.addData("going straight", leftwheelpower);}
            }

            rightWheel.setPower(rightwheelpower);
            leftWheel.setPower(leftwheelpower);

            //Setting Up Colors For Sensor Right
            TempByte = ColorRightreader.read(0x05, 1);
            redlevelRight = TempByte[0];
            TempByte = ColorRightreader.read(0x07, 1);
            bluelevelRight = TempByte[0];
            //Setting Up Colors foe sensor Left
            TempByte = ColorLeftreader.read(0x05, 1);
            redlevelLeft = TempByte[0];
            TempByte = ColorLeftreader.read(0x07, 1);
            bluelevelLeft = TempByte[0];
            if (do_telemetry) {telemetry.update();}

           //too_far_away = redlevelLeft < red_good && redlevelRight < red_good && bluelevelLeft < blue_good && bluelevelRight < blue_good;
           if (touchSensor.isPressed()){
               too_far_away = false;
           }
        }// end of too_far_away

        found_white = false;  // Turn right until it gets back on the line
        while (!found_white && opModeIsActive()) {
            leftWheel.setPower(.1);
            rightWheel.setPower(-.1);
            light_reading = ODS.getLightDetected();
            found_white = light_reading >= perfect_value - fuzz_factor;
            if (do_telemetry) {telemetry.addData("Light reading", light_reading);}
            if (do_telemetry) {telemetry.update();}
        }

        leftWheel.setPower(.15);  // Now get a titch closer
        rightWheel.setPower(.15);
        sleep(50);

        leftWheel.setPower(0);
        rightWheel.setPower(0);

        while (!BluePressed && opModeIsActive()) {

            if (bluelevelRight >= blue_good) {
                btn_servo_position = init_btn_servo_position - btn_servo_degrees;
                btn_servo.setPosition(btn_servo_position);
            } else if (bluelevelLeft >= blue_good) {
                btn_servo_position = init_btn_servo_position + btn_servo_degrees;
                btn_servo.setPosition(btn_servo_position);
            }
            sleep(2000);
            btn_servo.setPosition(init_btn_servo_position);

            //Setting Up Colors For Sensor Right
            TempByte = ColorRightreader.read(0x05, 1);
            redlevelRight = TempByte[0];
            TempByte = ColorRightreader.read(0x07, 1);
            bluelevelRight = TempByte[0];
            //Setting Up Colors foe sensor Left
            TempByte = ColorLeftreader.read(0x05, 1);
            redlevelLeft = TempByte[0];
            TempByte = ColorLeftreader.read(0x07, 1);
            bluelevelLeft = TempByte[0];

            BluePressed = bluelevelLeft >= 8 && bluelevelRight >= 8;
            if (do_telemetry) {telemetry.addData("Blue Level Left", bluelevelLeft);}
            if (do_telemetry) {telemetry.addData("Blue Level Right", bluelevelRight);}
        }

        leftWheel.setPower(-1);
        rightWheel.setPower(-1);
        sleep(300);

        if (do_telemetry) {telemetry.addData("Red Right", redlevelRight);}
        if (do_telemetry) {telemetry.addData("Blue Right", bluelevelRight);}
        if (do_telemetry) {telemetry.addData("Red Left", redlevelLeft);}
        if (do_telemetry) {telemetry.addData("Blue Left", bluelevelLeft);}

        requestOpModeStop();
    }// end off RunOpMode
}//end class