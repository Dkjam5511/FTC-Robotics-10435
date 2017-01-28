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

import android.test.InstrumentationTestRunner;

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

@Autonomous(name = "Beacon_Autonomous_Red", group = "Beacon")

public class Beacon_Autonomous_Red extends LinearOpMode {

    //Hardware
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

    // line follow variables
    double white_level;
    double inside_correction = -.08;
    double outside_correction = .16;
    double straight_speed = .16;
    double light_reading;
    double perfect_value = .26;
    double fuzz_factor = .05;
    double wheelpower_base = .15;
    double wheelpower_base_left = 0;
    double turnspeed =.25;

    // color sensor and button pushing variables
    int Passive = 1;
    int bluelevelRight;
    int redlevelRight;
    int bluelevelLeft;
    int redlevelLeft;
    int ShootTarget = 2880;
    int StartPositionS;
    boolean found_white = false;
    double rightwheelpower;
    double leftwheelpower;
    byte[] TempByte;
    double red_good = 4;
    boolean too_far_away = true;
    boolean RedPressed = false;
    double init_btn_servo_position = .45;
    double btn_servo_position;
    double btn_servo_degrees = .4;

    // telemetry
    boolean do_telemetry = false;

    private void Shoot() throws InterruptedException {
        StartPositionS = ShootMotor.getCurrentPosition();
        ShootMotor.setPower(1);
        ShootMotor.setTargetPosition(ShootTarget + StartPositionS);
        sleep(500);
        ShootMotor.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        btn_servo = hardwareMap.servo.get("button_servo");
        touchSensor = hardwareMap.touchSensor.get("TouchSensor");
        ball_gate_servo = hardwareMap.servo.get("ball_gate");
        ShootMotor = hardwareMap.dcMotor.get("shoot_motor");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        CDI.setLED(0, false);           //Blue light Off
        CDI.setLED(1, true);           //Red light On

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

        ShootMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        ball_gate_servo.setPosition(1);

        waitForStart();
        /*
        // Go forward fast for a bit
        leftWheel.setPower(1);
        rightWheel.setPower(1);
        sleep(400);
        */

        // Go until we see the white line, cross it and exit the loop
        found_white = false;
        while (!found_white && opModeIsActive()) {
            if (!do_telemetry) {leftWheel.setPower(wheelpower_base + wheelpower_base_left);}  // The .01 is because our left motors is weak
            if (!do_telemetry) {rightWheel.setPower(wheelpower_base);}
            light_reading = ODS.getLightDetected();
            found_white = light_reading >= perfect_value - fuzz_factor;
            if (found_white) {
                if (!do_telemetry) {leftWheel.setPower(wheelpower_base);}
                if (!do_telemetry) {rightWheel.setPower(wheelpower_base);}
                sleep(360);  // Go over the line until back wheels over the line
                leftWheel.setPower(0);
                rightWheel.setPower(0);
                sleep(200);
            }
            if (do_telemetry) {telemetry.addData("Light reading", light_reading);}
            if (do_telemetry) {telemetry.update();}
        }

        found_white = false;  // Turn left until it gets back on the line
        if (!do_telemetry) {rightWheel.setPower(turnspeed);}
        if (!do_telemetry) {leftWheel.setPower(-turnspeed);}
        while (!found_white && opModeIsActive()) {
            light_reading = ODS.getLightDetected();
            found_white = light_reading >= perfect_value - fuzz_factor;
            if (do_telemetry) {telemetry.addData("Light reading tl", light_reading);}
            if (do_telemetry) {telemetry.update();}
        }

        leftWheel.setPower(0);
        rightWheel.setPower(0);

        // Line follow code
        too_far_away = true;
        while (too_far_away && opModeIsActive()) {

            light_reading = ODS.getLightDetected();
            if (do_telemetry) {telemetry.addData("Light reading", light_reading);}
            white_level = light_reading - perfect_value;

            if (white_level < -fuzz_factor) {                    // Gray - turn left
                leftwheelpower = inside_correction + wheelpower_base_left;
                rightwheelpower = outside_correction;
                if (do_telemetry) {telemetry.addData("turning left lw", leftwheelpower);}
                if (do_telemetry) {telemetry.addData("turning left rw", rightwheelpower);}
            } else if (white_level > fuzz_factor){               // White - turn right
                rightwheelpower = inside_correction;
                leftwheelpower = outside_correction + wheelpower_base_left;
                if (do_telemetry) {telemetry.addData("turning right lw", leftwheelpower);}
                if (do_telemetry) {telemetry.addData("turning right rw", rightwheelpower);}
            } else {                                            // go straight
                leftwheelpower = straight_speed;
                rightwheelpower = straight_speed;
                if (do_telemetry) {telemetry.addData("going straight", leftwheelpower);}
            }

            if (!do_telemetry) {leftWheel.setPower(leftwheelpower);}
            if (!do_telemetry) {rightWheel.setPower(rightwheelpower);}

            if (do_telemetry) {telemetry.update();}

            // Old code was trying to use red and blue levels to see if we're close enough.  Now we're using touch sensor
            // too_far_away = redlevelLeft < red_good && redlevelRight < red_good && bluelevelLeft < blue_good && bluelevelRight < blue_good;
            if (touchSensor.isPressed()){
                too_far_away = false;
            }

        } // end of too_far_away

        // Stop
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        sleep(50); // give it time to stop

        // Now back up a tiny bit
        leftWheel.setPower(-.1);
        rightWheel.setPower(-.1);
        sleep(50);

        // Stop
        leftWheel.setPower(0);
        rightWheel.setPower(0);

        // Just in case we're not on the line, turn left until it gets back on the line
        /*
        found_white = false;
        while (!found_white && opModeIsActive()) {
            light_reading = ODS.getLightDetected();
            found_white = light_reading >= perfect_value - fuzz_factor;
            if (!do_telemetry) {rightWheel.setPower(.1);}
            if (!do_telemetry) {leftWheel.setPower(-.1);}
            if (do_telemetry) {telemetry.addData("Light reading lta", light_reading);}
            if (do_telemetry) {telemetry.update();}
        }
        */

        // Do the first color reads
        TempByte = ColorRightreader.read(0x05, 1);
        redlevelRight = TempByte[0];
        TempByte = ColorRightreader.read(0x07, 1);
        bluelevelRight = TempByte[0];
        TempByte = ColorLeftreader.read(0x05, 1);
        redlevelLeft = TempByte[0];
        TempByte = ColorLeftreader.read(0x07, 1);
        bluelevelLeft = TempByte[0];

        // Button pressing section
        RedPressed = false;
        while (!RedPressed && opModeIsActive()) {

            if (redlevelRight > redlevelLeft && redlevelRight >= red_good) {
                btn_servo_position = init_btn_servo_position - btn_servo_degrees;
                btn_servo.setPosition(btn_servo_position);
            } else if (redlevelLeft > redlevelRight && redlevelLeft >= red_good) {
                btn_servo_position = init_btn_servo_position + btn_servo_degrees;
                btn_servo.setPosition(btn_servo_position);
            }
            sleep(1000);
            btn_servo.setPosition(init_btn_servo_position);

            //Re-Read color sensosrs
            TempByte = ColorRightreader.read(0x05, 1);
            redlevelRight = TempByte[0];
            TempByte = ColorRightreader.read(0x07, 1);
            bluelevelRight = TempByte[0];
            TempByte = ColorLeftreader.read(0x05, 1);
            redlevelLeft = TempByte[0];
            TempByte = ColorLeftreader.read(0x07, 1);
            bluelevelLeft = TempByte[0];

            RedPressed = redlevelLeft >= red_good && redlevelRight >= red_good;
            if (do_telemetry) {telemetry.addData("Red Right", redlevelRight);}
            if (do_telemetry) {telemetry.addData("Blue Right", bluelevelRight);}
            if (do_telemetry) {telemetry.addData("Red Left", redlevelLeft);}
            if (do_telemetry) {telemetry.addData("Blue Left", bluelevelLeft);}
            if (do_telemetry) {telemetry.update();}
        }

        // Back up a bit
        if (!do_telemetry) {leftWheel.setPower(-1);}
        if (!do_telemetry) {rightWheel.setPower(-1);}
        sleep(300);

        // Stop
        leftWheel.setPower(0);
        rightWheel.setPower(0);

        Shoot();

        ball_gate_servo.setPosition(0);

        sleep(2000);

        Shoot();

        sleep(300);

        ball_gate_servo.setPosition(1);


        // Turn right 90 degrees
        if (!do_telemetry) {leftWheel.setPower(.2 + wheelpower_base_left);}
        if (!do_telemetry) {rightWheel.setPower(-.2);}
        sleep(2090);

        /*
        // Go forward fast for a bit
        leftWheel.setPower(1);
        rightWheel.setPower(1);
        sleep(400);
        */

        // Go until we see the white line, cross it and exit the loop
        found_white = false;
        while (!found_white && opModeIsActive()) {
            if (!do_telemetry) {leftWheel.setPower(wheelpower_base + wheelpower_base_left);}
            if (!do_telemetry) {rightWheel.setPower(wheelpower_base);}
            light_reading = ODS.getLightDetected();
            found_white = light_reading >= perfect_value - fuzz_factor;
            if (found_white) {
                if (!do_telemetry) {leftWheel.setPower(wheelpower_base + wheelpower_base_left);}
                if (!do_telemetry) {rightWheel.setPower(wheelpower_base);}
                sleep(270);  // Go over the line until back wheels over the line
                leftWheel.setPower(0);
                rightWheel.setPower(0);
                sleep(200);
            }
            if (do_telemetry) {telemetry.addData("Light reading", light_reading);}
            if (do_telemetry) {telemetry.update();}
        }

        found_white = false;  // Turn left until it gets back on the line
        if (!do_telemetry) {rightWheel.setPower(turnspeed + .1);}
        if (!do_telemetry) {leftWheel.setPower(-turnspeed - .1);}
        while (!found_white && opModeIsActive()) {
            light_reading = ODS.getLightDetected();
            found_white = light_reading >= perfect_value - fuzz_factor;
            if (do_telemetry) {telemetry.addData("Light reading tl", light_reading);}
            if (do_telemetry) {telemetry.update();}
        }

        leftWheel.setPower(0);
        rightWheel.setPower(0);

        telemetry.addData("Found white", light_reading);
        telemetry.update();

        // Line follow code
        too_far_away = true;
        while (too_far_away && opModeIsActive()) {

            light_reading = ODS.getLightDetected();
            if (do_telemetry) {telemetry.addData("Light reading", light_reading);}
            white_level = light_reading - perfect_value;

            if (white_level < -fuzz_factor) {                    // Gray - turn left
                leftwheelpower = inside_correction + wheelpower_base_left;
                rightwheelpower = outside_correction;
                if (do_telemetry) {telemetry.addData("turning left lw", leftwheelpower);}
                if (do_telemetry) {telemetry.addData("turning left rw", rightwheelpower);}
            } else if (white_level > fuzz_factor){               // White - turn right
                rightwheelpower = inside_correction;
                leftwheelpower = outside_correction + wheelpower_base_left;
                if (do_telemetry) {telemetry.addData("turning right lw", leftwheelpower);}
                if (do_telemetry) {telemetry.addData("turning right rw", rightwheelpower);}
            } else {                                            // go straight
                leftwheelpower = straight_speed;
                rightwheelpower = straight_speed;
                if (do_telemetry) {telemetry.addData("going straight", leftwheelpower);}
            }

            if (!do_telemetry) {leftWheel.setPower(leftwheelpower);}
            if (!do_telemetry) {rightWheel.setPower(rightwheelpower);}

            if (do_telemetry) {telemetry.update();}

            // Old code was trying to use red and blue levels to see if we're close enough.  Now we're using touch sensor
            // too_far_away = redlevelLeft < red_good && redlevelRight < red_good && bluelevelLeft < blue_good && bluelevelRight < blue_good;
            if (touchSensor.isPressed()){
                too_far_away = false;
            }

        } // end of too_far_away

        // Stop
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        sleep(50); // give it time to stop

        // Now back up a tiny bit
        leftWheel.setPower(-.1);
        rightWheel.setPower(-.1);
        sleep(50);

        // Stop
        leftWheel.setPower(0);
        rightWheel.setPower(0);

        // Just in case we're not on the line, turn left until it gets back on the line
        /*
        found_white = false;
        while (!found_white && opModeIsActive()) {
            light_reading = ODS.getLightDetected();
            found_white = light_reading >= perfect_value - fuzz_factor;
            if (!do_telemetry) {rightWheel.setPower(.1);}
            if (!do_telemetry) {leftWheel.setPower(-.1);}
            if (do_telemetry) {telemetry.addData("Light reading lta", light_reading);}
            if (do_telemetry) {telemetry.update();}
        }
        */

        // Do the first color reads
        TempByte = ColorRightreader.read(0x05, 1);
        redlevelRight = TempByte[0];
        TempByte = ColorRightreader.read(0x07, 1);
        bluelevelRight = TempByte[0];
        TempByte = ColorLeftreader.read(0x05, 1);
        redlevelLeft = TempByte[0];
        TempByte = ColorLeftreader.read(0x07, 1);
        bluelevelLeft = TempByte[0];

        // Button pressing section
        RedPressed = false;
        while (!RedPressed && opModeIsActive()) {

            if (redlevelRight > redlevelLeft && redlevelRight >= red_good) {
                btn_servo_position = init_btn_servo_position - btn_servo_degrees;
                btn_servo.setPosition(btn_servo_position);
            } else if (redlevelLeft > redlevelRight && redlevelLeft >= red_good) {
                btn_servo_position = init_btn_servo_position + btn_servo_degrees;
                btn_servo.setPosition(btn_servo_position);
            }
            sleep(1000);
            btn_servo.setPosition(init_btn_servo_position);

            //Re-Read color sensosrs
            TempByte = ColorRightreader.read(0x05, 1);
            redlevelRight = TempByte[0];
            TempByte = ColorRightreader.read(0x07, 1);
            bluelevelRight = TempByte[0];
            TempByte = ColorLeftreader.read(0x05, 1);
            redlevelLeft = TempByte[0];
            TempByte = ColorLeftreader.read(0x07, 1);
            bluelevelLeft = TempByte[0];

            RedPressed = redlevelLeft >= red_good && redlevelRight >= red_good;
            if (do_telemetry) {telemetry.addData("Red Right", redlevelRight);}
            if (do_telemetry) {telemetry.addData("Blue Right", bluelevelRight);}
            if (do_telemetry) {telemetry.addData("Red Left", redlevelLeft);}
            if (do_telemetry) {telemetry.addData("Blue Left", bluelevelLeft);}
            if (do_telemetry) {telemetry.update();}
        }

        // Back up a bit
        if (!do_telemetry) {leftWheel.setPower(-1);}
        if (!do_telemetry) {rightWheel.setPower(-1);}
        sleep(300);

        // Stop
        leftWheel.setPower(0);
        rightWheel.setPower(0);

        requestOpModeStop();

    }// end off RunOpMode
}//end class