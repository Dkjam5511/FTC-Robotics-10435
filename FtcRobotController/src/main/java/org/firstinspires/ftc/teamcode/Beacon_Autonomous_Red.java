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

@Autonomous(name = "Beacon_Autonomous_Red", group = "Beacon")

public class Beacon_Autonomous_Red extends LinearOpMode {

    //Instance of OpticalDistanceSensor
    OpticalDistanceSensor ODS;

    //Motors
    DcMotor leftWheel;
    DcMotor rightWheel;
    I2cDevice ColorRight;
    I2cDeviceSynch ColorRightreader;  // right beacon sensor
    I2cDevice ColorLeft;
    I2cDeviceSynch ColorLeftreader;   // left beacon sensor
    DeviceInterfaceModule CDI;
    Servo btn_servo;
    double white_level;
    double inside_correction = .05;
    double outside_correction = .10;
    double straight_speed = .13;
    double light_reading;
    double perfect_value = .28;
    double wheelpower_base = .5;
    double fuzz_factor = .05;
    double init_btn_servo_position = .63;
    double btn_servo_position;
    double btn_servo_degrees = .15;

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
    double red_good = 9;
    double blue_good = 11;
    boolean too_far_away = true;

    @Override
    public void runOpMode() throws InterruptedException {

        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        btn_servo = hardwareMap.servo.get("button_servo");
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

        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            /*
            leftWheel.setPower(1);
            rightWheel.setPower(1);
            sleep(400);
            */

            while (too_far_away) {

                while (!found_white) {
                    leftWheel.setPower(wheelpower_base);
                    rightWheel.setPower(wheelpower_base);
                    light_reading = ODS.getLightDetected();
                    found_white = light_reading >= perfect_value - fuzz_factor;
                    if (found_white) {
                        leftWheel.setPower(0);
                        rightWheel.setPower(0);
                        sleep(200);
                    }
                    telemetry.addData("Light reading", light_reading);
                    telemetry.update();
                }

                light_reading = ODS.getLightDetected();

                white_level = light_reading - perfect_value;

                telemetry.addData("Light reading", light_reading);

            /* Now correction should be between 0 and .56 */

                if (white_level < -fuzz_factor) {                    // turn left
                    leftwheelpower = -outside_correction;
                    rightwheelpower = inside_correction;
                    telemetry.addData("turning left lw", leftwheelpower);
                    telemetry.addData("turning left rw", rightwheelpower);
                } else if (white_level > fuzz_factor){               // turn right
                    leftwheelpower = inside_correction;
                    rightwheelpower = -outside_correction ;
                    telemetry.addData("turning right lw", leftwheelpower);
                    telemetry.addData("turning right rw", rightwheelpower);
                } else {                                            // go straight
                    leftwheelpower = straight_speed;
                    rightwheelpower = straight_speed;
                    telemetry.addData("going straight", leftwheelpower);
                }

                leftWheel.setPower(leftwheelpower);
                rightWheel.setPower(rightwheelpower);

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
                telemetry.update();

                too_far_away = redlevelLeft < red_good && redlevelRight < red_good && bluelevelLeft < blue_good && bluelevelRight < blue_good;

            } // end of too_far_away
            leftWheel.setPower(0);
            rightWheel.setPower(0);

            if (redlevelRight >= red_good){
                btn_servo_position = init_btn_servo_position - btn_servo_degrees;
                btn_servo.setPosition(btn_servo_position);
            }else if (redlevelLeft >= red_good){
                btn_servo_position = init_btn_servo_position + btn_servo_degrees;
                btn_servo.setPosition(btn_servo_position);
            }
            sleep(2000);
            btn_servo.setPosition(init_btn_servo_position);

            leftWheel.setPower(-1);
            rightWheel.setPower(-1);

            telemetry.addData("Red Right", redlevelRight);
            telemetry.addData("Blue Right", bluelevelRight);
            telemetry.addData("Red Left", redlevelLeft);
            telemetry.addData("Blue Left", bluelevelLeft);

            requestOpModeStop();
        } // end of opModeisActive
    }// end off RunOpMode
}//end class