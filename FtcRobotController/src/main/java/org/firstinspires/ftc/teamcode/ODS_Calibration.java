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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "ODS_Calibration", group = "Tests and Calibration")

public class ODS_Calibration extends LinearOpMode {

    //Instance of OpticalDistanceSensor
    OpticalDistanceSensor ODS;

    //Motors
    DcMotor leftWheel;
    DcMotor rightWheel;

    double white_level;
    double correction;
    double correction_sensitivity = 1;
    double light_reading;
    double perfect_value = .34;
    double wheelpower_base = .1;
    double fuzz_factor = .03;
    boolean found_white = false;
    boolean turn_right = false;
    double rightwheelpower;
    double leftwheelpower;

    @Override
    public void runOpMode() throws InterruptedException {

        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");

        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            while (!found_white) {
                //leftWheel.setPower(wheelpower_base);
                //rightWheel.setPower(wheelpower_base);
                light_reading = ODS.getLightDetected();
                found_white = light_reading >= perfect_value - fuzz_factor;
                if (found_white) {
                    leftWheel.setPower(0);
                    rightWheel.setPower(0);
                    sleep(2000);
                }
                telemetry.addData("Light reading", light_reading);
                telemetry.addData("Found White", found_white);
                telemetry.update();
            }

            light_reading = ODS.getLightDetected();

            white_level = light_reading - perfect_value;

            turn_right = false;

            if (white_level > 0) {
                correction = white_level * 14 / 8;  // because max white is .08 higher than perfect_value and min white is .14 lower
            } else {
                correction = -white_level;
            }
            /* Now correction should be between 0 and .14 */

            telemetry.addData("Correction1", correction);
            telemetry.addData("Light reading", light_reading);

            correction = correction_sensitivity * correction;
            if (correction < .08) {
                correction = .08;
            }
            telemetry.addData("Correction2", correction);
            /* Now correction should be between 0 and .56 */

            if (white_level < -fuzz_factor) {                    // turn left
                //leftwheelpower = -correction;
                //rightwheelpower = correction;
                telemetry.addData("turning left lw", leftwheelpower);
                telemetry.addData("turning left rw", rightwheelpower);
            } else if (white_level > fuzz_factor){               // turn right
               //leftwheelpower = correction;
                //rightwheelpower = -correction;
                telemetry.addData("turning right lw", leftwheelpower);
                telemetry.addData("turning right rw", rightwheelpower);
            } else {                                            // go straight
                //leftwheelpower = wheelpower_base;
                //rightwheelpower = wheelpower_base;
                telemetry.addData("going straight", leftwheelpower);
            }

            //leftWheel.setPower(leftwheelpower);
            //rightWheel.setPower(rightwheelpower);
            telemetry.addData("Found White", found_white);


            telemetry.update();
        }
    }
}//end class