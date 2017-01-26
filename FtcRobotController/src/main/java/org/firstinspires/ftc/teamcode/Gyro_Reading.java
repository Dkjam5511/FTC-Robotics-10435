package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Drew on 1/24/2017.
 */
@TeleOp(name = "Gyro Reading", group = "Tests and Calibration")
public class Gyro_Reading extends OpMode {

    ModernRoboticsI2cGyro gyro;

    @Override
    public void init() {

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (gyro.isCalibrating()){
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

    }

    @Override
    public void loop() {

        telemetry.addData("IntergradedZ", gyro.getIntegratedZValue());
        telemetry.addData("RawZ", gyro.rawZ());
        telemetry.addData("Heading", gyro.getHeading());

    }
}
