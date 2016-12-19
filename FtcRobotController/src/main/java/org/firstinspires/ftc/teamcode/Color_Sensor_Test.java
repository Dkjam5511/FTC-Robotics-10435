package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Drew on 11/26/2016.
 *
 */
@TeleOp(name="ColorSensor Test", group="Tests and Calibration")
public class Color_Sensor_Test extends OpMode{

    DcMotor leftWheel;
    DcMotor rightWheel;
    I2cDevice ColorRight;
    I2cDeviceSynch ColorRightreader;  // right beacon sensor
    I2cDevice ColorLeft;
    I2cDeviceSynch ColorLeftreader;   // left beacon sensor
    DeviceInterfaceModule CDI;
    int bluelevelRight;
    int redlevelRight;
    int whitelevelRight;
    int bluelevelLeft;
    int redlevelLeft;
    int whitelevelLeft;
    int Passive = 1;
    double leftWheelPower = 0;
    double rightWheelPower = 0;
    byte[] TempByte;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Defining All our Machines in the Hardware Map
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        CDI.setLED(0, false);           //Blue light OFF
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
        
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Setting Up Colors For Sensor Right
        TempByte = ColorRightreader.read(0x05, 1);
        redlevelRight = TempByte[0];
        TempByte = ColorRightreader.read(0x07, 1);
        bluelevelRight = TempByte[0];
        TempByte = ColorRightreader.read(0x08, 1);
        whitelevelRight = TempByte[0];
        //Setting Up Colors foe sensor Left
        TempByte = ColorLeftreader.read(0x05, 1);
        redlevelLeft = TempByte[0];
        TempByte = ColorLeftreader.read(0x07, 1);
        bluelevelLeft = TempByte[0];
        TempByte = ColorLeftreader.read(0x08, 1);
        whitelevelLeft = TempByte[0];

        //Changing between Passive and Active light
        if (gamepad1.y){
            Passive = 1;
        }
        if (gamepad1.x){
            Passive = 0;}
        
        ColorRightreader.write8(3, Passive);
        ColorLeftreader.write8(3, Passive);

        // Blue light means passive, Red light means active
        //Using the CDI to show Passive and Active
        CDI.setLED(0, (Passive == 1));          //Blue light
        CDI.setLED(1, (Passive == 0));          //Red Light

        //Adding Screen Data
        telemetry.addData("Red Right", redlevelRight);
        telemetry.addData("Blue Right", bluelevelRight);
        telemetry.addData("White Right", whitelevelRight);
        telemetry.addData("Red Left", redlevelLeft);
        telemetry.addData("Blue Left", bluelevelLeft);
        telemetry.addData("White Left", whitelevelLeft);
        telemetry.addData("Passive", Passive);

        //Using wheel power to test as feedback for the color sensors
        /*
        if (redlevelRight > 2 && redlevelRight > bluelevelRight){
            rightWheelPower = 1;
            leftWheelPower = 0;
        }else if (bluelevelRight > 2 && bluelevelRight > redlevelRight){
            leftWheelPower = 1;
            rightWheelPower = 0;
        }else {
            leftWheelPower = 0;
            rightWheelPower = 0;
        }

        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);
        */



    }
}
