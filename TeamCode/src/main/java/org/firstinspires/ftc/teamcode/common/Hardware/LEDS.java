package org.firstinspires.ftc.teamcode.common.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.hydraulichydras.hydrauliclib.Util.Contraption;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Intake;
import org.firstinspires.ftc.teamcode.common.Hardware.Contraptions.Launcher;

@Config
public class LEDS extends Contraption {

    public DigitalChannel LED_GreenL;
    public DigitalChannel LED_RedL;
    public DigitalChannel LED_GreenR;
    public DigitalChannel LED_RedR;

    public DigitalChannel ALED_Green;
    public DigitalChannel ALED_Red;
    public DigitalChannel BLED_Green;
    public DigitalChannel BLED_Red;

    public static DistanceSensor distanceBackdrop;
    public static int count = 0;


    @Override
    public void initialize(HardwareMap hardwareMap) {
        LED_GreenL = hardwareMap.get(DigitalChannel.class, "LED_Green-L");
        LED_GreenL.setMode(DigitalChannel.Mode.OUTPUT);

        LED_RedL = hardwareMap.get(DigitalChannel.class, "LED_Red-L");
        LED_RedL.setMode(DigitalChannel.Mode.OUTPUT);

        LED_GreenR = hardwareMap.get(DigitalChannel.class, "LED_Green-R");
        LED_GreenR.setMode(DigitalChannel.Mode.OUTPUT);

        LED_RedR = hardwareMap.get(DigitalChannel.class, "LED_Red-R");
        LED_RedR.setMode(DigitalChannel.Mode.OUTPUT);

        ALED_Green = hardwareMap.get(DigitalChannel.class, "ALED_Green");
        ALED_Green.setMode(DigitalChannel.Mode.OUTPUT);

        ALED_Red = hardwareMap.get(DigitalChannel.class, "ALED_Red");
        ALED_Red.setMode(DigitalChannel.Mode.OUTPUT);

        BLED_Green = hardwareMap.get(DigitalChannel.class, "BLED_Green");
        BLED_Green.setMode(DigitalChannel.Mode.OUTPUT);

        BLED_Red = hardwareMap.get(DigitalChannel.class, "BLED_Red");
        BLED_Red.setMode(DigitalChannel.Mode.OUTPUT);

        distanceBackdrop = hardwareMap.get(DistanceSensor.class, "distanceBackdrop");

    }

    @Override
    public void loop(Gamepad gamepad) {

        if (Globals.IS_AUTO) {
            if (Globals.LOCATION == 3) {
                ALED_Green.setState(true);
                ALED_Red.setState(true);
                BLED_Green.setState(true);
                BLED_Red.setState(true);
                LED_GreenL.setState(true);
                LED_RedL.setState(true);
                LED_GreenR.setState(false);
                LED_RedR.setState(false);
            } else if (Globals.LOCATION == 2) {
                ALED_Green.setState(false);
                ALED_Red.setState(false);
                BLED_Green.setState(false);
                BLED_Red.setState(false);
                LED_GreenL.setState(true);
                LED_RedL.setState(true);
                LED_GreenR.setState(true);
                LED_RedR.setState(true);
            } else if (Globals.LOCATION == 1) {
                ALED_Green.setState(true);
                ALED_Red.setState(true);
                BLED_Green.setState(true);
                BLED_Red.setState(true);
                LED_GreenL.setState(false);
                LED_RedL.setState(false);
                LED_GreenR.setState(true);
                LED_RedR.setState(true);
            }
        } else {
            if (Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) >= 16
                    && Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) <= 22) {
                LED_RedL.setState(false);
                LED_GreenL.setState(true);
                LED_RedR.setState(false);
                LED_GreenR.setState(true);
            } else {
                LED_GreenL.setState(false);
                LED_RedL.setState(true);
                LED_GreenR.setState(false);
                LED_RedR.setState(true);
            }

            if (Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) >= 16 && Double.parseDouble(
                    JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) <= 22
                    && Intake.rotateBucket.getPosition() != 0.2) {
                ALED_Green.setState(true);
                ALED_Red.setState(false);
                BLED_Green.setState(true);
                BLED_Red.setState(false);
            } else if (Double.parseDouble(JavaUtil.formatNumber(distanceBackdrop.getDistance(DistanceUnit.CM), 0)) <= 40) {
                ALED_Green.setState(false);
                ALED_Red.setState(true);
                BLED_Green.setState(false);
                BLED_Red.setState(true);
            } else if (Launcher.launcher_angle.getPosition() < 0.4) {
                if (Launcher.droneTrigger.getPosition() > 0.5) {
                    ALED_Green.setState(true);
                    ALED_Red.setState(false);
                    BLED_Green.setState(true);
                    BLED_Red.setState(false);
                } else if (count == 1) {
                    ALED_Green.setState(false);
                    ALED_Red.setState(true);
                    BLED_Green.setState(false);
                    BLED_Red.setState(true);
                    count = 0;
                } else {
                    ALED_Green.setState(true);
                    ALED_Red.setState(true);
                    BLED_Green.setState(true);
                    BLED_Red.setState(true);
                    count = 1;
                }
            } else {
                ALED_Green.setState(true);
                ALED_Red.setState(true);
                BLED_Green.setState(true);
                BLED_Red.setState(true);
            }

        }

    }
}
