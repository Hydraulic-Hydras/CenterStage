package com.hydraulichydras.hydrauliclib.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Contraption is an abstract class for all contraptions on a robot. It contains methods and/or instance
 * variables common to all contraptions.
 *
 *  All robot contraption, including the main hardware map, should extend this abstract class.
 */
public abstract class Contraption {

    /**
     * OpMode context for a Mechanism class.
     */
    protected LinearOpMode opMode;

    /**
     * Initializes hardware on the robot. Gets and stores references to the robot configuration and
     * sets motors and servos to their starting positions.
     *
     * @param hwMap robot's hardware map
     */
    public abstract void init(HardwareMap hwMap);

    /**
     * Manages gamepad inputs and their corresponding mechanism response
     *
     * Implement when using only one gamepad, in slot 1
     * @param gamepad1
     */
    public void loop(Gamepad gamepad1) {}

    /**
     * Manages multiple gamepad inputs and their corresponding mechanism responses
     *
     * Implement when using two gamepads
     * @param gamepad1
     * @param gamepad2
     */
    public void loop(Gamepad gamepad1, Gamepad gamepad2) {}

    /**
     * Manages all telemetry data to driver phone or FTC Dashboard
     *
     * Where all telemetry.addData() calls should go
     * @param telemetry
     */
    public void telemetry(Telemetry telemetry) { }
}

