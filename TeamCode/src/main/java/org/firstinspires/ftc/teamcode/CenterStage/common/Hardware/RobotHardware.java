package org.firstinspires.ftc.teamcode.CenterStage.common.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Config
public class RobotHardware {

    // drivetrain
    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightFront;
    public DcMotorEx rightRear;

    // climber
    public DcMotorEx leftClimb;
    public DcMotorEx rightClimb;

    // extensions
    public DcMotorEx leftCont;
    public DcMotorEx rightCont;

    // intake and outtake
    public CRServo rollers;
    public CRServo boxy;
    /**
     * Voltage timer and voltage value.
     */
    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 0.0;

    // vision
    public TensorFlow camera;


    /**
     * HardwareMap storage.
     */
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    /**
     * Singleton variables.
     */
    private static RobotHardware instance = null;
    public boolean enabled;

    private IMU imu;
    private double imuAngle;
    public List<LynxModule> modules;

    /**
     * Creating the singleton the first time, instantiating.
     */
    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    /**
     * Created at the start of every OpMode.
     *
     * @param hardwareMap The HardwareMap of the robot, storing all hardware devices
     * @param telemetry Saved for later in the event FTC Dashboard used
     */

}
