package org.firstinspires.ftc.teamcode.Testers;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.hydraulichydras.hydrauliclib.Geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CenterStage.Side;
import org.firstinspires.ftc.teamcode.common.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.Hardware.Vision.TensorFlow;


@TeleOp
public class VisionTest extends CommandOpMode {

    public TensorFlow tensorFlow;
    public final RobotHardware robot = RobotHardware.getInstance();
    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap, telemetry);

        robot.read();
        while (opModeInInit()) {
            tensorFlow.initialize(hardwareMap);
            tensorFlow.telemetryOldProp(telemetry);

            telemetry.update();
        }

        Pose test = new Pose();

        Side side = tensorFlow.getLocation();

        switch (side) {
            case LEFT:
                test = new Pose(1, 1, 1);
                telemetry.addData("Pose: ", test);
                telemetry.addLine("Side: " + "LEFT");
                telemetry.update();
                break;
            case CENTER:
                test = new Pose(2, 2, 2);
                telemetry.addData("Pose: ", test);
                telemetry.addLine("Side: " + "CENTER");
                telemetry.update();
                break;
            case RIGHT:
                test = new Pose(3, 3, 3);
                telemetry.addData("Pose: ", test);
                telemetry.addLine("Side: " + "RIGHT");
                telemetry.update();
                break;
            default:
                // blank for now
                break;
        }
    }


    @Override
    public void run() {
        robot.read();

        // input
        super.run();
        robot.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
        robot.write();
        robot.clearBulkCache();
    }

}
