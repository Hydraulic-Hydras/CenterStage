package com.hydraulichydras.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Left {

    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(74, 74, Math.toRadians(180), Math.toRadians(180), 16.5)
                .setDimensions(17.5, 17.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62.5, Math.toRadians(90)))

                                /* === LEFT === */
                                // Team prop on Left
                                .lineToLinearHeading(new Pose2d(-35.00, -32.00, Math.toRadians(180.00)))
                                .waitSeconds(1)

                                // white stack
                                .strafeRight(1)
                                .splineToConstantHeading(new Vector2d(-59.00, -11.60), Math.toRadians(180.00))
                                .waitSeconds(1)

                                // center door pos
                                .setReversed(true)
                                .lineTo(new Vector2d(20,-11.60))
                                .waitSeconds(2)

                                // backdrop
                                .splineToConstantHeading(new Vector2d(48,-29), Math.toRadians(0))
                                .waitSeconds(1)

                                // back to white stack cycle
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(20, -11.60), Math.toRadians(180))
                                .lineTo(new Vector2d(-59,-11.6))
                                .waitSeconds(1)

                                // repeat this for cycle and travel

                                .setReversed(true)
                                .lineTo(new Vector2d(20,-11.60))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(48,-29), Math.toRadians(0))
                                .waitSeconds(1)

                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(left)
                .start();
    }
}
