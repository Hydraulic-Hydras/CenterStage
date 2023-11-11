package com.hydraulichydras.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeft {

    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(74, 74, Math.toRadians(180), Math.toRadians(180), 16.5)
                .setDimensions(17.5, 17.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 62.5, Math.toRadians(270)))

                                .lineToLinearHeading(new Pose2d(-35, 32, Math.toRadians(180)))
                                .waitSeconds(1)

                                .strafeLeft(1)
                                .splineToConstantHeading(new Vector2d(-59,11.50), Math.toRadians(180))
                                .waitSeconds(1)

                                .setReversed(true)
                                .lineTo(new Vector2d(29,11.50))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(left)
                .start();
    }
}
