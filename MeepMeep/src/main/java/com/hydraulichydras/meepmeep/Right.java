package com.hydraulichydras.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Right {

    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(74, 74, Math.toRadians(180), Math.toRadians(180), 16.68)
                .setDimensions(17.5,17.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))

                                /* === RIGHT === */

                                // Right Prop
                                //.splineToConstantHeading(new Vector2d(13, 22), Math.toRadians(90))
                               // .lineToLinearHeading(new Pose2d(50, -40, Math.toRadians(180)))
                               // .strafeLeft(17)
                               // .back(10)


                                // Center Prop
                               // .lineTo(new Vector2d(12, -34.5))
                               // .lineToLinearHeading(new Pose2d(50, -34.5, Math.toRadians(180)))
                               // .strafeLeft(25)
                               // .back(10)

                                // Left Prop
                                //.lineToLinearHeading(new Pose2d(12, -33.5, Math.toRadians(180)))
                                //.lineTo(new Vector2d(50, -33.5))
                                //.strafeLeft(27)
                                // .back(10)


                                .splineToConstantHeading(new Vector2d(47, -17), Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(47, -30, Math.toRadians(-270)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueRight)
                .start();

    }

}