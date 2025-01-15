package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep)
                .setDimensions(18, 16)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        robot.runAction(robot.getDrive().actionBuilder(new Pose2d(-13, 64, Math.toRadians(270)))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-2, 38))   // go back a little
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-36, 38))  // slide
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-36, 13))  // go forward
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-46, 12))  // slide and drag
                .strafeToConstantHeading(new Vector2d(-46, 56))  // ^
                .strafeToConstantHeading(new Vector2d(-46, 13))  // ^
                .strafeToConstantHeading(new Vector2d(-52, 12))  // slide and drag
                .strafeToConstantHeading(new Vector2d(-52, 56))  // ^
                .strafeToConstantHeading(new Vector2d(-52, 13))  // ^
                .strafeToConstantHeading(new Vector2d(-59, 12))  // slide and drag
                .strafeToConstantHeading(new Vector2d(-59, 56))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}
