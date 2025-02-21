package org.firstinspires.ftc.teamcode._autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode._autonomous.AutoComponents.*;

import java.util.Arrays;


@Config
@Autonomous(name = "SpecimenAutonomousRR", group = "Autonomous")
public class SpecimenAutonomousRR extends LinearOpMode {

    @Override
    public void runOpMode(){
        // initial pose on map (specimen)
        Pose2d initialPose = new Pose2d(-13, 64, Math.toRadians(270));

        // initialize movement and attachments
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);

        // velocity and acceleration constraints for fast movements and accurate movements
        VelConstraint accurateVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(40),
                new AngularVelConstraint(Math.PI / 3)
        ));
        AccelConstraint accurateAccel = new ProfileAccelConstraint(-15, 15);

        VelConstraint fastVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(80),
                new AngularVelConstraint(Math.PI)
        ));
        AccelConstraint fastAccel = new ProfileAccelConstraint(-45, 45);
        AccelConstraint superAccel = new ProfileAccelConstraint(-60, 60);

        // go to rung where the robot currently holds a specimen
        TrajectoryActionBuilder hangSamplePos1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-7, 31));

        // routine to drag samples from default space to the observation zone
        // then position itself to get the first specimen
        TrajectoryActionBuilder dragSamples = hangSamplePos1.endTrajectory().fresh()
//                .strafeToConstantHeading(new Vector2d(-2, 38), fastVel, fastAccel)   // go back a little
                .strafeToConstantHeading(new Vector2d(-36, 42), fastVel, superAccel)  // slide
                .strafeToConstantHeading(new Vector2d(-36, 13), fastVel, superAccel)  // go forward
                .strafeToConstantHeading(new Vector2d(-46, 12), fastVel, superAccel)  // slide and drag
                .strafeToConstantHeading(new Vector2d(-46, 56), fastVel, superAccel)  // ^
//                .strafeToConstantHeading(new Vector2d(-46, 13), fastVel, fastAccel)  // ^
//                .strafeToConstantHeading(new Vector2d(-52, 12), fastVel, fastAccel)  // slide and drag
//                .strafeToConstantHeading(new Vector2d(-52, 56), fastVel, fastAccel)  // ^
//                .strafeToConstantHeading(new Vector2d(-52, 13), fastVel, fastAccel)  // ^
//                .strafeToConstantHeading(new Vector2d(-61, 12), fastVel, fastAccel)  // slide and drag
//                .strafeToConstantHeading(new Vector2d(-61, 56), fastVel, fastAccel)  // ^
//                .strafeToConstantHeading(new Vector2d(-61, 48), fastVel, fastAccel)  // move out of way and position to get specimen
                .strafeToConstantHeading(new Vector2d(-52, 45), fastVel, superAccel)  // move out of way and position to get specimen
                .strafeToSplineHeading(new Vector2d(-49, 47.5), Math.toRadians(90));

        // gets the specimen and moves to rung
        TrajectoryActionBuilder hangSample1 = dragSamples.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-4, 40), Math.toRadians(270), fastVel, superAccel)
                .strafeToConstantHeading(new Vector2d(-4, 30), accurateVel);


        // moves to observation zone and goes to hang location then parks in the
        // observation zone
        TrajectoryActionBuilder collectSpecimen2 = hangSample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-49, 49), Math.toRadians(90), fastVel, fastAccel);

        TrajectoryActionBuilder hangSpecimen2 = collectSpecimen2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-1, 40), Math.toRadians(270), fastVel, superAccel)
                .strafeToConstantHeading(new Vector2d(-1, 31), accurateVel);

        TrajectoryActionBuilder collectSpecimen3 = hangSpecimen2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-50, 45), Math.toRadians(90), accurateVel);

        TrajectoryActionBuilder hangSpecimen3 = collectSpecimen3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(2, 40), Math.toRadians(270), fastVel, superAccel)
                .strafeToConstantHeading(new Vector2d(2, 31), accurateVel);

        TrajectoryActionBuilder park = hangSpecimen3.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-50, 60), fastVel, fastAccel);

        // closes claw to grab starting specimen
        Actions.runBlocking(claw.closeClaw());

        // builds trajectories beforehand
        Action HangSamplePos1 = hangSamplePos1.build();
        Action DragSamples = dragSamples.build();
        Action HangSamplePos2 = hangSample1.build();
        Action CollectSpecimenPos3 = collectSpecimen2.build();
        Action HangSamplePos3 = hangSpecimen2.build();
        Action CollectSpecimenPos4 = collectSpecimen3.build();
        Action HangSamplePos4 = hangSpecimen3.build();
        Action Park = park.build();

        while (!isStopRequested() && !opModeIsActive()){
            telemetry.addData("Ready", true);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;


        Actions.runBlocking(
            new SequentialAction(
                    // starting specimen
                    new ParallelAction(
                            HangSamplePos1,
                            slide.specimenUp()
                    ),
                    pivot.smallPivotForward(),
                    slide.deploySpecimen(),
                    claw.openClaw(),
                    new ParallelAction(
                            pivot.goToStart(),
                            slide.idleDown(),
                            DragSamples
                    ),
                    // second specimen
                    pivot.specimenPickup(),
                    claw.closeClaw(),
                    new ParallelAction(
                            pivot.goToStart(),
                            claw.adjustSpecimen(),
                            slide.specimenUp(),
                            HangSamplePos2
                    ),
                    pivot.smallPivotForward(),
                    slide.deploySpecimen(),
                    claw.openClaw(),
                    pivot.goToStart(),
                    // third specimen
                    new ParallelAction(
                            slide.idleDown(),
                            CollectSpecimenPos3
                    ),
                    pivot.specimenPickup(),
                    claw.closeClaw(),
                    new ParallelAction(
                            pivot.goToStart(),
                            claw.adjustSpecimen(),
                            slide.specimenUp(),
                            HangSamplePos3
                    ),
                    pivot.smallPivotForward(),
                    slide.deploySpecimen(),
                    claw.openClaw(),
                    pivot.goToStart(),
//                    // fourth specimen
//                    new ParallelAction(
//                            slide.idleDown(),
//                            CollectSpecimenPos4
//                    ),
//                    pivot.specimenPickup(),
//                    claw.closeClaw(),
//                    pivot.goToStart(),
//                    new ParallelAction(
//                            slide.specimenUp(),
//                            HangSamplePos4
//                    ),
//                    pivot.smallPivotForward(),
//                    slide.deploySpecimen(),
//                    claw.openClaw(),
//                    pivot.goToStart(),
                    Park
            )

        );
    }

}
