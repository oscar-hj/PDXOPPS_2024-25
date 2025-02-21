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
@Autonomous(name = "SampleAutonomousRR", group = "Autonomous")
public class SampleAutonomousRR extends LinearOpMode {

    @Override
    public void runOpMode(){
        // initial pose on map (specimen)
        Pose2d initialPose = new Pose2d(13, 64, Math.toRadians(270));

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


        // go to rung where the robot currently holds a specimen
        TrajectoryActionBuilder gotoBasket1 = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(56, 56), Math.toRadians(60));

        TrajectoryActionBuilder gotoSample2 = gotoBasket1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(50, 40), Math.toRadians(270), fastVel, fastAccel);

        TrajectoryActionBuilder gotoBasket2 = gotoSample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(56, 56), Math.toRadians(60), fastVel, fastAccel);

        TrajectoryActionBuilder gotoSample3 = gotoBasket2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(61, 40), Math.toRadians(270));

        TrajectoryActionBuilder gotoBasket3 = gotoSample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(56, 56), Math.toRadians(60));

        TrajectoryActionBuilder park = gotoBasket3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(36, 6), Math.toRadians(180), fastVel, fastAccel);

        TrajectoryActionBuilder parkTouch = park.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(24, 6), fastVel, fastAccel);
//        TrajectoryActionBuilder gotoSample3 = gotoBasket2.endTrajectory().fresh()
//                .





        // closes claw to grab starting specimen
        Actions.runBlocking(claw.closeClaw());

        // builds trajectories beforehand
        Action GotoBasket1 = gotoBasket1.build();
        Action GoToSample2 = gotoSample2.build();
        Action GoToBasket2 = gotoBasket2.build();
        Action GoToSample3 = gotoSample3.build();
        Action GoToBasket3 = gotoBasket3.build();
        Action Park = park.build();
        Action TouchPark = parkTouch.build();


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
                            GotoBasket1,
                            slide.basketSample()
                    ),
                    pivot.basketPivot(),
                    claw.openClaw(),
                    new ParallelAction(
                            pivot.goToStart(),
                            slide.idleDown()
                    ),
                    GoToSample2,
                    pivot.specimenPickup(),
                    claw.closeClaw(),
                    new ParallelAction(
                            GoToBasket2,
                            pivot.goToStart()
                    ),
                    slide.basketSample(),
                    pivot.basketPivot(),
                    claw.openClaw(),
                    pivot.goToStart(),
                    new ParallelAction(
                            slide.idleDown(),
                            GoToSample3
                    ),
                    pivot.specimenPickup(),
                    claw.closeClaw(),
                    new ParallelAction(
                            GoToBasket3,
                            pivot.goToStart()
                    ),
                    slide.basketSample(),
                    pivot.basketPivot(),
                    claw.openClaw(),
                    pivot.goToStart(),
                    new ParallelAction(
                        slide.parkDown(),
                        Park
                    ),
                    TouchPark,
                    pivot.touchPivot()
            )

        );
    }

}
