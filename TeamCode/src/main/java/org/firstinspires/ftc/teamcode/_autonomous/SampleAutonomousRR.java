package org.firstinspires.ftc.teamcode._autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode._autonomous.AutoComponents.*;


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

        TrajectoryActionBuilder gotoBasket1 = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(56, 56), Math.toRadians(60));

        // rotates to sample
        TrajectoryActionBuilder gotoSample2 = gotoBasket1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(55, 56), Math.toRadians(265));

        // goes to high basket and deploys
        TrajectoryActionBuilder gotoBasket2 = gotoSample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(56, 56), Math.toRadians(45));

        // rotates to sample
        TrajectoryActionBuilder gotoSample3 = gotoBasket2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(55, 56), Math.toRadians(280));

        // goes to high basket and deploys
        TrajectoryActionBuilder gotoBasket3 = gotoSample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(56, 56), Math.toRadians(45));

        // rotates a bit before sample to pivot down safely
        TrajectoryActionBuilder positionSample4 = gotoBasket3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(55.5, 56), Math.toRadians(280));

        // rotates the rest of the way to sample
        TrajectoryActionBuilder gotoSample4 = positionSample4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(55, 56), Math.toRadians(300));

        // goes to high basket and deploys
        TrajectoryActionBuilder gotoBasket4 = gotoSample4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(56, 56), Math.toRadians(45));

        // goes to parking position
        TrajectoryActionBuilder park = gotoBasket4.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(24, 6, Math.toRadians(180)), Math.toRadians(180));


        // closes claw to grab starting specimen
        Actions.runBlocking(claw.closeClaw());

        // builds trajectories beforehand
        Action GotoBasket1 = gotoBasket1.build();
        Action GoToSample2 = gotoSample2.build();
        Action GoToBasket2 = gotoBasket2.build();
        Action GoToSample3 = gotoSample3.build();
        Action GoToBasket3 = gotoBasket3.build();
        Action PositionSample4 = positionSample4.build();
        Action GoToSample4 = gotoSample4.build();
        Action GoToBasket4 = gotoBasket4.build();
        Action Park = park.build();


        // loop after init but before start
        while (!isStopRequested() && !opModeIsActive()){
            telemetry.addData("Parallel (Left) Encoder Check", drive.leftFront.getCurrentPosition());
            telemetry.addData("Perpendicular (Right) Encoder Check", drive.rightBack.getCurrentPosition());

            // if either of the encoders are reading 0, outputs warning to check encoders (Encoders are not working properly)
            // **encoders restart after turning off**
            if(drive.leftFront.getCurrentPosition() == 0 || drive.rightBack.getCurrentPosition() == 0){
                telemetry.addData("WARNING", "CHECK ENCODERS");
                telemetry.addData("WARNING!", "CHECK ENCODERS");
                telemetry.addData("WARNING!!", "CHECK ENCODERS");
                telemetry.addData("Ready", "FALSE");
            } else{
                telemetry.addData("Ready", "True");
            }

            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;


        // Autonomous Routine
        Actions.runBlocking(
                new SequentialAction(
                        // starting sample
                        new ParallelAction(
                                GotoBasket1,
                                slide.basketSample()
                        ),
                        pivot.basketPivot(),
                        claw.openClaw(),
                        pivot.goToStart(),
                        // second sample
                        new ParallelAction(
                                GoToSample2,
                                slide.extendedSamplePickup()
                        ),
                        pivot.specimenPickup(),
                        claw.closeClaw(),
                        pivot.goToStart(),
                        new ParallelAction(
                                GoToBasket2,
                                slide.basketSample()
                        ),
                        pivot.basketPivot(),
                        claw.openClaw(),
                        pivot.goToStart(),
                        //third sample
                        new ParallelAction(
                                slide.extendedSamplePickup(),
                                GoToSample3
                        ),
                        pivot.specimenPickup(),
                        claw.closeClaw(),
                        pivot.goToStart(),
                        new ParallelAction(
                                GoToBasket3,
                                slide.basketSample()
                        ),
                        pivot.basketPivot(),
                        claw.openClaw(),
                        pivot.goToStart(),
                        //fourth sample
                        new ParallelAction(
                                PositionSample4,
                                slide.extendedSamplePickup()
                        ),
                        pivot.specimenPickup(),
                        GoToSample4,
                        claw.closeClaw(),
                        pivot.goToStart(),
                        new ParallelAction(
                                GoToBasket4,
                                slide.basketSample()
                        ),
                        pivot.basketPivot(),
                        claw.openClaw(),
                        // parking
                        new ParallelAction(
                                slide.idleDown(),
                                Park
                        ),
                        pivot.touchPivot()
                )

        );
    }

}
