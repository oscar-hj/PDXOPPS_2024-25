package org.firstinspires.ftc.teamcode._autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


@Config
@Autonomous(name = "RoadrunnerTest", group = "Autonomous")
public class SpecimenAutonomousRR extends LinearOpMode {
    // slide class
    public class Slide {
        private final DcMotor slide;

        public Slide(HardwareMap hardwareMap){
            slide = hardwareMap.get(DcMotor.class, "slideMotor");
            slide.setTargetPosition(slide.getCurrentPosition());
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
        }

        public class DeploySpecimen implements Action{
            private boolean initialized = false;
            int targetPos = slide.getCurrentPosition() + 1900;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    packet.put("targetPos", targetPos);
                    slide.setTargetPosition(targetPos);
                    initialized = true;
                    sleep(1000);
                }

                boolean isBusy = slide.isBusy();
                packet.put("isBusy", isBusy);
                return isBusy;
            }
        }
        public Action deploySpecimen(){
            return new Slide.DeploySpecimen();
        }

        public class IdleDown implements Action{
            private boolean initialized = false;
            int targetPos = slide.getCurrentPosition() + 1000;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    packet.put("targetPos", targetPos);
                    slide.setTargetPosition(targetPos);
                    initialized = true;
                }

                boolean isBusy = slide.isBusy();
                packet.put("isBusy", isBusy);
                return slide.isBusy();
            }
        }
        public Action idleDown(){
            return new Slide.IdleDown();
        }

        public class SpecimenUp implements Action{
            private boolean initialized = false;
            int targetPos = slide.getCurrentPosition() + 3500;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    packet.put("targetPos", targetPos);
                    slide.setTargetPosition(targetPos);
                    initialized = true;
                }

                boolean isBusy = slide.isBusy();
                packet.put("isBusy", isBusy);
                return slide.isBusy();
            }
        }
        public Action specimenUp(){
            return new Slide.SpecimenUp();
        }
    }

    public class Pivot {
        private final DcMotor pivotMotor;

        public Pivot(HardwareMap hardwareMap){
            pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
            pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition());
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotMotor.setPower(0.5);
            pivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class SmallPivotForward implements Action{
            private boolean initialized = false;
            int targetPos = pivotMotor.getCurrentPosition() + 140;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
                }

                boolean isBusy = pivotMotor.isBusy();
                packet.put("isBusy", isBusy);
                return pivotMotor.isBusy();
            }
        }

        public Action smallPivotForward(){
            return new SmallPivotForward();
        }

        public class GoToStart implements Action{
            private boolean initialized = false;
            int targetPos = pivotMotor.getCurrentPosition();

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
                }

                boolean isBusy = pivotMotor.isBusy();
                packet.put("isBusy", isBusy);
                return pivotMotor.isBusy();
            }
        }
        public Action goToStart(){
            return new GoToStart();
        }

        public class SpecimenPickup implements Action{
            private boolean initialized = false;
            int targetPos = pivotMotor.getCurrentPosition() + 1300;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
                    sleep(1000);
                }

                boolean isBusy = pivotMotor.isBusy();
                packet.put("isBusy", isBusy);
                return isBusy;

            }
        }

        public Action specimenPickup(){
            return new SpecimenPickup();
        }

    }

    // claw class
    public class Claw {
        private final CRServo clawServo;

        // pivot servo not in use
        //private CRServo pivotServo;

        public Claw(HardwareMap hardwareMap){
            clawServo = hardwareMap.get(CRServo.class, "clawServo");
            //pivotServo = hardwareMap.get(CRServo.class, "pivotServo");
        }

        public class OpenClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                clawServo.setPower(0);
                sleep(500);
                return false;
            }
        }
        public Action openClaw(){
            return new OpenClaw();
        }

        public class CloseClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                clawServo.setPower(1);
                sleep(500);
                return false;
            }
        }
        public Action closeClaw(){
            return new CloseClaw();
        }
    }

    @Override
    public void runOpMode(){
        // initial pose on map (specimen)
        Pose2d initialPose = new Pose2d(-13, 64, Math.toRadians(270));

        // initialize movement and attachments
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);

        // go to rung where the robot currently holds a specimen
        TrajectoryActionBuilder hangSamplePos1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(6, 34));

        // routine to drag samples from default space to the observation zone
        // then position itself to get the first specimen
        TrajectoryActionBuilder dragSamples = hangSamplePos1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-2, 38))   // go back a little
                .strafeToConstantHeading(new Vector2d(-36, 38))  // slide
                .strafeToConstantHeading(new Vector2d(-36, 13))  // go forward
                .strafeToConstantHeading(new Vector2d(-46, 12))  // slide and drag
                .strafeToConstantHeading(new Vector2d(-46, 56))  // ^
                .strafeToConstantHeading(new Vector2d(-46, 13))  // ^
                .strafeToConstantHeading(new Vector2d(-52, 12))  // slide and drag
                .strafeToConstantHeading(new Vector2d(-52, 56))  // ^
                .strafeToConstantHeading(new Vector2d(-52, 13))  // ^
                .strafeToConstantHeading(new Vector2d(-58, 12))  // slide and drag
                .strafeToConstantHeading(new Vector2d(-58, 56))  // ^
                .strafeToConstantHeading(new Vector2d(-58, 48))  // move out of way and position to get specimen
                .strafeToLinearHeading(new Vector2d(-45, 48), Math.toRadians(90));

        // gets the specimen and moves to rung
        TrajectoryActionBuilder hangSample1 = dragSamples.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(4, 40), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(4, 33));


        // moves to specimen collection location and goes to hang location then parks in the
        // observation zone
        TrajectoryActionBuilder collectSpecimen2 = hangSample1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-45, 48), Math.toRadians(90));

        TrajectoryActionBuilder hangSpecimen2 = collectSpecimen2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(2, 40), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(2, 33));

        TrajectoryActionBuilder collectSpecimen3 = hangSpecimen2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-45, 48), Math.toRadians(90));

        TrajectoryActionBuilder hangSpecimen3 = collectSpecimen3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, 40), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(0, 33));

        TrajectoryActionBuilder park = hangSpecimen3.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-45, 60));

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
                    pivot.goToStart(),
                    new ParallelAction(
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
                    pivot.goToStart(),
                    new ParallelAction(
                            slide.specimenUp(),
                            HangSamplePos3
                    ),
                    pivot.smallPivotForward(),
                    slide.deploySpecimen(),
                    claw.openClaw(),
                    pivot.goToStart(),
                    // fourth specimen
                    new ParallelAction(
                            slide.idleDown(),
                            CollectSpecimenPos4
                    ),
                    pivot.specimenPickup(),
                    claw.closeClaw(),
                    pivot.goToStart(),
                    new ParallelAction(
                            slide.specimenUp(),
                            HangSamplePos4
                    ),
                    pivot.smallPivotForward(),
                    slide.deploySpecimen(),
                    claw.openClaw(),
                    pivot.goToStart(),
                    Park
            )

        );
    }

}
