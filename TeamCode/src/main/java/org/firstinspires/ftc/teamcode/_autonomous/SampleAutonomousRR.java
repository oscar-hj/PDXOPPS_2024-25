package org.firstinspires.ftc.teamcode._autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;


@Config
@Autonomous(name = "SampleAutonomousRR", group = "Autonomous")
public class SampleAutonomousRR extends LinearOpMode {
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
            int targetPos = (int) (slide.getCurrentPosition() + 1750 * (537.7/1425.1));

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    packet.put("targetPos", targetPos);
                    slide.setTargetPosition(targetPos);
                    initialized = true;
//                    sleep(1000);
                }

                boolean isBusy = slide.isBusy();
                packet.put("isBusy", isBusy);
                return slide.isBusy();
            }
        }
        public Action deploySpecimen(){
            return new DeploySpecimen();
        }

        public class IdleDown implements Action{
            private boolean initialized = false;
            int targetPos = (int) (slide.getCurrentPosition() + 1000 * (537.7/1425.1));

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
            return new IdleDown();
        }

        public class ParkDown implements Action{
            private boolean initialized = false;
            int targetPos = (int) (slide.getCurrentPosition() + 3500 * (537.7/1425.1));

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
        public Action parkDown(){
            return new ParkDown();
        }

        public class SpecimenUp implements Action{
            private boolean initialized = false;
//            int targetPos = slide.getCurrentPosition() + 3500;
            int targetPos = (int) (slide.getCurrentPosition() + 3400 * (537.7/1425.1));

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
            return new SpecimenUp();
        }

        public class BasketSample implements Action{
            private boolean initialized = false;
            int targetPos = (int) (slide.getCurrentPosition() + 9400 * (537.7/1425.1));

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

        public Action basketSample(){
            return new BasketSample();
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
            private final ElapsedTime timer = new ElapsedTime();
            int targetPos = pivotMotor.getCurrentPosition() + 150;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    timer.reset();
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
                    sleep(500);
                }

//                double time = timer.time();
//                packet.put("time", time);
//                return time < 5;

                boolean isBusy = pivotMotor.isBusy();
                packet.put("isBusy", isBusy);
                return pivotMotor.isBusy();
            }
        }

        public Action smallPivotForward(){
            return new SmallPivotForward();
        }


        public class BasketPivot implements Action{
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();
            int targetPos = pivotMotor.getCurrentPosition() + 240;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    timer.reset();
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
//                    sleep(500);
                }

//                double time = timer.time();
//                packet.put("time", time);
//                return time < 5;

                boolean isBusy = pivotMotor.isBusy();
                packet.put("isBusy", isBusy);
                return pivotMotor.isBusy();
            }
        }

        public Action basketPivot(){
            return new BasketPivot();
        }


        public class TouchPivot implements Action{
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();
            int targetPos = pivotMotor.getCurrentPosition() + 400;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    timer.reset();
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
//                    sleep(500);
                }

//                double time = timer.time();
//                packet.put("time", time);
//                return time < 5;

                boolean isBusy = pivotMotor.isBusy();
                packet.put("isBusy", isBusy);
                return pivotMotor.isBusy();
            }
        }

        public Action touchPivot(){
            return new TouchPivot();
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
            private final ElapsedTime timer = new ElapsedTime();
            int targetPos = pivotMotor.getCurrentPosition() + 1350;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    timer.reset();
                    pivotMotor.setPower(0.6);
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
//                    sleep(1000);
                }

                boolean isBusy = pivotMotor.isBusy();
                packet.put("isBusy", isBusy);

                if (pivotMotor.isBusy()){
                    return true;
                } else{
                    pivotMotor.setPower(0.5);
                    return false;
                }
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

            // locks the rotation servo in place
            CRServo rotationServo = hardwareMap.get(CRServo.class, "pivotServo");
            rotationServo.setPower(0);
        }

        public class OpenClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                clawServo.setPower(-1);
                sleep(750);
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
                sleep(750);
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
