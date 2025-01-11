package org.firstinspires.ftc.teamcode._autonomous;

import android.view.animation.LinearInterpolator;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.net.HttpURLConnection;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "RoadrunnerTest", group = "Autonomous")
public class RoadrunnerTest extends LinearOpMode {
    // slide class
    public class Slide {
        private DcMotor slide;

        public Slide(HardwareMap hardwareMap){
            slide = hardwareMap.get(DcMotor.class, "slideMotor");
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class SlideUp implements Action{
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    slide.setPower(0.8);
                    timer.reset();
                    initialized = true;
                }

                double time = timer.time();
                packet.put("time", time);
                if (time < 2){
                    return true;
                }else{
                    slide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideUp(){
            return new SlideUp();
        }

        public class SlideDown implements Action{
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    slide.setPower(-0.8);
                    timer.reset();
                    initialized = true;
                }

                double time = timer.time();
                packet.put("time", time);
                if (time < .5){
                    return true;
                }else{
                    slide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideDown(){
            return new SlideDown();
        }

        public class SpecimenSlideDown implements Action{
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    sleep(1000);
                    slide.setPower(-0.8);
                    timer.reset();
                    initialized = true;
                }

                double time = timer.time();
                packet.put("time", time);
                if (time < 1){
                    return true;
                }else{
                    slide.setPower(0);
                    return false;
                }
            }
        }
        public Action specimenSlideDown(){
            return new SpecimenSlideDown();
        }
    }

    public class Pivot {
        private DcMotor pivotMotor;

        public Pivot(HardwareMap hardwareMap){
            pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
            pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition());
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotMotor.setPower(0.5);
            pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

                return pivotMotor.isBusy();
            }
        }

        public Action smallPivotForward(){
            return new SmallPivotForward();
        }

        public class SmallPivotBackwards implements Action{
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();
            int targetPos = pivotMotor.getCurrentPosition() - 140;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
                }

                double t = timer.time();
                packet.put("t", t);
                if (t < 3){
                    return pivotMotor.isBusy();
                } else{
                    return false;
                }

            }
        }

        public Action smallPivotBackwards(){
            return new SmallPivotBackwards();
        }

    }

    // claw class
    public class Claw {
        private CRServo clawServo;
        private CRServo pivotServo;

        public Claw(HardwareMap hardwareMap){
            clawServo = hardwareMap.get(CRServo.class, "clawServo");
            pivotServo = hardwareMap.get(CRServo.class, "pivotServo");
        }

        public class OpenClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                clawServo.setPower(-1);
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
                return false;
            }
        }
        public Action closeClaw(){
            return new CloseClaw();
        }
    }

    @Override
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(-13, 64, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);


        TrajectoryActionBuilder startGoToRung = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-8, 34));
        TrajectoryActionBuilder dragSamples = startGoToRung.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-8, 38))   // go back a little
                .strafeToConstantHeading(new Vector2d(-36, 38))  // slide
                .strafeToConstantHeading(new Vector2d(-36, 12))  // go forward
                .strafeToConstantHeading(new Vector2d(-46, 12))  // slide and drag
                .strafeToConstantHeading(new Vector2d(-46, 56))  // ^
                .strafeToConstantHeading(new Vector2d(-46, 12))  // ^
                .strafeToConstantHeading(new Vector2d(-52, 12))  // slide and drag
                .strafeToConstantHeading(new Vector2d(-52, 56))  // ^
                .strafeToConstantHeading(new Vector2d(-52, 12))  // ^
                .strafeToConstantHeading(new Vector2d(-60, 12))  // slide and drag
                .strafeToConstantHeading(new Vector2d(-60, 56)); // ^



        Action StartGoToRung = startGoToRung.build();
        Action DragSamples = dragSamples.build();

        Actions.runBlocking(claw.closeClaw());

        while (!isStopRequested() && !opModeIsActive()){
            telemetry.addData("Ready", true);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                StartGoToRung,
                                slide.slideUp()
                        ),
                        new SequentialAction(
                                pivot.smallPivotForward(),
                                slide.specimenSlideDown(),
                                claw.openClaw(),
                                pivot.smallPivotBackwards(),
                                slide.slideDown(),
                                DragSamples
                        )
                )

        );
    }

}
