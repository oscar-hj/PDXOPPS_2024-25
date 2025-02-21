package org.firstinspires.ftc.teamcode._autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class AutoComponents {
    /*
    Slide Component
     */
    public static class Slide {
        private final DcMotor slide;

        public Slide(HardwareMap hardwareMap) {
            slide = hardwareMap.get(DcMotor.class, "slideMotor");
            slide.setTargetPosition(slide.getCurrentPosition());
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
        }

        public class DeploySpecimen implements Action {
            private boolean initialized = false;
            int targetPos = (int) (slide.getCurrentPosition() + 1750 * (537.7 / 1425.1));

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    packet.put("targetPos", targetPos);
                    slide.setTargetPosition(targetPos);
                    initialized = true;
                }

                boolean isBusy = slide.isBusy();
                packet.put("isBusy", isBusy);
                return slide.isBusy();
            }
        }

        public Action deploySpecimen() {
            return new DeploySpecimen();
        }


        public class IdleDown implements Action {
            private boolean initialized = false;
            int targetPos = (int) (slide.getCurrentPosition() + 1000 * (537.7 / 1425.1));

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    packet.put("targetPos", targetPos);
                    slide.setTargetPosition(targetPos);
                    initialized = true;
                }

                boolean isBusy = slide.isBusy();
                packet.put("isBusy", isBusy);
                return slide.isBusy();
            }
        }

        public Action idleDown() {
            return new IdleDown();
        }

        public class ParkDown implements Action {
            private boolean initialized = false;
            int targetPos = (int) (slide.getCurrentPosition() + 3500 * (537.7 / 1425.1));

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    packet.put("targetPos", targetPos);
                    slide.setTargetPosition(targetPos);
                    initialized = true;
                }

                boolean isBusy = slide.isBusy();
                packet.put("isBusy", isBusy);
                return slide.isBusy();
            }
        }

        public Action parkDown() {
            return new ParkDown();
        }

        public class SpecimenUp implements Action {
            private boolean initialized = false;
            //            int targetPos = slide.getCurrentPosition() + 3500;
            int targetPos = (int) (slide.getCurrentPosition() + 3400 * (537.7 / 1425.1));

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    packet.put("targetPos", targetPos);
                    slide.setTargetPosition(targetPos);
                    initialized = true;
                }

                boolean isBusy = slide.isBusy();
                packet.put("isBusy", isBusy);
                return slide.isBusy();
            }
        }

        public Action specimenUp() {
            return new SpecimenUp();
        }

        public class BasketSample implements Action {
            private boolean initialized = false;
            int targetPos = (int) (slide.getCurrentPosition() + 9400 * (537.7 / 1425.1));

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    packet.put("targetPos", targetPos);
                    slide.setTargetPosition(targetPos);
                    initialized = true;
                }

                boolean isBusy = slide.isBusy();
                packet.put("isBusy", isBusy);
                return slide.isBusy();
            }
        }

        public Action basketSample() {
            return new BasketSample();
        }
    }



    /*
    Pivot Component
    */
    public static class Pivot {
        private final DcMotor pivotMotor;

        public Pivot(HardwareMap hardwareMap) {
            pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
            pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition());
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotMotor.setPower(0.5);
            pivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class SmallPivotForward implements Action {
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();
            int targetPos = pivotMotor.getCurrentPosition() + 150;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
                }


                boolean isBusy = pivotMotor.isBusy();
                packet.put("isBusy", isBusy);
                return pivotMotor.isBusy();
            }
        }

        public Action smallPivotForward() {
            return new SmallPivotForward();
        }


        public class BasketPivot implements Action {
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();
            int targetPos = pivotMotor.getCurrentPosition() + 240;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
                }

                boolean isBusy = pivotMotor.isBusy();
                packet.put("isBusy", isBusy);
                return pivotMotor.isBusy();
            }
        }

        public Action basketPivot() {
            return new BasketPivot();
        }


        public class TouchPivot implements Action {
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();
            int targetPos = pivotMotor.getCurrentPosition() + 400;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
                }

                boolean isBusy = pivotMotor.isBusy();
                packet.put("isBusy", isBusy);
                return pivotMotor.isBusy();
            }
        }

        public Action touchPivot() {
            return new TouchPivot();
        }

        public class GoToStart implements Action {
            private boolean initialized = false;
            int targetPos = pivotMotor.getCurrentPosition();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
                }

                boolean isBusy = pivotMotor.isBusy();
                packet.put("isBusy", isBusy);
                return pivotMotor.isBusy();
            }
        }

        public Action goToStart() {
            return new GoToStart();
        }

        public class SpecimenPickup implements Action {
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();
            int targetPos = pivotMotor.getCurrentPosition() + 1350;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    pivotMotor.setPower(0.6);
                    packet.put("targetPos", targetPos);
                    pivotMotor.setTargetPosition(targetPos);
                    initialized = true;
//                    sleep(1000);
                }

                boolean isBusy = pivotMotor.isBusy();
                packet.put("isBusy", isBusy);

                if (pivotMotor.isBusy()) {
                    return true;
                } else {
                    pivotMotor.setPower(0.5);
                    return false;
                }
            }
        }

        public Action specimenPickup() {
            return new SpecimenPickup();
        }

    }

    /*
    Claw Component
     */
    public static class Claw {
        private final CRServo clawServo;

        // pivot servo not in use
        //private CRServo pivotServo;

        public Claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(CRServo.class, "clawServo");

            // locks the rotation servo in place
            CRServo rotationServo = hardwareMap.get(CRServo.class, "pivotServo");
            rotationServo.setPower(0);
        }

        public class OpenClaw implements Action {
            ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    clawServo.setPower(-1);
                    initialized = true;
                }

                double time = timer.time();
                return time < 0.75;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action {
            ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    clawServo.setPower(1);
                    initialized = true;
                }

                double time = timer.time();
                return time < 0.75;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }


        public class AdjustSpecimen implements Action{
            ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    timer.reset();
                    clawServo.setPower(-1);
                    initialized = true;
                }

                double time = timer.time();

                if (time < 0.25){
                    return true;
                }else{
                    clawServo.setPower(1);
                    return false;
                }
            }
        }
        public Action adjustSpecimen(){
            return new AdjustSpecimen();
        }
    }
}
