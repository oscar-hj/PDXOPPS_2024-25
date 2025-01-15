package org.firstinspires.ftc.teamcode.roadrunner;

import java.lang.Math;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "PivotTest")
public class PivotTest extends LinearOpMode{
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    DcMotor pivotMotor, slideMotor, hookMotor;
    CRServo clawServo, clawPivot;
    int pivotPos;
    double gp1LSX, gp1LSY, gp1RSX, gp1RSY, gp1LT, gp1RT, gp2LSY, gp2RSY, gp2LT, gp2RT;
    boolean gp2DPadUp, gp2DPadDown, gp2PS, gp1LB, gp1RB;
    double motorSpeed, slowSpeed = 0.1, normSpeed = 0.5, boostSpeed = 1;

    @Override
    public void runOpMode() {
        initMotors();
        waitForStart();

        if(opModeIsActive()){
            pivotPos = pivotMotor.getCurrentPosition();
            pivotMotor.setTargetPosition(pivotPos);
            while(opModeIsActive()) {
                //reads the game pad inputs and assigns them to variables
                readGP1();
                readGP2();

                if(gp1LB){
                    motorSpeed = slowSpeed;
                } else if (gp1RB) {
                    motorSpeed = boostSpeed;
                } else {
                    motorSpeed = normSpeed;
                }

                // move forward and backwards, rotate left and right
                if (!(Math.abs(gp1RSX) > 0.1)) {
                    if (Math.abs(gp1LSY) > 0.1) {
                        driveForwardBackward(gp1LSY, motorSpeed);
                    } else if (Math.abs(gp1LSX) > 0.1) {
                        rotateRobot(-gp1LSX, motorSpeed);
                    } else {
                        stopDriveMotors();
                    }
                }

                //sliding
                if (!(Math.abs(gp1LSX) > 0 || Math.abs(gp1LSY) > 0)) {
                    if (gp1RSX > 0.1) {
                        slideRight(gp1RSX, motorSpeed);
                    } else if (gp1RSX < -0.1) {
                        slideLeft(Math.abs(gp1RSX), motorSpeed);
                    } else {
                        stopDriveMotors();
                    }
                }

//                if(gp1LT > 0.05){
//                    pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    pivotMotor.setPower(-gp1LT * motorSpeed);
//                    pivotEncoderCount = pivotMotor.getCurrentPosition();
//                } else if (gp1RT > 0.05) {
//                    pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    pivotMotor.setPower(gp1RT * motorSpeed);
//                    pivotEncoderCount = pivotMotor.getCurrentPosition();
//                } else{
//                    pivotMotor.setTargetPosition(pivotEncoderCount);
//                    pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    pivotMotor.setPower(0.5);
//
//                }

                //pivot slide
                if (gp1LT > 0.05){
                    drivePivotMotor(-gp1LT, motorSpeed);
                    pivotPos = pivotMotor.getCurrentPosition();
                } else if (gp1RT > 0.05) {
                    drivePivotMotor(gp1RT, motorSpeed);
                    pivotPos = pivotMotor.getCurrentPosition();
                } else{
                    lockPivotMotor(pivotPos);
                }

                //slide slide
                if(gamepad1.dpad_up){
                    slideMotor.setPower(0.5 * motorSpeed);
                } else if(gamepad1.dpad_down){
                    slideMotor.setPower(-0.5 * motorSpeed);
                }else{
                    slideMotor.setPower(0);
                }

                // claw servo
                if (gamepad1.a){
                    clawServo.setPower(1);
                } else if (gamepad1.b) {
                    clawServo.setPower(-1);
                } else{
                    clawServo.setPower(0);
                }

                // claw pivot servo
                if (gamepad1.x){
                    clawPivot.setPower(1);
                } else if (gamepad1.y) {
                    clawPivot.setPower(-1);
                } else{
                    clawPivot.setPower(0);
                }

                //move hook
                if (gp2DPadUp) {
                    hookMotor.setPower(0.5);
                }else if(gp2DPadDown){
                    hookMotor.setPower(-0.5);
                }else{
                    hookMotor.setPower(0);
                }

                telemetry.update();
            }
        }
    }

    public void getDriveMotorTele(){
        telemetry.addData("frontLeft", frontLeftMotor.getPower());
        telemetry.addData("frontRight", frontRightMotor.getPower());
        telemetry.addData("backLeft", backLeftMotor.getPower());
        telemetry.addData("backRight", backRightMotor.getPower());
        telemetry.update();
    }
    public void readGP1(){
        gp1LSX = gamepad1.left_stick_x;
        gp1LSY = gamepad1.left_stick_y;

        gp1RSX = gamepad1.right_stick_x;
        gp1RSY = gamepad1.right_stick_y;

        gp1LT = gamepad1.left_trigger;
        gp1RT = gamepad1.right_trigger;

        gp1LB = gamepad1.left_bumper;
        gp1RB = gamepad1.right_bumper;
    }

    public void readGP2(){
        gp2LSY = gamepad2.left_stick_y;
        gp2RSY = gamepad2.right_stick_y;

        gp2LT = gamepad2.left_trigger;
        gp2RT = gamepad2.right_trigger;

        gp2DPadUp = gamepad2.dpad_up;
        gp2DPadDown = gamepad2.dpad_down;

        gp2PS = gamepad2.ps;

    }
    public void initMotors(){
        backRightMotor = hardwareMap.get(DcMotor.class, "BRM");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        hookMotor = hardwareMap.get(DcMotor.class, "hookMotor");

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);


        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        clawServo = hardwareMap.get(CRServo.class, "clawServo");
        clawPivot = hardwareMap.get(CRServo.class, "clawPivot");
    }

    public void drivePivotMotor(double val, double speed) {
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setPower(val * speed);
        pivotPos = pivotMotor.getCurrentPosition();
    }

    public void lockPivotMotor(int pivotEncoderPos){
        pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition());
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(0.5);
    }

    public void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        getDriveMotorTele();
    }

    public void slideLeft(double val, double speed){
        frontLeftMotor.setPower(-val * speed);
        frontRightMotor.setPower(-val * speed);
        backLeftMotor.setPower(val * speed);
        backRightMotor.setPower(val * speed);

        getDriveMotorTele();
    }

    public void slideRight(double val, double speed){
        frontLeftMotor.setPower(val * speed);
        frontRightMotor.setPower(val * speed);
        backLeftMotor.setPower(-val * speed);
        backRightMotor.setPower(-val * speed);

        getDriveMotorTele();
    }

    public void driveForwardBackward(double val, double speed){
        frontLeftMotor.setPower(val * speed);
        frontRightMotor.setPower(val * speed);
        backLeftMotor.setPower(val * speed);
        backRightMotor.setPower(val * speed);

        getDriveMotorTele();
    }

    public void rotateRobot(double val, double speed){
        frontLeftMotor.setPower(val * speed);
        frontRightMotor.setPower(-val * speed);
        backLeftMotor.setPower(val * speed);
        backRightMotor.setPower(-val * speed);

        getDriveMotorTele();
    }
}