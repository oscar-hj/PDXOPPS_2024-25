package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends LinearOpMode{
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    DcMotor pivotMotor, slideMotor, hookMotor;
    CRServo clawServo, clawPivot;
    DistanceSensor distanceSensor;
    GoBildaPinpointDriver odo;
    int pivotPos;
    double gp1LSX, gp1LSY, gp1RSX, gp1RSY, gp1LT, gp1RT, gp2LSY, gp2RSY, gp2LT, gp2RT;
    boolean gp1LB, gp1RB, gp2DPadUp, gp2DPadDown, gp2DPadLeft, gp2DPadRight, gp2PS, gp2A, gp2B;
    double motorSpeed, slowSpeed = 0.1, normSpeed = 0.5, boostSpeed = 1, deadzone = 0.05;

    @Override
    public void runOpMode() {
        initMotors();
        waitForStart();

        if(opModeIsActive()){
            pivotPos = pivotMotor.getCurrentPosition();
            pivotMotor.setTargetPosition(pivotPos);
            while(opModeIsActive()) {
                //reads the game-pad inputs and assigns them to variables
                readGP1();
                readGP2();

                // sets speed
                if(gp1LB){
                    motorSpeed = slowSpeed;
                } else if (gp1RB) {
                    motorSpeed = boostSpeed;
                } else {
                    motorSpeed = normSpeed;
                }

                // 2D movement and rotation
                if(abs(gp1LSX) > deadzone | abs(gp1LSY) > deadzone | abs(gp1RSX) > deadzone) {
                    Drive2D(gp1LSX, gp1LSY, gp1RSX, motorSpeed);
                } else{
                    stopDriveMotors();
                }

                //pivot slide
                if (gp2LT > deadzone){
                    pivotPos = drivePivotMotor(-gp2LT, motorSpeed);
                } else if (gp2RT > deadzone) {
                    pivotPos = drivePivotMotor(gp2RT, motorSpeed);
                } else{
                    lockPivotMotor(pivotPos);
                }

                // slide
                if(abs(gp2LSY) > deadzone){
                    slideMotor.setPower(gp2LSY * motorSpeed);
                } else {
                    slideMotor.setPower(0);
                }

                // open/close claw
                if (gp2A){
                    clawServo.setPower(1);
                } else if (gp2B) {
                    clawServo.setPower(-1);
                } else{
                    clawServo.setPower(0);
                }

                // rotate claw
                if (gp2DPadLeft){
                    clawPivot.setPower(1);
                } else if (gp2DPadRight) {
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
    }
    public void readGP1(){
        /*
        Left Stick X,Y: Flat Movement
        Right Stick X: Rotation

        Left Bumper: Slow Mode
        Right Bumper: Turbo MOde
         */
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
        /*

         */
        gp2LSY = gamepad2.left_stick_y;
        gp2RSY = gamepad2.right_stick_y;

        gp2LT = gamepad2.left_trigger;
        gp2RT = gamepad2.right_trigger;

        gp2DPadUp = gamepad2.dpad_up;
        gp2DPadDown = gamepad2.dpad_down;
        gp2DPadLeft = gamepad2.dpad_left;
        gp2DPadRight = gamepad2.dpad_right;

        gp2A = gamepad2.a;
        gp2B = gamepad2.b;

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

    public void Drive2D(double valx, double valy, double valr, double speed){
        frontLeftMotor.setPower((valy - valx - valr) * speed);
        frontRightMotor.setPower((valy + valx + valr) * speed);
        backLeftMotor.setPower((valy + valx - valr) * speed);
        backRightMotor.setPower((valy - valx + valr) * speed);

        getDriveMotorTele();
    }

    public int drivePivotMotor(double val, double speed) {
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setPower(val * speed);
        return pivotMotor.getCurrentPosition();
    }

    public void lockPivotMotor(int pivotEncoderPosition){
        pivotMotor.setTargetPosition(pivotEncoderPosition);
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

    public void rotateRobot(double val, double speed){
        frontLeftMotor.setPower(val * speed);
        frontRightMotor.setPower(-val * speed);
        backLeftMotor.setPower(val * speed);
        backRightMotor.setPower(-val * speed);

        getDriveMotorTele();
    }
}