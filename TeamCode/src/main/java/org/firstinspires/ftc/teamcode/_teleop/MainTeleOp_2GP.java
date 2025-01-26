package org.firstinspires.ftc.teamcode._teleop;

import static java.lang.Math.abs;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MainTeleOp_2GP")
public class MainTeleOp_2GP extends LinearOpMode{
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    DcMotor pivotMotor, slideMotor, hookMotor;
    CRServo clawServo, pivotServo;
    DistanceSensor distanceSensor;
    HuskyLens lens;
    int pivotPos, specimenPickupPos;
    double gp1LSX, gp1LSY, gp1RSX, gp1RSY, gp1LT, gp1RT, gp2LSY, gp2RSY, gp2LT, gp2RT, gp2LSX, gp2RSX;
    boolean gp1LB, gp1RB, gp2DPadUp, gp2DPadDown, gp2DPadLeft, gp2DPadRight, gp2PS, gp2A, gp2B, gp2LB, gp2RB;
    double motorSpeedgp1, motorSpeedgp2, dis, slowSpeed = 0.1, normSpeed = 0.5, boostSpeed = 1, deadzone = 0.05;

    @Override
    public void runOpMode() {
        initAll();
        waitForStart();

        if(opModeIsActive()){
            pivotPos = pivotMotor.getCurrentPosition();
            specimenPickupPos = pivotMotor.getCurrentPosition() + 1300;
            pivotMotor.setTargetPosition(pivotPos);
            while(opModeIsActive()) {
                //reads the game-pad inputs and assigns them to variables
                readGP1();
                readGP2();


                // GAMEPAD 1 CONTROL
                // sets speed
                if(gp1LB){
                    motorSpeedgp1 = slowSpeed;
                } else if (gp1RB) {
                    motorSpeedgp1 = boostSpeed;
                } else {
                    motorSpeedgp1 = normSpeed;
                }

                // 2D movement and rotation
                if(abs(gp1LSX) > deadzone | abs(gp1LSY) > deadzone | abs(gp1RSX) > deadzone) {
                    drive2D(gp1LSX, gp1LSY, gp1RSX, motorSpeedgp1);
                } else{
                    stopDriveMotors();
                }

                // rotate hooks
                if (gp1LT > deadzone){
                    hookMotor.setPower(gp1LT * motorSpeedgp1);
                } else if (gp1RT > deadzone){
                    hookMotor.setPower(-gp1RT * motorSpeedgp1);
                } else{
                    hookMotor.setPower(0);
                }


                //GAMEPAD 2 CONTROL
                // sets speed
                if(gp2LB){
                    motorSpeedgp2 = slowSpeed;
                } else if (gp2RB) {
                    motorSpeedgp2 = boostSpeed;
                } else {
                    motorSpeedgp2 = normSpeed;
                }

                //pivot slide
                if (gp2LT > deadzone){
                    pivotPos = drivePivotMotor(-gp2LT, motorSpeedgp2);
                } else if (gp2RT > deadzone) {
                    pivotPos = drivePivotMotor(gp2RT, motorSpeedgp2);
                } else if (gp2LT < deadzone && gp2RT < deadzone && !gp2PS){
                    lockPivotMotor(pivotPos);
                }

                // auto position slide (HOLD)
                if (gp2PS){
                    lockPivotMotor(specimenPickupPos);
                }

                // slide
                if (gp2DPadUp) {
                    if(gp2LB){
                        motorSpeedgp2 = 0.1;
                    }
                    slideMotor.setPower(1 * motorSpeedgp2);
                }else if(gp2DPadDown){
                    slideMotor.setPower(-1 * motorSpeedgp2);
                }else{
                    slideMotor.setPower(0);
                }

                // open/close claw
                if (gp2A){
                    clawServo.setPower(1);
                } else if (gp2B) {
                    clawServo.setPower(-0.5);
                }

                // rotate claw
                if (abs(gp2RSX) > deadzone){
                    pivotServo.setPower(gp2RSX);
                }else{
                    pivotServo.setPower(0);
                }

                /*
                 HUSKY LENS DELAYED
                 if (gp2PS){
                     autoRotate();
                 }

                 doHusky();
                */

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
        gp2LSX = gamepad2.left_stick_x;

        gp2RSY = gamepad2.right_stick_y;
        gp2RSX = gamepad2.left_stick_x;

        gp2LT = gamepad2.left_trigger;
        gp2RT = gamepad2.right_trigger;

        gp2RB = gamepad2.right_bumper;
        gp2LB = gamepad2.left_bumper;

        gp2DPadUp = gamepad2.dpad_up;
        gp2DPadDown = gamepad2.dpad_down;
        gp2DPadLeft = gamepad2.dpad_left;
        gp2DPadRight = gamepad2.dpad_right;

        gp2A = gamepad2.a;
        gp2B = gamepad2.b;

        gp2PS = gamepad2.ps;

    }
    public void initAll(){
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
        pivotServo = hardwareMap.get(CRServo.class, "pivotServo");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        lens = hardwareMap.get(HuskyLens.class, "lens");
        lens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public void drive2D(double valx, double valy, double valr, double speed){
        frontLeftMotor.setPower((valy - valx - valr) * speed);
        frontRightMotor.setPower((valy + valx + valr) * speed);
        backLeftMotor.setPower((valy + valx - valr) * speed);
        backRightMotor.setPower((valy - valx + valr) * speed);

        getDriveMotorTele();
    }

    public int drivePivotMotor(double val, double speed) {
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setPower(val * speed);
        telemetry.addData("pivotEncoder", pivotMotor.getCurrentPosition());
        return pivotMotor.getCurrentPosition();
    }

    public void lockPivotMotor(int pivotEncoderPosition){
        pivotMotor.setTargetPosition(pivotEncoderPosition);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("pivotEncoder", pivotEncoderPosition);
        pivotMotor.setPower(0.5);
    }

    public void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        getDriveMotorTele();
    }

//    public void autoRotate(){
//        HuskyLens.Block[] blockArr = lens.blocks();
//
//        if (blockArr.length >= 2){
//            telemetry.addLine("WARNING: THERE ARE 2 OR MORE SAMPLES DETECTED");
//        }
//    }
//    public void doHusky(){
//        /* Color IDs
//        YELLOW: ??
//        RED: ??
//        BLUE: ??
//        */
//        HuskyLens.Block[] blockArr = lens.blocks();
//
//        for (HuskyLens.Block block : blockArr) {
//            telemetry.addData("Block", block.toString());
//        }
//    }
}