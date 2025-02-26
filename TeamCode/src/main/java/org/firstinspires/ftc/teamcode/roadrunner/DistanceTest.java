package org.firstinspires.ftc.teamcode.roadrunner;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "Distance Test")
public class DistanceTest extends LinearOpMode {
    int pivotPos;
    double distance;
    DistanceSensor disSense;
    DcMotor pivotMotor;

    @Override
    public void runOpMode(){
        disSense = hardwareMap.get(DistanceSensor.class, "disSense");
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivotPos = pivotMotor.getCurrentPosition();

        waitForStart();
        while (opModeIsActive()){
            distance = disSense.getDistance(DistanceUnit.INCH);

            if (gamepad2.ps) {
                if (distance < 5) {
                    pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    pivotMotor.setPower(-0.1);
                } else if (distance > 6) {
                    pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    pivotMotor.setPower(0.1);
                } else {
                    pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition());
                    pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } else {
                if (gamepad1.left_trigger > 0.05){
                    drivePivotMotor(-gamepad1.left_trigger, 0.5);
                    pivotPos = pivotMotor.getCurrentPosition();
                } else if (gamepad1.right_trigger > 0.05) {
                    drivePivotMotor(gamepad1.right_trigger, 0.5);
                    pivotPos = pivotMotor.getCurrentPosition();
                } else{
                    lockPivotMotor(pivotPos);
                }
            }

            telemetry.addData("Distance", distance);
            telemetry.update();
        }
    }

    public void drivePivotMotor(double val, double speed) {
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setPower(val * speed);
        pivotPos = pivotMotor.getCurrentPosition();
    }

    public void lockPivotMotor(int pivotEncoderPos){
        pivotMotor.setTargetPosition(pivotPos);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(0.5);
    }
}
