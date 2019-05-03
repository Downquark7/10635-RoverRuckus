package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class numberObtainingCode extends OpMode {
    DcMotor fLeftMotor, fRightMotor, bLeftMotor, bRightMotor;

    public void init() {
        fLeftMotor = hardwareMap.get(DcMotor.class, "flmotor");
        fRightMotor = hardwareMap.get(DcMotor.class, "frmotor");
        bLeftMotor = hardwareMap.get(DcMotor.class, "blmotor");
        bRightMotor = hardwareMap.get(DcMotor.class, "brmotor");
        fRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        fLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    final double sqrt2 = Math.sqrt(2);
    double r, robotAngle, rightX;

    public void loop() {
        if (gamepad1.left_bumper) {
            r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            robotAngle = Math.atan2(-gamepad1.left_stick_y * sqrt2, gamepad1.left_stick_x) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
        }
        telemetry.addData("r", r);
        telemetry.addData("robotAngle", robotAngle);
        telemetry.addData("rightX", rightX);

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        if (gamepad1.a) {
            fLeftMotor.setPower(v1 * 0.2);
            fRightMotor.setPower(v2 * 0.2);
            bLeftMotor.setPower(v3 * 0.2);
            bRightMotor.setPower(v4 * 0.2);
        } else if (gamepad1.b) {
            fLeftMotor.setPower(-v1 * 0.2);
            fRightMotor.setPower(-v2 * 0.2);
            bLeftMotor.setPower(-v3 * 0.2);
            bRightMotor.setPower(-v4 * 0.2);
        } else if(gamepad1.x) {
            double fLeftMotorPower = -fLeftMotor.getCurrentPosition();
            double fRightMotorPower = -fRightMotor.getCurrentPosition();
            double bLeftMotorPower = -bLeftMotor.getCurrentPosition();
            double bRightMotorPower = -bRightMotor.getCurrentPosition();
            double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

            if (Math.abs(fLeftMotorPower) < 10)
                fLeftMotorPower = 0;
            if (Math.abs(fRightMotorPower) < 10)
                fRightMotorPower = 0;
            if (Math.abs(bRightMotorPower) < 10)
                bRightMotorPower = 0;
            if (Math.abs(bLeftMotorPower) < 10)
                bLeftMotorPower = 0;

            max /= 0.3;

            if (max > 0) {
                fLeftMotor.setPower(fLeftMotorPower / max);
                fRightMotor.setPower(fRightMotorPower / max);
                bLeftMotor.setPower(bLeftMotorPower / max);
                bRightMotor.setPower(bRightMotorPower / max);
            } else {
                fLeftMotor.setPower(0);
                fRightMotor.setPower(0);
                bLeftMotor.setPower(0);
                bRightMotor.setPower(0);
            }
        } else {
            fLeftMotor.setPower(0);
            fRightMotor.setPower(-0);
            bLeftMotor.setPower(0);
            bRightMotor.setPower(-0);
        }

        telemetry.addData("fLeftMotor", fLeftMotor.getCurrentPosition());
        telemetry.addData("fRightMotor", fRightMotor.getCurrentPosition());
        telemetry.addData("bLeftMotor", bLeftMotor.getCurrentPosition());
        telemetry.addData("bRightMotor", bRightMotor.getCurrentPosition());

    }
}