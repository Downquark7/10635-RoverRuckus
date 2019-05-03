package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.teamcode.robotconfig.armMotor;
import static org.firstinspires.ftc.teamcode.robotconfig.getGoldPosition;
import static org.firstinspires.ftc.teamcode.robotconfig.goldLocation.UNKNOWN;
import static org.firstinspires.ftc.teamcode.robotconfig.lastGoldLocation;
import static org.firstinspires.ftc.teamcode.robotconfig.lastLocation;
import static org.firstinspires.ftc.teamcode.robotconfig.slide;
import static org.firstinspires.ftc.teamcode.teleOp.armController5;
import static org.firstinspires.ftc.teamcode.teleOp.armPositionFinder;
import static org.firstinspires.ftc.teamcode.teleOp.getAngleValue;
import static org.firstinspires.ftc.teamcode.teleOp.liftTargetPosition;
import static org.firstinspires.ftc.teamcode.teleOp.maxArmSensorValue;
import static org.firstinspires.ftc.teamcode.teleOp.minArmSensorValue;
import static org.firstinspires.ftc.teamcode.teleOp.pGain;
import static org.firstinspires.ftc.teamcode.teleOp.slidePositions;

public class auxillaryDevices {
    ExpansionHubEx hub;
    ExpansionHubMotor lift, slide, arm, hang;
    CRServo vexy, vexy2;
    ElapsedTime timer;
    AnalogInput pot, pot2;

    auxillaryDevices(HardwareMap hardwareMap) {
        timer = new ElapsedTime();
        pot = hardwareMap.analogInput.get("pot");
        pot2 = hardwareMap.analogInput.get("pot2");
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        lift = hardwareMap.get(ExpansionHubMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide = hardwareMap.get(ExpansionHubMotor.class, "slide");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        hang = hardwareMap.get(ExpansionHubMotor.class, "hang");
        arm = hardwareMap.get(ExpansionHubMotor.class, "armMotor");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vexy = hardwareMap.crservo.get("vexy");
        vexy2 = hardwareMap.crservo.get("vexy2");
    }

    public void activateIntake() {
        vexy.setPower(0.75);
        vexy2.setPower(-0.75);
    }

    public void activateIntakeM() {
        vexy.setPower(0.5);
        vexy2.setPower(-0.5);
    }

    public void activateIntakeSlow() {
        vexy.setPower(0.25);
        vexy2.setPower(-0.25);
    }

    public void reverseIntakeSlow() {
        vexy.setPower(-0.25);
        vexy2.setPower(0.25);
    }

    public void deactivateIntake() {
        vexy.setPower(0);
        vexy2.setPower(-0);
    }

    public void reverseIntake() {
        vexy.setPower(-0.75);
        vexy2.setPower(0.75);
    }

    public boolean isUnhungWithArm() {
        if (!hub.getStandardModule().isEngaged()) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        RevBulkData data = hub.getBulkInputData();
        if (data == null) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        if (setArm(armTarget.vertical, data))
            setArm(armTarget.vertical, slidePositions[0], data);
        if (timer.milliseconds() > 100 && data.getMotorCurrentPosition(lift) < 100)
            startLiftUp();
        if (getGoldPosition() != UNKNOWN)
            hang.setPower(0.5);
        return data.getMotorCurrentPosition(lift) > liftTargetPosition * 0.9;
    }

    public boolean setArm(double target, int slidetarget) {
        if (!hub.getStandardModule().isEngaged()) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        RevBulkData data = hub.getBulkInputData();
        if (data == null) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        return setArm(target, slidetarget, data);
    }

    public boolean setArmSlowly(double target, int slidetarget) {
        if (!hub.getStandardModule().isEngaged()) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        RevBulkData data = hub.getBulkInputData();
        if (data == null) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        return setArmSlowly(target, slidetarget, data);
    }

    public boolean setArmSlowlyGroundFollowing(int slidetarget) {
        if (!hub.getStandardModule().isEngaged()) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        RevBulkData data = hub.getBulkInputData();
        if (data == null) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        return setArmSlowly(armPositionFinder(data.getMotorCurrentPosition(slide)), slidetarget, data);
    }

    public boolean setArm(double target) {
        if (!hub.getStandardModule().isEngaged()) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        RevBulkData data = hub.getBulkInputData();
        if (data == null) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        setArm(target, data);
        return Math.abs(arm.getPower()) < 0.3;
    }

    public boolean setArmSlowSlide(double target, int slidetarget) {
        if (!hub.getStandardModule().isEngaged()) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        RevBulkData data = hub.getBulkInputData();
        if (data == null) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        setArm(target, data);
        slide.setPower(Range.clip(0.004 * (slidetarget - data.getMotorCurrentPosition(slide)), -0.2, 0.5));
        return Math.abs(arm.getPower()) < 0.3;
    }

    public boolean intakeMinerals(int slidetarget) {
        if (!hub.getStandardModule().isEngaged()) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        RevBulkData data = hub.getBulkInputData();
        if (data == null) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        setArm(armPositionFinder(data.getMotorCurrentPosition(slide)), data);
        slide.setPower(0.7);
        if (data.getMotorCurrentPosition(slide) > slidetarget) {
            slide.setPower(0);
            arm.setPower(0);
            return true;
        }
        return false;
    }

    public boolean setArm(double target, int slidetarget, RevBulkData data) {
        if (!hub.getStandardModule().isEngaged()) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        if (data == null) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        slide.setPower((data.getAnalogInputValue(pot) / 1000.0) > armTarget.horizontal || (data.getAnalogInputValue(pot) / 1000.0) < armTarget.vertical ? Math.max(-0.1, 0.004 * (slidetarget - data.getMotorCurrentPosition(slide))) : Math.max(-0.7, 0.004 * (slidetarget - data.getMotorCurrentPosition(slide))));
        if (data.getAnalogInputValue(pot) / 1000.0 > minArmSensorValue && data.getAnalogInputValue(pot) / 1000.0 < maxArmSensorValue)
            arm.setPower(armController5(data.getAnalogInputValue(pot) / 1000.0, target, data.getMotorCurrentPosition(slide)));
        else
            arm.setPower(0);
        return Math.abs(slide.getPower()) < 0.3 && Math.abs(arm.getPower()) < 0.3;
    }

    public boolean setArmSlowly(double target, int slidetarget, RevBulkData data) {
        slide.setPower((data.getAnalogInputValue(pot) / 1000.0) > armTarget.horizontal || (data.getAnalogInputValue(pot) / 1000.0) < armTarget.vertical ? Math.max(-.1, 0.004 * (slidetarget - data.getMotorCurrentPosition(slide))) : Math.max(-0.7, 0.004 * (slidetarget - data.getMotorCurrentPosition(slide))));
        if (data.getAnalogInputValue(pot) / 1000.0 > minArmSensorValue && data.getAnalogInputValue(pot) / 1000.0 < maxArmSensorValue)
            arm.setPower(Range.clip(pGain * (data.getAnalogInputValue(pot) / 1000.0 - target), -0.4, 0.8));
        else
            arm.setPower(0);
        return Math.abs(slide.getPower()) < 0.3 && Math.abs(arm.getPower()) < 0.3;
    }

    public boolean setArmSlowly(double target, RevBulkData data) {
        if (data == null) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        if (data.getAnalogInputValue(pot) / 1000.0 > minArmSensorValue && data.getAnalogInputValue(pot) / 1000.0 < maxArmSensorValue)
            arm.setPower(Range.clip(pGain * (data.getAnalogInputValue(pot) / 1000.0 - target), -0.4, 1));
        else
            arm.setPower(0);
        return Math.abs(slide.getPower()) < 0.3 && Math.abs(arm.getPower()) < 0.3;
    }

    public boolean setArm(double target, RevBulkData data) {
        if (data == null) {
            arm.setPower(0);
            slide.setPower(0);
            return false;
        }
        if (data.getAnalogInputValue(pot) / 1000.0 > minArmSensorValue && data.getAnalogInputValue(pot) / 1000.0 < maxArmSensorValue)
            arm.setPower(armController5(data.getAnalogInputValue(pot) / 1000.0, target, data.getMotorCurrentPosition(slide)));
        else
            arm.setPower(0);
        return Math.abs(arm.getPower()) < 0.3;
    }

    public void startLiftUp() {
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        lift.setTargetPosition(liftTargetPosition);
    }

    public void startLiftDown() {
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        lift.setTargetPosition(0);
    }
}
