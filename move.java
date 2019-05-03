package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.robotconfig.bLeftMotor;
import static org.firstinspires.ftc.teamcode.robotconfig.bRightMotor;
import static org.firstinspires.ftc.teamcode.robotconfig.debugMode;
import static org.firstinspires.ftc.teamcode.robotconfig.fLeftMotor;
import static org.firstinspires.ftc.teamcode.robotconfig.fRightMotor;
import static org.firstinspires.ftc.teamcode.robotconfig.getAngularSpeed;
import static org.firstinspires.ftc.teamcode.robotconfig.getCurrentAngle;
import static org.firstinspires.ftc.teamcode.robotconfig.move;

//import static org.firstinspires.ftc.teamcode.robotconfig.addlog;
//import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by mail2 on 6/12/2017.
 */

/***
 * move is a class with functions for getting the robot to move the drive motors
 * exclusive to things like bettermove and move
 */
public class move {
    //assume bettermove will still be applicable
    static int fLeftMotorTarget = 0;
    static int fRightMotorTarget = 0;
    static int bLeftMotorTarget = 0;
    static int bRightMotorTarget = 0;
    static int bettermoveMovementDeadzone = 10;
    static int bettermoveCompletionDeadzone = 50;

    /***
     * sets the customised target position of all the motors in pulses for user with bettermove
     * this adds the targets to the current position
     * probably won't work in a logical direction if less than 2 values are equal to zero
     *
     * @param forward pulses in the forward direction
     * @param right   pulses in sliding to the right (or left if negative)
     * @param spin    pulses in spinning clockwise
     */
    static void setMyMotorTargets(int forward, int right, int spin) {
        Log.d("Downquark7", String.format(Locale.ENGLISH, "setMyMotorTargets was called - f:r:s: %d : %d : %d", forward, right, spin));
        if (debugMode) return;
        if (areMotorsAtTargets()) {
            incrementMyMotorTargets(forward, right, spin);
        } else {
            fLeftMotorTarget = (fLeftMotor.getCurrentPosition() + forward + right + spin);
            fRightMotorTarget = (fRightMotor.getCurrentPosition() + forward - right - spin);
            bLeftMotorTarget = (bLeftMotor.getCurrentPosition() + forward - right + spin);
            bRightMotorTarget = (bRightMotor.getCurrentPosition() + forward + right - spin);
        }
    }

    static void incrementMyMotorTargets(int forward, int right, int spin) {
        Log.d("Downquark7", String.format(Locale.ENGLISH, "incrementMyMotorTargets was called - f:r:s: %d : %d : %d", forward, right, spin));
        if (debugMode) return;
        fLeftMotorTarget += forward + right + spin;
        fRightMotorTarget += forward - right - spin;
        bLeftMotorTarget += forward - right + spin;
        bRightMotorTarget += forward + right - spin;
    }

    static void setMyMotorTargets(int fLeft, int fRight, int bLeft, int bRight) {
        Log.d("Downquark7", String.format(Locale.ENGLISH, "setMyMotorTargets was called - fL:fR:bL:bR %d : %d : %d : %d", fLeft, fRight, bLeft, bRight));
        if (debugMode) return;
        if (areMotorsAtTargets()) {
            incrementMyMotorTargets(fLeft, fRight, bLeft, bRight);
        } else {
            fLeftMotorTarget = (fLeftMotor.getCurrentPosition() + fLeft);
            fRightMotorTarget = (fRightMotor.getCurrentPosition() + fRight);
            bLeftMotorTarget = (bLeftMotor.getCurrentPosition() + bLeft);
            bRightMotorTarget = (bRightMotor.getCurrentPosition() + bRight);
        }
    }

    static void incrementMyMotorTargets(int fLeft, int fRight, int bLeft, int bRight) {
        Log.d("Downquark7", String.format(Locale.ENGLISH, "setMyMotorTargets was called - fL:fR:bL:bR %d : %d : %d : %d", fLeft, fRight, bLeft, bRight));
        if (debugMode) return;
        fLeftMotorTarget += fLeft;
        fRightMotorTarget += fRight;
        bLeftMotorTarget += bLeft;
        bRightMotorTarget += bRight;
    }

//    static void setMyMotorTargets(int fLeft, int fRight, int bLeft, int bRight, RevBulkData data) {
//        Log.d("Downquark7", String.format(Locale.ENGLISH, "setMyMotorTargets was called - fL:fR:bL:bR %d : %d : %d : %d", fLeft, fRight, bLeft, bRight));
//        if (debugMode) return;
//        fLeftMotorTarget = (data.getMotorCurrentPosition(fLeftMotor) + fLeft);
//        fRightMotorTarget = (data.getMotorCurrentPosition(fRightMotor) + fRight);
//        bLeftMotorTarget = (data.getMotorCurrentPosition(bLeftMotor) + bLeft);
//        bRightMotorTarget = (data.getMotorCurrentPosition(bRightMotor) + bRight);
//    }
//
//    static void setMyMotorTargets(int forward, int right, int spin, RevBulkData data) {
//        Log.d("Downquark7", String.format(Locale.ENGLISH, "setMyMotorTargets was called - f:r:s: %d : %d : %d", forward, right, spin));
//        if (debugMode) return;
//        fLeftMotorTarget = (data.getMotorCurrentPosition(fLeftMotor) + forward + right + spin);
//        fRightMotorTarget = (data.getMotorCurrentPosition(fRightMotor) + forward - right - spin);
//        bLeftMotorTarget = (data.getMotorCurrentPosition(bLeftMotor) + forward - right + spin);
//        bRightMotorTarget = (data.getMotorCurrentPosition(bRightMotor) + forward + right - spin);
//    }

    /**
     * Set motor targets to move robot a specific amount from the current position
     *
     * @param left  number of encoder pulses for motors on the left side
     * @param right number of encoder pulses for motors on the right side
     */
    static void setMyMotorTankTargets(int left, int right) {
//        addlog(dl, "robot", String.format(Locale.ENGLISH, "setMotorTankTargets was called - L:r: %d : %d", left, right));
        if (debugMode) return;
        fLeftMotorTarget = fLeftMotor.getCurrentPosition() + left;
        fRightMotorTarget = fRightMotor.getCurrentPosition() + right;
        bLeftMotorTarget = bLeftMotor.getCurrentPosition() + left;
        bRightMotorTarget = bRightMotor.getCurrentPosition() + right;
    }

    static boolean areMotorsAtTargets() {
        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());

        if (Math.abs(fLeftMotorPower) > 100)
            return false;
        if (Math.abs(fRightMotorPower) > 100)
            return false;
        if (Math.abs(bRightMotorPower) > 100)
            return false;
        if (Math.abs(bLeftMotorPower) > 100)
            return false;

        return true;
    }

    static double spinToHeadingController(double targetHeading) {
        double error = (getCurrentAngle() - targetHeading);
        double angSpeed = getAngularSpeed();
        double spin = error * 0.05 + 0.005 * angSpeed;
        return spin;
    }

    //make sure both of these have the same equation
    static boolean spinToHeading(double targetHeading) {
        double error = (getCurrentAngle() - targetHeading);
        double angSpeed = getAngularSpeed();
        double spin = error * 0.05 + 0.005 * angSpeed;
        move(0, 0, spin);
//        double previousMaxMotorPower = Math.max(Math.max(Math.abs(fLeftMotor.getPower()), Math.abs(bLeftMotor.getPower())), Math.max(Math.abs(fRightMotor.getPower()), Math.abs(bRightMotor.getPower())));
//        move(0,0, Range.clip(spin, -previousMaxMotorPower - 0.1, previousMaxMotorPower + 0.1));
        return Math.abs(error) < 1 && Math.abs(angSpeed) < 10;
    }

    /***
     * bettermove is a function to efficiently set the power values of all 4 drive train motors in a way to get all 4 motors to their targets at the same time as fast as possible
     * @return true if completed move
     */
    static boolean bettermove() {

//        addlog(dl, "robot", "bettermove was called");
        if (debugMode) return true;

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone)
            bLeftMotorPower = 0;

        double previousMaxMotorPower = Math.max(Math.max(Math.abs(fLeftMotor.getPower()), Math.abs(bLeftMotor.getPower())), Math.max(Math.abs(fRightMotor.getPower()), Math.abs(bRightMotor.getPower())));
        max /= Math.min(Math.min(0.7, previousMaxMotorPower + 0.1), Math.max(Math.abs(max / 1200), 0.1)); //should probably be the best because it doesn't jump to full speed instantly

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


//        return Math.abs(fLeftMotorPower + bLeftMotorPower) < bettermoveCompletionDeadzone * 2 && Math.abs(fRightMotorPower + bRightMotorPower) < bettermoveCompletionDeadzone * 2;
        boolean output = Math.abs(fLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(fRightMotorPower) < bettermoveCompletionDeadzone && Math.abs(bLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(bRightMotorPower) < bettermoveCompletionDeadzone;
        Log.v("Downquark7", String.format("bettermove(%b) powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f; %d, %d, %d, %d; %.0f, %.0f, %.0f, %.0f", output, fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower(), fLeftMotor.getCurrentPosition(), fRightMotor.getCurrentPosition(), bLeftMotor.getCurrentPosition(), bRightMotor.getCurrentPosition(), fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower));
        return output;
    }

    static boolean bettermoveNoSlowDown() {

//        addlog(dl, "robot", "bettermove was called");
        if (debugMode) return true;

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone)
            bLeftMotorPower = 0;

//        max /= Math.min(1, Math.max(Math.abs(max / 1200), 0.2));
//        max /= Math.min(0.7, Math.max(Math.abs(max / 500), 0.2));

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

        double completionDeadzone = 400;

//        return Math.abs(fLeftMotorPower + bLeftMotorPower) < bettermoveCompletionDeadzone * 2 && Math.abs(fRightMotorPower + bRightMotorPower) < bettermoveCompletionDeadzone * 2;
        boolean output = Math.abs(fLeftMotorPower) < completionDeadzone && Math.abs(fRightMotorPower) < completionDeadzone && Math.abs(bLeftMotorPower) < completionDeadzone && Math.abs(bRightMotorPower) < completionDeadzone;
        Log.v("Downquark7", String.format("bettermove(%b) powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f; %d, %d, %d, %d; %.0f, %.0f, %.0f, %.0f", output, fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower(), fLeftMotor.getCurrentPosition(), fRightMotor.getCurrentPosition(), bLeftMotor.getCurrentPosition(), bRightMotor.getCurrentPosition(), fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower));
        return output;
    }

    static boolean bettermove(double power) {

//        addlog(dl, "robot", "bettermove was called");
        if (debugMode) return true;

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone)
            bLeftMotorPower = 0;

        double previousMaxMotorPower = Math.max(Math.max(Math.abs(fLeftMotor.getPower()), Math.abs(bLeftMotor.getPower())), Math.max(Math.abs(fRightMotor.getPower()), Math.abs(bRightMotor.getPower())));
        max /= Math.min(Math.min(power, previousMaxMotorPower + 0.1), Math.max(Math.abs(max / 1200), 0.2)); //should probably be the best because it doesn't jump to full speed instantly

//        max /= Math.min(0.7, Math.max(Math.abs(max / 500), 0.2));

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


//        return Math.abs(fLeftMotorPower + bLeftMotorPower) < bettermoveCompletionDeadzone * 2 && Math.abs(fRightMotorPower + bRightMotorPower) < bettermoveCompletionDeadzone * 2;
        boolean output = Math.abs(fLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(fRightMotorPower) < bettermoveCompletionDeadzone && Math.abs(bLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(bRightMotorPower) < bettermoveCompletionDeadzone;
        Log.v("Downquark7", String.format("bettermove(%b) powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f; %d, %d, %d, %d; %.0f, %.0f, %.0f, %.0f", output, fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower(), fLeftMotor.getCurrentPosition(), fRightMotor.getCurrentPosition(), bLeftMotor.getCurrentPosition(), bRightMotor.getCurrentPosition(), fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower));
        return output;
    }

    static boolean bettermoveRampUp() {

//        addlog(dl, "robot", "bettermove was called");
        if (debugMode) return true;

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone)
            bLeftMotorPower = 0;

        double previousMaxMotorPower = Math.max(Math.max(Math.abs(fLeftMotor.getPower()), Math.abs(bLeftMotor.getPower())), Math.max(Math.abs(fRightMotor.getPower()), Math.abs(bRightMotor.getPower())));


        max /= Math.min(Math.min(0.7, previousMaxMotorPower + 0.1), Math.max(Math.abs(max / 1200), 0.1));
//        max /= Math.min(0.7, Math.max(Math.abs(max / 500), 0.2));

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


//        return Math.abs(fLeftMotorPower + bLeftMotorPower) < bettermoveCompletionDeadzone * 2 && Math.abs(fRightMotorPower + bRightMotorPower) < bettermoveCompletionDeadzone * 2;
        boolean output = Math.abs(fLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(fRightMotorPower) < bettermoveCompletionDeadzone && Math.abs(bLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(bRightMotorPower) < bettermoveCompletionDeadzone;
        Log.v("Downquark7", String.format("bettermove(%b) powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f; %d, %d, %d, %d; %.0f, %.0f, %.0f, %.0f", output, fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower(), fLeftMotor.getCurrentPosition(), fRightMotor.getCurrentPosition(), bLeftMotor.getCurrentPosition(), bRightMotor.getCurrentPosition(), fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower));
        return output;
    }

    static boolean bettermoveNoRamp() {

//        addlog(dl, "robot", "bettermove was called");
        if (debugMode) return true;

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone)
            bLeftMotorPower = 0;

//        double previousMaxMotorPower = Math.max(Math.max(Math.abs(fLeftMotor.getPower()), Math.abs(bLeftMotor.getPower())), Math.max(Math.abs(fRightMotor.getPower()), Math.abs(bRightMotor.getPower())));

//        max /= Math.min(Math.min(1, previousMaxMotorPower + 0.01), Math.max(Math.abs(max / 1200), 0.2));
//        max /= Math.min(0.7, Math.max(Math.abs(max / 500), 0.2));

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


//        return Math.abs(fLeftMotorPower + bLeftMotorPower) < bettermoveCompletionDeadzone * 2 && Math.abs(fRightMotorPower + bRightMotorPower) < bettermoveCompletionDeadzone * 2;
        boolean output = Math.abs(fLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(fRightMotorPower) < bettermoveCompletionDeadzone && Math.abs(bLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(bRightMotorPower) < bettermoveCompletionDeadzone;
        Log.v("Downquark7", String.format("bettermove(%b) powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f; %d, %d, %d, %d; %.0f, %.0f, %.0f, %.0f", output, fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower(), fLeftMotor.getCurrentPosition(), fRightMotor.getCurrentPosition(), bLeftMotor.getCurrentPosition(), bRightMotor.getCurrentPosition(), fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower));
        return output;
    }

    /***
     * bettermove is a function to efficiently set the power values of all 4 drive train motors in a way to get all 4 motors to their targets at the same time as fast as possible
     * @return true if completed move
     */
//    static boolean bettermove(RevBulkData data) {
//
////        addlog(dl, "robot", "bettermove was called");
//        if (debugMode) return true;
//
//        double fLeftMotorPower = (fLeftMotorTarget - data.getMotorCurrentPosition(fLeftMotor));
//        double fRightMotorPower = (fRightMotorTarget - data.getMotorCurrentPosition(fRightMotor));
//        double bLeftMotorPower = (bLeftMotorTarget - data.getMotorCurrentPosition(bLeftMotor));
//        double bRightMotorPower = (bRightMotorTarget - data.getMotorCurrentPosition(bRightMotor));
//        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));
//
//        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone)
//            fLeftMotorPower = 0;
//        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone)
//            fRightMotorPower = 0;
//        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone)
//            bRightMotorPower = 0;
//        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone)
//            bLeftMotorPower = 0;
//
//        max /= Math.min(1, Math.max(Math.abs(max / 1200), 0.2));
////        max /= Math.min(0.7, Math.max(Math.abs(max / 500), 0.2));
//
//        if (max > 0) {
//            fLeftMotor.setPower(fLeftMotorPower / max);
//            fRightMotor.setPower(fRightMotorPower / max);
//            bLeftMotor.setPower(bLeftMotorPower / max);
//            bRightMotor.setPower(bRightMotorPower / max);
//        } else {
//            fLeftMotor.setPower(0);
//            fRightMotor.setPower(0);
//            bLeftMotor.setPower(0);
//            bRightMotor.setPower(0);
//        }
//
//
////        return Math.abs(fLeftMotorPower + bLeftMotorPower) < bettermoveCompletionDeadzone * 2 && Math.abs(fRightMotorPower + bRightMotorPower) < bettermoveCompletionDeadzone * 2;
//        boolean output = Math.abs(fLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(fRightMotorPower) < bettermoveCompletionDeadzone && Math.abs(bLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(bRightMotorPower) < bettermoveCompletionDeadzone;
//        Log.v("Downquark7", String.format("bettermove(%b) powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f; %d, %d, %d, %d; %.0f, %.0f, %.0f, %.0f", output, fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower(), data.getMotorCurrentPosition(fLeftMotor), data.getMotorCurrentPosition(fRightMotor), data.getMotorCurrentPosition(bLeftMotor), data.getMotorCurrentPosition(bRightMotor), fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower));
//        return output;
//    }
    static boolean bettermoveMattEdition() {

//        addlog(dl, "robot", "bettermoveMattEdition was called");
        if (debugMode) return true;

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        Log.d("Downquark7", String.format("bettermoveMattEdition initial powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f, %.2f", fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower, max));

        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone)
            bLeftMotorPower = 0;

        max /= Math.min(0.7, Math.max(Math.abs(max / 1200), 0.222));

        Log.d("Downquark7", String.format("bettermoveMattEdition processed powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f, %.2f", fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower, max));

        double tempfLeftMotor = ((fLeftMotorPower + bLeftMotorPower) / max);
        double tempbLeftMotor = ((fLeftMotorPower + bLeftMotorPower) / max);
        double tempfRightMotor = ((fRightMotorPower + bRightMotorPower) / max);
        double tempbRightMotor = ((fRightMotorPower + bRightMotorPower) / max);

        double topLimit = 0.75;
        double negTopLimit = -0.75;

        if (tempfLeftMotor < negTopLimit) tempfLeftMotor = negTopLimit;
        if (tempbLeftMotor < negTopLimit) tempbLeftMotor = negTopLimit;
        if (tempfRightMotor < negTopLimit) tempfRightMotor = negTopLimit;
        if (tempbRightMotor < negTopLimit) tempbRightMotor = negTopLimit;

        if (tempfLeftMotor > topLimit) tempfLeftMotor = topLimit;
        if (tempbLeftMotor > topLimit) tempbLeftMotor = topLimit;
        if (tempfRightMotor > topLimit) tempfRightMotor = topLimit;
        if (tempbRightMotor > topLimit) tempbRightMotor = topLimit;

        double scaleFactor = 0.75;

        tempfLeftMotor *= scaleFactor;
        tempbLeftMotor *= scaleFactor;
        tempfRightMotor *= scaleFactor;
        tempbRightMotor *= scaleFactor;


        if (max > 0) {
            fLeftMotor.setPower(tempfLeftMotor);
            bLeftMotor.setPower(tempbLeftMotor);
            fRightMotor.setPower(tempfRightMotor);
            bRightMotor.setPower(tempbRightMotor);
        } else {
            fLeftMotor.setPower(0);
            fRightMotor.setPower(0);
            bLeftMotor.setPower(0);
            bRightMotor.setPower(0);
        }

        Log.d("Downquark7", String.format("bettermoveMattEdition final powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f; %d, %d, %d, %d; %.0f, %.0f, %.0f, %.0f", fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower(), fLeftMotor.getCurrentPosition(), fRightMotor.getCurrentPosition(), bLeftMotor.getCurrentPosition(), bRightMotor.getCurrentPosition(), fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower));

        return Math.abs(fLeftMotorPower + bLeftMotorPower) < bettermoveCompletionDeadzone * 2 && Math.abs(fRightMotorPower + bRightMotorPower) < bettermoveCompletionDeadzone * 2;
    }

    static boolean bettermovejjkdEdition() {

        Log.d("Downquark7", "bettermovejjkdEdition called...");

        if (debugMode) return true;

        double fLeftMotorToTravel = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorToTravel = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorToTravel = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorToTravel = (bRightMotorTarget - bRightMotor.getCurrentPosition());

        double maxDistanceToTravel = Math.max(Math.max(Math.abs(fLeftMotorToTravel), Math.abs(bLeftMotorToTravel)), Math.max(Math.abs(fRightMotorToTravel), Math.abs(bRightMotorToTravel)));

        Log.d("Downquark7", String.format("bettermovejjkdEdition initial Distance to Travel are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f, %.2f", fLeftMotorToTravel, fRightMotorToTravel, bLeftMotorToTravel, bRightMotorToTravel, maxDistanceToTravel));

        if (Math.abs(fLeftMotorToTravel) < bettermoveMovementDeadzone)
            fLeftMotorToTravel = 0;
        if (Math.abs(fRightMotorToTravel) < bettermoveMovementDeadzone)
            fRightMotorToTravel = 0;
        if (Math.abs(bRightMotorToTravel) < bettermoveMovementDeadzone)
            bRightMotorToTravel = 0;
        if (Math.abs(bLeftMotorToTravel) < bettermoveMovementDeadzone)
            bLeftMotorToTravel = 0;

        double tempfLeftMotor = ((fLeftMotorToTravel + bLeftMotorToTravel) / 2);
        double tempbLeftMotor = ((fLeftMotorToTravel + bLeftMotorToTravel) / 2);
        double tempfRightMotor = ((fRightMotorToTravel + bRightMotorToTravel) / 2);
        double tempbRightMotor = ((fRightMotorToTravel + bRightMotorToTravel) / 2);

        double topLimit = 0.75;
        double bottomLimit = 0.15;

        if (tempfLeftMotor > topLimit) tempfLeftMotor = topLimit;
        if (tempbLeftMotor > topLimit) tempbLeftMotor = topLimit;
        if (tempfRightMotor > topLimit) tempfRightMotor = topLimit;
        if (tempbRightMotor > topLimit) tempbRightMotor = topLimit;

        if (tempfLeftMotor < bottomLimit) tempfLeftMotor = bottomLimit;
        if (tempbLeftMotor < bottomLimit) tempbLeftMotor = bottomLimit;
        if (tempfRightMotor < bottomLimit) tempfRightMotor = bottomLimit;
        if (tempbRightMotor < bottomLimit) tempbRightMotor = bottomLimit;

        if (maxDistanceToTravel > bettermoveMovementDeadzone) {
            fLeftMotor.setPower(tempfLeftMotor);
            bLeftMotor.setPower(tempbLeftMotor);
            fRightMotor.setPower(tempfRightMotor);
            bRightMotor.setPower(tempbRightMotor);
        } else {
            fLeftMotor.setPower(0);
            fRightMotor.setPower(0);
            bLeftMotor.setPower(0);
            bRightMotor.setPower(0);
        }

      /*  Log.d("Downquark7", String.format( "bettermovejjkdEdition exiting powers are fl:fr:bl:br, "
                Log.d("Downquark7", String.format( "bettermovejjkdEdition exiting powers are fl:fr:bl:br, " +
                "%.2f, %.2f, %.2f, %.2f; %d, %d, %d, %d; %.0f, %.0f, %.0f, %.0f", fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower(), fLeftMotor.getCurrentPosition(), fRightMotor.getCurrentPosition(), bLeftMotor.getCurrentPosition(), bRightMotor.getCurrentPosition(), fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower));
*/
        return Math.abs(fLeftMotorToTravel + bLeftMotorToTravel) < bettermoveCompletionDeadzone * 2 && Math.abs(fRightMotorToTravel + bRightMotorToTravel) < bettermoveCompletionDeadzone * 2;
    }

    static boolean bettermoveSlowEdition() {

//        addlog(dl, "robot", "bettermoveSlowEdition was called");
        if (debugMode) return true;

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone)
            bLeftMotorPower = 0;

        max /= 0.08;

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

        Log.d("Downquark7", String.format("bettermove powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f", fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower()));

        return Math.abs(fLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(fRightMotorPower) < bettermoveCompletionDeadzone && Math.abs(bLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(bRightMotorPower) < bettermoveCompletionDeadzone;
    }

    static boolean bettermoveFastEdition() {

//        addlog(dl, "robot", "bettermoveFastEdition was called");
        if (debugMode) return true;

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone * 3)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone * 3)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone * 3)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone * 3)
            bLeftMotorPower = 0;

//        max /= Math.min(0.7, Math.max(Math.abs(max / 500), 0.2));
        max /= 0.7;

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

        Log.d("Downquark7", String.format("bettermoveFastEdition powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f", fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower()));

        return Math.abs(fLeftMotorPower + bLeftMotorPower) < bettermoveCompletionDeadzone * 3 && Math.abs(fRightMotorPower + bRightMotorPower) < bettermoveCompletionDeadzone * 3;
//        return Math.abs(fLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(fRightMotorPower) < bettermoveCompletionDeadzone && Math.abs(bLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(bRightMotorPower) < bettermoveCompletionDeadzone;
    }

    static boolean bettermoveIntakeEdition() {

//        addlog(dl, "robot", "bettermoveIntakeEdition was called");
        if (debugMode) return true;

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone * 3)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone * 3)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone * 3)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone * 3)
            bLeftMotorPower = 0;

//        max /= Math.min(0.7, Math.max(Math.abs(max / 500), 0.2));
        max /= 0.4;

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

        Log.d("Downquark7", String.format("bettermove powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f", fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower()));

        return Math.abs(fLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(fRightMotorPower) < bettermoveCompletionDeadzone && Math.abs(bLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(bRightMotorPower) < bettermoveCompletionDeadzone;
    }

    static boolean gyromove(double targetHeading) {

//        addlog(dl, "robot", "bettermove was called");

        double spin = Range.clip(spinToHeadingController(targetHeading), -0.5, 0.5);

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        fLeftMotorPower += spin * Math.min(bettermoveCompletionDeadzone, max);
        fRightMotorPower += -spin * Math.min(bettermoveCompletionDeadzone, max);
        bLeftMotorPower += spin * Math.min(bettermoveCompletionDeadzone, max);
        bRightMotorPower += -spin * Math.min(bettermoveCompletionDeadzone, max);

        max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone)
            bLeftMotorPower = 0;

        double previousMaxMotorPower = Math.max(Math.max(Math.abs(fLeftMotor.getPower()), Math.abs(bLeftMotor.getPower())), Math.max(Math.abs(fRightMotor.getPower()), Math.abs(bRightMotor.getPower())));
        max /= Math.min(Math.min(0.7, previousMaxMotorPower + 0.1), Math.max(Math.abs(max / 1200), 0.1)); //should probably be the best because it doesn't jump to full speed instantly

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

        Log.d("Downquark7", String.format("bettermove powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f", fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower()));

        return Math.abs(fLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(fRightMotorPower) < bettermoveCompletionDeadzone && Math.abs(bLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(bRightMotorPower) < bettermoveCompletionDeadzone;
    }

    /***
     * bettermove is a function to efficiently set the power values of all 4 drive train motors in a way to get all 4 motors to their targets at the same time as fast as possible
     * @param maxpower max power level
     * @param minpower min power level
     * @return true if completed move
     */
    static boolean bettermove(double maxpower, double minpower) {

//        addlog(dl, "robot", String.format(Locale.ENGLISH, "bettermove was called with powers, %.2f, %.2f", maxpower, minpower));
        if (debugMode) return true;

        double fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        double fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        double bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        double bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        double max = Math.max(Math.max(Math.abs(fLeftMotorPower), Math.abs(bLeftMotorPower)), Math.max(Math.abs(fRightMotorPower), Math.abs(bRightMotorPower)));

        if (Math.abs(fLeftMotorPower) < bettermoveMovementDeadzone)
            fLeftMotorPower = 0;
        if (Math.abs(fRightMotorPower) < bettermoveMovementDeadzone)
            fRightMotorPower = 0;
        if (Math.abs(bRightMotorPower) < bettermoveMovementDeadzone)
            bRightMotorPower = 0;
        if (Math.abs(bLeftMotorPower) < bettermoveMovementDeadzone)
            bLeftMotorPower = 0;

        max /= Math.min(maxpower, Math.max(Math.abs(max / 3000), minpower));

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

//        addlog(dl, "robot", String.format(Locale.ENGLISH, "bettermove powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f", fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower()));

        return Math.abs(fLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(fRightMotorPower) < bettermoveCompletionDeadzone && Math.abs(bLeftMotorPower) < bettermoveCompletionDeadzone && Math.abs(bRightMotorPower) < bettermoveCompletionDeadzone;
    }

    /***
     * for debugging bettermove
     *
     * @return (fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower)
     */
    static String getErrors() {
        int fLeftMotorPower = (fLeftMotorTarget - fLeftMotor.getCurrentPosition());
        int fRightMotorPower = (fRightMotorTarget - fRightMotor.getCurrentPosition());
        int bLeftMotorPower = (bLeftMotorTarget - bLeftMotor.getCurrentPosition());
        int bRightMotorPower = (bRightMotorTarget - bRightMotor.getCurrentPosition());
        return String.format(Locale.ENGLISH, "%d, %d, %d, %d", fLeftMotorPower, fRightMotorPower, bLeftMotorPower, bRightMotorPower);
    }

}
