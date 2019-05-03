package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.measurements.pi;
import static org.firstinspires.ftc.teamcode.measurements.ppr;
import static org.firstinspires.ftc.teamcode.measurements.wheelDiagonal;
import static org.firstinspires.ftc.teamcode.measurements.wheelDiameter;
import static org.firstinspires.ftc.teamcode.robotconfig.isMotorBusy;
import static org.firstinspires.ftc.teamcode.robotconfig.resetMotorEncoders;
import static org.firstinspires.ftc.teamcode.robotconfig.theLinearOpMode;

//import static org.firstinspires.ftc.teamcode.robotconfig.addlog;
//import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by mail2 on 10/31/2016.
 * Project: ftc_app_for_2016_robot
 */

/***
 * library to use math for precise movement, primarily for autonomous
 */
class preciseMovement {

    private static ElapsedTime runtime = new ElapsedTime();

    /***
     * uses math to convert an amount of degrees into how much distance the wheel spins
     *
     * @param degrees amount of degrees to spin clockwise, negative if backwards
     * @return distance a wheel will spin in mm
     */
    static double spin2mm(double degrees) {
        return (degrees / 360) * (wheelDiagonal * pi);
    }

    /***
     * uses math to convert a distance in millimeters to the number of pulses the motor should generate to go that far
     *
     * @param mm a distance in millimeters
     * @return number of pulses generated
     */
    static int mm2pulses(double mm) {
        return (int) ((ppr / (pi * wheelDiameter)) * mm);//moved sprocket change to measurements.ppr
    }

//    /***
//     * function takes measurements in mm to move the robot
//     *
//     * @param forward amount to move forward in mm
//     * @param right   amount to move right in mm
//     * @param spin    amount to spin degrees clockwise
//     * @param timeout time to take doing movement
//     */
//    static void move(double forward, double right, double spin, double timeout) {
//        addlog(dl, "pm.move", String.format(Locale.ENGLISH, "called with, forward %.2f, right %.2f, spin %.2f, timeout", forward, right, spin, timeout));
//        setMotorTargets(mm2pulses(forward), mm2pulses(right), mm2pulses(spin2mm(spin)));
//        waitForMotors(forward, right, spin, timeout);
//    }
//
//    static void movesegments(double forward, double right, double spin, double timeout, int segments) {
//        for (int i = 0; i < segments; i++) {
//            move(forward / segments, right / segments, spin / segments, timeout / segments);
//        }
//    }

//    private static void waitForMotors(double forward, double right, double spin, double timeout) {
//        addlog(dl, "pm.waitforMotors with telemetry", "called with right:" + String.format(Locale.ENGLISH, "%.2f", right) + " and spin:" + String.format(Locale.ENGLISH, "%.2f", spin) + " and forward:" + String.format(Locale.ENGLISH, "%.2f", forward));
//        runtime.reset();
//        while (robot.isMotorBusy() && (runtime.seconds() < timeout) && theLinearOpMode.opModeIsActive()) {
//            telemetry.addData("Path0", "Go to %f in : %f in : %f in", (forward / measurements.mmPerInch), (right / measurements.mmPerInch), spin);
//            telemetry.addData("Path0", "Go to %f in : %f in : %f in", (forward / measurements.mmPerInch), (right / measurements.mmPerInch), spin);
//            telemetry.addData("Path1", "At  %7d :%7d :%7d :%7d", robot.fLeftMotor.getCurrentPosition(), robot.fRightMotor.getCurrentPosition(), robot.bLeftMotor.getCurrentPosition(), robot.bRightMotor.getCurrentPosition());
//            telemetry.addData("Path2", "End %7d :%7d :%7d :%7d", robot.fLeftMotor.getTargetPosition(), robot.fRightMotor.getTargetPosition(), robot.bLeftMotor.getTargetPosition(), robot.bRightMotor.getTargetPosition());
//            telemetry.update();
//            Thread.yield();
//        }
//        // Need to gracefully exit loop here as we have either timed out or a stop has been requested
//
//        if (runtime.seconds() >= timeout) {
//            addlog(dl, "pm.waitforMotors", "timed out: " + String.format(Locale.ENGLISH, "%.2f", runtime.seconds()));
//        } else if (!theLinearOpMode.opModeIsActive()) {
//            robot.resetMotorEncoders();
//            addlog(dl, "pm.waitforMotors", "Stop of opmode was requested");
//        } else {
//            addlog(dl, "pm.waitforMotors", "exited move normally");
//        }
//    }

    static void waitForMotors(double forward, double right, double spin, double timeout) {
//        addlog(dl, "pm.waitForMotors", String.format(Locale.ENGLISH, "called with, forward %.2f, right %.2f, spin %.2f, timeout", forward, right, spin, timeout));
        runtime.reset();
        while (isMotorBusy() && (runtime.seconds() < timeout) && theLinearOpMode.opModeIsActive()) {
            //addlog(dl, "pm.waitforMotors", "looping");
            Thread.yield();
        }
        // Need to gracefully exit loop here as we have either timed out or a stop has been requested

        //addlog(dl, "pm.waitforMotors", "done looping");
        //addlog(dl, "pm.waitforMotors", "runtime.seconds() is " + String.format(Locale.ENGLISH, "%.2f", runtime.seconds()));
        //addlog(dl, "pm.waitforMotors", "timeout is " + String.format(Locale.ENGLISH, "%.2f", timeout));

        if (runtime.seconds() >= timeout) {
//            addlog(dl, "pm.waitforMotors", "timed out: " + String.format(Locale.ENGLISH, "%.2f", runtime.seconds()));
        } else if (!theLinearOpMode.opModeIsActive()) {
            resetMotorEncoders();
//            addlog(dl, "pm.waitforMotors", "Stop of opmode was requested");
        } else {
//            addlog(dl, "pm.waitforMotors", "exited move normally");
        }
    }

//    /***
//     * is supposed to square up the robot to the nearest 45 degrees
//     */
//    static void automaticSquareUp() {
//        addlog(dl, "pm.automaticSquareUp with telemetry", "called");
//        move(0, 0, (getCurrentAngle() - (Math.round(getCurrentAngle() / 45) * 45)), 1);
//    }

}
