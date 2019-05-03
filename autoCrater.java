package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREV;

import static org.firstinspires.ftc.teamcode.autoPaths.*;
import static org.firstinspires.ftc.teamcode.autoPaths.startpos;
import static org.firstinspires.ftc.teamcode.robotconfig.initTfod;
import static org.firstinspires.ftc.teamcode.robotconfig.initVuforia;
import static org.firstinspires.ftc.teamcode.robotconfig.lastGoldLocation;
import static org.firstinspires.ftc.teamcode.robotconfig.tfod;
import static org.firstinspires.ftc.teamcode.teleOp.slidePositions;


@Autonomous
@Disabled
public class autoCrater extends LinearOpMode {
    SampleMecanumDriveBase drive;
    auxillaryDevices aux;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        drive = new SampleMecanumDriveREV(hardwareMap);
        aux = new auxillaryDevices(hardwareMap);

        autoPaths.generatePaths();
        drive.setPoseEstimate(startpos);

        initVuforia(hardwareMap);
        initTfod(hardwareMap);

        telemetry.addData("lastGoldLocation", () -> lastGoldLocation);

        telemetry.addData("stuff", "yeah...");
        telemetry.update();
        while(!isStarted())
            idle();

        if (isStopRequested()) return;

        aux.startLiftUp();
        aux.timer.reset();
        tfod.activate();
        while (!isStopRequested() && opModeIsActive() && !aux.isUnhungWithArm()) {
            idle();
        }
        tfod.deactivate();

        //############################### beginning of marker ###############################

        drive.followTrajectory(markerTrajectory);

        while (!isStopRequested() && opModeIsActive() && drive.isFollowingTrajectory()) {
            Pose2d currentPose = drive.getPoseEstimate();
            if (currentPose.getX() > 0) {
                aux.setArmSlowly(armTarget.horizontal, slidePositions[3]);
            } else
                aux.setArm(armTarget.autohh, slidePositions[0]);
            sendPositionalErrorTelemetry();
            drive.update();
        }

        aux.reverseIntake();
        aux.startLiftDown();

        drive.followTrajectory(returnFromMarker);

        while (!isStopRequested() && opModeIsActive() && drive.isFollowingTrajectory()) {
            Pose2d currentPose = drive.getPoseEstimate();
//            if (currentPose.getX() > 0)
            aux.setArmSlowly(armTarget.vertical, slidePositions[0]);
//            else {
//                aux.slide.setPower(0);
//                aux.setArm(armTarget.vertical);
//            }
            sendPositionalErrorTelemetry();
            drive.update();
        }

        aux.lift.setPower(0);

        //############################### beginning of minerals ###############################

        aux.activateIntake();

        for (int i = 0; i < (lastGoldLocation == robotconfig.goldLocation.CENTER ? 3 : 2); i++) {

            switch (lastGoldLocation) {
                case LEFT:
                    drive.followTrajectory(leftPath1);
                    break;
                case CENTER:
                    if (i == 0)
                        drive.followTrajectory(centerPath1c);
                    else
                        drive.followTrajectory(centerPath1);
                    break;
                case RIGHT:
                    drive.followTrajectory(rightPath1);
                    break;
                case UNKNOWN:
                    if (i == 0)
                        drive.followTrajectory(centerPath1c);
                    else
                        drive.followTrajectory(centerPath1);
                    break;
            }

            if (lastGoldLocation == robotconfig.goldLocation.CENTER) {

                aux.activateIntake();

                while (!isStopRequested() && opModeIsActive() && drive.isFollowingTrajectory()) {
                    aux.setArm(i == 0 ? 2.5 : armTarget.autohh, i == 0 ? 2 * slidePositions[0] : slidePositions[0]);//change to these values to down more if picking up a cube is desired
                    sendPositionalErrorTelemetry();
                    drive.update();
                }

                if (i > 0)
                    while (!isStopRequested() && opModeIsActive() && !aux.setArm(armTarget.autohh, slidePositions[0])) {
                        sendPositionalErrorTelemetry();
                        drive.update();
                    }

                if (i > 0)
                    while (!isStopRequested() && opModeIsActive() && !aux.intakeMinerals((int) (0.6 * slidePositions[3] + slidePositions[3] * 0.2 * (i - 1)))) {
                        sendPositionalErrorTelemetry();
                        drive.update();
                    }

                //spit out extra minerals
                if (i > 0) {
                    while (!isStopRequested() && opModeIsActive() && !aux.setArm(armTarget.slightUp, slidePositions[0])) {
                        sendPositionalErrorTelemetry();
                        drive.update();
                    }
                    aux.reverseIntake();
                    sleep2(400);
                }

            } else {

                while (!isStopRequested() && opModeIsActive() && drive.isFollowingTrajectory()) {
                    aux.setArm(armTarget.autohh, slidePositions[0]);//change to these values to down more if picking up a cube is desired
                    sendPositionalErrorTelemetry();
                    drive.update();
                }

                aux.activateIntake();

                while (!isStopRequested() && opModeIsActive() && !aux.intakeMinerals((int) (0.6 * slidePositions[3] + slidePositions[3] * 0.2 * i))) {
                    sendPositionalErrorTelemetry();
                    drive.update();
                }

                //spit out extra minerals
                while (!isStopRequested() && opModeIsActive() && !aux.setArm(armTarget.slightUp, slidePositions[0])) {
                    sendPositionalErrorTelemetry();
                    drive.update();
                }

                aux.reverseIntake();
                sleep2(400);
            }

            aux.activateIntake();

            //go to scoring position
            switch (lastGoldLocation) {
                case LEFT:
                    drive.followTrajectory(leftPath2);
                    break;
                case CENTER:
                    if (i == 0)
                        drive.followTrajectory(centerPath2c);
                    else
                        drive.followTrajectory(centerPath2);
                    break;
                case RIGHT:
                    drive.followTrajectory(rightPath2);
                    break;
                case UNKNOWN:
                    if (i == 0)
                        drive.followTrajectory(centerPath1c);
                    else
                        drive.followTrajectory(centerPath1);
                    break;
            }

            while (!isStopRequested() && opModeIsActive() && drive.isFollowingTrajectory()) {
                aux.setArm(armTarget.hh, slidePositions[0]);
                sendPositionalErrorTelemetry();
                drive.update();
            }

            //score minerals
            aux.activateIntake();
            aux.timer.reset();
            while (!isStopRequested() && opModeIsActive() && aux.timer.seconds() < (i == 0 && lastGoldLocation == robotconfig.goldLocation.CENTER ? 0.8 : 1.2)) {
                aux.setArm(armTarget.teleOpY, slidePositions[5]);
                sendPositionalErrorTelemetry();
                drive.update();
            }
        }

        //############################### park at end ###############################
        switch (lastGoldLocation) {
            case LEFT:
                drive.followTrajectory(leftPath1);
                break;
            case CENTER:
                drive.followTrajectory(centerPath1);
                break;
            case RIGHT:
                drive.followTrajectory(rightPath1);
                break;
            case UNKNOWN:
                drive.followTrajectory(centerPath1);
                break;
        }

        while (!isStopRequested() && opModeIsActive() && drive.isFollowingTrajectory()) {
            aux.setArm(armTarget.slightUp, lastGoldLocation == robotconfig.goldLocation.CENTER ? slidePositions[1] : slidePositions[0]);
            sendPositionalErrorTelemetry();
            drive.update();
        }

        while (!isStopRequested() && opModeIsActive() && !aux.setArm(armTarget.horizontal, slidePositions[1])) {
            sendPositionalErrorTelemetry();
            drive.update();
        }

        aux.arm.setPower(0);
        aux.slide.setPower(0);
        drive.setMotorPowers(0, 0, 0, 0);
        aux.deactivateIntake();
    }

//    void move(Trajectory trajectory) {
//        drive.followTrajectory(trajectory);
//
//        while (!isStopRequested() && opModeIsActive() && drive.isFollowingTrajectory()) {
//            Pose2d currentPose = drive.getPoseEstimate();
//            Pose2d error = drive.getFollowingError();
//
//            TelemetryPacket packet = new TelemetryPacket();
////                Canvas fieldOverlay = packet.fieldOverlay();
////
//            packet.put("xError", error.getX());
//            packet.put("yError", error.getY());
//            double headErr = error.getHeading();
//            for (; headErr < -Math.PI; headErr += 2 * Math.PI) {
//            }
//            packet.put("headingError", headErr);
//
//            dashboard.sendTelemetryPacket(packet);
//
// sendPositionalErrorTelemetry();
// drive.update();
//        }
//    }

    void sendPositionalErrorTelemetry() {
        Pose2d error = drive.getFollowingError();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("xError", error.getX());
        packet.put("yError", error.getY());
        double headErr = error.getHeading();
        for (; headErr < -Math.PI; headErr += 2 * Math.PI) {
        }
        packet.put("headingError", headErr);
        dashboard.sendTelemetryPacket(packet);
        idle();
    }


    public void sleep2(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
        }
    }
}
