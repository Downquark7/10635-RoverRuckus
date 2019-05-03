package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREV;

import static org.firstinspires.ftc.teamcode.autoPaths.centerPath1;
import static org.firstinspires.ftc.teamcode.autoPaths.centerPath1c;
import static org.firstinspires.ftc.teamcode.autoPaths.centerPath2;
import static org.firstinspires.ftc.teamcode.autoPaths.centerPath2c;
import static org.firstinspires.ftc.teamcode.autoPaths.leftPath1;
import static org.firstinspires.ftc.teamcode.autoPaths.leftPath2;
import static org.firstinspires.ftc.teamcode.autoPaths.markerTrajectory;
import static org.firstinspires.ftc.teamcode.autoPaths.returnFromMarkerCenterc;
import static org.firstinspires.ftc.teamcode.autoPaths.returnFromMarkerLeft;
import static org.firstinspires.ftc.teamcode.autoPaths.returnFromMarkerRight;
import static org.firstinspires.ftc.teamcode.autoPaths.rightPath1;
import static org.firstinspires.ftc.teamcode.autoPaths.rightPath2;
import static org.firstinspires.ftc.teamcode.autoPaths.startpos;
import static org.firstinspires.ftc.teamcode.robotconfig.initTfod;
import static org.firstinspires.ftc.teamcode.robotconfig.initVuforia;
import static org.firstinspires.ftc.teamcode.robotconfig.lastGoldLocation;
import static org.firstinspires.ftc.teamcode.robotconfig.tfod;
import static org.firstinspires.ftc.teamcode.teleOp.armPositionFinder;
import static org.firstinspires.ftc.teamcode.teleOp.slidePositions;


@Autonomous
@Disabled
public class autoDepot extends LinearOpMode {
    SampleMecanumDriveBase drive;
    auxillaryDevices aux;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        drive = new SampleMecanumDriveREV(hardwareMap);
        aux = new auxillaryDevices(hardwareMap);

        autoPaths.generatePaths();
        Pose2d depotStartpos = new Pose2d(13, -14, Math.toRadians(-45));
        drive.setPoseEstimate(depotStartpos);

        initVuforia(hardwareMap);
        initTfod(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        aux.startLiftUp();
        aux.timer.reset();
        tfod.activate();
        while (!isStopRequested() && !aux.isUnhungWithArm()) {
        }
        tfod.deactivate();

        //############################### beginning of marker ###############################

        drive.followTrajectory(drive.trajectoryBuilder()
                .turn(Math.toRadians(15))
                .forward(2)
                .turnTo(Math.toRadians(-45))
                .build());

        while (!isStopRequested() && drive.isFollowingTrajectory()) {
            Pose2d currentPose = drive.getPoseEstimate();
            aux.setArm(armTarget.horizontal, slidePositions[0]);
            sendPositionalErrorTelemetry();
            drive.update();
        }

        aux.startLiftDown();

        drive.followTrajectory(drive.trajectoryBuilder()
                .forward(15)
                .build());

        while (!isStopRequested() && drive.isFollowingTrajectory()) {
            Pose2d currentPose = drive.getPoseEstimate();
            aux.setArm(armTarget.horizontal, slidePositions[3]);
            sendPositionalErrorTelemetry();
            drive.update();
        }

        aux.reverseIntake();

        drive.followTrajectory(drive.trajectoryBuilder()
                .waitFor(0.2)
                .back(15)
                .build());

        while (!isStopRequested() && drive.isFollowingTrajectory()) {
            Pose2d currentPose = drive.getPoseEstimate();
            aux.setArm(armTarget.horizontal, slidePositions[0]);
            sendPositionalErrorTelemetry();
            drive.update();
        }

        //############################### beginning of minerals ###############################

        aux.activateIntake();

        switch (lastGoldLocation) {
            case LEFT:
                drive.followTrajectory(drive.trajectoryBuilder()
                        .turn(Math.toRadians(28))
                        .forward(22)
                        .build());
                break;
            case CENTER:
                drive.followTrajectory(drive.trajectoryBuilder()
                        .turn(Math.toRadians(0))
                        .forward(22)
                        .build());
                break;
            case RIGHT:
                drive.followTrajectory(drive.trajectoryBuilder()
                        .turn(Math.toRadians(-25))
                        .forward(22)
                        .build());
                break;
            case UNKNOWN:
                drive.followTrajectory(drive.trajectoryBuilder()
                        .turn(Math.toRadians(0))
                        .forward(22)
                        .build());
                break;
        }

        while (!isStopRequested() && drive.isFollowingTrajectory()) {
            aux.setArm(2.4, 2 * slidePositions[0]);
            sendPositionalErrorTelemetry();
            drive.update();
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .back(22)
                .turnTo(Math.toRadians(-45))
                .build());

        while (!isStopRequested() && drive.isFollowingTrajectory()) {
            aux.setArm(armTarget.slightUp, slidePositions[0]);
            sendPositionalErrorTelemetry();
            drive.update();
        }

        aux.timer.reset();
        while (!isStopRequested() && aux.timer.seconds() < 1) {
            aux.setArm(armTarget.depotSideY, slidePositions[1]);
            sendPositionalErrorTelemetry();
            drive.update();
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .turn(Math.toRadians(10))
                .splineTo(new Pose2d(62, 13, Math.toRadians(90)))
                .build());

        while (!isStopRequested() && drive.isFollowingTrajectory()) {
            aux.setArm(armTarget.hh, slidePositions[0]);
            sendPositionalErrorTelemetry();
            drive.update();
        }

        while (!isStopRequested() && !aux.setArm(armTarget.horizontal, slidePositions[1])) {
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
//        while (!isStopRequested() && drive.isFollowingTrajectory()) {
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
    }
}
