package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREV;

/*
 * Op mode for tuning follower PID coefficients. This is the final step in the tuning process.
 */
@Autonomous
@Disabled
public class autoPaths extends LinearOpMode {
    SampleMecanumDriveBase drive;
    FtcDashboard dashboard;

    static Pose2d startpos;
    static Pose2d beforeSamplePos;

    static Trajectory markerTrajectory;
    static Trajectory returnFromMarker;
    static Trajectory goToDoubleSample;
    static Trajectory returnFromMarkerAfterDoubleSample;
    static Trajectory leftPath1;
    static Trajectory leftPath2;
    static Trajectory centerPath1;
    static Trajectory centerPath2;
    static Trajectory rightPath1;
    static Trajectory rightPath2;

    static Trajectory leftPath1b;
    static Trajectory leftPath2b;
    static Trajectory centerPath1b;
    static Trajectory centerPath2b;
    static Trajectory rightPath1b;
    static Trajectory rightPath2b;

    static Trajectory leftPath1c;
    static Trajectory leftPath2c;
    static Trajectory centerPath1c;
    static Trajectory centerPath2c;
    static Trajectory rightPath1c;
    static Trajectory rightPath2c;

    static Trajectory returnFromMarkerLeft;
    static Trajectory returnFromMarkerCenterc;
    static Trajectory returnFromMarkerRight;

    static Pose2d centerMineralPose;
    static Pose2d leftMineralPose;
    static Pose2d rightMineralPose;

    static Pose2d centerMineralPoseb;
    static Pose2d leftMineralPoseb;
    static Pose2d rightMineralPoseb;

    static Pose2d centerMineralPosec;
    static Pose2d leftMineralPosec;
    static Pose2d rightMineralPosec;

    static Pose2d craterPos;
    static Pose2d scorePos;


    public static void generatePoints() {
        startpos = new Pose2d(-14, -13, Math.toRadians(-135));
        beforeSamplePos = new Pose2d(-12.5, -18, Math.toRadians(-145));
        scorePos = new Pose2d(-12.5, -18, Math.toRadians(-147));
        craterPos = new Pose2d(-32,-22, Math.toRadians(-145));

        //drive through mineral close to crater as possible
        centerMineralPose = new Pose2d(-34, -34, Math.toRadians(-135));
        leftMineralPose = new Pose2d(-34 + 9, -34 - 9, Math.toRadians(-135));
        rightMineralPose = new Pose2d(-32 - 12, -32 + 12, Math.toRadians(-135));

        //pick up mineral by going straight to right before it
        leftMineralPoseb = new Pose2d(-22.93,-34.13,Math.toRadians(-115.02));
        centerMineralPoseb = new Pose2d(-27.15,-28.15,Math.toRadians(-145.21));
        rightMineralPoseb = new Pose2d(-34.08,-23.60,Math.toRadians(-173.29));

        //optimized drive through mineral that ends up further away but should have more speed
        leftMineralPosec = new Pose2d(-19.8,-34.8,Math.toRadians(-115.0));
        centerMineralPosec = new Pose2d(-28,-28,Math.toRadians(-142.2));
        rightMineralPosec = new Pose2d(-34.9,-19.8,Math.toRadians(-155.0));
    }

    public static void generatePaths() { //aka statelist on steroids but only for the drivetrain

        DriveConstraints constraints = new MecanumConstraints(DriveConstants.BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH);

        generatePoints();

        markerTrajectory = new TrajectoryBuilder(startpos, constraints)
                .turn(Math.toRadians(15))
                .beginComposite()
                .splineTo(new Pose2d(-12, -39, Math.toRadians(-65)))
                .splineTo(new Pose2d(12, -60, 0))
                .forward(8)
                .closeComposite()
                .build();

        returnFromMarker = new TrajectoryBuilder(new Pose2d(20, -60, 0), constraints)
                .waitFor(0.1)
                .reverse()
                .beginComposite()
                .splineTo(beforeSamplePos)
                .closeComposite()
                .build();

        goToDoubleSample = new TrajectoryBuilder(new Pose2d(36, -60, 0), constraints)
                .reverse()
                .forward(-24)
                .turnTo(Math.toRadians(45))
                .build();

        returnFromMarkerAfterDoubleSample = new TrajectoryBuilder(new Pose2d(12, -60, Math.toRadians(45)), constraints)
                .reverse()
                .beginComposite()
                .splineTo(new Pose2d(-12, -39, Math.toRadians(-65)))
                .splineTo(beforeSamplePos)
                .closeComposite()
                .build();

        leftPath1 = new TrajectoryBuilder(beforeSamplePos, constraints)
                .splineTo(leftMineralPose)
                .build();

        leftPath2 = new TrajectoryBuilder(leftMineralPose, constraints)
                .reverse()
                .splineTo(beforeSamplePos)
                .build();

        centerPath1 = new TrajectoryBuilder(beforeSamplePos, constraints)
                .splineTo(centerMineralPose)
                .build();

        centerPath2 = new TrajectoryBuilder(centerMineralPose, constraints)
                .reverse()
                .splineTo(beforeSamplePos)
                .build();

        rightPath1 = new TrajectoryBuilder(beforeSamplePos, constraints)
                .splineTo(rightMineralPose)
                .build();

        rightPath2 = new TrajectoryBuilder(rightMineralPose, constraints)
                .reverse()
                .splineTo(beforeSamplePos)
                .build();

        leftPath1b = new TrajectoryBuilder(beforeSamplePos, constraints)
                .splineTo(leftMineralPoseb)
                .build();

        leftPath2b = new TrajectoryBuilder(leftMineralPoseb, constraints)
                .reverse()
                .splineTo(beforeSamplePos)
                .build();

        centerPath1b = new TrajectoryBuilder(beforeSamplePos, constraints)
                .splineTo(centerMineralPoseb)
                .build();

        centerPath2b = new TrajectoryBuilder(centerMineralPoseb, constraints)
                .reverse()
                .splineTo(beforeSamplePos)
                .build();

        rightPath1b = new TrajectoryBuilder(beforeSamplePos, constraints)
                .splineTo(rightMineralPoseb)
                .build();

        rightPath2b = new TrajectoryBuilder(rightMineralPoseb, constraints)
                .reverse()
                .splineTo(beforeSamplePos)
                .build();

        leftPath1c = new TrajectoryBuilder(beforeSamplePos, constraints)
                .splineTo(leftMineralPosec)
                .build();

        leftPath2c = new TrajectoryBuilder(leftMineralPosec, constraints)
                .reverse()
                .splineTo(beforeSamplePos)
                .build();

        centerPath1c = new TrajectoryBuilder(beforeSamplePos, constraints)
                .splineTo(centerMineralPosec)
                .build();

        centerPath2c = new TrajectoryBuilder(centerMineralPosec, constraints)
                .reverse()
                .splineTo(beforeSamplePos)
                .build();

        rightPath1c = new TrajectoryBuilder(beforeSamplePos, constraints)
                .splineTo(rightMineralPosec)
                .build();

        returnFromMarkerLeft = new TrajectoryBuilder(new Pose2d(20, -60, 0), constraints)
                .reverse()
                .beginComposite()
                .splineTo(beforeSamplePos)
                .reverse()
                .splineTo(leftMineralPose)
                .closeComposite()
                .build();

        returnFromMarkerCenterc = new TrajectoryBuilder(new Pose2d(20, -60, 0), constraints)
                .reverse()
                .beginComposite()
                .splineTo(beforeSamplePos)
                .reverse()
                .splineTo(centerMineralPosec)
                .closeComposite()
                .build();

        returnFromMarkerRight = new TrajectoryBuilder(new Pose2d(20, -60, 0), constraints)
                .reverse()
                .beginComposite()
                .splineTo(beforeSamplePos)
                .reverse()
                .splineTo(rightMineralPose)
                .closeComposite()
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        drive = new SampleMecanumDriveREV(hardwareMap);

        generatePaths();

        waitForStart();

        if (isStopRequested()) return;

        move(markerTrajectory);
        move(returnFromMarker);
        move(leftPath1);
        move(leftPath2);
        move(centerPath1);
        move(centerPath2);
        move(rightPath1);
        move(rightPath2);
        move(centerPath1);

        Trajectory backToStart = drive.trajectoryBuilder()
                .reverse()
                .splineTo(startpos)
                .build();

        move(backToStart);

    }

    void move(Trajectory trajectory) {
        drive.followTrajectory(trajectory);

        while (!isStopRequested() && drive.isFollowingTrajectory()) {
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d error = drive.getFollowingError();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("xError", error.getX());
            packet.put("yError", error.getY());
            double headErr = error.getHeading();
            for (; headErr < -Math.PI; headErr += 2 * Math.PI) {
            }
            packet.put("headingError", headErr);
            dashboard.sendTelemetryPacket(packet);

            drive.update();
        }
    }
}
