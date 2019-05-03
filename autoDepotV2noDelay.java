package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREV;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.robotconfig.debugMode;
import static org.firstinspires.ftc.teamcode.robotconfig.goldLocation.UNKNOWN;
import static org.firstinspires.ftc.teamcode.robotconfig.lastGoldLocation;
import static org.firstinspires.ftc.teamcode.robotconfig.tfod;
import static org.firstinspires.ftc.teamcode.stateslist.autoTimer;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.teleOp.armPositionFinder;
import static org.firstinspires.ftc.teamcode.teleOp.maxSlidePosition;
import static org.firstinspires.ftc.teamcode.teleOp.slidePositions;

@Autonomous
public class autoDepotV2noDelay extends OpMode {

    //make list of the states that are going to be run
    private List<state> list = new ArrayList<>();
    //init the array that is going to be used when actually running
    private state[] runlist;

    SampleMecanumDriveBase drive;
    auxillaryDevices aux;
//    FtcDashboard dashboard;

    @Override
    public void init() {

        asyncVuforiaInit R1 = new asyncVuforiaInit(hardwareMap);
        R1.start();

//        dashboard = FtcDashboard.getInstance();
        drive = new SampleMecanumDriveREV(hardwareMap);
        aux = new auxillaryDevices(hardwareMap);

        Pose2d depotStartpos = new Pose2d(13, -14, Math.toRadians(-45));
        drive.setPoseEstimate(depotStartpos);

        telemetry.addData("state", () -> runlist[currentState].name);
        telemetry.addData("mineral order", () -> lastGoldLocation);

        autoTimer.reset();

        //start with the first state selected
        int selectedstate = 0;
        //set variable selecting to true for the selecting while loop

        addState(new state("unhang and stuff for victory") {
            @Override
            public void firstTime() {
                lastGoldLocation = UNKNOWN;
                aux.startLiftUp();
                aux.timer.reset();
                tfod.activate();
                aux.hang.setPower(0);
            }

            @Override
            public boolean everyTime() {
                return aux.isUnhungWithArm();
            }

            @Override
            public void onCompletion() {
                tfod.deactivate();
                aux.hang.setPower(0);
            }
        });

        addState(new state("get off hook") {
            @Override
            public void firstTime() {
                drive.followTrajectory(drive.trajectoryBuilder()
                        .turn(Math.toRadians(15))
                        .forward(2)
                        .turnTo(Math.toRadians(-45))
                        .build());
            }

            @Override
            public boolean everyTime() {
                aux.setArm(armTarget.horizontal, slidePositions[0]);
                sendPositionalErrorTelemetry();
                drive.update();
                return !drive.isFollowingTrajectory();
            }

            @Override
            public void onCompletion() {
                aux.startLiftDown();
            }
        });

        addState(new state("drop off marker") {
            @Override
            public void firstTime() {
                drive.followTrajectory(drive.trajectoryBuilder()
                        .forward(15)
                        .build());
            }

            @Override
            public boolean everyTime() {
                aux.setArm(armTarget.horizontal, maxSlidePosition);
                sendPositionalErrorTelemetry();
                drive.update();
                return !drive.isFollowingTrajectory();
            }

            @Override
            public void onCompletion() {
                aux.reverseIntake();
                telemetry.update();
                try {
                    sleep(300);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                telemetry.update();
            }
        });

        addState(new state("backup from team marker") {
            @Override
            public void firstTime() {
                drive.followTrajectory(drive.trajectoryBuilder()
                        .back(15)
                        .build());
            }

            @Override
            public boolean everyTime() {
                aux.setArm(armTarget.slightUp, slidePositions[0]);
                sendPositionalErrorTelemetry();
                drive.update();
                return !drive.isFollowingTrajectory();
            }
        });

        addState(new state("pickup gold") {
            @Override
            public void firstTime() {
                aux.activateIntakeM();

                switch (lastGoldLocation) {
                    case LEFT:
                        drive.followTrajectory(drive.trajectoryBuilder()
                                .turn(Math.toRadians(28))
                                .forward(22)
                                .build());
                        break;
                    case RIGHT:
                        drive.followTrajectory(drive.trajectoryBuilder()
                                .turn(Math.toRadians(-25))
                                .forward(22)
                                .build());
                        break;
                    default:
                        drive.followTrajectory(drive.trajectoryBuilder()
                                .turn(Math.toRadians(0))
                                .forward(22)
                                .build());
                        break;
                }
            }

            @Override
            public boolean everyTime() {
                aux.setArm(armPositionFinder(maxSlidePosition * 0.22), (int) (maxSlidePosition * 0.28));//change to these values to down more if picking up a cube is desired
                sendPositionalErrorTelemetry();
                drive.update();
                return !drive.isFollowingTrajectory();
            }
        });

        addState(new state("back up from gold after picked up and align for scoring it") {
            @Override
            public void firstTime() {
                drive.followTrajectory(drive.trajectoryBuilder()
                        .back(22)
                        .turnTo(Math.toRadians(-45))
                        .build());
            }

            @Override
            public boolean everyTime() {
                aux.setArm(armTarget.slightUp, slidePositions[0]);
                sendPositionalErrorTelemetry();
                drive.update();
                return !drive.isFollowingTrajectory();
            }
        });

        addState(new state("score cube") {
            @Override
            public void firstTime() {
                aux.timer.reset();
                aux.lift.setPower(0);
            }

            @Override
            public boolean everyTime() {
                if(aux.pot.getVoltage() < armTarget.vertical)
                    aux.reverseIntake();
                aux.setArm(armTarget.depotSideY, (int) (maxSlidePosition * 0.47));
                drive.update();
                return aux.timer.seconds() > 1;
            }
        });

        addState(new state("go to crater") {
            @Override
            public void firstTime() {
                drive.followTrajectory(drive.trajectoryBuilder()
                        .beginComposite()
//                        .splineTo(new Pose2d(36,-12,0))
                        .lineTo(new Vector2d(64, -6), new LinearInterpolator(Math.toRadians(-45),Math.toRadians(90)))
                        .lineTo(new Vector2d(62, 13),new ConstantInterpolator(Math.toRadians(90)))
                        .closeComposite()
                        .build());
            }

            @Override
            public boolean everyTime() {
                aux.setArm(armTarget.vertical, slidePositions[0]);
                sendPositionalErrorTelemetry();
                drive.update();
                return !drive.isFollowingTrajectory();
            }
        });

        addState(new state("extend out arm at end") {
            @Override
            public boolean everyTime() {
                sendPositionalErrorTelemetry();
                drive.update();
                return aux.setArmSlowly(armTarget.horizontal, slidePositions[1]);
            }
        });

        addState(new state("sleep0") {
            public void firstTime() {
                try {
                    sleep(0);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });

        //convert list of states to be run to array for theoretical performance reasons
        runlist = list.toArray(new state[list.size()]);

        //run each state multiple times until the state increases the currentState variable by 1
        currentState = 0;

        //add telementry data to display if debug mode is active, debug mode is used to test to make sure objects are oriented correctly without having actual hardware attached
        telemetry.addData("Say", "Hello Driver - debug mode is " + debugMode);
        displayStates();

    }

    @Override
    public void init_loop() {
        if (Math.abs(aux.slide.getCurrentPosition()) > 100 || Math.abs(aux.lift.getCurrentPosition()) > 100)
            aux.hang.setPower(0.5);
        else
            aux.hang.setPower(0);

        telemetry.addData("slide position", aux.slide.getCurrentPosition());
        telemetry.addData("lift position", aux.lift.getCurrentPosition());
        displayStates();
    }

    @Override
    public void stop() {
        aux.arm.setPower(0);
        aux.slide.setPower(0);
        drive.setMotorPowers(0, 0, 0, 0);
        aux.deactivateIntake();
    }

    @Override
    public void loop() {
        if (currentState + 1 < runlist.length) {
            //run the state from the runlist of the currentState index
            runlist[currentState].run();
        } else {
            requestOpModeStop();
        }
    }


    void displayStates() {
        telemetry.addLine();
        //display list of states that will be run on the phone's screen
        for (int index = 0; index < list.size(); index++) {
            state currentState = list.get(index);
            telemetry.addData(String.valueOf(index), currentState.name);
        }
    }

    void addState(state statea) {
        list.add(list.size(), statea);
    }

    void sendPositionalErrorTelemetry() {
//        Pose2d error = drive.getFollowingError();
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("xError", error.getX());
//        packet.put("yError", error.getY());
//        double headErr = error.getHeading();
//        for (; headErr < -Math.PI; headErr += 2 * Math.PI) {
//        }
//        packet.put("headingError", headErr);
//        dashboard.sendTelemetryPacket(packet);
    }
}
