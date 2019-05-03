package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREV;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.autoPaths.centerMineralPoseb;
import static org.firstinspires.ftc.teamcode.autoPaths.craterPos;
import static org.firstinspires.ftc.teamcode.autoPaths.leftMineralPoseb;
import static org.firstinspires.ftc.teamcode.autoPaths.rightMineralPoseb;
import static org.firstinspires.ftc.teamcode.autoPaths.beforeSamplePos;
import static org.firstinspires.ftc.teamcode.autoPaths.scorePos;
import static org.firstinspires.ftc.teamcode.autoPaths.startpos;
import static org.firstinspires.ftc.teamcode.robotconfig.debugMode;
import static org.firstinspires.ftc.teamcode.robotconfig.goldLocation.UNKNOWN;
import static org.firstinspires.ftc.teamcode.robotconfig.initTfod;
import static org.firstinspires.ftc.teamcode.robotconfig.initVuforia;
import static org.firstinspires.ftc.teamcode.robotconfig.lastGoldLocation;
import static org.firstinspires.ftc.teamcode.robotconfig.tfod;
import static org.firstinspires.ftc.teamcode.stateslist.autoTimer;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.teleOp.armPositionFinder;
import static org.firstinspires.ftc.teamcode.teleOp.maxSlidePosition;
import static org.firstinspires.ftc.teamcode.teleOp.slidePositions;

@Autonomous
public class autoCraterV2 extends OpMode {

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

        autoPaths.generatePoints();
        drive.setPoseEstimate(startpos);

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
                drive.setPoseEstimate(startpos);
            }
        });

        addState(new state("drop off marker") {
            @Override
            public void firstTime() {
                drive.followTrajectory(drive.trajectoryBuilder()
                        .turn(Math.toRadians(15))
                        .beginComposite()
                        .splineTo(new Pose2d(-12, -39, Math.toRadians(-65)))
                        .splineTo(new Pose2d(12, -60, 0))
                        .forward(8)
                        .closeComposite()
                        .build());
            }

            @Override
            public boolean everyTime() {
                Pose2d currentPose = drive.getPoseEstimate();
                if (currentPose.getX() > 5)
                    aux.setArmSlowly(armTarget.horizontal, slidePositions[3]);
                else
                    aux.setArm(armTarget.hh, slidePositions[0]);
                sendPositionalErrorTelemetry();
                drive.update();
                return !drive.isFollowingTrajectory();
            }

            @Override
            public void onCompletion() {
                aux.reverseIntake();
                aux.startLiftDown();
            }
        });

        addState(new state("return from marker") {
            @Override
            public void firstTime() {
                drive.followTrajectory(drive.trajectoryBuilder()
                        .waitFor(0.1)
                        .reverse()
                        .beginComposite()
                        .splineTo(beforeSamplePos)
                        .closeComposite()
                        .build());
            }

            @Override
            public boolean everyTime() {
                Pose2d currentPose = drive.getPoseEstimate();
                if (currentPose.getY() > -48)
                    aux.setArm(armTarget.down, slidePositions[0]);
                else
                    aux.setArmSlowSlide(armTarget.hh, slidePositions[0]);
                sendPositionalErrorTelemetry();
                drive.update();
                return !drive.isFollowingTrajectory();
            }

            @Override
            public void onCompletion() {
                aux.lift.setPower(0);
            }
        });


        addState(new state("pickup gold") {
            @Override
            public void firstTime() {
                aux.activateIntake();
                switch (lastGoldLocation) {
                    case LEFT:
                        drive.followTrajectory(drive.trajectoryBuilder().splineTo(leftMineralPoseb).build());
                        break;
                    case RIGHT:
                        drive.followTrajectory(drive.trajectoryBuilder().splineTo(rightMineralPoseb).build());
                        break;
                    default:
                        drive.followTrajectory(drive.trajectoryBuilder().splineTo(centerMineralPoseb).build());
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

        addState(new state("drive to lander scoring pos for gold") {
            @Override
            public void firstTime() {
                drive.followTrajectory(drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(scorePos)
                        .build());
            }

            @Override
            public boolean everyTime() {
                aux.setArm(armTarget.hh, slidePositions[0]);
                sendPositionalErrorTelemetry();
                drive.update();
                return !drive.isFollowingTrajectory();
            }
        });

        addState(new state("score minerals for gold") {
            @Override
            public void firstTime() {
                aux.timer.reset();
            }

            @Override
            public boolean everyTime() {
                if(aux.pot.getVoltage() < armTarget.vertical)
                aux.reverseIntake();
                aux.setArm(armTarget.autoScorePos, (int) (maxSlidePosition * 0.53));
                sendPositionalErrorTelemetry();
                drive.update();
                return aux.timer.seconds() > 0.9;
            }
        });


        for (int i = 1; i < 2; i++) {
            addState(new state("go to minerals") {
                @Override
                public void firstTime() {
                    aux.activateIntake();
                    drive.followTrajectory(drive.trajectoryBuilder().waitFor(0.3).splineTo(craterPos).build());
                }

                @Override
                public boolean everyTime() {
                    aux.setArmSlowly(armTarget.horizontal, slidePositions[0]);
                    sendPositionalErrorTelemetry();
                    drive.update();
                    return !drive.isFollowingTrajectory();
                }
            });

            addState(new state("arm out so it doesn't get stuck") {
                @Override
                public void firstTime() {
                    aux.activateIntake();
                }

                @Override
                public boolean everyTime() {
                    sendPositionalErrorTelemetry();
                    drive.update();
                    return aux.setArmSlowly(armTarget.horizontal, (int) (maxSlidePosition * 0.5));
                }
            });

            int finalI = i;
            addState(new state("intake minerals") {
                @Override
                public void firstTime() {
                    aux.activateIntake();
                }

                @Override
                public boolean everyTime() {
                    sendPositionalErrorTelemetry();
                    drive.update();
                    return aux.intakeMinerals((int) (0.75 * slidePositions[3] + slidePositions[3] * 0.2 * finalI));
                }
            });

            addState(new state("spit out extra minerals") {
                @Override
                public boolean everyTime() {
                    sendPositionalErrorTelemetry();
                    drive.update();
                    return aux.setArm(armTarget.autohh, slidePositions[1]);
                }

                @Override
                public void onCompletion() {
                    aux.reverseIntake();
                    aux.arm.setPower(0);
                    aux.slide.setPower(0);
//                    telemetry.update();
//                    try {
//                        sleep(300);
//                    } catch (InterruptedException e) {
//                        e.printStackTrace();
//                    }
                    telemetry.update();
                    try {
                        sleep(300);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    telemetry.update();
                    aux.activateIntake();
                }
            });

            addState(new state("drive to lander scoring pos") {
                @Override
                public void firstTime() {
                    drive.followTrajectory(drive.trajectoryBuilder()
                            .reverse()
                            .splineTo(scorePos)
                            .build());
                }

                @Override
                public boolean everyTime() {
                    aux.setArm(armTarget.hh, slidePositions[0]);
                    sendPositionalErrorTelemetry();
                    drive.update();
                    return !drive.isFollowingTrajectory();
                }
            });

            addState(new state("score minerals") {
                @Override
                public void firstTime() {
                    aux.timer.reset();
                }

                @Override
                public boolean everyTime() {
                    if(aux.pot.getVoltage() < armTarget.vertical)
                        aux.reverseIntake();
                    aux.setArm(armTarget.autoScorePos, (int) (maxSlidePosition * 0.53));
                    sendPositionalErrorTelemetry();
                    drive.update();
                    return aux.timer.seconds() > 1.2;
                }
            });
        }

        addState(new state("park at end") {
            @Override
            public void firstTime() {
                aux.activateIntake();
                drive.followTrajectory(drive.trajectoryBuilder().splineTo(new Pose2d(-28, -28, Math.toRadians(-135))).build());
            }

            @Override
            public boolean everyTime() {
                aux.setArm(armTarget.hh, slidePositions[1]);
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
                aux.setArmSlowly(armTarget.hh, slidePositions[1]);
                return false;
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
//        packet.put("headingError", headErr);
//        dashboard.sendTelemetryPacket(packet);
    }
}
