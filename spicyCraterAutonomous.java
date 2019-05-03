package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.move.bettermove;
import static org.firstinspires.ftc.teamcode.move.setMyMotorTargets;
import static org.firstinspires.ftc.teamcode.preciseMovement.mm2pulses;
import static org.firstinspires.ftc.teamcode.preciseMovement.spin2mm;
import static org.firstinspires.ftc.teamcode.robotconfig.armMotor;
import static org.firstinspires.ftc.teamcode.robotconfig.debugMode;
import static org.firstinspires.ftc.teamcode.robotconfig.enableMotorBreak;
import static org.firstinspires.ftc.teamcode.robotconfig.enableMotorEncoders;
import static org.firstinspires.ftc.teamcode.robotconfig.goldLocation.CENTER;
import static org.firstinspires.ftc.teamcode.robotconfig.goldLocation.LEFT;
import static org.firstinspires.ftc.teamcode.robotconfig.initTfod;
import static org.firstinspires.ftc.teamcode.robotconfig.initVuforia;
import static org.firstinspires.ftc.teamcode.robotconfig.lastGoldLocation;
import static org.firstinspires.ftc.teamcode.robotconfig.lift;
import static org.firstinspires.ftc.teamcode.robotconfig.move;
import static org.firstinspires.ftc.teamcode.robotconfig.rinit;
import static org.firstinspires.ftc.teamcode.robotconfig.slide;
import static org.firstinspires.ftc.teamcode.robotconfig.vexy;
import static org.firstinspires.ftc.teamcode.robotconfig.vexy2;
import static org.firstinspires.ftc.teamcode.stateslist.autoTimer;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.teleOp.armPositionFinder;
import static org.firstinspires.ftc.teamcode.teleOp.getAngleValue;
import static org.firstinspires.ftc.teamcode.teleOp.maxArmSensorValue;
import static org.firstinspires.ftc.teamcode.teleOp.minArmSensorValue;
import static org.firstinspires.ftc.teamcode.teleOp.pGain;
import static org.firstinspires.ftc.teamcode.teleOp.slidePositions;


@Autonomous(group = "crater")
@Disabled
//this one scores 2-4 minerals in the lander and samples both cubes
public class spicyCraterAutonomous extends OpMode {

    boolean goBackAtEnd = false;
    //import file of states
    private stateslist states = new stateslist();
    //make list of the states that are going to be run
    private List<state> list = new ArrayList<>();
    //init the array that is going to be used when actually running
    private state[] runlist;

    @Override
    public void init() {

        // send whole LinearOpMode object and context to robotconfig init method
        rinit(this);
//        initGyro();
        enableMotorEncoders();
        enableMotorBreak();
        initVuforia();
        initTfod();
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        autoTimer.reset();

        //start with the first state selected
        int selectedstate = 0;
        //set variable selecting to true for the selecting while loop

        addState(states.dropNScannArmVertical);
//        addState(states.scan);

        addState(states.getOffHookPt1);
        addState(states.getOffHookPt2);

        addState(states.startLoweringLift);
        addState(new state("rotate right towards wall", 0, 0, -55 + 15));
        addState(new state("move towards wall", 38, 0, 0));
        addState(new state("rotate towards crater", 0, 0, -80) {
            @Override
            public boolean everyTime() {
                double target = armTarget.vertical;
                slide.setPower(0.001 * (slidePositions[0] - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(pGain * (getAngleValue() - target));
                else
                    armMotor.setPower(0);
                return bettermove();
            }

            @Override
            public void onCompletion() {
                slide.setPower(0);
                armMotor.setPower(0);
                move(0, 0, 0);
            }
        });
        addState(states.stopLift);
//        addState(states.octoganUp);
        addState(new state("markerTime", 18, 0, 0) {
            @Override
            public boolean everyTime() {
                double target = armTarget.horizontal;
                slide.setPower(0.001 * (slidePositions[3] - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(pGain * (getAngleValue() - target) + 0.4);
                else
                    armMotor.setPower(0);
                return bettermove();
            }

            @Override
            public void onCompletion() {
                slide.setPower(0);
                armMotor.setPower(0);
                move(0, 0, 0);
            }
        });

        addState(states.reverseVexy);
        addState(states.sleep200);
//        addState(states.octoganUp);
        addState(states.sleep100);

        addState(new state("get arm ready to pick up far sampled cube", 0, 0, -47) {
            @Override
            public boolean everyTime() {
                double target = 0;
                switch (lastGoldLocation) {
                    case RIGHT:
                        target = armTarget.down;
                        break;
                    case CENTER:
                        target = armPositionFinder(2545 + 1000);
                        break;
                    case LEFT:
                        target = (armPositionFinder(slidePositions[3]) + armTarget.horizontal) / 2;
                        break;
                }
                int slideTarget = 0;
                switch (lastGoldLocation) {
                    case RIGHT:
                        slideTarget = slidePositions[0];
                        break;
                    case CENTER:
                        slideTarget = 2545;
                        break;
                    case LEFT:
                        slideTarget = slidePositions[3];
                        break;
                }
                slide.setPower(0.001 * (slideTarget - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(pGain * (getAngleValue() - target));
                else
                    armMotor.setPower(0);
                return (lastGoldLocation == LEFT ? bettermove(0.5) : bettermove()) && Math.abs(slide.getPower()) < 0.3;
            }

            @Override
            public void onCompletion() {
                super.onCompletion();
                armMotor.setPower(0);
                slide.setPower(0);
            }
        });

        addState(states.activateVexy);

        addState(new state("stop on gamepad1.A for debug") {
            @Override
            public boolean everyTime() {
                return !gamepad1.a;
            }
        });

//        addState(states.squareUp);

        addState(new state("get far sampled cube",5,0,0) {
            @Override
            public boolean everyTime() {
                double target = armPositionFinder(slide.getCurrentPosition());
                int slideTarget = 0;
                switch (lastGoldLocation) {
                    case RIGHT:
                        target = 2.533;
                        slideTarget = 836;
                        break;
                    case CENTER:
                        slideTarget = 2545 + 1000;
                        break;
                    case LEFT:
                        slideTarget = slidePositions[3] + 1000;
                        break;
                }
                slide.setPower(0.001 * (slideTarget - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(pGain * (getAngleValue() - target));
                else
                    armMotor.setPower(0);
                return bettermove(0.3);
            }

            @Override
            public void onCompletion() {
                super.onCompletion();
                slide.setPower(0);
            }
        });

        addState(new state("stop on gamepad1.A for debug") {
            @Override
            public boolean everyTime() {
                return !gamepad1.a;
            }
        });

//        addState(states.squareUp);

        addState(new state("back up from first mineral",-5,0,0) {
            @Override
            public boolean everyTime() {
                double target = armTarget.vertical;
                int slideTarget = slidePositions[0];
                slide.setPower(0.001 * (slideTarget - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(pGain * (getAngleValue() - target));
                else
                    armMotor.setPower(0);
                return bettermove() && Math.abs(slide.getPower()) < 0.3;
            }
        });

        addState(new state("return from sample", 0, 0, 47) {
            @Override
            public boolean everyTime() {
                double target = armTarget.vertical;
                int slideTarget = slidePositions[0];
                slide.setPower(0.001 * (slideTarget - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(pGain * (getAngleValue() - target));
                else
                    armMotor.setPower(0);
                return bettermove() && Math.abs(slide.getPower()) < 0.3;
            }
        });

        addState(new state("backup from marker position", -18, 0, 0) {
            @Override
            public boolean everyTime() {
                double target = armTarget.vertical;
                slide.setPower(0.001 * (slidePositions[0] - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(pGain * (getAngleValue() - target));
                else
                    armMotor.setPower(0);
                return bettermove() && Math.abs(slide.getPower()) < 0.3 && Math.abs(armMotor.getPower()) < 0.3;
            }

            @Override
            public void onCompletion() {
                armMotor.setPower(0);
                slide.setPower(0);
                move(0, 0, 0);
            }
        });
        addState(states.deactivateVexy);
//        addState(states.octoganUp);
        addState(new state("rotate towards crater", 0, 0, 80));
        addState(new state("move towards wall", -38, 0, 0));
        addState(new state("rotate right towards wall", 0, 0, 55) {
            @Override
            public boolean everyTime() {
                double target = armTarget.down;
                slide.setPower(0.001 * (slidePositions[0] - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(pGain * (getAngleValue() - target));
                else
                    armMotor.setPower(0);
                return bettermove() && Math.abs(slide.getPower()) < 0.3 && Math.abs(armMotor.getPower()) < 0.3;
            }

            @Override
            public void onCompletion() {
                armMotor.setPower(0);
                slide.setPower(0);
                move(0, 0, 0);
            }
        });

//        addState(states.squareUp);

        addState(new state("obtain cube with rotate") {
            @Override
            public void firstTime() {
                autoTimer.reset();
                vexy.setPower(0.75);
                vexy2.setPower(-.75);
                switch (lastGoldLocation) {
                    case LEFT:
                        setMyMotorTargets(0, 0, mm2pulses(spin2mm(-23)));
                        break;
                    case CENTER:
                        setMyMotorTargets(0, 0, mm2pulses(spin2mm(3)));
                        break;
                    case RIGHT:
                        setMyMotorTargets(0, 0, mm2pulses(spin2mm(31)));
                        break;
                }
            }

            @Override
            public boolean everyTime() {
                double x = 1.0 * slide.getCurrentPosition();
                double target = slide.getCurrentPosition() > slidePositions[0] * 2 ? armPositionFinder(x) : ((x - slidePositions[0]) / slidePositions[0]) * (armPositionFinder(slidePositions[0] * 2) - 2.33) + 2.33;
                slide.setPower(0.7);
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(pGain * (getAngleValue() - target));
                else
                    armMotor.setPower(0);
                return bettermove() && slide.getCurrentPosition() > (lastGoldLocation == CENTER ? slidePositions[2] * 0.9 : slidePositions[2]);
            }

            @Override
            public void onCompletion() {
                slide.setPower(0);
                armMotor.setPower(0);
                move(0, 0, 0);
            }
        });

        addState(states.scoreCubeWithRotateCraterEdition);

        addState(states.sleep200);

        addState(states.reverseVexy);

        addState(new state("rotate again for crater side", 0, 0, -20));

        addState(states.activateVexy);

//        addState(states.squareUp);

        int numberOfScoringAttempts = 1; //change this number if you want

        for (int i = 0; i < numberOfScoringAttempts; i++) {
            addState(new state("get more minerals", 15, 0, 0) {
                @Override
                public boolean everyTime() {
                    double target = armTarget.horizontal;
                    slide.setPower(0.001 * (slidePositions[4] - slide.getCurrentPosition()));
                    if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                        armMotor.setPower(Math.max(pGain * (getAngleValue() - target), -0.5));
                    else
                        armMotor.setPower(0);
                    return bettermove() || lastGoldLocation != CENTER;
                }
            });
            addState(new state("obtain cube full extension beta") {
                @Override
                public void firstTime() {
                    autoTimer.reset();
                    vexy.setPower(0.75);
                    vexy2.setPower(-.75);
                }

                @Override
                public boolean everyTime() {
                    double x = 1.0 * slide.getCurrentPosition();
                    double target = slide.getCurrentPosition() > slidePositions[0] * 2 ? armPositionFinder(x) : ((x - slidePositions[0]) / slidePositions[0]) * (armPositionFinder(slidePositions[0] * 2) - armTarget.down) + armTarget.down;
                    slide.setPower(0.7);
                    if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                        armMotor.setPower(pGain * (getAngleValue() - target));
                    else
                        armMotor.setPower(0);
                    return (bettermove() && slide.getCurrentPosition() > slidePositions[3]) || lastGoldLocation != CENTER;
                }

                @Override
                public void onCompletion() {
                    slide.setPower(0);
                    armMotor.setPower(0);
                    move(0, 0, 0);
                }
            });
            addState(new state("back up from minerals", -15, 0, 0) {
                @Override
                public boolean everyTime() {
                    double target = armTarget.hh;
                    slide.setPower(0.001 * (slidePositions[1] - slide.getCurrentPosition()));
                    if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                        armMotor.setPower(pGain * (getAngleValue() - target));
                    else
                        armMotor.setPower(0);
                    return bettermove() || lastGoldLocation != CENTER;
                }
            });
            addState(states.scoreMineralsCraterSideIfNotCenter);
            addState(states.sleep500);
            addState(states.reverseVexy);
            addState(new state("rotate again for crater side", 0, 0, -20) {
                @Override
                public boolean everyTime() {
                    double target = armTarget.hh;
                    slide.setPower(0.001 * (slidePositions[1] - slide.getCurrentPosition()));
                    if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                        armMotor.setPower(pGain * (getAngleValue() - target));
                    else
                        armMotor.setPower(0);
                    return bettermove() || lastGoldLocation != CENTER;
                }
            });
            addState(states.activateVexy);
        }

        addState(new state("drive forward at end", 13, 0, 0) {
            @Override
            public boolean everyTime() {
                double target = armTarget.horizontal;
                slide.setPower(0.001 * (slidePositions[3] - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(Math.max(pGain * (getAngleValue() - target), -0.5));
                else
                    armMotor.setPower(0);
                return bettermove();
            }
        });

        addState(states.holdArmExtendedForever);

        addState(states.sleep0);
        //add telementry to report that init completed
//        addlog(dl, "autonomous", "Done with robot.init --- waiting for start in " + this.getClass().getSimpleName());

        //convert list of states to be run to array for theoretical performance reasons
        runlist = list.toArray(new state[list.size()]);

        //run each state multiple times until the state increases the currentState variable by 1
        currentState = 0;

        //add telementry data to display if debug mode is active, debug mode is used to test to make sure objects are oriented correctly without having actual hardware attached
        telemetry.addData("Say", "Hello Driver - debug mode is " + debugMode);

        displayStates();

    }

    @Override
    public void loop() {
        Log.v("Downquark7", String.format("angleValue: %f", getAngleValue()));

//            addlog(dl, "Mainline", "Beginning state machine pass " + String.format(Locale.ENGLISH, "%d", currentState));
        //check if the currentState is more than the last index of the runlist
        if (currentState + 1 < runlist.length) {
            telemetry.addData("state", runlist[currentState].name);
            telemetry.addData("mineral order", lastGoldLocation);
            telemetry.update();
            //run the state from the runlist of the currentState index
            runlist[currentState].run();
            //add log of name of state that ran for debugging
//                addlog(dl, "Mainline", "Ending state machine pass of " + runlist[currentState].name);
        } else {
            //if completed last state, stop
            move(0, 0, 0);
            //add log to tell us that the program finished all selected states before stopping
//                addlog(dl, "StateMachine", "stop requested");
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

}
