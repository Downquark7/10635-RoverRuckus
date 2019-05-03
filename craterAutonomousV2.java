package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.move.gyromove;
import static org.firstinspires.ftc.teamcode.move.spinToHeading;
import static org.firstinspires.ftc.teamcode.robotconfig.armMotor;
import static org.firstinspires.ftc.teamcode.robotconfig.debugMode;
import static org.firstinspires.ftc.teamcode.robotconfig.enableMotorEncoders;
import static org.firstinspires.ftc.teamcode.robotconfig.getCurrentAngle;
import static org.firstinspires.ftc.teamcode.robotconfig.goldLocation.CENTER;
import static org.firstinspires.ftc.teamcode.robotconfig.initDrivetrainTelemetry;
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
public class craterAutonomousV2 extends OpMode {

    boolean goBackAtEnd = false;
    //import file of states
    private stateslist states = new stateslist();
    //make list of the states that are going to be run
    private List<state> list = new ArrayList<>();
    //init the array that is going to be used when actually running
    private state[] runlist;

    @Override
    public void init() {

        // send whole OpMode object and context to robotconfig init method
        rinit(this);
        enableMotorEncoders();
        initVuforia();
        initTfod(); //tfod = tensor flow object detection

        telemetry.addData("gyro", new Func<Float>() {
            @Override
            public Float value() {
                return (float) (getCurrentAngle());
            }
        });
        telemetry.addData("state", new Func<String>() {
            @Override
            public String value() {
                return runlist[currentState].name;
            }
        });
        telemetry.addData("mineral order", new Func<robotconfig.goldLocation>() {
            @Override
            public robotconfig.goldLocation value() {
                return lastGoldLocation;
            }
        });

        initDrivetrainTelemetry(telemetry);

        autoTimer.reset();

        //start with the first state selected
        int selectedstate = 0;
        //set variable selecting to true for the selecting while loop

        addState(states.dropNScannArmVertical);
//        addState(states.scan);

        addState(new state("get off hook", 15.0));
        addState(new state("getOffHookPt2", 2, 0, 0, 15.0));

        addState(states.startLoweringLift);
        addState(new state("rotate right towards wall", 50.0));
        addState(new state("move towards wall", 40, 0, 0, 50.0));
        addState(new state("rotate towards box", 125.0) {
            @Override
            public boolean everyTime() {
                double target = armTarget.vertical;
                slide.setPower(0.001 * (slidePositions[0] - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(pGain * (getAngleValue() - target));
                else
                    armMotor.setPower(0);
                return super.everyTime();
            }

            @Override
            public void onCompletion() {
                slide.setPower(0);
                armMotor.setPower(0);
                move(0, 0, 0);
            }
        });
        addState(states.stopLift);
        addState(new state("markerTime", 30, 0, 0, 125.0) {
            @Override
            public boolean everyTime() {
                double target = armTarget.horizontal;
                slide.setPower(0.001 * (slidePositions[4] - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(pGain * (getAngleValue() - target) + 0.4);
                else
                    armMotor.setPower(0);
                return super.everyTime();
            }

            @Override
            public void onCompletion() {
                slide.setPower(0);
                armMotor.setPower(0);
                move(0, 0, 0);
            }
        });
        addState(states.reverseVexy);
        addState(states.sleep500);
        addState(new state("backup from marker position", -30, 0, 0, 125.0) {
            @Override
            public boolean everyTime() {
                double target = armTarget.vertical;
                slide.setPower(0.001 * (slidePositions[0] - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(pGain * (getAngleValue() - target));
                else
                    armMotor.setPower(0);
                return super.everyTime() && Math.abs(slide.getPower()) < 0.3 && Math.abs(armMotor.getPower()) < 0.3;
            }

            @Override
            public void onCompletion() {
                armMotor.setPower(0);
                slide.setPower(0);
                move(0, 0, 0);
            }
        });
        addState(states.deactivateVexy);
        addState(new state("rotate towards crater", 50.0));
        addState(new state("move away from wall", -40, 0, 0, 50.0) {
            @Override
            public boolean everyTime() {
                double target = armTarget.down;
                slide.setPower(0.001 * (slidePositions[0] - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(Range.clip(pGain * (getAngleValue() - target), -0.4, 0.4));
                else
                    armMotor.setPower(0);
                return super.everyTime() && Math.abs(slide.getPower()) < 0.3 && Math.abs(armMotor.getPower()) < 0.3;
            }

            @Override
            public void onCompletion() {
                armMotor.setPower(0);
                slide.setPower(0);
                move(0, 0, 0);
            }
        });

        addState(new state("obtain cube with rotate", 0.0) {
            double targetHeading = -50;

            @Override
            public void firstTime() {
                autoTimer.reset();
                vexy.setPower(0.75);
                vexy2.setPower(-.75);
                switch (lastGoldLocation) {
                    case LEFT:
//                        setMyMotorTargets(0, 0, mm2pulses(spin2mm(-22)));
                        targetHeading = 20;
//                        gyromove(targetHeading);
                        break;
                    case CENTER:
//                        setMyMotorTargets(0, 0, mm2pulses(spin2mm(4)));
                        targetHeading = 3;
//                        gyromove(targetHeading);
                        break;
                    case RIGHT:
//                        setMyMotorTargets(0, 0, mm2pulses(spin2mm(28)));
                        targetHeading = -20;
//                        gyromove(targetHeading);
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
                return spinToHeading(targetHeading) && slide.getCurrentPosition() > (lastGoldLocation == CENTER ? slidePositions[2] * 0.9 : slidePositions[2]);
            }

            @Override
            public void onCompletion() {
                slide.setPower(0);
                armMotor.setPower(0);
                move(0, 0, 0);
            }
        });

        addState(states.armAtCarry);

        addState(states.scoreCubeWithRotateCraterEdition);

        addState(states.sleep200);

        addState(new state("rotate again for crater side", 0.0));

        int numberOfScoringAttempts = 1; //change this number if you want

        for (int i = 0; i < numberOfScoringAttempts; i++) {
            addState(new state("get more minerals", 15, 0, 0, 0.0) {
                @Override
                public boolean everyTime() {
                    double target = armTarget.horizontal;
                    slide.setPower(0.001 * (slidePositions[4] - slide.getCurrentPosition()));
                    if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                        armMotor.setPower(Math.max(pGain * (getAngleValue() - target), -0.5));
                    else
                        armMotor.setPower(0);
                    return super.everyTime();
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
                    double target = slide.getCurrentPosition() > slidePositions[0] * 2 ? armPositionFinder(x) : ((x - slidePositions[0]) / slidePositions[0]) * (armPositionFinder(slidePositions[0] * 2) - 2.33) + 2.33;
                    slide.setPower(0.7);
                    if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                        armMotor.setPower(pGain * (getAngleValue() - target));
                    else
                        armMotor.setPower(0);
                    return (gyromove(0) && slide.getCurrentPosition() > slidePositions[3]);
                }

                @Override
                public void onCompletion() {
                    slide.setPower(0);
                    armMotor.setPower(0);
                    move(0, 0, 0);
                }
            });
            addState(new state("get rid of extra elements") {
                @Override
                public boolean everyTime() {
                    double target = armTarget.hh;
                    slide.setPower(0.001 * (slidePositions[4] - slide.getCurrentPosition()));
                    if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                        armMotor.setPower(pGain * (getAngleValue() - target));
                    else
                        armMotor.setPower(0);
                    return Math.abs(armMotor.getPower()) < 0.3;
                }

                @Override
                public void onCompletion() {
                    slide.setPower(0);
                    armMotor.setPower(0);
                }
            });
            addState(states.activateVexy);
            addState(new state("back up from minerals", -15, 0, 0, 0.0) {
                @Override
                public boolean everyTime() {
                    double target = armTarget.hh;
                    slide.setPower(0.001 * (slidePositions[1] - slide.getCurrentPosition()));
                    if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                        armMotor.setPower(pGain * (getAngleValue() - target));
                    else
                        armMotor.setPower(0);
                    return super.everyTime();
                }
            });
            addState(states.activateVexy);
            addState(states.scoreMineralsCraterSide);
            addState(states.sleep500);
            addState(new state("rotate again for crater side", 0.0) {
                @Override
                public boolean everyTime() {
                    double target = armTarget.hh;
                    slide.setPower(0.001 * (slidePositions[1] - slide.getCurrentPosition()));
                    if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                        armMotor.setPower(pGain * (getAngleValue() - target));
                    else
                        armMotor.setPower(0);
                    return spinToHeading(0);
                }
            });
        }

        addState(new state("drive forward at end", 15, 0, 0, 0.0) {
            @Override
            public boolean everyTime() {
                double target = armTarget.hh;
                slide.setPower(0.001 * (slidePositions[4] - slide.getCurrentPosition()));
                if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                    armMotor.setPower(Math.max(pGain * (getAngleValue() - target), -0.5));
                else
                    armMotor.setPower(0);
                return super.everyTime();
            }

            @Override
            public void onCompletion() {
                super.onCompletion();
            }
        });

        addState(states.holdArmExtendedForever);

        //the lower two states can replace the upper two for the potential to get more minerals ready for teleop

//        addState(new state("get more minerals", 15, 0, 0) {
//            @Override
//            public boolean everyTime() {
//                double target = armTarget.horizontal;
//                slide.setPower(0.001 * (slidePositions[4] - slide.getCurrentPosition()));
//                if .getAngleValue() > minArmSensorValue &&.getAngleValue() < maxArmSensorValue)
//                    armMotor.setPower(Math.max(pGain * .getAngleValue() - target), -0.5));
//                else
//                    armMotor.setPower(0);
//                return super.everytime();
//            }
//        });
//        addState(states.obtainCubeFullExtension);

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
//        Log.v("Downquark7", String.format("angleValue: %f", getAngleValue()));

//            addlog(dl, "Mainline", "Beginning state machine pass " + String.format(Locale.ENGLISH, "%d", currentState));
        //check if the currentState is more than the last index of the runlist
        if (currentState + 1 < runlist.length) {
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
