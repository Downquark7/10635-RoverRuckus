package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.robotconfig.angle;
import static org.firstinspires.ftc.teamcode.robotconfig.debugMode;
import static org.firstinspires.ftc.teamcode.robotconfig.enableMotorEncoders;
import static org.firstinspires.ftc.teamcode.robotconfig.initTfod;
import static org.firstinspires.ftc.teamcode.robotconfig.initVuforia;
import static org.firstinspires.ftc.teamcode.robotconfig.lastGoldLocation;
import static org.firstinspires.ftc.teamcode.robotconfig.lift;
import static org.firstinspires.ftc.teamcode.robotconfig.move;
import static org.firstinspires.ftc.teamcode.robotconfig.rinit;
import static org.firstinspires.ftc.teamcode.stateslist.autoTimer;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;
import static org.firstinspires.ftc.teamcode.teleOp.getAngleValue;


@Autonomous(group="farSide")
@Disabled
public class noticeMePlsNoCrater extends OpMode {

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
        enableMotorEncoders();
        initVuforia();
        initTfod();
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        autoTimer.reset();

        //start with the first state selected
        int selectedstate = 0;
        //set variable selecting to true for the selecting while loop

        askState(states.dropNScannWithArm);

        askState(states.getOffHookPt1);
        askState(states.getOffHookPt2);
        askState(states.getOffHookPt3);

        askState(states.extendArmForMarkerWithDriveForward15);
        askState(states.reverseVexy);
        askState(states.holdArmExtendedForASecond);
        askState(states.armAtCarryWithDriveBackward15);

        askState(states.activateVexy);
        askState(states.startLoweringLift);

        askState(states.obtainCubeWithRotateFullExtension);
        askState(states.scoreCubeWithRotate); //if doesn't work replace with states.rotateFromSample followed by states.scoreCube
        askState(states.sleep500);
        askState(states.armVertical);

        askState(states.deactivateVexy);

        askState(states.sleep0);

        //add telementry to report that init completed
//        addlog(dl, "autonomous", "Done with robot.init --- waiting for start in " + this.getClass().getSimpleName());

        //convert list of states to be run to array for theoretical performance reasons
        runlist = list.toArray(new state[list.size()]);

        //run each state multiple times until the state increases the currentState variable by 1
        currentState = 0;

        //add telementry data to display if debug mode is active, debug mode is used to test to make sure objects are oriented correctly without having actual hardware attached
        telemetry.addData("Say", "Hello Driver - debug mode is " + debugMode);

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

    void askState(state statea) {
        list.add(list.size(), statea);
    }

    void askState(state statea, state stateb) {
        list.add(list.size(), ask(statea, stateb));
    }

    void askState(state statea, state stateb, state statec) {
        list.add(list.size(), ask(statea, stateb, statec));
    }

    void askState(state statea, state stateb, state statex, state statey) {
        list.add(list.size(), ask(statea, stateb, statex, statey));
    }

    boolean ask(String statea, String stateb) {
        telemetry.addData("A", statea);
        telemetry.addData("B", stateb);
        displayStates();
        telemetry.update();
        while (autoTimer.seconds() < 40) {
            if (gamepad1.a) {
                while (gamepad1.a)
                    try {
                        sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                return true;
            } else if (gamepad1.b) {
                while (gamepad1.b)
                    try {
                        sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                return false;
            }
        }
        return true;
    }

    /***
     * ask is a method that returns a state object containing code for autonomous
     *
     * @param statea the first state object to choose
     * @param stateb the second state object to choose
     * @return a state to be added to the runList
     */
    state ask(state statea, state stateb) {
        //display options with the gamepad buttons to press
        telemetry.addData("A", statea.name);
        telemetry.addData("B", stateb.name);
        //show what was already added
        displayStates();
        telemetry.update();
        //check to make sure it is still in init
        while (autoTimer.seconds() < 40) {
            if (gamepad1.a) {
                //loop while held to avoid double press
                while (gamepad1.a)
                    try {
                        sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                //return state to add to runList
                return statea;
            } else if (gamepad1.b) {
                //loop while held to avoid double press
                while (gamepad1.b)
                    try {
                        sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                //return state to add to runList
                return stateb;
            }
        }
        //return state if program was stopped to allow a quick restart
        return statea;
    }

    state ask(state statea, state stateb, state statex) {
        telemetry.addData("A", statea.name);
        telemetry.addData("B", stateb.name);
        telemetry.addData("X", statex.name);
        displayStates();
        telemetry.update();
        while (autoTimer.seconds() < 40) {
            if (gamepad1.a) {
                while (gamepad1.a)
                    try {
                        sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                return statea;
            } else if (gamepad1.b) {
                while (gamepad1.b)
                    try {
                        sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                return stateb;
            } else if (gamepad1.x) {
                while (gamepad1.x)
                    try {
                        sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                return statex;
            }
        }
        return statea;
    }

    state ask(state statea, state stateb, state statex, state statey) {
        telemetry.addData("A", statea.name);
        telemetry.addData("B", stateb.name);
        telemetry.addData("X", statex.name);
        telemetry.addData("Y", statey.name);
        displayStates();
        telemetry.update();
        while (autoTimer.seconds() < 40) {
            if (gamepad1.a) {
                while (gamepad1.a)
                    try {
                        sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                return statea;
            } else if (gamepad1.b) {
                while (gamepad1.b)
                    try {
                        sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                return stateb;
            } else if (gamepad1.x) {
                while (gamepad1.x)
                    try {
                        sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                return statex;
            } else if (gamepad1.y) {
                while (gamepad1.y)
                    try {
                        sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                return statey;
            }
        }
        return statea;
    }

}
