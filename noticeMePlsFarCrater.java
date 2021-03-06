package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

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
public class noticeMePlsFarCrater extends OpMode {

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

        addState(states.dropNScannArmVertical);

        addState(states.getOffHookPt1);
        addState(states.getOffHookPt2);
        addState(states.getOffHookPt3);
        addState(states.startLoweringLift);

        addState(states.extendArmForMarkerWithDriveForward15);
        addState(states.reverseVexy);
        addState(states.sleep500);
        addState(states.armAtCarryWithDriveBackward15);

        addState(states.activateVexy);

        addState(states.rotateToSample);
        addState(states.obtainCubeFullExtension);
//        addState(states.obtainCubeWithRotateFullExtension);
        addState(states.armAtCarry);
        addState(states.scoreCubeWithRotate); //if doesn't work replace with states.rotateFromSample followed by states.scoreCube
        addState(states.sleep500);
        addState(states.armVertical);

        addState(states.stopLift);

        addState(new state("rotate right towards wall", 0, 0, -55));
        addState(new state("move towards wall", 38, 0, 0));
        addState(new state("rotate towards crater", 0, 0, -80));
        addState(new state("square up to wall", 0, 16, 0));
        addState(new state("distance from wall", 0, -3, 0));
        addState(new state("drive towards far crater", 22, 0, 0));
        addState(states.armDownAtCarry);
        addState(states.obtainCubeFullExtension);
        addState(states.armAtCarry);
        addState(states.reverseVexy);
        addState(states.sleep500);
        addState(states.deactivateVexy);
        addState(states.coastForward);

        addState(states.sleep0);

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
