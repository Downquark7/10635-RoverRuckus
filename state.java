package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.measurements.mmPerInch;
import static org.firstinspires.ftc.teamcode.move.bettermove;
import static org.firstinspires.ftc.teamcode.move.gyromove;
import static org.firstinspires.ftc.teamcode.move.setMyMotorTargets;
import static org.firstinspires.ftc.teamcode.move.spinToHeading;
import static org.firstinspires.ftc.teamcode.preciseMovement.mm2pulses;
import static org.firstinspires.ftc.teamcode.preciseMovement.spin2mm;
import static org.firstinspires.ftc.teamcode.robotconfig.move;
import static org.firstinspires.ftc.teamcode.stateslist.currentState;

//import static org.firstinspires.ftc.teamcode.robotconfig.addlog;
//import static org.firstinspires.ftc.teamcode.robotconfig.dl;

/**
 * Created by mail2 on 6/12/2017.
 */
interface substate {
    /***
     * method to be run first time of the state
     */
    void firstTime();

    /***
     * method to be run every time of the state
     * @return true if should exit
     */
    boolean everyTime();

    /***
     * method to be run once on completion of state
     */
    void onCompletion();

    /***
     * method to copy firstTime but setting the robot up to reverse the movement
     */
    void initReverse();

    /***
     * method to be run on every time to have autonomous run in reverse
     */
    boolean reverse();
}

public class state implements substate {

    public String name;
    public int runCount = 0;
    public ElapsedTime runtimecounter = new ElapsedTime();
    private boolean isFirstTime = true;
    private int moveTargetToBeSetForward = 0;
    private int moveTargetToBeSetRight = 0;
    private int moveTargetToBeSetSpin = 0;
    private boolean moveOnlyFunction = false;
    private boolean spinFunction = false;
    private int bl, br, fl, fr;
    private double targetHeading = 0;

//    ToneGenerator toneGen1 = new ToneGenerator(AudioManager.STREAM_MUSIC, 100);

    public state(String name) {
        this.name = name;
    }

    public state(String name, Boolean moveOnlyFunction) {
        this.name = name;
        this.moveOnlyFunction = moveOnlyFunction;
    }

    /***
     * a state programed to be a move function
     * @param name the name to appear
     * @param forward the distance to move forward in inches
     * @param right the distance to strafe right in inches
     * @param spin the distance to spin clockwise in degrees
     */
    public state(String name, int forward, int right, int spin) {
        this.name = name;
        this.moveTargetToBeSetForward = mm2pulses(forward * mmPerInch);
        this.moveTargetToBeSetRight = mm2pulses(right * mmPerInch);
        this.moveTargetToBeSetSpin = mm2pulses(spin2mm(spin));
        this.moveOnlyFunction = this.moveTargetToBeSetForward != 0 || this.moveTargetToBeSetRight != 0 || this.moveTargetToBeSetSpin != 0;
    }

    public state(String name, int forward, int right, int spin, double targetHeading) {
        this.name = name;
        this.moveTargetToBeSetForward = mm2pulses(forward * mmPerInch);
        this.moveTargetToBeSetRight = mm2pulses(right * mmPerInch);
        this.moveTargetToBeSetSpin = mm2pulses(spin2mm(spin));
        this.targetHeading = targetHeading;
        this.moveOnlyFunction = this.moveTargetToBeSetForward != 0 || this.moveTargetToBeSetRight != 0 || this.moveTargetToBeSetSpin != 0;
        this.spinFunction = true;
    }

    /***
     *
     * @param name the name to appear
     * @param fl encoder target
     * @param fr encoder target
     * @param bl encoder target
     * @param br encoder target
     */
    public state(String name, int fl, int fr, int bl, int br) {
        this.name = name;
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.moveOnlyFunction = true;
    }

    public state(String name, double targetHeading) {
        this.name = name;
        this.targetHeading = targetHeading;
        this.spinFunction = true;
    }

    /***
     * method to be run first time of the state
     */
    public void firstTime() {
        if (this.moveOnlyFunction) {
            if (this.moveTargetToBeSetForward != 0 || this.moveTargetToBeSetRight != 0 || this.moveTargetToBeSetSpin != 0)
                setMyMotorTargets(this.moveTargetToBeSetForward, this.moveTargetToBeSetRight, this.moveTargetToBeSetSpin);
            else
                setMyMotorTargets(fl, fr, bl, br);
        }
    }

    /***
     * method to be run every time of the state
     * @return true if should exit
     */
    public boolean everyTime() {
        if (this.moveOnlyFunction && this.spinFunction)
            return gyromove(this.targetHeading);
        else if (this.moveOnlyFunction)
            return bettermove();
        else if (this.spinFunction)
            return spinToHeading(this.targetHeading);
        else
            return true;
    }

    public boolean reverse() {
        return this.everyTime();
    }

    /***
     * method to be run once on completion of state
     */
    public void onCompletion() {}

    public void initReverse() {
        if (this.moveOnlyFunction)
            setMyMotorTargets(-this.moveTargetToBeSetForward, -this.moveTargetToBeSetRight, -this.moveTargetToBeSetSpin);
    }

    public void run() {
        if (isFirstTime) {
//            toneGen1.startTone(ToneGenerator.TONE_PROP_BEEP,150);
            runtimecounter.reset();
            Log.d("Downquark7", name + " has started");
            firstTime();
            Log.d("Downquark7", name + " first time has ended");
//            addlog(dl, "#" + name);
//            addlog(dl, name, "Execution has started");
            isFirstTime = false;
            runCount = 0;
        }
        runCount++;
        if (everyTime()) {
//            toneGen1.startTone(ToneGenerator.TONE_PROP_BEEP2,150);
            onCompletion();
            currentState++;
            isFirstTime = true;
            Log.i("Downquark7", String.format("Execution of %s has been completed in, %d, intervals over a time of, %f.3, seconds", name, runCount, runtimecounter.seconds()));
//            addlog(dl, name, String.format(Locale.ENGLISH, "Execution has been completed in, %d, intervals over a time of, %f.3, seconds", name, runCount, runtimecounter.seconds()));
        }
    }

    public void runReverse() {
        if (isFirstTime) {
//            toneGen1.startTone(ToneGenerator.TONE_PROP_BEEP,150);
            Log.d("Downquark7", name + " has started in reverse");
            initReverse();
            Log.d("Downquark7", name + " initReverse has ended");
//            addlog(dl, "#" + name);
//            addlog(dl, name, "Execution has started");
            isFirstTime = false;
            runtimecounter.reset();
            runCount = 0;
        }
        runCount++;
        if (reverse()) {
//            toneGen1.startTone(ToneGenerator.TONE_PROP_BEEP2,150);
            onCompletion();
            Log.i("Downquark7", String.format("Execution of %s has been completed in reverse in, %d, intervals over a time of, %f.3, seconds", name, runCount, runtimecounter.seconds()));
            currentState--;
            isFirstTime = true;
//            addlog(dl, name, String.format(Locale.ENGLISH, "Execution has been completed in, %d, intervals over a time of, %f.3, seconds", name, runCount, runtimecounter.seconds()));
        }
    }
}
