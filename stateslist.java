package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.measurements.mmPerInch;
import static org.firstinspires.ftc.teamcode.move.bettermove;
import static org.firstinspires.ftc.teamcode.move.setMyMotorTargets;
import static org.firstinspires.ftc.teamcode.move.spinToHeading;
import static org.firstinspires.ftc.teamcode.preciseMovement.mm2pulses;
import static org.firstinspires.ftc.teamcode.preciseMovement.spin2mm;
import static org.firstinspires.ftc.teamcode.robotconfig.armMotor;
import static org.firstinspires.ftc.teamcode.robotconfig.disableMotorBreak;
import static org.firstinspires.ftc.teamcode.robotconfig.enableMotorBreak;
import static org.firstinspires.ftc.teamcode.robotconfig.getCurrentAngle;
import static org.firstinspires.ftc.teamcode.robotconfig.getGoldPosition;
import static org.firstinspires.ftc.teamcode.robotconfig.goldLocation.CENTER;
import static org.firstinspires.ftc.teamcode.robotconfig.goldLocation.UNKNOWN;
import static org.firstinspires.ftc.teamcode.robotconfig.lastGoldLocation;
import static org.firstinspires.ftc.teamcode.robotconfig.lift;
import static org.firstinspires.ftc.teamcode.robotconfig.move;
import static org.firstinspires.ftc.teamcode.robotconfig.oldRobot;
import static org.firstinspires.ftc.teamcode.robotconfig.slide;
import static org.firstinspires.ftc.teamcode.robotconfig.tfod;
import static org.firstinspires.ftc.teamcode.robotconfig.vexy;
import static org.firstinspires.ftc.teamcode.robotconfig.vexy2;
import static org.firstinspires.ftc.teamcode.teleOp.armPositionFinder;
import static org.firstinspires.ftc.teamcode.teleOp.getAngleValue;
import static org.firstinspires.ftc.teamcode.teleOp.liftTargetPosition;
import static org.firstinspires.ftc.teamcode.teleOp.maxArmSensorValue;
import static org.firstinspires.ftc.teamcode.teleOp.minArmSensorValue;
import static org.firstinspires.ftc.teamcode.teleOp.pGain;
import static org.firstinspires.ftc.teamcode.teleOp.slidePositions;

// import static org.firstinspires.ftc.teamcode.robotconfig.robert;

/**
 * Created by mail2 on 11/15/2016.
 * Project: ftc_app_for_2016_robot
 */

/***
 * list of states for the state machine
 * each state can be referenced in other programs for use in various autonomous programs
 */
class stateslist {

    static ElapsedTime autoTimer = new ElapsedTime();

    static int currentState;//variable is used to control which state the state machine is currently running

    state sleep0 = new state("sleep0") {
        public void firstTime() {
            //try-catch is needed because it throws errors without it
            try {
                sleep(0);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };

    state sleep100 = new state("sleep100") {
        public void firstTime() {
            //try-catch is needed because it throws errors without it
            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };

    state sleep200 = new state("sleep200") {
        public void firstTime() {
            //try-catch is needed because it throws errors without it
            try {
                sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };

    state sleep500 = new state("sleep500") {
        public void firstTime() {
            //try-catch is needed because it throws errors without it
            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };

    state sleep1000 = new state("sleep1000") {
        public void firstTime() {
            //try-catch is needed because it throws errors without it
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };

    state sleep2000 = new state("sleep2000") {
        public void firstTime() {
            //try-catch is needed because it throws errors without it
            try {
                sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    };

    state squareUp = new state("squareUp") {
        public void firstTime() {
            double currentAngle = getCurrentAngle();
            Log.i("Downquark7", "currentAngle: " + currentAngle);
            if (Math.abs(currentAngle - (Math.round(currentAngle / 90) * 90)) < 90 / 3)
                setMyMotorTargets(0, 0, mm2pulses(spin2mm(currentAngle - (Math.round(currentAngle / 90) * 90))));
        }

        public boolean everyTime() {
            return bettermove();
        }

        public void onCompletion() {
            move(0, 0, 0);
        }
    };

    state triangleUp = new state("triangleUp") {
        public void firstTime() {
            double currentAngle = getCurrentAngle();
            Log.i("Downquark7", "currentAngle: " + currentAngle);
            setMyMotorTargets(0, 0, mm2pulses(spin2mm(currentAngle - (Math.round(currentAngle / 60) * 60))));
        }

        public boolean everyTime() {
            return bettermove();
        }

        public void onCompletion() {
            move(0, 0, 0);
        }
    };

    state notSureWhatThisIsUp = new state("notSureWhatThisIsUp") {
        public void firstTime() {
            double currentAngle = getCurrentAngle();
            Log.i("Downquark7", "currentAngle: " + currentAngle);
            setMyMotorTargets(0, 0, mm2pulses(spin2mm(currentAngle - (Math.round(currentAngle / 120) * 120))));
        }

        public boolean everyTime() {
            return bettermove();
        }

        public void onCompletion() {
            move(0, 0, 0);
        }
    };

    state octoganUp = new state("octoganUp") {
        public void firstTime() {
            double currentAngle = getCurrentAngle();
            Log.i("Downquark7", "currentAngle: " + currentAngle);
            if (Math.abs(currentAngle - (Math.round(currentAngle / 45)) * 45) < 45 / 3)
                setMyMotorTargets(0, 0, mm2pulses(spin2mm(currentAngle - (Math.round(currentAngle / 45) * 45))));
        }

        public boolean everyTime() {
            return bettermove();
        }

        public void onCompletion() {
            move(0, 0, 0);
        }
    };

    state dodecagonUp = new state("dodecagonUp") {
        public void firstTime() {
            double currentAngle = getCurrentAngle();
            Log.i("Downquark7", "currentAngle: " + currentAngle);
            setMyMotorTargets(0, 0, mm2pulses(spin2mm(currentAngle - (Math.round(currentAngle / 30) * 30))));
        }

        public boolean everyTime() {
            return bettermove();
        }

        public void onCompletion() {
            move(0, 0, 0);
        }
    };

    state noscope = new state("noscope") {//its name is a meme, plz disregard it
        public void firstTime() {
            //This is used to test the spin function
            setMyMotorTargets(0, 0, mm2pulses(spin2mm(360)));
        }

        public boolean everyTime() {
            //use the default custom autonomous move function=
            return bettermove();
        }

        public void onCompletion() {
            //stop on completion to measure where it ended
            move(0, 0, 0);
        }
    };

    state dropNScannArmVertical = new state("dropNScannArmVertical") {
        @Override
        public void firstTime() {
            tfod.activate();
            autoTimer.reset();
            if (!oldRobot) {
                lift.setPower(1);//giving power to lift
                lift.setTargetPosition(liftTargetPosition);//telling the lift how far to go
                vexy.setPower(0);
                vexy2.setPower(0);
            }
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.vertical;
            slide.setPower(0.001 * (slidePositions[0] - slide.getCurrentPosition()));
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target));
            else
                armMotor.setPower(0);
//                slide.setPower(0.001 * (1300 - slide.getCurrentPosition()));
            return lift.getCurrentPosition() > liftTargetPosition * 0.9 && (getGoldPosition() != UNKNOWN || autoTimer.seconds() > 8);
        }

        @Override
        public void onCompletion() {
            tfod.shutdown();
            slide.setPower(0);
        }
    };

    state dropNScann = new state("dropNscann") {
        @Override
        public void firstTime() {
            tfod.activate();
            autoTimer.reset();
            if (!oldRobot) {
                lift.setPower(1);//giving power to lift
                lift.setTargetPosition(liftTargetPosition);//telling the lift how far to go
                vexy.setPower(0);
                vexy2.setPower(0);
            }
        }

        @Override
        public boolean everyTime() {
//                slide.setPower(0.001 * (1300 - slide.getCurrentPosition()));
            return lift.getCurrentPosition() > liftTargetPosition * 0.9 && (getGoldPosition() != UNKNOWN || autoTimer.seconds() > 8);
        }

        @Override
        public void onCompletion() {
            tfod.shutdown();
            slide.setPower(0);
        }
    };


    state scan = new state("scan") {
        @Override
        public void firstTime() {
            tfod.activate();
            autoTimer.reset();
        }

        @Override
        public boolean everyTime() {
            return (getGoldPosition() != UNKNOWN || autoTimer.seconds() > 8);
        }

        @Override
        public void onCompletion() {
            tfod.shutdown();
        }
    };

    state dropNScannWithArm = new state("dropNscann with arm") {
        @Override
        public void firstTime() {
            tfod.activate();
            autoTimer.reset();
            if (!oldRobot) {
                lift.setPower(1);//giving power to lift
                lift.setTargetPosition(liftTargetPosition);//telling the lift how far to go
                vexy.setPower(0);
                vexy2.setPower(0);
            }
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.vertical;
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target));
            else
                armMotor.setPower(0);
//            slide.setPower(0.001 * (slidePositions[3] - slide.getCurrentPosition()));
            return lift.getCurrentPosition() > liftTargetPosition * 0.9 && (getGoldPosition() != UNKNOWN || autoTimer.seconds() > 8);
        }

        @Override
        public void onCompletion() {
            tfod.shutdown();
//            slide.setPower(0);
        }
    };

    state startLoweringLift = new state("startLoweringLift") {
        @Override
        public void firstTime() {
            lift.setTargetPosition(0);
        }
    };

    state startRaisingLift = new state("startRaisingLift") {
        @Override
        public void firstTime() {
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
            lift.setTargetPosition(liftTargetPosition);
        }
    };

    state lowerLift = new state("lowerLift") {
        @Override
        public void firstTime() {
            lift.setPower(1);
            lift.setTargetPosition(0);
        }

        @Override
        public boolean everyTime() {
            return !lift.isBusy();
        }
    };

    state stopLift = new state("stop lift") {
        @Override
        public void firstTime() {
            lift.setPower(0);
        }
    };

    state getOffHookPt1 = new state("getOffHookPt1", 0, 0, -15);
    state getOffHookPt2 = new state("getOffHookPt2", 2, 0, 0);
    state getOffHookPt3 = new state("getOffHookPt3", 0, 0, 15);

    state extendArmForMarkerWithDriveForward15 = new state("extend arm for marker with drive forward 15") {
        @Override
        public void firstTime() {
            setMyMotorTargets(mm2pulses(15 * mmPerInch), 0, 0);
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.horizontal;
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target) + 0.4);
            else
                armMotor.setPower(0);
            slide.setPower(0.001 * (slidePositions[3] - slide.getCurrentPosition()));
            return bettermove();
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0.3);
        }

    };

    state holdArmExtendedForASecond = new state("holds arm extended for a second") {
        ElapsedTime armTimer = new ElapsedTime();

        @Override
        public void firstTime() {
            armTimer.reset();
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.horizontal;
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target) + 0.4);
            else
                armMotor.setPower(0);
            return armTimer.seconds() > 1;
        }

        @Override
        public void onCompletion() {
            armMotor.setPower(0);
        }
    };

    //hi evan

    state holdArmExtendedForever = new state("holds arm extended forever") {

        @Override
        public boolean everyTime() {
            double target = armTarget.hh;
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target));
            else
                armMotor.setPower(0);
            return false;
        }

        @Override
        public void onCompletion() {
            armMotor.setPower(0);
        }
    };

    state extendArm = new state("extends out the arm") {
        @Override
        public boolean everyTime() {
            double target = armTarget.horizontal;
            slide.setPower(1);
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target) + 0.4);
            else
                armMotor.setPower(0);
            return slide.getCurrentPosition() > slidePositions[2];
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
        }
    };

    state rotateToSample = new state("rotateToSample") {
        @Override
        public void firstTime() {
            switch (lastGoldLocation) {
                case LEFT:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(-25)));
                    break;
                case CENTER:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(0)));
                    break;
                case RIGHT:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(28)));
                    break;
            }
        }

        @Override
        public boolean everyTime() {
            return bettermove();
        }
    };

    state rotateFromSample = new state("rotateFromSample") {
        @Override
        public void firstTime() {
            switch (lastGoldLocation) {
                case LEFT:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(28)));
                    break;
                case CENTER:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(0)));
                    break;
                case RIGHT:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(-28)));
                    break;
            }
        }

        @Override
        public boolean everyTime() {
            return bettermove();
        }
    };

    state rotateToBox = new state("rotate to box") {
        @Override
        public void firstTime() {
            switch (lastGoldLocation) {
                case LEFT:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(56)));
                    break;
                case CENTER:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(0)));
                    break;
                case RIGHT:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(-56)));
                    break;
            }
        }

        @Override
        public boolean everyTime() {
            return bettermove();
        }
    };

    state slowDownVexy = new state("slow down vexy") {
        @Override
        public void firstTime() {
            vexy.setPower(0.2);
            vexy2.setPower(-.2);
        }
    };

    state goToBox = new state("go to box") {
        @Override
        public void firstTime() {
            switch (lastGoldLocation) {
                case LEFT:
                    setMyMotorTargets(mm2pulses(mmPerInch * 32), 0, 0);
                    break;
                case CENTER:
                    setMyMotorTargets(mm2pulses(mmPerInch * 20), 0, 0);
                    break;
                case RIGHT:
                    setMyMotorTargets(mm2pulses(mmPerInch * 32), 0, 0);
                    break;
                case UNKNOWN:
                    setMyMotorTargets(mm2pulses(mmPerInch * 20), 0, 0);
                    break;
            }
        }

        @Override
        public boolean everyTime() {
            return bettermove();
        }
    };

/*
state resetMarkerArm = new state("reset marker arm") {
        @Override
        public void initReverse() {
            robert.setPosition(0);
        }
    };

    state dropMarker = new state("deposit marker") {
        @Override
        public void firstTime() {
            robert.setPosition(1);
        }
    };\
*/

    state reverseVexy = new state("reverse vexy") {
        @Override
        public void firstTime() {
            vexy.setPower(-0.75);
            vexy2.setPower(.75);
        }
    };

    state deactivateVexy = new state("deactivate vexy") {
        @Override
        public void firstTime() {
            vexy.setPower(0);
            vexy2.setPower(0);
        }
    };

    state activateVexy = new state("activate vexy") {
        @Override
        public void firstTime() {
            vexy.setPower(0.75);
            vexy2.setPower(-.75);
        }
    };

    state obtainCubeWithRotate = new state("obtain cube with rotate") {
        @Override
        public void firstTime() {
            autoTimer.reset();
            vexy.setPower(0.75);
            vexy2.setPower(-.75);
            switch (lastGoldLocation) {
                case LEFT:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(-25)));
                    break;
                case CENTER:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(3)));
                    break;
                case RIGHT:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(29)));
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
    };

    state obtainCubeWithRotateFullExtension = new state("obtain cube with rotate full extension") {
        double targetHeading = 0;
        @Override
        public void firstTime() {
            autoTimer.reset();
            vexy.setPower(0.75);
            vexy2.setPower(-.75);
            switch (lastGoldLocation) {
                case LEFT:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(-25)));
                    targetHeading = -25;
                    break;
                case CENTER:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(1)));
                    targetHeading = -1;
                    break;
                case RIGHT:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(29)));
                    targetHeading = -29;
                    break;
            }
        }

        @Override
        public boolean everyTime() {
            double x = 1.0 * slide.getCurrentPosition();
            double target = slide.getCurrentPosition() > slidePositions[0] * 2 ? armPositionFinder(x) : ((x - slidePositions[0]) / slidePositions[0]) * (armPositionFinder(slidePositions[0] * 2) - 2.33) + 2.33;
            slide.setPower(1);
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target) + 0.2);
            else
                armMotor.setPower(0);
            return spinToHeading(targetHeading) && slide.getCurrentPosition() > slidePositions[3];
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
            move(0, 0, 0);
        }
    };
    state obtainCubeFullExtension = new state("obtain cube full extension") {
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
            return (bettermove() && slide.getCurrentPosition() > slidePositions[3]);
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
            move(0, 0, 0);
        }
    };


    state obtainCube = new state("obtain cube") {
        @Override
        public void firstTime() {
            autoTimer.reset();
            vexy.setPower(0.75);
            vexy2.setPower(-.75);
        }

        @Override
        public boolean everyTime() {
            double x = 1.0 * slide.getCurrentPosition();
            double target = armPositionFinder(x);
            slide.setPower(0.7);
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target) + 0.2);
            else
                armMotor.setPower(0);
            return slide.getCurrentPosition() > slidePositions[3];
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
            move(0, 0, 0);
        }
    };

    state scoreCube = new state("score cube") {
        @Override
        public void firstTime() {
            autoTimer.reset();
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.autoScorePos;
            slide.setPower(0.001 * (slidePositions[1] - slide.getCurrentPosition()));
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target));
            else
                armMotor.setPower(0);
            return slide.getCurrentPosition() > slidePositions[1] && Math.abs(armMotor.getPower()) < 0.3;
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
        }
    };

    state scoreCubeWithRotate = new state("score cube with rotate") {
        @Override
        public void firstTime() {
            autoTimer.reset();

            switch (lastGoldLocation) {
                case LEFT:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(28)));
                    break;
                case CENTER:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(0)));
                    break;
                case RIGHT:
                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(-28)));
                    break;
            }
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.depotSideY;
            slide.setPower(0.001 * (slidePositions[1] - slide.getCurrentPosition()));
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target));
            else
                armMotor.setPower(0);
            return spinToHeading(0) && Math.abs(slide.getPower()) < 0.3 && Math.abs(armMotor.getPower()) < 0.3;
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
            move(0, 0, 0);
        }
    };

    state scoreCubeWithRotateCraterEdition = new state("score cube with rotate for the crater side") {
        @Override
        public void firstTime() {
            autoTimer.reset();
//            switch (lastGoldLocation) {
//                case LEFT:
//                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(48)));
//                    break;
//                case CENTER:
//                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(20)));
//                    break;
//                case RIGHT:
//                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(-8)));
//                    break;
//                case UNKNOWN:
//                    setMyMotorTargets(0, 0, mm2pulses(spin2mm(20)));
//                    break;
//            }
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.autoScorePos;
            slide.setPower(0.001 * (slidePositions[5] - slide.getCurrentPosition()));
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target));
            else
                armMotor.setPower(0);
            return spinToHeading(-15) && Math.abs(slide.getPower()) < 0.3 && Math.abs(armMotor.getPower()) < 0.3;
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
            move(0, 0, 0);
        }
    };

    state scoreMineralsCraterSide = new state("score minerals for the crater side") {
        @Override
        public void firstTime() {
            autoTimer.reset();
            setMyMotorTargets(0, 0, mm2pulses(spin2mm(20)));
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.autoScorePos;
            slide.setPower(0.001 * (slidePositions[5] - slide.getCurrentPosition()));
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target));
            else
                armMotor.setPower(0);
            return spinToHeading(-15) && Math.abs(slide.getPower()) < 0.3 && Math.abs(armMotor.getPower()) < 0.3;
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
            move(0, 0, 0);
        }
    };

    state scoreMineralsCraterSideIfNotCenter = new state("score minerals for the crater side if not center") {
        @Override
        public void firstTime() {
            autoTimer.reset();
            setMyMotorTargets(0, 0, mm2pulses(spin2mm(20)));
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.autoScorePos;
            slide.setPower(0.001 * (slidePositions[5] - slide.getCurrentPosition()));
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target));
            else
                armMotor.setPower(0);
            return (bettermove() && Math.abs(slide.getPower()) < 0.3 && Math.abs(armMotor.getPower()) < 0.3) || lastGoldLocation != CENTER;
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
            move(0, 0, 0);
        }
    };

    state armVertical = new state("put arm into vertical position") {
        @Override
        public void firstTime() {
            autoTimer.reset();
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.vertical;
            slide.setPower(0.001 * (slidePositions[0] - slide.getCurrentPosition()));
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target));
            else
                armMotor.setPower(0);
            return Math.abs(slide.getPower()) < 0.3 && Math.abs(armMotor.getPower()) < 0.3;
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
        }
    };

    state armAtCarry = new state("put arm into carry position") {
        @Override
        public void firstTime() {
            autoTimer.reset();
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.horizontal;
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target) + 0.1);
            else
                armMotor.setPower(0);
            slide.setPower(0.001 * (slidePositions[0] - slide.getCurrentPosition()));
            return slide.getCurrentPosition() < 1.1 * slidePositions[0] && Math.abs(armMotor.getPower()) < 0.3;
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
            move(0, 0, 0);
        }
    };

    state armDownAtCarry = new state("put arm down at carry at the get ready to intake carry position") {
        @Override
        public void firstTime() {
            autoTimer.reset();
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.down;
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target) + 0.1);
            else
                armMotor.setPower(0);
            slide.setPower(0.001 * (slidePositions[0] - slide.getCurrentPosition()));
            return slide.getCurrentPosition() < 1.1 * slidePositions[0] && Math.abs(armMotor.getPower()) < 0.3;
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
            move(0, 0, 0);
        }
    };

    state armAtCarryWithDriveBackward15 = new state("put arm into carry position with drive backward 15") {
        @Override
        public void firstTime() {
            autoTimer.reset();
            setMyMotorTargets(mm2pulses(-15 * mmPerInch), 0, 0);
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.horizontal;
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target) + 0.1);
            else
                armMotor.setPower(0);
            slide.setPower(0.001 * (slidePositions[0] - slide.getCurrentPosition()));
            return bettermove() && slide.getCurrentPosition() < 1.1 * slidePositions[0] && Math.abs(armMotor.getPower()) < 0.3;
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
            move(0, 0, 0);
        }
    };

    state extendedArmAtCarryWithDriveBackward15 = new state("put arm into an extended carry position with drive backward 15") {
        @Override
        public void firstTime() {
            autoTimer.reset();
            setMyMotorTargets(mm2pulses(-15 * mmPerInch), 0, 0);
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.horizontal;
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target) + 0.1);
            else
                armMotor.setPower(0);
            slide.setPower(0.001 * (slidePositions[0] * 2 - slide.getCurrentPosition()));
            return bettermove() && slide.getCurrentPosition() < 2.1 * slidePositions[0] && Math.abs(armMotor.getPower()) < 0.3;
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
            move(0, 0, 0);
        }
    };

    state armAtCarryWithDriveForward15 = new state("put arm into carry position with drive forward 15") {
        @Override
        public void firstTime() {
            autoTimer.reset();
            setMyMotorTargets(mm2pulses(15 * mmPerInch), 0, 0);
        }

        @Override
        public boolean everyTime() {
            double target = armTarget.horizontal;
            if (getAngleValue() > minArmSensorValue && getAngleValue() < maxArmSensorValue)
                armMotor.setPower(pGain * (getAngleValue() - target) + 0.1);
            else
                armMotor.setPower(0);
            slide.setPower(0.001 * (slidePositions[0] - slide.getCurrentPosition()));
            return bettermove() && slide.getCurrentPosition() < 1.1 * slidePositions[0];
        }

        @Override
        public void onCompletion() {
            slide.setPower(0);
            armMotor.setPower(0);
            move(0, 0, 0);
        }
    };

    state coastForward = new state("coast forward") {
        @Override
        public void firstTime() {
            disableMotorBreak();
            move(1, 0, 0);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            move(0, 0, 0);
        }
    };

    state coastBackward = new state("coast backward") {
        @Override
        public void firstTime() {
            disableMotorBreak();
            move(-1, 0, 0);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            move(0, 0, 0);
            enableMotorBreak();
        }
    };
}
