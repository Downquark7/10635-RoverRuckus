package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.openftc.revextensions2.ExpansionHubEx;

import static org.firstinspires.ftc.teamcode.robotconfig.angle;
import static org.firstinspires.ftc.teamcode.robotconfig.angle2;
import static org.firstinspires.ftc.teamcode.robotconfig.armMotor;
import static org.firstinspires.ftc.teamcode.robotconfig.enableMotorBreak;
import static org.firstinspires.ftc.teamcode.robotconfig.enableMotorEncoders;
import static org.firstinspires.ftc.teamcode.robotconfig.hang;
import static org.firstinspires.ftc.teamcode.robotconfig.hub3;
import static org.firstinspires.ftc.teamcode.robotconfig.hub;
import static org.firstinspires.ftc.teamcode.robotconfig.initGyro;
import static org.firstinspires.ftc.teamcode.robotconfig.initHubs;
import static org.firstinspires.ftc.teamcode.robotconfig.move;
import static org.firstinspires.ftc.teamcode.robotconfig.rinit;
import static org.firstinspires.ftc.teamcode.robotconfig.slide;
import static org.firstinspires.ftc.teamcode.robotconfig.vexy;
import static org.firstinspires.ftc.teamcode.robotconfig.vexy2;

@TeleOp
@Disabled
public class teleOp extends OpMode {

    //powers for all motors
    private double forward, right, spin, lift, armSwing, slidePower;

    // variable that says whether slide proportional controller is enabled
    private boolean slideHolding = false;

    // main hanging lift: if 1, automatically controlled; if 0, manually controlled
    int liftButtonState = 0;
    //timer to stop lift motor after a set amount of time
    ElapsedTime liftTimer = new ElapsedTime();

    //target slide positions for slide proportional controller {carry pos, scoring pos, autonomous pos, maximum pos}
    static final int OriginalSlidePositions[] = {750, 2280, 3700, 5000, 2000, 2600};
    static final double slideRatio = 13.7 / 19.5;
    static final int maxSlidePosition = 2900;
    static final int slidePositions[] = {(int) (maxSlidePosition * 0.2), (int) (maxSlidePosition * 0.5), (int) (3700 * slideRatio), maxSlidePosition, (int) (2000 * slideRatio), 2000};
    static int liftTargetPosition = 7500;

    //true if arm porportional controller is active
    private boolean armHolding = false;
    //set min and max values of arms to disable arm controller if outside of the acceptable range to prevent damage
    static final double minArmSensorValue = 0.2;
    static final double maxArmSensorValue = 2.82;
    //other arm motor variables
    boolean intakeMode1 = false;
    boolean intakeMode2 = false;
    static final double pGain = 2.5;
    double target = (minArmSensorValue + maxArmSensorValue) / 2;
    boolean previousA1State = false;
    boolean previousA2State = false;
    boolean gamepad1MovedArm = false;
    boolean gamepad2MovedArm = false;

    //variable used to have a multiple stage interface for going beyond the minimum arm position
    int armMovingDownState = 0;

    //slide motor variables
    final int slideMotorPPr = (int) (536.7 / 3); //allows slide positions to be adjusted on the fly in increments of one third rotation
    int slideMotorOffset = 0;
    boolean previousDpad2State = false;

    //timers
    ElapsedTime pTimer = new ElapsedTime();
    //    ElapsedTime sTimer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();
    double loopTime = 0;
    int loopCount = 0;

    boolean isDepotSide = false;

    //for vex motor efficiency
    boolean vexyAtZero = true;

    static public double getAngleValue() {
        double angleValue = angle.getVoltage();
        if (angleValue > minArmSensorValue && angleValue < maxArmSensorValue)
            return angleValue;
        double angleValue2 = angle2.getVoltage();
        return 6.63 - 7.12 * angleValue2 + 3.2 * Math.pow(angleValue2, 2) - 0.514 * Math.pow(angleValue2, 3);
    }


    //hubs for power draw data and faster usb connections
//    private ExpansionHubEx hub;
//    private ExpansionHubEx hub2;
//    private RevBulkData data;
//    private RevBulkData data2;

    double groundFollowingOffset = 0;

//    boolean extendArmMode = false;

    double previousRight = 0;

//    FtcDashboard dashboard;
//    ElapsedTime yButtonTimer = new ElapsedTime();

    MediaPlayer media = null;

    @Override
    public void init() {

        FtcDashboard.stop();


        //init robot motors and servos
        rinit(this);
//        initGyro();

        enableMotorEncoders();
        enableMotorBreak();
        robotconfig.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotconfig.lift.setPower(0);

        initHubs();
        telemetry.addData("arm current", () -> (float) (hub3.getMotorCurrentDraw(2) / 1000));
        telemetry.addData("total power", () -> (float) ((hub3.getTotalModuleCurrentDraw() + hub.getTotalModuleCurrentDraw()) * hub3.read12vMonitor() / 1000000));

//        initGyro();

        move(0, 0, 0);

        // init telemetry as functions in init for increased performance


        telemetry.addData("depot side", () -> isDepotSide);

        telemetry.addData("loop", () -> (int) loopTime);

//        initDrivetrainTelemetry(telemetry);

        telemetry.addData("lift", () -> robotconfig.lift.getCurrentPosition());

        telemetry.addData("slide", () -> slide.getCurrentPosition());

        telemetry.addData("arm", () -> (float) (angle.getVoltage()));

        telemetry.addData("arm2", () -> (float) (angle2.getVoltage()));

//        telemetry.addData("arm e", () -> (float) (angle.getVoltage() - target));

        telemetry.addData("arm target", () -> (float) (target));

//        telemetry.addData("gyro", new Func<Float>() { @Override public Float value() { return (float) (getCurrentAngle()); } });

        //power draw data
//
//        telemetry.addData("lift current", new Func<Float>() { @Override public Float value() { return (float) (hub.getMotorCurrentDraw(0) / 1000); } });
//
//        telemetry.addData("slides current", new Func<Float>() { @Override public Float value() { return (float) (hub.getMotorCurrentDraw(1) / 1000); } });
//

        //and push telemetry
        telemetry.update();

        try {
            media = MediaPlayer.create(hardwareMap.appContext, R.raw.tunak);
        } catch (Exception e) {
            Log.e("Downquark7", "error playing media: " + e.getMessage());
        }
    }


    @Override
    public void start() {
        pTimer.reset();
        loopTimer.reset();
        hang.setPower(0);

        if (media != null) media.start();
    }

    //equation was made by gathering points and putting them into a regression program
    static double armPositionFinder(double x) {
        return Math.min(2500.0, -0.152108 * x + 2540) / 1000.0;
    }

    static double armPositionFinder(double x, double offset) {
        return armPositionFinder(x);
    }

    void initArmHolding(double targetPosition) {
        target = targetPosition;
        armHolding = true;
    }

    int slideTargetPosition = 0;

    void initSlideHolding(int targetPosition) {
        slideTargetPosition = targetPosition;
        slideHolding = true;
    }

    public void init_loop() {

        //listen for depot side toggle during init as well as during the loop
        if (gamepad1.start)
            if (gamepad1.y)
                isDepotSide = true;
            else if (gamepad1.x)
                isDepotSide = false;

        //and the same for the second joystick
        if (gamepad2.start)
            if (gamepad2.y)
                isDepotSide = true;
            else if (gamepad2.x)
                isDepotSide = false;

        if (Math.abs(slide.getCurrentPosition()) > 100)
            hang.setPower(0.5);
        else
            hang.setPower(0);
    }

    Boolean previousRightBumper = false;

    @Override
    public void loop() {

//        RevBulkData data = hub3.getBulkInputData();
        double slidePos = 1.0 * slide.getCurrentPosition() + slideMotorOffset; //offset allows slide encoder position to be moved on the fly if the string falls off the pulley

        double armLengthTurningCoeff = 0.25 * (1.2 * maxSlidePosition) / (slidePos + 0.2 * maxSlidePosition);

        // drive motor powers use both controllers with other modes at different speeds
        forward = -1 * (gamepad1.left_stick_y);
        right = (intakeMode1 ? (gamepad1.left_stick_x) * 0.5 : (gamepad1.left_stick_x)) + (gamepad2.left_stick_x) * 0.5;
        spin = (gamepad1.left_bumper || intakeMode1 ? (gamepad1.right_stick_x) * (intakeMode1 ? armLengthTurningCoeff : 0.5) : Math.pow(gamepad1.right_stick_x, 3)) + (gamepad2.right_stick_x) * armLengthTurningCoeff;
//        if(Math.abs(spin) < 0.3) {
//            double currentAngle = getCurrentAngle();
//            double error = 0;
//            double angSpeed = 0;
//
//            if (!intakeMode2 && !intakeMode1 && (gamepad1.right_bumper || Math.abs(currentAngle) < 10)) {
//                error = currentAngle;
//                angSpeed = getAngularSpeed();
//                spin = error * 0.03 - 0.002 * angSpeed;
//            }
//        }

        //slide extension power
        slidePower = -1 * (gamepad2.left_stick_y);

//        double slidePos = 1.0 * data.getMotorCurrentPosition(slide) + slideMotorOffset;

        //drive the robot forward instead of extending out the slide if the slide is at the max distance
        if (slidePos > slidePositions[3] && slidePower > 0) {
            forward += slidePower * 0.5;
            slidePower = 0;
        }

        //code for intake mode so the robot can work with a single driver
        if (intakeMode1) {
            if (forward > 0) {
                if (slidePos > slidePositions[3]) {
                    forward *= 0.5;
                } else {
                    slidePower += forward;
                    forward = 0;
                }
            } else {
                if (slidePos < slidePositions[0]) {
                    forward *= 0.5;
                } else {
                    slidePower += forward;
                    forward = 0;
                }
            }
            forward += gamepad1.right_trigger - gamepad1.left_trigger;
        }

        forward += gamepad2.right_trigger - gamepad2.left_trigger;

        if (gamepad1.right_bumper) {
            if (!previousRightBumper) {
                initArmHolding(armTarget.horizontal);
                initSlideHolding(slidePositions[1]);
                previousRightBumper = true;
            }
        } else if (previousRightBumper) {
            intakeMode1 = false;
            if (!intakeMode2) {
                initArmHolding(armTarget.hh);
                initSlideHolding(slidePositions[0]);
            }
            previousRightBumper = false;
        }

        //code for increasing or decreasing the slide motor encoder offset when the dpad is pressed
        if (gamepad2.dpad_up) {
            if (!previousDpad2State)
                if (intakeMode2)
                    groundFollowingOffset -= 0.05;
                else
                    slideMotorOffset -= slideMotorPPr;
            previousDpad2State = true;
        } else if (gamepad2.dpad_down) {
            if (!previousDpad2State)
                if (intakeMode2)
                    groundFollowingOffset += 0.05;
                else
                    slideMotorOffset += slideMotorPPr;
            previousDpad2State = true;
        } else
            previousDpad2State = false;

        final double rightIncrement = 0.3;
        right = Range.clip(right, previousRight - rightIncrement, previousRight + rightIncrement);
        previousRight = right;

        //convert these variables into the powers to go to the motors and set motor powers
        move(forward, right, spin);

//        double angleValue = data.getAnalogInputValue(angle) > minArmSensorValue * 1000 ? data.getAnalogInputValue(angle) / 1000.0 : 0.0007076 * data.getAnalogInputValue(angle2) + 0.14474;
        //code to get the potentiometer value; if the first one is broken or unplugged, it uses the second one automatically
        double angleValue = getAngleValue();
        //(angle.getVoltage() > minArmSensorValue ? angle.getVoltage() : 0.7076 * angle2.getVoltage() + 0.14474);

        //curve joystick value to set as initial arm power
        armSwing = -1 * Math.pow(gamepad2.right_stick_y, 3);

        //intake motor code
        if (gamepad2.right_bumper || (!intakeMode1 && gamepad1.a)) {
            vexy.setPower(-0.75);
            vexy2.setPower(.75);
            vexyAtZero = false;
        } else if (gamepad2.left_bumper || intakeMode1) {
            vexy.setPower(0.75);
            vexy2.setPower(-.75);
            vexyAtZero = false;
        } else if (angleValue < armTarget.vertical && angleValue > minArmSensorValue) { //automatically open mineral door
            if (vexyAtZero) {
                vexy.setPower(-.75);
                vexy2.setPower(.75);
                vexyAtZero = false;
            }
//        } else if (angleValue < armTarget.hh && angleValue > armTarget.vertical + 0.2 && angleValue > minArmSensorValue) { //automatically open mineral door
//            if (vexyAtZero) {
//                vexy.setPower(.75);
//                vexy2.setPower(-.75);
//                vexyAtZero = false;
//            }
        } else if (!vexyAtZero) {
            vexy.setPower(0.25);
            vexy2.setPower(-0.25);
            vexyAtZero = true;
        }

        //initial lift power is set by the joystick triggers
        if (!intakeMode1)
            lift = gamepad1.right_trigger - gamepad1.left_trigger;

        //start automatically moving up to full height
        if (gamepad1.dpad_up) {
            robotconfig.lift.setTargetPosition(liftTargetPosition);
            liftTimer.reset();
            if (liftButtonState == 0) {
                robotconfig.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotconfig.lift.setPower(1);
            }
            liftButtonState = 1;
        }

        //start automatically moving down to start position
        if (gamepad1.dpad_down) {
            robotconfig.lift.setTargetPosition(0);
            liftTimer.reset();
            if (liftButtonState == 0) {
                robotconfig.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotconfig.lift.setPower(1);
            }
            liftButtonState = 1;
        }

        //disable automatic lift movement as soon as manual movement buttons are pressed or it times out
        if (liftButtonState == 1 && (Math.abs(lift) > 0.1 || liftTimer.seconds() > 7)) {
            robotconfig.lift.setPower(0);
            robotconfig.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftButtonState = 0;
        }

        //set power if in manual mode
        if (liftButtonState == 0)
            robotconfig.lift.setPower(lift);

        //disable intake mode if arm moved by second gamepad
        if (Math.abs(armSwing) > 0.1) {
            intakeMode2 = false;
            gamepad2MovedArm = true;
        }

        //add in first joystick to arm movement if in intake mode
        if (intakeMode1 && Math.abs(gamepad1.right_stick_y) > 0.3) {
            armSwing += -1 * Math.pow(gamepad1.right_stick_y, 3);
            gamepad1MovedArm = true;
        }

        //actively hold arm position when it's not manually moved
        if (Math.abs(armSwing) > 0.2) {
            armHolding = false;
            pTimer.reset();
        } else if (!armHolding && pTimer.milliseconds() > 300)
            initArmHolding(angleValue);

//        //write values to logcat to calculate regression equations
        if (gamepad2.dpad_right)
            Log.i("new arm values", "val:" + angle2.getVoltage() + ", " + angle.getVoltage());
//        if (gamepad2.left_trigger > 0.1) {
////            if (!previousDpad)
//            Log.i("new values", "val:" + slide.getCurrentPosition() + " " + angleValue);
//            previousDpad = true;
//        } else previousDpad = false;

        //actively hold slide at current position when not manually being moved
        if (Math.abs(slidePower) > 0.2) {
            slideHolding = false;
//            sTimer.reset();
        } else if (!slideHolding)
            initSlideHolding((int) slidePos);

        //set slide power to proportional control value that takes into account the angle of the arm so it doesn't get stuck on the lander
        if (slideHolding)
//            slidePower = Math.max(0.0025 * (slideTargetPosition - slidePos), -0.8);
            slidePower = ((angleValue > armTarget.vertical && angleValue < armTarget.down) || slidePos > slidePositions[1] ? Math.max(0.004 * (slideTargetPosition - slidePos), -0.4) : Math.max(0.004 * (slideTargetPosition - slidePos), 0));
        slide.setPower(slidePower);

//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("slidePos", slidePos / 1000);
//        packet.put("angleValue", angleValue);
//        packet.put("target", target);
//        packet.put("yButtonTimer", Math.min(12, yButtonTimer.seconds()) / 2);
////        packet.put("dashboard loop timer", dashboardTimer.milliseconds() / 10);
////        dashboardTimer.reset();
//        packet.put("arm motor power", armMotor.getPower());
//        packet.put("slide motor power", slide.getPower());
//        dashboard.sendTelemetryPacket(packet);

        //hold arm position if in intake mode
        if (((intakeMode1 && !gamepad1MovedArm) || intakeMode2) && angleValue > armTarget.slightUp && slidePos > slidePositions[0])
            initArmHolding(slidePos > maxSlidePosition * 0.25 ? armPositionFinder(slidePos, groundFollowingOffset) : (armPositionFinder(maxSlidePosition * 0.25, groundFollowingOffset) - armTarget.horizontal) * ((slidePos) / (maxSlidePosition * 0.25)) + armTarget.horizontal);

//        if (!intakeMode1 && !intakeMode2 && gamepad2.right_trigger > 0.2) {
//            initArmHolding(armTarget.horizontal);
//            initSlideHolding((int) (slidePositions[3] * 0.4));
//            intakeMode2 = true;
//        }

        //move to next state after stick is released
        if (armMovingDownState == 1 && Math.abs(armSwing) < 0.1) {
            armMovingDownState = 2;
        }

        //go to slide position on button press
        if (gamepad1.x && !gamepad1.start) {
            initSlideHolding(0);
//            extendArmMode = false;
            if (angleValue > armTarget.horizontal || angleValue < minArmSensorValue) {
                intakeMode1 = true;
            }
        }
        //same for gamepad 2
        if (gamepad2.x && !gamepad2.start) {
            initSlideHolding(0);
//            extendArmMode = false;
            if (angleValue > armTarget.horizontal || angleValue < minArmSensorValue) {
                intakeMode2 = true;
            }
        }

        //go to slide and arm positions on button press and activate intake mode
        if (gamepad1.a && !gamepad1.start && gamepad1.right_bumper) {
            if (!previousA1State) {
                intakeMode2 = false;
//                extendArmMode = false;
                if (intakeMode1) {
                    initSlideHolding(slidePositions[0]);
                    intakeMode1 = false;
                } else {
                    if (angleValue > armTarget.vertical || angleValue < minArmSensorValue)
                        intakeMode1 = true;
                    else
                        initSlideHolding(slidePositions[0]);
                }

                if (target < armTarget.vertical && angleValue > armTarget.vertical)
                    armHolding = false;
                else
                    initArmHolding(intakeMode1 ? armTarget.down : armTarget.hh);

                gamepad1MovedArm = false;
            }
            previousA1State = true;
        } else
            previousA1State = false;

        //go to slide and arm positions on button press and activate intake mode
        if (gamepad2.a && !gamepad2.start) {
            if (!previousA2State) {
                intakeMode1 = false;
//                extendArmMode = false;
                if (gamepad1.right_bumper && !intakeMode2) {
                    intakeMode2 = true;
                } else {
                    if (intakeMode2 || gamepad2MovedArm) {
                        initSlideHolding(slidePositions[0]);
                        intakeMode2 = false;
//                    extendArmMode = true;
                    } else {
                        if (angleValue > armTarget.vertical)
                            intakeMode2 = true;
                        else
                            initSlideHolding(slidePositions[0]);
                    }
                }

                if (target < armTarget.vertical && angleValue > armTarget.vertical)
                    armHolding = false;
                else
                    initArmHolding(intakeMode2 ? armTarget.down : (angleValue < armTarget.vertical ? armTarget.horizontal : armTarget.hh));

                gamepad2MovedArm = false;
            }

            previousA2State = true;
        } else
            previousA2State = false;

//        if (gamepad2MovedArm)
//            extendArmMode = false;
//
//        if (extendArmMode) {
//            if (slidePos > 0.4 * maxSlidePosition && slidePower > 0.3) {
//                if (forward > 0.3)
//                    initArmHolding(armTarget.horizontal);
//                extendArmMode = false;
//            }
//        }

        //slide and arm to last scoring position
        if ((gamepad1.b && !gamepad1.start) || (gamepad2.b && !gamepad2.start)) {
            initArmHolding(armTarget.teleOpB);
            initSlideHolding((int) (maxSlidePosition * 0.4));
//            extendArmMode = false;
            intakeMode1 = false;
            intakeMode2 = false;
            armMotor.setPower(1);
        }

        if (gamepad1.start)
            if (gamepad1.y)
                isDepotSide = true;
            else if (gamepad1.x)
                isDepotSide = false;

        if (gamepad2.start)
            if (gamepad2.y)
                isDepotSide = true;
            else if (gamepad2.x)
                isDepotSide = false;

        //slide and arm to main scoring position
        if ((gamepad1.y && !gamepad1.start) || (gamepad2.y && !gamepad2.start)) {
            initArmHolding(isDepotSide ? armTarget.depotSideY : armTarget.depotSideY);
            initSlideHolding(isDepotSide ? (int) (maxSlidePosition * 0.46) : (int) (maxSlidePosition * 0.5));
//            extendArmMode = false;
            intakeMode1 = false;
            intakeMode2 = false;
            armMotor.setPower(1);
//            yButtonTimer.reset();
        }

        //manage manual and automatic arm control
        if (!armHolding) {
            if (armSwing > 0) {
                armMotor.setPower(armSwing);
                armMovingDownState = 0;
            } else {
                armMotor.setPower(angleValue < maxArmSensorValue || armMovingDownState == 2 ? armSwing : 0);
                if (angleValue > maxArmSensorValue) {
                    if (armMovingDownState == 0 && Math.abs(armSwing) > 0.1) {
                        armMovingDownState = 1;
                    }
                } else
                    armMovingDownState = 0;
            }
        } else {
            if (!gamepad1.y && !gamepad2.y && !gamepad1.b && !gamepad2.b)
                if (angleValue > minArmSensorValue)
                    if (angleValue > armTarget.horizontal)
                        armMotor.setPower((Math.abs(spin) > 0.1 || Math.abs(right) > 0.1) ? armController5(angleValue, target, slidePos) + 0.3 : armController5(angleValue, target, slidePos));
                    else if (angleValue < maxArmSensorValue)
                        armMotor.setPower(armController5(angleValue, target, slidePos));
                    else
                        armMotor.setPower(0);
                else
                    armMotor.setPower(0);
        }

        //check loop time
        if (loopCount % 100 == 0) {
            loopTime = loopTimer.milliseconds() / 100;
            loopTimer.reset();
        }
        loopCount++;
    }

    static double armController(double angleValue, double target) {
        double error = (angleValue - target);
        return pGain * error;
    }

    static double armControllerSlowed(double angleValue, double target) {
        double error = (angleValue - target);
        return 0.8 * pGain * error;
    }


    static double armControllerFast(double angleValue, double target) {
        double error = (angleValue - target);
        return 2 * pGain * error;
    }

    //new arm controller with probably more control
    static double armController2(double angleValue, double target) {
        double error = (angleValue - target);
        return Math.pow(error, 3) * 12 + 0.5 * error;
    }

    //new arm controller with a larger deadzone thing where it's practically at zero
    static double armController3(double angleValue, double target) {
        double error = (angleValue - target);
        return Math.pow(error, 3) * 12;
    }

    static private double armI = 0;

    static double armController4(double angleValue, double target) {
        double error = pGain * (angleValue - target);
        armI += 0.01 * error;
        armI = Range.clip(armI, 0, 0.5);
        if (Math.abs(error) > 0.5)
            armI = 0;
        return error + armI;
    }

    static double armController5(double angleValue, double target, double slidePos) {
        double error = (angleValue - target);
        if ((target >= armTarget.horizontal) || (target == armTarget.hh && angleValue > armTarget.hh))
            error *= 0.5;
        else if (target < armTarget.vertical && angleValue < target)
            error *= 0.1;
        else if (target < armTarget.vertical)
            error *= 1.2;
        return pGain * error;
    }
}
