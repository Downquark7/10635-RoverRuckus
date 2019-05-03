package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.util.JoystickTransform;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.teamcode.teleOp.armController5;
import static org.firstinspires.ftc.teamcode.teleOp.armPositionFinder;
import static org.firstinspires.ftc.teamcode.teleOp.liftTargetPosition;
import static org.firstinspires.ftc.teamcode.teleOp.maxArmSensorValue;
import static org.firstinspires.ftc.teamcode.teleOp.maxSlidePosition;
import static org.firstinspires.ftc.teamcode.teleOp.minArmSensorValue;
import static org.firstinspires.ftc.teamcode.teleOp.slidePositions;

@TeleOp
public class alphaOp extends LinearOpMode {

    boolean asyncLocationTracking = true;

    @Override
    public void runOpMode() {
        rinit();
        asyncLocationTracking locationTracking = new asyncLocationTracking(hardwareMap, drive.getLocalizer());
        while (!opModeIsActive() && !isStopRequested()) {
            rinit_loop();
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        if (asyncLocationTracking)
            locationTracking.start();
        rstart();
        telemetry.update();
        while (opModeIsActive() && !isStopRequested())
            rloop();
        locationTracking.stop();
    }

    //powers for all motors
    private double forward, right, spin, lift, armSwing, slidePower;

    // variable that says whether slide proportional controller is enabled
    private boolean slideHolding = false;

    // main hanging lift: if 1, automatically controlled; if 0, manually controlled
    int liftButtonState = 0;
    //timer to stop lift motor after a set amount of time
    ElapsedTime liftTimer = new ElapsedTime();

    //true if arm porportional controller is active
    private boolean armHolding = false;
    //set min and max values of arms to disable arm controller if outside of the acceptable range to prevent damage

    //other arm motor variables
    boolean intakeMode1 = false;
    boolean intakeMode2 = false;
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

    double groundFollowingOffset = 0;

    double previousRight = 0;

    SampleMecanumDriveREV drive;
    auxillaryDevices aux;


    void rinit() {

        FtcDashboard.stop();

        drive = new SampleMecanumDriveREV(hardwareMap);
        aux = new auxillaryDevices(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(-28, -28, Math.toRadians(-135)));

        // init telemetry as functions in init for increased performance
        telemetry.addData("depot side", () -> isDepotSide);
        telemetry.addData("loop", () -> (int) loopTime);
        telemetry.addData("target", () -> (float) target);
        telemetry.addData("slideTarget", () -> (float) slideTargetPosition);

        //and push telemetry
        telemetry.update();

        aux.lift = hardwareMap.get(ExpansionHubMotor.class, "lift");
        aux.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aux.lift.setPower(0);

        aux.lift = hardwareMap.get(ExpansionHubMotor.class, "lift");
        aux.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aux.lift.setPower(0);

        drive.setPoseEstimate(new Pose2d(-28, -28, Math.toRadians(-135)));

        transform.setMode(JoystickTransform.MODE.LINEAR);
    }

    void rstart() {
        pTimer.reset();
        loopTimer.reset();
        aux.hang.setPower(0);
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

    void rinit_loop() {

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

        if (Math.abs(aux.slide.getCurrentPosition()) > 100)
            aux.hang.setPower(0.5);
        else
            aux.hang.setPower(0);
    }

    Boolean previousRightBumper = false;

//    Pose2d scorePos = new Pose2d(-12.5, -18, Math.toRadians(-147));
//    DriveConstraints fastConstraints = new DriveConstraints(60, 25, Math.PI, Math.PI);

    Boolean trajectoryInterupted = false;
    double slidePos = 0, angleValue = 0;
    private JoystickTransform transform;
    Boolean fieldOrientedDriving = true;


    void rloop() {

        if (!asyncLocationTracking)
            drive.getLocalizer().update();

        if (aux.hub.getStandardModule().isEngaged()) {
            RevBulkData data = aux.hub.getBulkInputData();

            if (data == null) {
                slidePos = slideTargetPosition;
                angleValue = 0;
            } else {
                slidePos = 1.0 * data.getMotorCurrentPosition(aux.slide) + slideMotorOffset; //offset allows slide encoder position to be moved on the fly if the string falls off the pulley
                angleValue = data.getAnalogInputValue(aux.pot) / 1000.0;
            }
        } else {
            slidePos = slideTargetPosition;
            angleValue = 0;
        }

        double armLengthTurningCoeff = Range.clip(0.25 * (1.2 * maxSlidePosition) / (slidePos + 0.2 * maxSlidePosition), 0.2, 0.5);

        // drive motor powers use both controllers with other modes at different speeds
        forward = -1 * (gamepad1.left_stick_y);
        right = (intakeMode1 ? (gamepad1.left_stick_x) * 0.5 : (gamepad1.left_stick_x)) + (Math.abs(gamepad2.left_stick_x) > 0.3 ? gamepad2.left_stick_x : 0) * 0.5;
        spin = (gamepad1.left_bumper || intakeMode1 ? (gamepad1.right_stick_x) * (intakeMode1 ? armLengthTurningCoeff : 0.5) : gamepad1.right_stick_x) + (gamepad2.right_stick_x) * armLengthTurningCoeff;
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
        //code for intake mode so the robot can work with a single driver
        if (intakeMode1) {
            slidePower += forward;
            forward = 0;
            forward += gamepad1.right_trigger - gamepad1.left_trigger;
        }

        forward += gamepad2.right_trigger - gamepad2.left_trigger;

        if (0.004 * (maxSlidePosition - slidePos) < 0.7 && slidePower > 0) {
            forward += Math.max(0.7 * slidePower - 0.004 * (maxSlidePosition - slidePos), 0);
            slidePower = 0.004 * (maxSlidePosition - slidePos);
        } else if (target == armTarget.horizontal && 0.004 * (0.5 * maxSlidePosition - slidePos) < 1 && slidePower > 0.5) {
            slidePower = 0.004 * (0.5 * maxSlidePosition - slidePos);
        }

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
            trajectoryInterupted = false;
//            drive.followTrajectory(drive.trajectoryBuilder().reverse().splineTo(scorePos).build());
//            drive.followTrajectory(drive.trajectoryBuilder().reverse().splineTo(scorePos, new SplineInterpolator(drive.getPoseEstimate().getHeading(), scorePos.getHeading()), fastConstraints).build());
            previousRightBumper = false;
        }

//        if (gamepad1.left_bumper)
//            scorePos = drive.getPoseEstimate();

        //code for increasing or decreasing the slide motor encoder offset when the dpad is pressed
        if (gamepad2.dpad_up) {
            if (!previousDpad2State)
//                if (intakeMode2)
//                    groundFollowingOffset -= 0.01;
//                else
                slideMotorOffset -= slideMotorPPr;
            previousDpad2State = true;
        } else if (gamepad2.dpad_down) {
            if (!previousDpad2State)
//                if (intakeMode2)
//                    groundFollowingOffset += 0.01;
//                else
                slideMotorOffset += slideMotorPPr;
            previousDpad2State = true;
        } else
            previousDpad2State = false;

//        if (Math.abs(spin) > 0.5 || Math.abs(forward) > 0.5 || Math.abs(right) > 0.5)
//            trajectoryInterupted = true;

//        final double driveRampIncrement = 0.3;
//        forward = Range.clip(forward, previousForward - driveRampIncrement, previousForward + driveRampIncrement);
//        previousForward = forward;
//        right = Range.clip(right, previousRight - driveRampIncrement, previousRight + driveRampIncrement);
//        previousRight = right;
//        spin = Range.clip(spin, previousSpin - driveRampIncrement, previousSpin + driveRampIncrement);
//        previousSpin = spin;

//        if (!trajectoryInterupted && !gamepad1.right_bumper && drive.isFollowingTrajectory() && angleValue < armTarget.horizontal)
//            drive.update();
//        else {
//        drive.setMotorPowers(forward + right + spin, forward - right + spin, forward + right - spin, forward - right - spin);
//            drive.getLocalizer().update();
//        }

        if (intakeMode1 || intakeMode2)
            drive.setMotorPowers(forward + right + spin, forward - right + spin, forward + right - spin, forward - right - spin);
        else {
            Pose2d v;
            if (fieldOrientedDriving)
                v = transform.transform(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x), drive.getPoseEstimate());
            else
                v = transform.transform(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x));
            drive.setVelocity(v);
        }

        //code to get the potentiometer value; if the first one is broken or unplugged, it uses the second one automatically
//        double angleValue = getAngleValue();
//        (angle.getVoltage() > minArmSensorValue ? angle.getVoltage() : 0.7076 * angle2.getVoltage() + 0.14474);

        if (target == armTarget.horizontal && angleValue < armTarget.slightUp && slidePos > maxSlidePosition * 0.5)
            slidePower = Math.min(slidePower, 0);

        //curve joystick value to set as initial arm power
        armSwing = -1 * Math.pow(gamepad2.right_stick_y, 3);

        //intake motor code
        if (gamepad2.right_bumper || (!intakeMode1 && gamepad1.a)) {
            aux.reverseIntake();
            vexyAtZero = false;
        } else if (gamepad2.left_bumper || intakeMode1) {
            aux.activateIntake();
            vexyAtZero = false;
        } else if (angleValue < armTarget.vertical && angleValue > minArmSensorValue) {
            if (vexyAtZero) {
                aux.activateIntake();
                vexyAtZero = false;
            }
        } else if (!vexyAtZero) {
            aux.activateIntakeSlow();
            vexyAtZero = true;
        }

        //initial lift power is set by the joystick triggers
        if (!intakeMode1)
            lift = gamepad1.right_trigger - gamepad1.left_trigger;

        //start automatically moving up to full height
        if (gamepad1.dpad_up) {
            aux.lift.setTargetPosition(liftTargetPosition);
            liftTimer.reset();
            if (liftButtonState == 0) {
                aux.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                aux.lift.setPower(1);
            }
            liftButtonState = 1;
        }

        //start automatically moving down to start position
        if (gamepad1.dpad_down) {
            aux.lift.setTargetPosition(0);
            liftTimer.reset();
            if (liftButtonState == 0) {
                aux.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                aux.lift.setPower(1);
            }
            liftButtonState = 1;
        }

        //disable automatic lift movement as soon as manual movement buttons are pressed or it times out
        if (liftButtonState == 1 && (Math.abs(lift) > 0.1 || liftTimer.seconds() > 7)) {
            aux.lift.setPower(0);
            aux.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftButtonState = 0;
        }

        //set power if in manual mode
        if (liftButtonState == 0)
            aux.lift.setPower(lift);

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
//        if (gamepad2.dpad_right)
//            Log.i("new arm values", "val:" + angle2.getVoltage() + ", " + angle.getVoltage());
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
        aux.slide.setPower(slidePower);

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

            if (!intakeMode2 && target == armTarget.hh)
                initSlideHolding(slidePositions[0]);

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
            aux.arm.setPower(1);
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
            initArmHolding(isDepotSide ? armTarget.depotSideY : 0.82);
            initSlideHolding(isDepotSide ? (int) (maxSlidePosition * 0.46) : (int) (maxSlidePosition * 0.5));
//            extendArmMode = false;
            intakeMode1 = false;
            intakeMode2 = false;
            aux.arm.setPower(1);
//            yButtonTimer.reset();
        }

        //manage manual and automatic arm control
        if (!armHolding) {
            if (armSwing > 0) {
                aux.arm.setPower(armSwing);
                armMovingDownState = 0;
            } else {
                aux.arm.setPower(angleValue < maxArmSensorValue || armMovingDownState == 2 ? armSwing : 0);
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
                        aux.arm.setPower((Math.abs(spin) > 0.1 || Math.abs(right) > 0.1) ? armController5(angleValue, target, slidePos) + 0 : armController5(angleValue, target, slidePos));
                    else if (angleValue < maxArmSensorValue)
                        aux.arm.setPower(armController5(angleValue, target, slidePos));
                    else
                        aux.arm.setPower(0);
                else
                    aux.arm.setPower(0);
        }

        //check loop time
        if (loopCount % 100 == 0) {
            loopTime = loopTimer.milliseconds() / 100;
            loopTimer.reset();
        }
        loopCount++;

        telemetry.update();
    }
}
