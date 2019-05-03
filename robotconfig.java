package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


/***
 * oldrobotconfig is a simple and effective way to import the robot configuration information into every program.
 * To enable it, simply add the code?
 *
 * @code import static org.firstinspires.ftc.teamcode.oldrobotconfig.*;
 * into the op mode file.c
 * Every new motor, servo, and sensor should be added to this file.
 */
public class robotconfig {

    //everything becomes static so robotconfig is not cloned which caused problems until they were fixed and this way should be simpler
//    static DataLogger dl;
    static LinearOpMode theLinearOpMode;
    static boolean debugMode = false;
    //assume 4 drive motors
    static DcMotorEx fLeftMotor;//naming front left motor
    static DcMotorEx bLeftMotor;//naming back left motor
    static DcMotorEx bRightMotor;//naming back right motor
    static DcMotorEx fRightMotor;//naming front right motor
    // The IMU sensor object4
    static BNO055IMU imu;
    static HardwareMap hwMap = null;
    static OpMode theOpMode;
    //accessory motor(s)
//    static RelicRecoveryVuMark vuMark;
    static DcMotorEx armMotor;
    static DcMotor lift;//namign lifting arm 'lift'
    //    static Servo robert;
    static CRServo vexy;
    static CRServo vexy2;

//    static DistanceSensor sensorRange;

    static AnalogInput angle;
    static AnalogInput angle2;

    static double topJoyLimit = 1.0;
    static double curveJoyPower = 2.0;
    static DcMotor slide;//naming slide arm 'slide'
    static DcMotor hang;

    /* Checks if external storage is available for read and write */
    public static boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            return true;
        }
        return false;
    }

    /* Checks if external storage is available to at least read */
    public static boolean isExternalStorageReadable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state) ||
                Environment.MEDIA_MOUNTED_READ_ONLY.equals(state)) {
            return true;
        }
        return false;
    }

    /* Constructor */
    public robotconfig() {

    }

    static double curveJoyValue(double input) { // note: to change the maximum power associated with the maximum joystick input, use
        //  topJoyLimit as the multiplier

        if (input > 0.0) {
            return ((Math.pow(input, curveJoyPower) * topJoyLimit));
        } else {
            return (-1 * (Math.abs(Math.pow(input, curveJoyPower)) * topJoyLimit));
        }

    }

    /***
     * enables the motor brake for the drive train motors.
     * This function only has to be run if one wants the brake to be enabled.
     */
    static void enableMotorBreak() {
//        addlog(dl, "robot", "enableMotorBreak was called");
        fLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//telling motors to brake when not given power
        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//look up 1 line
        bLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//telling motors to brake when not given power
        bRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//telling motors to brake when not given power
    }

    /***
     * disables the motor brake for the drive train motors.
     * This function only has to be run if one wants disable the brake after it is manually enabled
     */
    static void disableMotorBreak() {
//        addlog(dl, "robot", "disableMotorBreak was called");
        fLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//restarting motors so they can continue to move after braking
        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//restarting motors so they can continue to move after braking
        bLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//restarting motors so they can continue to move after braking
        bRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//restarting motors so they can continue to move after braking
    }

    /***
     * resets the encoders of the drive train motors but doesn't put them back to the normal mode
     */
    static void resetMotorEncoders() {
//        addlog(dl, "robot", "resetMotorEncoders was called");
        fLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting encoder
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting encoder
        bLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting encoder
        bRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting encoder
    }

    /***
     * makes the drive train motors use the RUN_USING_ENCODER mode
     */
    static void enableMotorEncoders() {
//        addlog(dl, "robot", "enableMotorEncoders was called");
        fLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /***
     * makes the drive train motors use the RUN_TO_POSITION mode
     */
    static void enableEncodersToPosition() {
//        addlog(dl, "robot", "enableEncodersToPosition was called");
        fLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /***
     * sets the drive train motor mode to RUN_WITHOUT_ENCODER
     */
    static void disableMotorEncoders() {
//        addlog(dl, "robot", "disableMotorEncoders was called");
        fLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     * simply detects if one of the drive train motors is busy in run to position mode
     *
     * @return true if any one of the motors are busy
     */
    static boolean isMotorBusy() {
//        addlog(dl, "robot", "isMotorBusy was called, returning " + String.valueOf(!debugMode && (fLeftMotor.isBusy() || fRightMotor.isBusy() || bLeftMotor.isBusy() || bRightMotor.isBusy())));
        return (!debugMode && (fLeftMotor.isBusy() || fRightMotor.isBusy() || bLeftMotor.isBusy() || bRightMotor.isBusy()));
    }

    /***
     * init of drivetrain motors in method so it is the same for both teleop and autonomous
     */
    static void initDrivetrainMotors() {

//        addlog(dl, "r.init", "initDrivetrainMotors was called")

        bLeftMotor = (DcMotorEx) hwMap.dcMotor.get("blmotor");
        fRightMotor = (DcMotorEx) hwMap.dcMotor.get("frmotor");
        fLeftMotor = (DcMotorEx) hwMap.dcMotor.get("flmotor");
        bRightMotor = (DcMotorEx) hwMap.dcMotor.get("brmotor");

        fRightMotor.setDirection(DcMotor.Direction.REVERSE); //reverse for andymark motor in drivetrain
        bRightMotor.setDirection(DcMotor.Direction.REVERSE); //reverse for andymark motor in drivetrain

        DriveKp = 12; //bLeftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;

        DriveKi = 8; //bLeftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;

        DriveKd = 4;

        DriveKf = 13; //bLeftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f;

        setDriveMotorPIDF();

        fLeftMotor.setPower(0);
        fRightMotor.setPower(0);
        bLeftMotor.setPower(0);
        bRightMotor.setPower(0);
    }

    static void initDrivetrainTelemetry(Telemetry telemetry) {
        telemetry.addLine()
                .addData("Drive velocity PIDF: Kp", () -> (float) DriveKp)
                .addData("Ki", () -> (float) DriveKi)
                .addData("Kd", () -> (float) DriveKd)
                .addData("Kf", () -> (float) DriveKf);

        telemetry.addLine()
                .addData("Drive speeds: fl", () -> (float) bRightMotor.getVelocity(AngleUnit.RADIANS))
                .addData("fr", () -> (float) bRightMotor.getVelocity(AngleUnit.RADIANS))
                .addData("bl", () -> (float) bRightMotor.getVelocity(AngleUnit.RADIANS))
                .addData("br", () -> (float) bRightMotor.getVelocity(AngleUnit.RADIANS));

        telemetry.addLine()
                .addData("Drive powers: fl", () -> (float) bRightMotor.getPower())
                .addData("fr", () -> (float) bRightMotor.getPower())
                .addData("bl", () -> (float) bRightMotor.getPower())
                .addData("br", () -> (float) bRightMotor.getPower());
    }

    static double DriveKp, DriveKi, DriveKd, DriveKf;

    static void setDriveMotorPIDF() {
        bLeftMotor.setVelocityPIDFCoefficients(DriveKp, DriveKi, DriveKd, DriveKf);
        bRightMotor.setVelocityPIDFCoefficients(DriveKp, DriveKi, DriveKd, DriveKf);
        fLeftMotor.setVelocityPIDFCoefficients(DriveKp, DriveKi, DriveKd, DriveKf);
        fRightMotor.setVelocityPIDFCoefficients(DriveKp, DriveKi, DriveKd, DriveKf);
    }

    static ExpansionHubEx hub3, hub;

    static void initGyro() {

//        addlog(dl, "r.init", "initGyro was called");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        RevExtensions2.init();

        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // note: this strategy is still applicable even if the drive motors are split between hubs
        hub = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub3 = hwMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        hub.setPhoneChargeEnabled(false);
        hub3.setPhoneChargeEnabled(false);

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
    }

    static void initHubs() {

//        addlog(dl, "r.init", "initGyro was called");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        RevExtensions2.init();

        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // note: this strategy is still applicable even if the drive motors are split between hubs
        hub = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub3 = hwMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        hub.setPhoneChargeEnabled(false);
        hub3.setPhoneChargeEnabled(false);

    }

    static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    static final String VUFORIA_KEY = "AVpT0jz/////AAABmZ4+U5OnbklZtSvifQkuDS5xG2MIaXTqPLDP0llFjl6QKDfo3TAb2OJMez0CJERLdZLWxqqQRrWVes0hYOYrGGvYL5CdWVV2ZdZ+9fENvFq6ReMeBzhM2kt2aQGauDYAmQSCSczEw/7u5r8PH3rKqRR0dded061qHncbKVsXNsGvBKRbG9ij4dJg6w9xLaakGXdRa8oUrdflaaWQ1idISjvMjSp2PSREk8k0H8917cXUW0D1m4baQLWkjztsN5QYRD9kjMWEISK2khParlL9UtU3sDalZIdR9T8HdgioTdNYB3dXcUQuvO07lfo8XaDwkBu5iQdNqt6yPXCs23n4em/gknfdvxSlfHgqid7m3t9r";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    static VuforiaLocalizer vuforia;
    static VuforiaLocalizer vuforiaPhone;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    static TFObjectDetector tfod;

    static boolean oldRobot = false;

    /* Initialize standard Hardware interfaces - LinearOpMode */

    static WebcamName webcamName;
    static File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    static void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    static void initVuforia(HardwareMap hwMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    public static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;

    public static OpenGLMatrix lastLocation = null;
    public static boolean targetVisible = false;
    public static VuforiaTrackables targetsRoverRuckus;

    static VuforiaTrackable blueRover;
    static VuforiaTrackable redFootprint;
    static VuforiaTrackable frontCraters;
    static VuforiaTrackable backSpace;
    static List<VuforiaTrackable> allTrackables;

    static void initVuforiaNavigation() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        vuforiaPhone = ClassFactory.getInstance().createVuforia(parameters);

//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = vuforiaPhone.loadTrackablesFromAsset("RoverRuckus");
        blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT = -60;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 130;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = -190;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

//        final int CAMERA_FORWARD_DISPLACEMENT = 0;   // eg: Camera is 110 mm in front of robot center
//        final int CAMERA_VERTICAL_DISPLACEMENT = 140;   // eg: Camera is 200 mm above ground
//        final int CAMERA_LEFT_DISPLACEMENT = -220;     // eg: Camera is ON the robot's center line
//
//        OpenGLMatrix robotFromCamera = OpenGLMatrix
//                .translation(0,0,-CAMERA_LEFT_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZY,
//                        AngleUnit.DEGREES, 0, 0, 0));
//
//        /**  Let all the trackable listeners know where the phone is.  */
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        }

    }

    static void refreshRobotLocation() {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
        }
        return;
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    static void initTfod() {

        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }

    static void initTfod(HardwareMap hwMap) {

        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }

    enum goldLocation {
        LEFT,
        RIGHT,
        CENTER,
        UNKNOWN
    }

    static goldLocation lastGoldLocation = goldLocation.UNKNOWN; //defULTS FROM LAST FOUND POSITION to unknown

    static goldLocation getGoldPosition() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            if (updatedRecognitions.size() == 3) {
                int goldMineralX = -1; //defines gold to camera
                int silverMineral1X = -1;//defines silver to camera
                int silverMineral2X = -1; //defines silver to camera
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) { //recognizes gold
                        goldMineralX = (int) recognition.getLeft();
                    } else if (silverMineral1X == -1) { //if silver move positionb to next
                        silverMineral1X = (int) recognition.getLeft();
                    } else {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }
                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        lastGoldLocation = goldLocation.LEFT;
                        return goldLocation.LEFT;
                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        lastGoldLocation = goldLocation.RIGHT;
                        return goldLocation.RIGHT;
                    } else {
                        lastGoldLocation = goldLocation.CENTER;
                        return goldLocation.CENTER;
                    }
                }
            }

            if (updatedRecognitions.size() >= 3) {
                int goldMineralX = -1; //defines gold to camera
                int silverMineral1X = -1;//defines silver to camera
                int silverMineral2X = -1; //defines silver to camera
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getTop() > 100)
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) { //recognizes gold
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) { //if silver move positionb to next
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                }
                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        lastGoldLocation = goldLocation.LEFT;
                        return goldLocation.LEFT;
                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        lastGoldLocation = goldLocation.RIGHT;
                        return goldLocation.RIGHT;
                    } else {
                        lastGoldLocation = goldLocation.CENTER;
                        return goldLocation.CENTER;
                    }
                }
            }
        }
        return goldLocation.UNKNOWN;
    }

    static void rinit(LinearOpMode linearOpMode) { //don't use for new files because of init timeout problems

        theLinearOpMode = linearOpMode;

        // Save reference to Hardware map in class variable

        hwMap = linearOpMode.hardwareMap;

        initDrivetrainMotors();


        angle = hwMap.analogInput.get("pot");
        angle2 = hwMap.analogInput.get("pot2");

        lift = hwMap.dcMotor.get("lift");
        slide = hwMap.dcMotor.get("slide");
        armMotor = (DcMotorEx) hwMap.dcMotor.get("armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robert = hwMap.servo.get("robert");
//        robert.setPosition(0.1);
        vexy = hwMap.crservo.get("vexy");
        vexy2 = hwMap.crservo.get("vexy2");
        vexy2.setPower(0);
        vexy.setPower(0);
        Thread.yield();
        disableMotorEncoders();
        //pause for milliseconds
        Thread.yield();
        //make it easier to handle.
        enableMotorBreak();
    }

    /* Initialize standard Hardware interfaces */

    static void rinit(OpMode opMode) {
        theOpMode = opMode;

        // Save reference to Hardware map in class variable

        hwMap = opMode.hardwareMap;
        initDrivetrainMotors();
//        initGyro();

//        try {
//        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");
//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        angle = hwMap.analogInput.get("pot");
        angle2 = hwMap.analogInput.get("pot2");

        lift = hwMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide = hwMap.dcMotor.get("slide");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor = (DcMotorEx) hwMap.dcMotor.get("armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang = hwMap.dcMotor.get("hang");
//        robert = hwMap.servo.get("robert");
//        robert.setPosition(0.1);
        vexy = hwMap.crservo.get("vexy");
        vexy2 = hwMap.crservo.get("vexy2");
        vexy.setPower(0);
        vexy2.setPower(0);
//        } catch (Exception err) {
//            oldRobot = true;
//        }
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reset motor encoders because this might help with performance
        //resetMotorEncoders();
        //pause for milliseconds
        Thread.yield();
        disableMotorEncoders();
        //pause for milliseconds
        Thread.yield();
        enableMotorEncoders();
        //make it easier to handle.
        Thread.yield();
        enableMotorBreak();
        Thread.yield();


//        addlog(dl, "r.init", "r.init finished (a)");
    }

    /***
     * is a shortcut to get the intrinsic angle of the robot
     *
     * @return imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
     */
    static double getCurrentAngle() {
//        addlog(dl, "robot", "getCurrentAngle was invoked, returning 45");
//        if (debugMode) return 45;
//        return 0;
        return imu.getAngularOrientation().firstAngle;
    }

    static double getAngularSpeed() {
//        addlog(dl, "robot", "getCurrentAngle was invoked, returning 45");
//        if (debugMode) return 45;
//        return 0;
        return imu.getAngularVelocity().zRotationRate;
    }

    /***
     * move is a function to efficiently set the power values of all 4 drive train motors in one quick line.
     *
     * @param forward double: ranges from 1=forward to -1=backward
     * @param right   double: ranges from 1=slide right to -1=slide left
     * @param spin    double: ranges from 1=turn clockwise to -1=turn counterclockwise
     */
    static void move(double forward, double right, double spin) {

//        addlog(dl, "robot", String.format(Locale.ENGLISH, "move was called - f:r:s:, %.2f, %.2f, %.2f", forward, right, spin));

        fLeftMotor.setPower(forward + right + spin);
        fRightMotor.setPower(forward - right - spin);
        bLeftMotor.setPower(forward - right + spin);
        bRightMotor.setPower(forward + right - spin);

//        addlog(dl, "robot", String.format(Locale.ENGLISH, "move powers are fl:fr:bl:br, %.2f, %.2f, %.2f, %.2f", fLeftMotor.getPower(), fRightMotor.getPower(), bLeftMotor.getPower(), bRightMotor.getPower()));
    }

}

