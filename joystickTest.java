package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by mail2 on 6/12/2017.
 */
@TeleOp(name = "Joystick Test")

public class joystickTest extends OpMode {

    public void init() {

    }

    public void init_loop() {

        telemetry.addData("a", gamepad1.a);
        telemetry.addData("b", gamepad1.b);
        telemetry.addData("x", gamepad1.x);
        telemetry.addData("y", gamepad1.y);

        telemetry.addData("dpad_up", gamepad1.dpad_up);
        telemetry.addData("dpad_down", gamepad1.dpad_down);
        telemetry.addData("dpad_left", gamepad1.dpad_left);
        telemetry.addData("dpad_right", gamepad1.dpad_right);

        telemetry.addData("left_bumper", gamepad1.left_bumper);
        telemetry.addData("right_bumper", gamepad1.right_bumper);
        telemetry.addData("left_trigger", gamepad1.left_trigger);
        telemetry.addData("right_trigger", gamepad1.right_trigger);

        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("right_stick_y", gamepad1.right_stick_y);

        telemetry.addData("left_stick_button", gamepad1.left_stick_button);
        telemetry.addData("right_stick_button", gamepad1.right_stick_button);

        telemetry.addData("back", gamepad1.back);
        telemetry.addData("start", gamepad1.start);
        telemetry.addData("guide", gamepad1.guide);

    }

    public void loop() {

        telemetry.addData("a", gamepad1.a);
        telemetry.addData("b", gamepad1.b);
        telemetry.addData("x", gamepad1.x);
        telemetry.addData("y", gamepad1.y);

        telemetry.addData("dpad_up", gamepad1.dpad_up);
        telemetry.addData("dpad_down", gamepad1.dpad_down);
        telemetry.addData("dpad_left", gamepad1.dpad_left);
        telemetry.addData("dpad_right", gamepad1.dpad_right);

        telemetry.addData("left_bumper", gamepad1.left_bumper);
        telemetry.addData("right_bumper", gamepad1.right_bumper);
        telemetry.addData("left_trigger", gamepad1.left_trigger);
        telemetry.addData("right_trigger", gamepad1.right_trigger);

        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("right_stick_y", gamepad1.right_stick_y);

        telemetry.addData("left_stick_button", gamepad1.left_stick_button);
        telemetry.addData("right_stick_button", gamepad1.right_stick_button);

        telemetry.addData("back", gamepad1.back);
        telemetry.addData("start", gamepad1.start);
        telemetry.addData("guide", gamepad1.guide);

    }
}
