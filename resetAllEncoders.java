package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.RevExtensions2;

@TeleOp
public class resetAllEncoders extends LinearOpMode {
    @Override
    public void runOpMode() {
        RevExtensions2.init();
        auxillaryDevices aux = new auxillaryDevices(hardwareMap);
        aux.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aux.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aux.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aux.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        aux.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        aux.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (Math.abs(aux.lift.getCurrentPosition()) < 50 && Math.abs(aux.slide.getCurrentPosition()) < 50)
            aux.hang.setPower(0.5);
        else {
            aux.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            aux.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            aux.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            aux.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            aux.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            aux.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (Math.abs(aux.lift.getCurrentPosition()) < 50 && Math.abs(aux.slide.getCurrentPosition()) < 50)
                aux.hang.setPower(0.5);
        }
        sleep(5000);
    }
}
