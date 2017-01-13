package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import BasicLib4997.Motors.DefenseBot;
import BasicLib4997.Motors.TankDrive.TankDrive;
/**
 * TeleOp NFS
 */
@TeleOp(name="DefenseTeleOp", group="Final")// change name

public class DefenseTeleOp extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }

    @Override
    public void runOpMode() throws InterruptedException {

        DefenseBot chimera = new DefenseBot(telemetry);
        while (!isStarted()) {
            telemetry.addLine("helloWorls");
            telemetry.update();
            idle();
        }

        waitForStart();
        while (opModeIsActive()) {
            float move = -gamepad1.left_stick_y;
            float turn = -gamepad1.right_stick_x;
            double collector = -1.5;
            double left = move - turn;
            double right = move + turn;

            if (gamepad1.a) {
                left /= 3;
                right /= 3;
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            }
            if (left > 1.0) {
                left /= left;
                right /= left;
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            } else if (right > 1.0) {
                left /= right;
                right /= right;
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            } else {
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            }
        }
    }
}