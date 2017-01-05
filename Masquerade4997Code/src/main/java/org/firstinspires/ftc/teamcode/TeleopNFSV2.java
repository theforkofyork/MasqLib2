package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import BasicLib4997.Motors.TankDrive.TankDrive;

/**
 * TeleOp NFS
 */
@TeleOp(name="TeleOpFSV2", group="Final")// change name

public class TeleopNFSV2 extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        TankDrive chimera = new TankDrive(telemetry);
        while (!isStarted()) {
            chimera.setIndexer(0);
            chimera.setCapClamp(1);
            chimera.liftClamp.setPosition(1);
            telemetry.update();
            idle();
        }

        waitForStart();
        double power = 0;
        double revUpFactor = 0.1;
        double revDownFactor = 0.01;
        double targetPower = - 0.7;
        double lowPowerFactor = 0.1;
        chimera.shooter.resetEncoder();
        while (opModeIsActive()) {
            //chimera.shooter.runUsingEncoder();
            float move = -gamepad1.left_stick_y;
            float turn = -gamepad1.right_stick_x;
            double collector = -1.5;
            double left = move - turn;
            double right = move + turn;

            if (gamepad1.a) {
                left/= 3;
                right /=3;
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            }
            if(left > 1.0) {
                left /= left;
                right /= left;
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            }

            else if (right > 1.0) {
                left /= right;
                right /= right;
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            }

            else {
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            }


            if (gamepad1.right_bumper) {
                chimera.setPowerCollector(collector);
            }
            else if (gamepad1.left_bumper) {
                chimera.setPowerCollector(-collector);
            }
            else {
                chimera.setPowerCollector(0);
            }

            if(gamepad2.right_bumper) {
                chimera.shooter.setPowerNoEncoder(-2200, 0.7);
            }


            else if(gamepad2.left_bumper) {
                power += revUpFactor;
                if (power > targetPower) {
                    power = targetPower + lowPowerFactor;
                    telemetry.addLine("Shooter is Revved Up.");
                }
            }
            else {
                power -= revDownFactor;
                if (power < targetPower) {
                    telemetry.addLine("Shooter is Not Revved Up.");
                }
                if (power < 0) {
                    power = 0;
                }
            }


            if (gamepad2.a){
                chimera.setBeaconPresser(1);
            }
            else if (gamepad2.y){
                chimera.setBeaconPresser(-1);
            }
            else {
                chimera.setBeaconPresser(0);
            }
            if (gamepad2.right_trigger > 0) {
                chimera.lift.moveUp(0.7);
            }
            else if (gamepad2.left_trigger > 0) {
                chimera.lift.moveDown(1);
            }
            else chimera.lift.moveDown(0);
            if (gamepad1.x) {
                chimera.liftClamp.setPosition(0);
            }
            else {
                chimera.liftClamp.setPosition(1);
            }
            if (gamepad1.b) {
                chimera.setCapClamp(0);
            }
            else {
                chimera.setCapClamp(1);
            }

            telemetry.addData("Shooter Power", chimera.shooter.getPower());
            telemetry.addData("Lift Power", chimera.lift.getPower());
            telemetry.addData("Left Power", left);
            telemetry.addData("Right Power", right);
            telemetry.addData("Rate", chimera.shooter.getRate());
            telemetry.update();


        }


    }







}