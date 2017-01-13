package org.firstinspires.ftc.teamcode;

import android.bluetooth.BluetoothA2dp;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import BasicLib4997.Motors.DefenseBot;
import BasicLib4997.Motors.TankDrive.Direction;
import BasicLib4997.Motors.TankDrive.TankDrive;

/**
 * Created by Archish on 10/6/16.
 */

@Autonomous(name = "DefenseAuto", group = "G3") // change name
public class DefsenseAuto extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }



    @Override
    public void runOpMode() throws InterruptedException {
        boolean telemetrizeModules;
        double LOW_POWER = 0.50;
        double POWER = 0.70;
        double HIGH_POWER = 0.90;
        DefenseBot chimera = new DefenseBot(telemetry);
        while (!isStarted()) {
            telemetry.update();
            idle();
        }
        waitForStart();
        while (opModeIsActive()) {
            chimera.setPowerRight(1);
            chimera.setPowerLeft(1);
            Thread.sleep(1000);
            chimera.setPowerRight(1);
            chimera.setPowerLeft(-1);
        }


    }
}