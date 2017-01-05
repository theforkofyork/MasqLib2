package org.firstinspires.ftc.teamcode;

import android.bluetooth.BluetoothA2dp;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import BasicLib4997.Motors.TankDrive.Direction;
import BasicLib4997.Motors.TankDrive.TankDrive;

/**
 * Created by Archish on 10/6/16.
 */

@Autonomous(name = "ShootParkRed", group = "G2") // change name

public class ShootParkRed extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }



    @Override
    public void runOpMode() throws InterruptedException {
        boolean telemetrizeModules;
        double Power_Optimal = 0.50;
        double POWER_LOW = 0.3;
        double HIGH_POWER = 0.90;
        TankDrive chimera = new TankDrive(telemetry);
        while (!isStarted()) {
            chimera.setIndexer(0.8);
            telemetry.addData("hue", chimera.beaconColor.hue());
            chimera.runAllTelemetry();
            telemetry.update();
            idle();
        }
        waitForStart();
        chimera.drivePID(Power_Optimal, 15, Direction.BACKWARD);
        chimera.setPowerShooter(-0.8);
        chimera.sleep(3000);
        chimera.setPowerCollector(-1);
        chimera.setIndexer(0.2);
        chimera.sleep(500);
        chimera.setIndexer(0.8);
        chimera.sleep(3000);
        chimera.setPowerCollector(-1);
        chimera.setIndexer(0.2);
        chimera.sleep(3000);
        chimera.setPowerShooter(-0.5);
        chimera.setPowerShooter(-0.3);
        chimera.setPowerShooter(0);
        chimera.setPowerCollector(0);
        chimera.drivePID(Power_Optimal, 130, Direction.BACKWARD);
        chimera.setPowerCollector(1);
        chimera.turnPID(Power_Optimal, 45, Direction.RIGHT);
        chimera.drivePID(Power_Optimal, 200, Direction.FORWARD);
        chimera.setBrakeMode(10000);

    }
}