package org.firstinspires.ftc.teamcode;

import android.bluetooth.BluetoothA2dp;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import BasicLib4997.Motors.TankDrive.Direction;
import BasicLib4997.Motors.TankDrive.TankDrive;

/**
 * Created by Archish on 10/6/16.
 */

@Autonomous(name = "BeaconsShootRed", group = "Test") // change name

public class BeaconShootParkRed extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }



    @Override
    public void runOpMode() throws InterruptedException {
        boolean telemetrizeModules;
        double POWER = 0.50;
        TankDrive chimera = new TankDrive(telemetry);
        while (!isStarted()) {
            chimera.setIndexer(0);
            chimera.runAllTelemetry(telemetrizeModules = false);
            telemetry.update();
            idle();
        }
        waitForStart();
        chimera.setPowerShooter(newShooterPower(-0.65));
        chimera.drivePID(POWER, 10, Direction.BACKWARD);
        chimera.sleep(2000);
        chimera.setIndexer(0.6);
        chimera.sleep(400);
        chimera.setIndexer(0);
        chimera.sleep(3000);
        chimera.setIndexer(0.6);
        chimera.sleep(1000);
        chimera.setPowerShooter(-0.5);
        chimera.setPowerShooter(-0.3);
        chimera.setPowerShooter(0);
        chimera.sleep(1000);
        chimera.turnPID(POWER, 37, Direction.LEFT);
        chimera.drivePIDRange(0.3, 20, Direction.BACKWARD, 2000);
        chimera.turnPID(POWER, 45, Direction.RIGHT);
        chimera.drivePIDRange(0.30, 50, Direction.BACKWARD);
        chimera.stopRed(POWER, Direction.FORWARD);
        chimera.drivePID(POWER, 5, Direction.FORWARD);
        chimera.setBeaconPresser(1);
        chimera.sleep(1000);
        chimera.setBeaconPresser(0);



    }
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    private double newShooterPower(double targetPower){
        double voltsge = getBatteryVoltage();
        double targetVoltage = 13.5;
        double error = targetVoltage - voltsge;
        return targetPower - (error * 0.02);
    }


}