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

import static android.R.attr.angle;
import static android.R.attr.imageButtonStyle;
import static com.qualcomm.hardware.hitechnic.HiTechnicNxtIrSeekerSensor.DIRECTION;

/**
 * Beacons + Shoot for Red8
 */

@Autonomous(name = "RedAuto", group = "G1") // change name

public class RedAuto extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }



    @Override
    public void runOpMode() throws InterruptedException {
        boolean telemetrizeModules;
        double POWER = 0.50;
        TankDrive chimera = new TankDrive(telemetry);
        while (!isStarted()) {
            chimera.setIndexer(0);
            chimera.imu.telemetryRun();
            chimera.beaconColor.telemetryRun();
            chimera.rangeSensor.telemetryRun();
            telemetry.update();
            idle();
        }
        waitForStart();
        //SetPower to the shooter and drive foreword in orer to make a shot
        chimera.setPowerShooter(newShooterPower(-0.75));
        double parallelAngle = chimera.imu.getHeading();
        chimera.drivePID(POWER, 10, Direction.FORWARD);
        // find how arr of we are from our orignial position
        double disruption = chimera.imu.getHeading();
        chimera.sleep(1000);
        chimera.setIndexer(0.6);
        chimera.sleep(400);
        chimera.setIndexer(0);
        chimera.sleep(2000);
        chimera.setIndexer(0.6);
        chimera.sleep(1000);
        chimera.setPowerShooter(-0.5);
        chimera.setPowerShooter(-0.3);
        chimera.setPowerShooter(0);
        chimera.sleep(1000);
        //turn away from the center vortex 48 degrees - our disruption

        chimera.turnPID(POWER, (int) (48 - disruption), Direction.LEFT, 3);
        // drive to the first beacon
        chimera.drivePID(0.7, 148, Direction.FORWARD, 1000, 48 - disruption);
        // turn parallel to the
        double changeTurn = chimera.imu.getHeading();
        double turn = changeTurn - parallelAngle;

        chimera.turnPID(POWER, (int) turn, Direction.RIGHT, 5);
        // Stop at first Beacon
        chimera.stopRed(0.4, Direction.BACKWARD);
        // drive in position to hit the beacon
        chimera.drivePID(POWER, 15, Direction.BACKWARD);
        // move the beacon presser
        chimera.setBeaconPresser(-1);
        chimera.sleep(2000);
        chimera.setBeaconPresser(1);
        // drive to the next beacon
        chimera.drivePID(0.7, 60, Direction.FORWARD, 1000, parallelAngle);

        chimera.setBeaconPresser(0);

        chimera.stopRed(0.4, Direction.BACKWARD);
        // drive in position to hit the beacon
        chimera.drivePID(POWER, 15, Direction.BACKWARD);
        // move the beacon presser
        chimera.setBeaconPresser(-1);
        chimera.sleep(2000);
        chimera.setBeaconPresser(1);








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