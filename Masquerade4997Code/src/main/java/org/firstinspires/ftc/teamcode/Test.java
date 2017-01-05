package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import BasicLib4997.Motors.TankDrive.Direction;
import BasicLib4997.Motors.TankDrive.TankDrive;

/**
 * Created by Archish on 11/4/16.
 */

@Autonomous(name = "Test", group = "G4")
public class Test extends LinearOpMode {
    public void main() throws InterruptedException {

    }
    @Override

    public void runOpMode() throws InterruptedException {
        boolean telemetrizeModules;
        double LOW_POWER = 0.30;
        double POWER = 0.50;
        double HIGH_POWER = 0.90;

        TankDrive chimera = new TankDrive(telemetry);
        while (!isStarted()) {
            chimera.runSensorTelemetry();
            telemetry.update();
        }
        waitForStart();








    }
}
