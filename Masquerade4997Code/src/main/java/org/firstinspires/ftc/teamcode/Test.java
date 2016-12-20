package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import BasicLib4997.Motors.TankDrive.Direction;
import BasicLib4997.Motors.TankDrive.TankDrive;

/**
 * Created by Archish on 11/4/16.
 */

@Autonomous(name = "TestTurn", group = "Test")
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
            chimera.runAllTelemetry(false);
            telemetry.update();
            idle();
        }
        waitForStart();

        chimera.drivePIDRange(POWER, 25, Direction.BACKWARD);
        TankDrive.getTelemetry().addSticky("RangeMovmentComplete");
        chimera.turnPID(POWER, 45, Direction.LEFT);




    }
}
