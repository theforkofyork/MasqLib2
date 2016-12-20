package BasicLib4997.Motors.TankDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import BasicLib4997.Motors.Motor;
import BasicLib4997.Motors.MotorSystem;
import BasicLib4997.Sensors.AdafruitIMU;
import BasicLib4997.Sensors.Clock;
import BasicLib4997.Sensors.I2CRangeSensor;
import BasicLib4997.Sensors.MRColorSensor;
import BasicLib4997.Sensors.Sensor_Thresholds;
import BasicLib4997.Servos.CR_Servo;
import BasicLib4997.Servos.Servo;

import static com.qualcomm.hardware.hitechnic.HiTechnicNxtIrSeekerSensor.DIRECTION;

/**
 * Created by Archish on 10/28/16.
 */

public class TankDrive implements PID_Constants, Sensor_Thresholds {

    public TankDrive(Telemetry telemetry){
        this.telemetry  = telemetry;
        instance = this;
    }
    public static TankDrive getTelemetry(){
        return instance;
    }
    private static TankDrive instance;
    private Telemetry telemetry;
    public void addTelemetry(String string) {
        telemetry.addLine(string);
    }
    public void addTelemetry(String string, Object data) {
        telemetry.addData(string, data);
    }
    public void addSticky(String string){
        telemetry.log().add(string);
        telemetry.update();
    }
    public void addSticky(String string, Object data){
        telemetry.log().add(string, data);
        telemetry.update();
    }

    // Motor and Motor Systems
    public MotorSystem driveTrain = new MotorSystem("leftFront", "leftBack", "rightFront", "rightBack");
    private Motor collector = new Motor("collector");
    public Motor shooter = new Motor("shooter");
    ///Clock
    public Clock clock = new Clock("clock");
    //Servos
    private CR_Servo beaconPresser = new CR_Servo("beaconPresser");
    private Servo indexer = new Servo("indexer");
    //IMU
    public AdafruitIMU imu = new AdafruitIMU("imu");
    //ColorSensor
    public MRColorSensor beaconColor = new MRColorSensor("Beacon");
    //RangeSensor
    public I2CRangeSensor rangeSensor = new I2CRangeSensor("rangeSensor");
    //
    private static final int DEFAULT_SLEEP_TIME = 1000;

    private boolean opModeIsActive() {
        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
    }

    public void drivePID(double power, int distance, Direction DIRECTION, int sleepTime) {
        double targetAngle = imu.getHeading();
        double integral = 0;
        double prevError = 0;
        int newDistance = driveTrain.convert(distance);
        driveTrain.resetEncoders();
        driveTrain.setDistance((int) ((-newDistance) * DIRECTION.value));
        driveTrain.runToPosition();
        driveTrain.setPowerLeft(power);
        driveTrain.setPowerRight(power);
        double previousTime = 0;
        while (driveTrain.rightIsBusy() && opModeIsActive()) {
            double tChange = System.nanoTime() - previousTime;
            tChange = tChange / 1e9;
            double newPowerLeft = power;
            double imuVal = imu.getHeading();
            double error = targetAngle - imuVal;
            double errorkp = error * KP_STRAIGHT;
            integral = (integral + error) * tChange;
            double integralki = integral * KI_STRAIGHT;
            double dervitive = (error - prevError) / tChange;
            double dervitiveKd = dervitive * KD_STRAIGHT;
            newPowerLeft = (newPowerLeft - (errorkp + dervitiveKd + integralki) * DIRECTION.value);
            driveTrain.setPowerRight(power);
            driveTrain.setPowerLeft(newPowerLeft);
            prevError = error;
            previousTime = System.nanoTime()/ 1e9;
            TankDrive.getTelemetry().addTelemetry("Heading", imuVal);
            TankDrive.getTelemetry().addTelemetry("ShooterPower", shooter.getPower());
            telemetry.update();
        }

        driveTrain.StopDriving();
        driveTrain.runUsingEncoder();
        sleep(sleepTime);
    }

    public void drivePID(double power, int distance, Direction DIRECTION) {
        drivePID(power, distance, DIRECTION, DEFAULT_SLEEP_TIME);
    }



    public void turnPID(double power, int angle, Direction DIRECTION, int sleepTime) {
        double targetAngle = imu.adjustAngle(imu.getHeading() + (DIRECTION.value * angle));
        double acceptableError = 0.5;
        double currentError = 1;
        double prevError = 0;
        double integral = 0;
        double newPower = power;
        double previousTime = 0;
        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError)) {
            double tChange = System.nanoTime() - previousTime;
            tChange = tChange / 1e9;
            double imuVAL = imu.getHeading();
            currentError = imu.adjustAngle(targetAngle - imuVAL);
            integral += currentError;
            double errorkp = currentError * KP_TURN;
            double integralki = currentError * KI_TURN * tChange;
            double dervitive = (currentError - prevError) / tChange;
            double dervitivekd = dervitive * KD_TURN;
            newPower = (errorkp + integralki + dervitivekd);
            driveTrain.setPowerRight(-newPower);
            driveTrain.setPowerLeft(newPower);
            prevError = currentError;
            previousTime = System.nanoTime() / 1e9;
            TankDrive.getTelemetry().addTelemetry("TargetAngle", targetAngle);
            TankDrive.getTelemetry().addTelemetry("Heading", imuVAL);
            TankDrive.getTelemetry().addTelemetry("AngleLeftToCover", currentError);
            telemetry.update();
        }
        driveTrain.StopDriving();
        sleep(sleepTime);
    }
    public void turnPID(double power, int distance, Direction DIRECTION)  {
        turnPID(power, distance, DIRECTION, DEFAULT_SLEEP_TIME);
    }

    public void drivePIDRange(double power, double distanceAway, Direction Direction , int sleepTime ) {
        double targetAngle = imu.getHeading();
        driveTrain.runUsingEncoder();
        while (rangeSensor.Ultrasonic() > distanceAway && opModeIsActive()){
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            newPower = (newPower - (error * KP_STRAIGHT) * Direction.value);
            driveTrain.setPowerLeft(newPower);
            driveTrain.setPowerRight(power);
            TankDrive.getTelemetry().addTelemetry("Heading", heading);
            TankDrive.getTelemetry().addTelemetry("Ultrasonic", rangeSensor.Ultrasonic());
            telemetry.update();
        }
        driveTrain.StopDriving();
        driveTrain.runWithoutEncoders();
        sleep(sleepTime);
    }

    public void drivePIDRange(double power, double distanceAway, Direction Direction ) {
        drivePIDRange(power, distanceAway, Direction, DEFAULT_SLEEP_TIME);
    }

        public void setBrakeMode(int time) {
        driveTrain.setBrakeMode();
        sleep(time);
    }

    public void stopRed(double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        int currentVal = beaconColor.red();
        while ((beaconColor.red() < currentVal + 1 || beaconColor.red() - currentVal < -10)   && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error * KP_STRAIGHT;
            newPower = newPower - (errorkp) * Direction.value;
            driveTrain.setPowerLeft(power);
            driveTrain.setPowerRight(newPower);
            TankDrive.getTelemetry().addTelemetry("Heading", heading);
            TankDrive.getTelemetry().addTelemetry("red Val", beaconColor.red());
            telemetry.update();
        }
        driveTrain.StopDriving();
    }

    public void stopBlue(double power, Direction Direction) {
        driveTrain.runUsingEncoder();
        double targetAngle = imu.getHeading();
        int currentVal = beaconColor.blue();
        while (beaconColor.blue() < currentVal + 1 && opModeIsActive()) {
            double newPower = power;
            double heading = imu.getHeading();
            double error = targetAngle - heading;
            double errorkp = error * KP_STRAIGHT;
            newPower = newPower - (errorkp) * Direction.value;
            driveTrain.setPowerLeft(power);
            driveTrain.setPowerRight(newPower);
            TankDrive.getTelemetry().addTelemetry("Heading", heading);
            telemetry.update();
        }
        driveTrain.StopDriving();
    }
    //rangeSensor
    //setPower
    public void setPowerLeft(double power) {
        driveTrain.setPowerLeft(power);
    }

    public void setPowerRight(double power) {
        driveTrain.setPowerRight(power);
    }

    public void setPowerCollector(double powerCollector) {
        collector.setPower(powerCollector);
    }

    public void setPowerShooter(double powerShooter){
      shooter.runUsingEncoder();
      TankDrive.getTelemetry().addTelemetry("shooterPower", shooter.getPower());
      shooter.setPower(powerShooter);
    }

    public void sleep() {
        sleep(1000);
    }

    public void sleep(int time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    //servos
    public void setIndexer(double position) {
     indexer.setPosition(position);
    }

    public void setBeaconPresser(double power) {
        beaconPresser.setPower(power);
    }

    public void runAllTelemetry(boolean telemetrzeModules) {
        imu.telemetryRun();
        beaconColor.telemetryRun();
        rangeSensor.telemetryRun();
        driveTrain.telemetryRun();
        collector.telemetryRun(true);
        shooter.telemetryRun(true);
        clock.telemetryRun();
        beaconPresser.telemetryRun();
        indexer.telemetryRun();
        if (telemetrzeModules)
            telemetrzeModules();
    }

    private void telemetrzeModules() {
        telemetry.addData("Motor Controller Left Serial#", driveTrain.getControllerLeft());
        telemetry.addData("Motor Controller Right Serial#", driveTrain.getControllerRight());
    }


}
