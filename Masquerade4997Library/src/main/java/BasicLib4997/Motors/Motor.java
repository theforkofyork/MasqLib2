package BasicLib4997.Motors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.internal.TelemetryImpl;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import BasicLib4997.Motors.TankDrive.TankDrive;

/**
 * This is a custom motor that includes stall detection and telemetry
 */
public class Motor {
    private Telemetry telemetry;
    private DcMotor motor;
    private String nameMotor;
    public Motor(String name){
        this.nameMotor = name;
        motor = FtcOpModeRegister.opModeManager.getHardwareMap().dcMotor.get(name);
    }
    public Motor(String name, DcMotor.Direction direction) {
        this.nameMotor = name;
        motor = FtcOpModeRegister.opModeManager.getHardwareMap().dcMotor.get(name);
        motor.setDirection(direction);
    }
    public void runWithoutEncoders () {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    boolean isStalled () {

        double prevPosition = motor.getCurrentPosition();
        boolean isStalled;
        if (motor.getCurrentPosition() > prevPosition) {
            isStalled = false;
        }
        else if(motor.getCurrentPosition() - 10 > prevPosition) {
            isStalled = false;
        }
        else {
            isStalled = true;
        }
        return isStalled;

    }
    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setPowerNoEncoder (double power) {
        double MAX_POWER = 14;
        double error = MAX_POWER - getBatteryVoltage();
        double kp = 0.02;
        double newPower = power + (error *kp);
        motor.setPower(newPower);
    }
    public void setPower (double power) {
        motor.setPower(power);
    }
    public void setDistance(int distance){
        motor.setTargetPosition(distance);
    }
    public void runUsingEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runToPosition(){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    boolean isBusy () {
        return motor.isBusy();
    }
    public void setbrakeMode () {
        setPower(0.001);
    }
    double getCurrentPos () {
        double currentPos;
        currentPos = motor.getCurrentPosition();
        return currentPos;
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : FtcOpModeRegister.opModeManager.getHardwareMap().voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    public double getPower() {
        return motor.getPower();
    }
    public DcMotorController getController() {
        return motor.getController();
    }
    public void telemetryRun (boolean showCurrentPos) {

        TankDrive.getTelemetry().addTelemetry(nameMotor + "telemetry");
        TankDrive.getTelemetry().addTelemetry("isStalled", isStalled());
        TankDrive.getTelemetry().addTelemetry("isBusy", isBusy());
        if (showCurrentPos) {
        TankDrive.getTelemetry().addTelemetry("Current Position", getCurrentPos());
        }
    }
}

