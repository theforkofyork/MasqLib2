package BasicLib4997.Motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import BasicLib4997.Motors.LiftSystem;
import BasicLib4997.Motors.Motor;
import BasicLib4997.Motors.MotorSystem;
import BasicLib4997.Motors.TankDrive.PID_Constants;
import BasicLib4997.Motors.TankDrive.TankDrive;
import BasicLib4997.Sensors.AdafruitIMU;
import BasicLib4997.Sensors.Clock;
import BasicLib4997.Sensors.I2CColorSensor;
import BasicLib4997.Sensors.MR_RangeSensor;
import BasicLib4997.Sensors.Sensor_Thresholds;
import BasicLib4997.Servos.CR_Servo;
import BasicLib4997.Servos.Servo;
import static BasicLib4997.Motors.MotorSystem.convert;

/**
 * Created by Archish on 10/28/16.
 */

public class DefenseBot implements PID_Constants, Sensor_Thresholds {

    public DefenseBot(Telemetry telemetry){
        this.telemetry  = telemetry;
        instance = this;
    }
    public static DefenseBot getTelemetry(){
        return instance;
    }
    private static DefenseBot instance;
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
    public void contUpdate () {
        new Thread(new Runnable() {
            public void run() {
                while (opModeIsActive()) {
                    telemetry.update();
                }
            }
        }).start();
    }

    // Motor and Motor Systems

    public MotorSystem driveTrain = new MotorSystem("leftFront", "leftBack", "rightFront", "rightBack");
    //
    private static final int DEFAULT_SLEEP_TIME = 1000;
    private static final double DEFAULT_TIMEOUT = 3;


    private boolean opModeIsActive() {
        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
    }
    //rangeSensor
    //setPower
    public void setPowerLeft(double power) {
        driveTrain.setPowerLeft(power);
    }

    public void setPowerRight(double power) {
        driveTrain.setPowerRight(power);
    }






}
