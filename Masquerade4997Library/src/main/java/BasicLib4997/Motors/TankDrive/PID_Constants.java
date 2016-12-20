package BasicLib4997.Motors.TankDrive;

/**
 * These are the constants used in PID
 */
public interface PID_Constants {
    double
            KP_TURN = 0.01,
            KI_TURN = 0.000001,
            KD_TURN = 0.7;
    double
            KP_STRAIGHT = 0.02,
            KI_STRAIGHT = 0,
            KD_STRAIGHT = 0;

    double tickForAndymark = 1120;
    double wheelDiameter = 4;
    double cmToInches = 2.54;
    double CLICKS_PER_CM = tickForAndymark / wheelDiameter * cmToInches / Math.PI;
}

