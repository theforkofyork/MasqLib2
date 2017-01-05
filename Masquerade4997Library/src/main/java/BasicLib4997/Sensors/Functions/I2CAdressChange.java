package BasicLib4997.Sensors.Functions;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import java.util.concurrent.locks.Lock;

import BasicLib4997.Motors.TankDrive.TankDrive;
import BasicLib4997.Sensors.MRColorSensor;

/**
 * Created by Archish on 12/1/16.
 */

public class I2CAdressChange {
    public static final int ADDRESS_SET_NEW_I2C_ADDRESS = 0x70;
    public static final byte TRIGGER_BYTE_1 = 0x55;
    public static final byte TRIGGER_BYTE_2 = (byte) 0xaa;

    public static final byte IR_SEEKER_V3_FIRMWARE_REV = 0x12;
    public static final byte IR_SEEKER_V3_SENSOR_ID = 0x49;
    public static final I2cAddr IR_SEEKER_V3_ORIGINAL_ADDRESS = I2cAddr.create8bit(0x38);

    public static final byte COLOR_SENSOR_FIRMWARE_REV = 0x10;
    public static final byte COLOR_SENSOR_SENSOR_ID = 0x43;
    public static final byte COLOR_SENSOR_ORIGINAL_ADDRESS = 0x3C;

    public static final byte MANUFACTURER_CODE = 0x4d;
    public static final byte FIRMWARE_REV = IR_SEEKER_V3_FIRMWARE_REV;
    public static final byte SENSOR_ID = IR_SEEKER_V3_SENSOR_ID;

    public static final int READ_MODE = 0x80;
    public static final int ADDRESS_MEMORY_START = 0x0;
    public static final int TOTAL_MEMORY_LENGTH = 0x0c;
    public static final int BUFFER_CHANGE_ADDRESS_LENGTH = 0x03;
    int port;
    String colorName;
    public I2CAdressChange(int port, String name) {
        this.colorName = name;
        this.port = port;
        changeAdress();
    }

    byte[] readCache;
    Lock readLock;
    byte[] writeCache;
    Lock writeLock;

    I2cAddr currentAddress = IR_SEEKER_V3_ORIGINAL_ADDRESS;
    // I2c addresses on Modern Robotics devices must be divisible by 2, and between 0x7e and 0x10
    // Different hardware may have different rules.
    // Be sure to read the requirements for the hardware you're using!
    // If you use an invalid address, you may make your device completely unusable.
    I2cAddr newAddress = I2cAddr.create8bit(0x42);

    DeviceInterfaceModule dim;
    public void changeAdress() {
        // set up the hardware devices we are going to use
        dim = FtcOpModeRegister.opModeManager.getHardwareMap().deviceInterfaceModule.get("DIM");
        MRColorSensor colorSensor = new MRColorSensor(colorName);
        readCache = dim.getI2cReadCache(port);
        readLock = dim.getI2cReadCacheLock(port);
        writeCache = dim.getI2cWriteCache(port);
        writeLock = dim.getI2cWriteCacheLock(port);

        // I2c addresses on Modern Robotics devices must be divisible by 2, and between 0x7e and 0x10
        // Different hardware may have different rules.
        // Be sure to read the requirements for the hardware you're using!
        ModernRoboticsUsbDeviceInterfaceModule.throwIfModernRoboticsI2cAddressIsInvalid(newAddress);

        // wait for the start button to be pressed
        performAction("read", port, currentAddress, ADDRESS_MEMORY_START, TOTAL_MEMORY_LENGTH);

        while (!dim.isI2cPortReady(port)) {
            sleep(1000);
        }

        // update the local cache
        dim.readI2cCacheFromController(port);

        // make sure the first bytes are what we think they should be.
        int count = 0;
        int[] initialArray = {READ_MODE, currentAddress.get8Bit(), ADDRESS_MEMORY_START, TOTAL_MEMORY_LENGTH, FIRMWARE_REV, MANUFACTURER_CODE, SENSOR_ID};
        while (!foundExpectedBytes(initialArray, readLock, readCache)) {
            dim.readI2cCacheFromController(port);
            sleep(1000);
            count++;
            // if we go too long with failure, we probably are expecting the wrong bytes.
            if (count >= 10) {
            }
        }

        // Enable writes to the correct segment of the memory map.
        performAction("write", port, currentAddress, ADDRESS_SET_NEW_I2C_ADDRESS, BUFFER_CHANGE_ADDRESS_LENGTH);

        // Write out the trigger bytes, and the new desired address.
        writeNewAddress();
        dim.setI2cPortActionFlag(port);
        dim.writeI2cCacheToController(port);


        // Changing the I2C address takes some time.
        sleep(60000);

        // Query the new address and see if we can get the bytes we expect.
        dim.enableI2cReadMode(port, newAddress, ADDRESS_MEMORY_START, TOTAL_MEMORY_LENGTH);
        dim.setI2cPortActionFlag(port);
        dim.writeI2cCacheToController(port);

        int[] confirmArray = {READ_MODE, newAddress.get8Bit(), ADDRESS_MEMORY_START, TOTAL_MEMORY_LENGTH, FIRMWARE_REV, MANUFACTURER_CODE, SENSOR_ID};
        while (!foundExpectedBytes(confirmArray, readLock, readCache)) {

            dim.readI2cCacheFromController(port);
            sleep(1000);
        }

        RobotLog.i("Successfully changed the I2C address." + String.format("New address: 0x%02x", newAddress));

        /**** IMPORTANT NOTE ******/
        // You need to add a line like this at the top of your op mode
        // to update the I2cAddress in the driver.
        colorSensor.setI2cAddress(newAddress);
        /***************************/


    }
    private boolean foundExpectedBytes(int[] byteArray, Lock lock, byte[] cache) {
        try {
            lock.lock();
            boolean allMatch = true;
            StringBuilder s = new StringBuilder(300 * 4);
            String mismatch = "";
            for (int i = 0; i < byteArray.length; i++) {
                s.append(String.format("expected: %02x, got: %02x \n", TypeConversion.unsignedByteToInt( (byte) byteArray[i]), cache[i]));
                if (TypeConversion.unsignedByteToInt(cache[i]) != TypeConversion.unsignedByteToInt( (byte) byteArray[i])) {
                    mismatch = String.format("i: %d, byteArray[i]: %02x, cache[i]: %02x", i, byteArray[i], cache[i]);
                    allMatch = false;
                }
            }
            RobotLog.e(s.toString() + "\n allMatch: " + allMatch + ", mismatch: " + mismatch);
            return allMatch;
        } finally {
            lock.unlock();
        }
    }

    private void performAction(String actionName, int port, I2cAddr i2cAddress, int memAddress, int memLength) {
        if (actionName.equalsIgnoreCase("read")) dim.enableI2cReadMode(port, i2cAddress, memAddress, memLength);
        if (actionName.equalsIgnoreCase("write")) dim.enableI2cWriteMode(port, i2cAddress, memAddress, memLength);

        dim.setI2cPortActionFlag(port);
        dim.writeI2cCacheToController(port);
        dim.readI2cCacheFromController(port);
    }

    private void writeNewAddress() {
        try {
            writeLock.lock();
            writeCache[4] = (byte) newAddress.get8Bit();
            writeCache[5] = TRIGGER_BYTE_1;
            writeCache[6] = TRIGGER_BYTE_2;
        } finally {
            writeLock.unlock();
        }
    }
    public void sleep (double time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}


