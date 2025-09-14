/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.DoubleSupplier;

import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcPidStorage;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcPidStorage;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements an Intake Subsystem. This implementation consists of one or two motors and optionally a
 * front and/or back digital sensor(s) that can detect object entering/exiting the intake.
 */
public class Spindexer extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Spindexer";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final boolean HAS_ENTRY_SENSOR            = true;
        public static final boolean HAS_EXIT_SENSOR             = true;

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final MotorType PRIMARY_MOTOR_TYPE        = MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;

        public static final String ENTRY_SENSOR_NAME            = SUBSYSTEM_NAME + ".entrySensor";
        public static final boolean ENTRY_SENSOR_INVERTED       = false;

        public static final String EXIT_SENSOR_NAME             = SUBSYSTEM_NAME + ".exitSensor";
        public static final boolean EXIT_SENSOR_INVERTED        = false;

        public static final double objectDistance = 120.0;
        public static final double movePower = 1.0;
        public static final int maxCapacity = 3;

        public static final double entryLowerTriggerThreshold = 0.0;
        public static final double entryUpperTriggerThreshold = 0.0;
        public static final double entryTriggerSettlingPeriod = 0.0;

        public static final double exitLowerTriggerThreshold = 0.0;
        public static final double exitUpperTriggerThreshold = 0.0;
        public static final double exitTriggerSettlingPeriod = 0.0;

        public static final double[] entryPresetPositions = {0.0, 120.0, 240.0};
        public static final double[] exitPresetPositions = {60.0, 180.0, 300.0};

    }   //class Params

    private final FtcDashboard dashboard;
    private final TrcPidStorage spindexer;
    private final RevColorSensorV3 entryAnalogSensor;
    private final RevColorSensorV3 exitAnalogSensor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Spindexer()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();

        FtcPidStorage.Params spindexerParams = new FtcPidStorage.Params()
                .setPrimaryMotor(Spindexer.Params.PRIMARY_MOTOR_NAME, Spindexer.Params.PRIMARY_MOTOR_TYPE, Spindexer.Params.PRIMARY_MOTOR_INVERTED)
                .setMaxCapacity(Params.maxCapacity)
                .setMovePower(Params.movePower)
                .setObjectDistance(Params.objectDistance);

        if (Params.HAS_ENTRY_SENSOR)
        {
            entryAnalogSensor = FtcOpMode.getInstance().hardwareMap.get(RevColorSensorV3.class, Params.ENTRY_SENSOR_NAME);
            spindexerParams.setEntryAnalogSourceTrigger(Params.ENTRY_SENSOR_NAME, this::getEntrySensorData, Params.entryLowerTriggerThreshold, Params.entryUpperTriggerThreshold, Params.entryTriggerSettlingPeriod, false, null, null);
        }

        if (Params.HAS_EXIT_SENSOR)
        {
            exitAnalogSensor = FtcOpMode.getInstance().hardwareMap.get(RevColorSensorV3.class, Params.EXIT_SENSOR_NAME);
            spindexerParams.setExitAnalogSourceTrigger(Params.EXIT_SENSOR_NAME, this::getExitSensorData, Params.exitLowerTriggerThreshold, Params.exitUpperTriggerThreshold, Params.exitTriggerSettlingPeriod, false, null, null);
        }

        spindexer = new FtcPidStorage(Params.SUBSYSTEM_NAME, spindexerParams).getPidStorage();
    }   //Intake

    /**
     * This method returns the created TrcRollerIntake.
     *
     * @return created Roller Intake.
     */
    public TrcPidStorage getPidStorage()
    {
        return spindexer;
    }   //getIntake

    private double getEntrySensorData()
    {
        if (entryAnalogSensor != null) {
            return entryAnalogSensor.getDistance(DistanceUnit.INCH);
        }
        else
        {
            return 0.0;
        }
    }

    private double getExitSensorData()
    {
        if (exitAnalogSensor != null) {
            return exitAnalogSensor.getDistance(DistanceUnit.INCH);
        }
        else
        {
            return 0.0;
        }
    }

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        // Spindexer does not need cancel.
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        // Intake does not need zero calibration.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Intake does not support resetState.
    }   //resetState

    /**
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (slowLoop)
        {
            dashboard.displayPrintf(
                lineNum++, "%s: sensorState=%s/%s, numObjects=%s",
                Params.SUBSYSTEM_NAME, spindexer.isEntrySensorActive(), spindexer.isExitSensorActive(), spindexer.getNumObjects());
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to prep the subsystem for tuning.
     *
     * @param subComponent specifies the sub-component of the Subsystem to be tuned, can be null if no sub-component.
     * @param tuneParams specifies tuning parameters.
     */
    @Override
    public void prepSubsystemForTuning(String subComponent, double... tuneParams)
    {
        // Intake subsystem doesn't need tuning.
    }   //prepSubsystemForTuning



}   //class Intake
