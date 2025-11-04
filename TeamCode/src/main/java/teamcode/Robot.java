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

package teamcode;

import androidx.annotation.NonNull;

import ftclib.drivebase.FtcRobotDrive;
import ftclib.driverio.FtcDashboard;
import ftclib.driverio.FtcMatchInfo;
import ftclib.robotcore.FtcOpMode;
import ftclib.sensor.FtcRobotBattery;
import teamcode.autotasks.TaskAutoPickup;
import teamcode.autotasks.TaskAutoShoot;
import teamcode.subsystems.Intake;
import teamcode.indicators.LEDIndicator;
import teamcode.subsystems.BaseDrive;
import teamcode.subsystems.Shooter;
import teamcode.subsystems.Spindexer;
import teamcode.vision.Vision;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.sensor.TrcDigitalInput;
import trclib.subsystem.TrcPidStorage;
import trclib.subsystem.TrcRollerIntake;
import trclib.subsystem.TrcShooter;
import trclib.subsystem.TrcSubsystem;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    private final String moduleName = getClass().getSimpleName();
    // Global objects.
    public final FtcOpMode opMode;
    public final TrcDbgTrace globalTracer;
    public final FtcDashboard dashboard;
    public static FtcMatchInfo matchInfo = null;
    private static TrcPose2D endOfAutoRobotPose = null;
    // Robot Drive.
    public BaseDrive baseDrive;
    public FtcRobotDrive.RobotInfo robotInfo;
    public FtcRobotDrive robotDrive;
    // Vision subsystems.
    public Vision vision;
    // Sensors and indicators.
    public LEDIndicator ledIndicator;
    public FtcRobotBattery battery;
    // Subsystems.
    public Intake intakeSubsystem;
    public TrcRollerIntake intake;
    public Spindexer spindexerSubsystem;
    public TrcPidStorage spindexer;
    public Shooter shooterSubsystem;
    public TrcShooter shooter;
    // Autotasks.
    public TaskAutoPickup autoPickupTask;
    public TaskAutoShoot autoShootTask;

    public int obeliskAprilTagId = 0;
    public Vision.ArtifactType[] obeliskMotif = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *        specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        // Initialize global objects.
        opMode = FtcOpMode.getInstance();
        globalTracer = TrcDbgTrace.getGlobalTracer();
        dashboard = FtcDashboard.getInstance();
        speak("Init starting");
        // Create and initialize Robot Base.
        baseDrive = new BaseDrive();
        robotInfo = baseDrive.getRobotInfo();
        robotDrive = baseDrive.getRobotDrive();
        // Create and initialize vision subsystems.
        if (RobotParams.Preferences.useVision &&
            (RobotParams.Preferences.useLimelightVision ||
             RobotParams.Preferences.useWebcamAprilTagVision ||
             RobotParams.Preferences.useArtifactVision ||
             RobotParams.Preferences.useClassifierVision))
        {
            vision = new Vision(this);
        }
        // If robotType is VisionOnly, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        if (RobotParams.Preferences.robotType != BaseDrive.RobotType.VisionOnly)
        {
            // Create and initialize sensors and indicators.
            ledIndicator = robotInfo.indicatorNames != null? new LEDIndicator(robotInfo.indicatorNames): null;
            battery = RobotParams.Preferences.useBatteryMonitor? new FtcRobotBattery(): null;
            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                // Create subsystems.
                if (RobotParams.Preferences.useIntake)
                {
                    intakeSubsystem = new Intake(this);
                    intake = intakeSubsystem.getIntake();
                }

                if (RobotParams.Preferences.useSpindexer)
                {
                    spindexerSubsystem = new Spindexer(this);
                    spindexer = spindexerSubsystem.getPidStorage();
                }

                if (RobotParams.Preferences.useShooter)
                {
                    shooterSubsystem = new Shooter(this);
                    shooter = shooterSubsystem.getShooter();
                }

                // Create autotasks.
                if (RobotParams.Preferences.useAutoPickup && intake != null && spindexer != null)
                {
                    autoPickupTask = new TaskAutoPickup(this);
                }

                if (RobotParams.Preferences.useAutoShoot && shooter != null && spindexer != null)
                {
                    autoShootTask = new TaskAutoShoot(this);
                }

                // Zero calibrate all subsystems only in Auto or if TeleOp is run standalone without prior Auto.
                // There is no reason to zero calibrate again if Auto was run right before TeleOp.
                if (runMode == TrcRobot.RunMode.AUTO_MODE || FtcAuto.autoChoices.alliance == null)
                {
                    zeroCalibrate(null, null);
                }
            }
        }
        speak("Init complete");
        Dashboard.DashboardParams.updateDashboardEnabled = RobotParams.Preferences.updateDashboard;
        if (Dashboard.DashboardParams.updateDashboardEnabled)
        {
            dashboard.enableDashboardUpdate(1, true);
        }
    }   //Robot

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @NonNull
    @Override
    public String toString()
    {
        return robotInfo != null? robotInfo.robotName: RobotParams.Preferences.robotType.toString();
    }   //toString

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        if (robotDrive != null)
        {
            //
            // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(true);
                // The following are performance counters, could be disabled for competition if you want.
                // But it might give you some insight if somehow autonomous wasn't performing as expected.
                robotDrive.gyro.setElapsedTimerEnabled(true);
            }
            //
            // Enable odometry for all opmodes. We may need odometry in TeleOp for auto-assist drive.
            //
            robotDrive.driveBase.setOdometryEnabled(true);
            if (runMode == TrcRobot.RunMode.TELEOP_MODE)
            {
                if (endOfAutoRobotPose != null)
                {
                    // We had a previous autonomous run that saved the robot position at the end, use it.
                    robotDrive.driveBase.setFieldPosition(endOfAutoRobotPose);
                    globalTracer.traceInfo(moduleName, "Restore saved RobotPose=" + endOfAutoRobotPose);
                }
            }
            // Consume it so it's no longer valid for next run.
            endOfAutoRobotPose = null;
        }

        TrcDigitalInput.setElapsedTimerEnabled(true);
        TrcMotor.setElapsedTimerEnabled(true);
        TrcServo.setElapsedTimerEnabled(true);
    }   //startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to stop, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        // Cancel all operations.
        cancelAll();
        //
        // Print all performance counters if there are any.
        //
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.printElapsedTime(globalTracer);
            robotDrive.gyro.setElapsedTimerEnabled(false);
        }
        TrcDigitalInput.printElapsedTime(globalTracer);
        TrcDigitalInput.setElapsedTimerEnabled(false);
        TrcMotor.printElapsedTime(globalTracer);
        TrcMotor.setElapsedTimerEnabled(false);
        TrcServo.printElapsedTime(globalTracer);
        TrcServo.setElapsedTimerEnabled(false);
        //
        // Disable vision.
        //
        if (vision != null)
        {
            if (vision.isLimelightVisionEnabled())
            {
                globalTracer.traceInfo(moduleName, "Disabling LimelightVision.");
                vision.setLimelightVisionEnabled(Vision.LimelightPipelineType.APRIL_TAG, false);
            }

            if (vision.isWebcamAprilTagVisionEnabled())
            {
                globalTracer.traceInfo(moduleName, "Disabling Webcam AprilTagVision.");
                vision.setWebcamAprilTagVisionEnabled(false);
            }

            if (vision.artifactVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling ArtifactVision.");
                vision.setArtifactVisionEnabled(Vision.ArtifactType.Any, false);
            }

            if (vision.classifierVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling ClassifierVision.");
                vision.setClassifierVisionEnabled(false);
            }

            vision.close();
       }

        if (robotDrive != null)
        {
            if (runMode == TrcRobot.RunMode.AUTO_MODE)
            {
                // Save current robot location at the end of autonomous so subsequent teleop run can restore it.
                endOfAutoRobotPose = robotDrive.driveBase.getFieldPosition();
                globalTracer.traceInfo(moduleName, "Saved robot pose=" + endOfAutoRobotPose);
            }
            //
            // Disable odometry.
            //
            robotDrive.driveBase.setOdometryEnabled(false);
            //
            // Disable gyro task.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(false);
            }
        }
    }   //stopMode

    /**
     * This method is called to cancel all pending operations and release the ownership of all subsystems.
     */
    public void cancelAll()
    {
        globalTracer.traceInfo(moduleName, "Cancel all operations.");
        // Cancel auto tasks.
        if (autoPickupTask != null)
        {
            autoPickupTask.cancel();
        }

        if (autoShootTask != null)
        {
            autoShootTask.cancel();
        }
        // Cancel subsystems.
        TrcSubsystem.cancelAll();
    }   //cancelAll

    /**
     * This method zero calibrates all subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param event specifies the event to signal when the zero calibration is done.
     */
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        globalTracer.traceInfo(moduleName, "Zero calibrate all subsystems.");
        TrcSubsystem.zeroCalibrateAll(owner, event);
    }   //zeroCalibrate

    /**
     * This method retracts all appendages for robot high speed travelling.
     */
    public void turtle()
    {
        globalTracer.traceInfo(moduleName, "Turtle mode.");
        TrcSubsystem.resetStateAll();
    }   //turtle

    /**
     * This method sets the robot's starting position according to the autonomous choices.
     *
     * @param autoChoices specifies all the auto choices.
     */
    public void setRobotStartPosition(FtcAuto.AutoChoices autoChoices)
    {
        TrcPose2D startPose = adjustPoseByAlliance(
            autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE? RobotParams.Game.STARTPOSE_RED_GOAL_ZONE:
            autoChoices.startPos == FtcAuto.StartPos.LOAD_CENTER? RobotParams.Game.STARTPOSE_RED_LOAD_CENTER:
                RobotParams.Game.STARTPOSE_RED_LOAD_CORNER,
            autoChoices.alliance, false);
        robotDrive.driveBase.setFieldPosition(startPose);
    }   //setRobotStartPosition

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param x specifies x position in the red alliance in the specified unit.
     * @param y specifies y position in the red alliance in the specified unit.
     * @param heading specifies heading in the red alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if x and y are in tile unit, false if in inches.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(
        double x, double y, double heading, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        TrcPose2D newPose = new TrcPose2D(x, y, heading);

        if (alliance == FtcAuto.Alliance.BLUE_ALLIANCE)
        {
            // Translate blue alliance pose to red alliance pose.
            if (RobotParams.Game.fieldIsMirrored)
            {
                // Field is mirrored on X axis.
                // Same X, Flip Y. Heading left becomes right and right becomes left.
                double angleDelta = (newPose.angle - 90.0)*2.0;
                newPose.angle -= angleDelta;
                newPose.y = -newPose.y;
            }
            else
            {
                // Field is symmetrical.
                // Flip X, Flip Y. Heading flips 180-degree.
                newPose.x = -newPose.x;
                newPose.y = -newPose.y;
                newPose.angle = (newPose.angle + 180.0) % 360.0;
            }
        }

        if (isTileUnit)
        {
            newPose.x *= RobotParams.Field.FULL_TILE_INCHES;
            newPose.y *= RobotParams.Field.FULL_TILE_INCHES;
        }

        return newPose;
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param x specifies x position in the red alliance in tile unit.
     * @param y specifies y position in the red alliance in tile unit.
     * @param heading specifies heading in the red alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(double x, double y, double heading, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(x, y, heading, alliance, false);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param pose specifies pose in the red alliance in the specified unit.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if pose is in tile units, false in inches.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        return adjustPoseByAlliance(pose.x, pose.y, pose.angle, alliance, isTileUnit);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param pose specifies pose in the red alliance in tile unit.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(pose, alliance, false);
    }   //adjustPoseByAlliance

    /**
     * This method sends the text string to the Driver Station to be spoken using text to speech.
     *
     * @param sentence specifies the sentence to be spoken by the Driver Station.
     */
    public void speak(String sentence)
    {
        opMode.telemetry.speak(sentence);
    }   //speak

}   //class Robot
