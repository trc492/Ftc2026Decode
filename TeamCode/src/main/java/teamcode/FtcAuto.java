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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Locale;

import ftclib.driverio.FtcChoiceMenu;
import ftclib.driverio.FtcMatchInfo;
import ftclib.driverio.FtcMenu;
import ftclib.driverio.FtcValueMenu;
import ftclib.robotcore.FtcOpMode;
import ftclib.vision.FtcLimelightVision;
import teamcode.autocommands.CmdDecodeAuto;
import teamcode.vision.Vision;
import trclib.command.CmdPidDrive;
import trclib.command.CmdTimedDrive;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.timer.TrcTimer;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name="FtcAutonomous", group="FtcTeam")
public class FtcAuto extends FtcOpMode
{
    private final String moduleName = getClass().getSimpleName();

    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public enum StartPos
    {
        GOAL_ZONE,
        LOAD_CENTER,
        LOAD_CORNER
    }   //enum StartPos

    public enum AutoStrategy
    {
        DECODE_AUTO,
        PID_DRIVE,
        TIMED_DRIVE,
        DO_NOTHING
    }   //enum AutoStrategy

    public enum PickupOption
    {
        SPIKEMARKS,
        LOADING_ZONE,
        BOTH
    }   //enum PickupOption

    public enum ParkOption
    {
        PARK,
        NO_PARK
    }   //enum ParkOption

    /**
     * This class stores the autonomous menu choices.
     */
    public static class AutoChoices
    {
        public double startDelay = 0.0;
        public Alliance alliance = null;
        public StartPos startPos = StartPos.GOAL_ZONE;
        public AutoStrategy strategy = AutoStrategy.DECODE_AUTO;
        public PickupOption pickupOption = PickupOption.SPIKEMARKS;
        public double spikeMarkCount = 0.0;
        public double shootDelay1 = 0.0;
        public double shootDelay2 = 0.0;
        public double shootDelay3 = 0.0;
        public ParkOption parkOption = ParkOption.PARK;
        public double xTarget = 0.0;
        public double yTarget = 0.0;
        public double turnTarget = 0.0;
        public double driveTime = 0.0;
        public double drivePower = 0.0;

        @NonNull
        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "startDelay=%.0f " +
                "alliance=\"%s\" " +
                "startPos=\"%s\" " +
                "strategy=\"%s\" " +
                "pickupOption=\"%s\" +" +
                "spikeMarkCount=\"%s\" " +
                "shootDelay1=%.0f " +
                "shootDelay2=%.0f " +
                "shootDelay3=%.0f " +
                "parkOption=\"%s\" " +
                "xTarget=%.1f " +
                "yTarget=%.1f " +
                "turnTarget=%.0f " +
                "driveTime=%.0f " +
                "drivePower=%.1f",
                startDelay, alliance, startPos, strategy, pickupOption, spikeMarkCount,
                shootDelay1, shootDelay2, shootDelay3, parkOption,
                xTarget, yTarget, turnTarget, driveTime, drivePower);
        }   //toString

    }   //class AutoChoices

    public static final AutoChoices autoChoices = new AutoChoices();
    private Robot robot;
    private TrcRobot.RobotCommand autoCommand;

    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void robotInit()
    {
        //
        // Create and initialize robot object.
        //
        robot = new Robot(TrcRobot.getRunMode());
        //
        // Open trace log.
        //
        if (RobotParams.Preferences.useTraceLog)
        {
            Robot.matchInfo = FtcMatchInfo.getMatchInfo();
            String filePrefix = String.format(
                Locale.US, "%s%02d_Auto", Robot.matchInfo.matchType, Robot.matchInfo.matchNumber);
            TrcDbgTrace.openTraceLog(RobotParams.Robot.LOG_FOLDER_PATH, filePrefix);
        }
        //
        // Create and run choice menus.
        //
        doAutoChoicesMenus();
        //
        // Create autonomous command according to chosen strategy.
        //
        switch (autoChoices.strategy)
        {
            case DECODE_AUTO:
                if (robot.robotDrive != null)
                {
                    autoCommand = new CmdDecodeAuto(robot, autoChoices);
                }
                break;

            case PID_DRIVE:
                if (robot.robotDrive != null)
                {
                    autoCommand = new CmdPidDrive(robot.robotDrive.driveBase, robot.robotDrive.pidDrive);
                }
                break;

            case TIMED_DRIVE:
                if (robot.robotDrive != null)
                {
                    autoCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, autoChoices.startDelay, autoChoices.driveTime,
                        0.0, autoChoices.drivePower, 0.0);
                }
                break;

            case DO_NOTHING:
            default:
                autoCommand = null;
                break;
        }

        if (robot.vision != null)
        {
            // Enabling vision early so we can detect Obelisk AprilTag before match starts.
            if (robot.vision.limelightVision != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision.");
                robot.vision.setLimelightVisionEnabled(Vision.LimelightPipelineType.APRIL_TAG.ordinal(), true);
            }
        }
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called periodically after robotInit() is called but before competition starts. For example,
     * we can put vision code here to detect target before autonomous starts, or code to move the subsystems to a
     * particular configuration to be within the 18-inch starting restriction.
     */
    @Override
    public void initPeriodic()
    {
        // Detect Obelisk AprilTag.
        if (robot.vision != null && robot.vision.isLimelightVisionEnabled())
        {
            TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> detectedAprilTag =
                robot.vision.getLimelightDetectedObject(
                    FtcLimelightVision.ResultType.Fiducial, RobotParams.Game.obeliskAprilTags, -1);
            if (detectedAprilTag != null)
            {
                robot.obeliskAprilTagId = (int) detectedAprilTag.detectedObj.objId;
                robot.motif = RobotParams.Game.motifPatterns[robot.obeliskAprilTagId - 21];
            }
        }
    }   //initPeriodic

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(
            moduleName, "***** Starting autonomous: " + TrcTimer.getCurrentTimeString() + " *****");
        if (Robot.matchInfo != null)
        {
            robot.globalTracer.logInfo(moduleName, "MatchInfo", Robot.matchInfo.toString());
        }
        robot.globalTracer.logInfo(moduleName, "AutoChoices", autoChoices.toString());
        robot.globalTracer.traceInfo(moduleName, "Obelisk AprilTag %d: %s", robot.obeliskAprilTagId, robot.motif);
        robot.dashboard.clearDisplay();
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(true);
        }

        if (autoChoices.strategy == AutoStrategy.PID_DRIVE && autoCommand != null)
        {
            ((CmdPidDrive) autoCommand).start(
                autoChoices.startDelay, autoChoices.drivePower, null,
                new TrcPose2D(autoChoices.xTarget*12.0, autoChoices.yTarget*12.0, autoChoices.turnTarget));
        }
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        //
        // OpMode is about to stop, cancel autonomous command in progress if any.
        //
        if (autoCommand != null)
        {
            autoCommand.cancel();
        }
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(false);
        }

        robot.globalTracer.traceInfo(
            moduleName, "***** Stopping autonomous: " + TrcTimer.getCurrentTimeString() + " *****");
        printPerformanceMetrics();

        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.closeTraceLog();
        }
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (autoCommand != null)
        {
            //
            // Run the autonomous command.
            //
            autoCommand.cmdPeriodic(elapsedTime);
        }

        Dashboard.updateDashboard(robot, 1);
    }   //periodic

    /**
     * This method creates the autonomous menus, displays them and stores the choices.
     */
    private void doAutoChoicesMenus()
    {
        //
        // Construct menus.
        //
        FtcValueMenu startDelayMenu = new FtcValueMenu("Start delay:", null, 0.0, 30.0, 1.0, 0.0, " %.0f sec");
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", startDelayMenu);
        FtcChoiceMenu<StartPos> startPosMenu = new FtcChoiceMenu<>("Start Position:", allianceMenu);
        FtcChoiceMenu<AutoStrategy> strategyMenu = new FtcChoiceMenu<>("Auto Strategies:", startPosMenu);
        FtcChoiceMenu<PickupOption> pickupOptionMenu = new FtcChoiceMenu<>("Pickup Option:", strategyMenu);
        FtcValueMenu spikeMarkCountMenu =
            new FtcValueMenu("SpikeMark Count:", pickupOptionMenu, 0.0, 3.0, 1.0, 2.0, " %.0f");
        FtcValueMenu shootDelay1Menu =
            new FtcValueMenu("First Shoot delay:", spikeMarkCountMenu, 0.0, 30.0, 1.0, 0.0, " %.0f sec");
        FtcValueMenu shootDelay2Menu =
            new FtcValueMenu("Second Shoot delay:", shootDelay1Menu, 0.0, 30.0, 1.0, 0.0, " %.0f sec");
        FtcValueMenu shootDelay3Menu =
            new FtcValueMenu("Third Shoot delay time:", shootDelay2Menu, 0.0, 30.0, 1.0, 0.0, " %.0f sec");
        FtcChoiceMenu<ParkOption> parkOptionMenu = new FtcChoiceMenu<>("Park Option:", pickupOptionMenu);

        FtcValueMenu xTargetMenu =
            new FtcValueMenu("xTarget:", strategyMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
        FtcValueMenu yTargetMenu =
            new FtcValueMenu("yTarget:", xTargetMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
        FtcValueMenu turnTargetMenu =
            new FtcValueMenu("turnTarget:", yTargetMenu, -180.0, 180.0, 5.0, 90.0, " %.0f deg");
        FtcValueMenu driveTimeMenu =
            new FtcValueMenu("Drive time:", strategyMenu, 0.0, 30.0, 1.0, 5.0, " %.0f sec");
        FtcValueMenu drivePowerMenu =
            new FtcValueMenu("Drive power:", strategyMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");

        // Link Value Menus to their children.
        startDelayMenu.setChildMenu(allianceMenu);
        shootDelay1Menu.setChildMenu(shootDelay2Menu);
        shootDelay2Menu.setChildMenu(shootDelay3Menu);
        shootDelay3Menu.setChildMenu(parkOptionMenu);
        xTargetMenu.setChildMenu(yTargetMenu);
        yTargetMenu.setChildMenu(turnTargetMenu);
        turnTargetMenu.setChildMenu(drivePowerMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        //
        // Populate choice menus.
        //
        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, startPosMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, startPosMenu);

        startPosMenu.addChoice("Start Position Goal Zone", StartPos.GOAL_ZONE, true, strategyMenu);
        startPosMenu.addChoice("Start Position Loading Zone", StartPos.LOAD_CENTER, false, strategyMenu);
        startPosMenu.addChoice("Start Position Loading Zone", StartPos.LOAD_CORNER, false, strategyMenu);

        strategyMenu.addChoice("Decode Auto", AutoStrategy.DECODE_AUTO, true, pickupOptionMenu);
        strategyMenu.addChoice("PID Drive", AutoStrategy.PID_DRIVE, false, xTargetMenu);
        strategyMenu.addChoice("Timed Drive", AutoStrategy.TIMED_DRIVE, false, driveTimeMenu);
        strategyMenu.addChoice("Do nothing", AutoStrategy.DO_NOTHING, false);

        pickupOptionMenu.addChoice("Spike Marks", PickupOption.SPIKEMARKS, true, spikeMarkCountMenu);
        pickupOptionMenu.addChoice("Loading Zone", PickupOption.LOADING_ZONE, false, parkOptionMenu);
        pickupOptionMenu.addChoice("Both", PickupOption.BOTH, false, spikeMarkCountMenu);

        parkOptionMenu.addChoice("Park", ParkOption.PARK, true);
        parkOptionMenu.addChoice("No Park", ParkOption.NO_PARK, false);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(startDelayMenu);
        //
        // Fetch choices.
        //
        autoChoices.startDelay = startDelayMenu.getCurrentValue();
        autoChoices.alliance = allianceMenu.getCurrentChoiceObject();
        autoChoices.startPos = startPosMenu.getCurrentChoiceObject();
        autoChoices.strategy = strategyMenu.getCurrentChoiceObject();
        autoChoices.pickupOption = pickupOptionMenu.getCurrentChoiceObject();
        autoChoices.spikeMarkCount = spikeMarkCountMenu.getCurrentValue();
        autoChoices.shootDelay1 = shootDelay1Menu.getCurrentValue();
        autoChoices.shootDelay2 = shootDelay2Menu.getCurrentValue();
        autoChoices.shootDelay3 = shootDelay3Menu.getCurrentValue();
        autoChoices.parkOption = parkOptionMenu.getCurrentChoiceObject();
        autoChoices.xTarget = xTargetMenu.getCurrentValue();
        autoChoices.yTarget = yTargetMenu.getCurrentValue();
        autoChoices.turnTarget = turnTargetMenu.getCurrentValue();
        autoChoices.driveTime = driveTimeMenu.getCurrentValue();
        autoChoices.drivePower = drivePowerMenu.getCurrentValue();
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(1, "Auto Choices: %s", autoChoices);
    }   //doAutoChoicesMenus

}   //class FtcAuto
