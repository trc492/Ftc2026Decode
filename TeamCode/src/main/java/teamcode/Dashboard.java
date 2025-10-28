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

import com.acmerobotics.dashboard.config.Config;

import ftclib.driverio.FtcDashboard;
import teamcode.autotasks.TaskAutoShoot;
import teamcode.subsystems.BaseDrive;
import teamcode.subsystems.Shooter;
import teamcode.subsystems.Spindexer;
import teamcode.vision.Vision;
import trclib.drivebase.TrcDriveBase;
import trclib.drivebase.TrcSwerveDriveBase;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.sensor.TrcTriggerThresholdRange;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;
import trclib.vision.TrcOpenCvColorBlobPipeline;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Dashboard
{
    @Config
    public static class DashboardParams
    {
        public static boolean updateDashboardEnabled = RobotParams.Preferences.updateDashboard;
        public static String tuneSubsystemName = "";
        public static FtcAuto.AutoChoices autoChoices = FtcAuto.autoChoices;
    }   //class DashboardParams

    @Config
    public static class Subsystem_Drivebase
    {
        public static TrcDriveBase.BaseParams driveBaseParams = BaseDrive.DecodeInfo.baseParams;
        public static TrcSwerveDriveBase.SwerveParams swerveDriveParams = BaseDrive.DecodeInfo.swerveParams;
        public static double steerPowerCompConstant = 0.0;
    }   //class Subsystem_Drivebase

    @Config
    public static class Subsystem_Vision
    {
        public static int trackedAprilTagId = 20;
        public static TrcOpenCvColorBlobPipeline.PipelineParams artifactVision = Vision.artifactPipelineParams;
        public static TrcOpenCvColorBlobPipeline.PipelineParams classifierVision = Vision.classifierPipelineParams;
    }   //class Subsystem_Vision

    @Config
    public static class Subsystem_Shooter
    {
        public static boolean tuneShootingTable = false;
        public static double shootMotor1Velocity = 5000.0;  // in RPM
        public static double tiltAngle = 26.0;              // in degrees
        public static TaskAutoShoot.TaskParams autoShootParams = TaskAutoShoot.autoShootParams;
        public static TrcMotor.TuneParams shootMotor1Pid = Shooter.shootMotor1PidParams;
        public static TrcMotor.TuneParams shootMotor2Pid = Shooter.shootMotor1PidParams;
        public static TrcMotor.TuneParams panMotorPid = Shooter.panMotorPidParams;
        public static TrcMotor.TuneParams tiltMotorPid = Shooter.tiltMotorPidParams;
        public static TrcServo.TuneParams launcherPos = Shooter.launcherParams;
    }   //class Subsystem_Shooter

    @Config
    public static class Subsystem_Spindexer
    {
        public static TrcMotor.TuneParams motorPid = Spindexer.motorPidParams;
        public static double[] entryTriggerPoints = Spindexer.entryTriggerThresholdPoints;
        public static TrcTriggerThresholdRange.TriggerParams exitTrigger = Spindexer.exitTriggerParams;
    }   //class Subsystem_Spindexer

    private static Double nextDashboardUpdateTime =  null;

    /**
     * This method enables/disables Dashboard Update.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public static void setUpdateDashboardEnabled(boolean enabled)
    {
        DashboardParams.updateDashboardEnabled = enabled;
        if (!enabled)
        {
            FtcDashboard.getInstance().clearDisplay();
        }
    }   //setUpdateDashboardEnabled

    /**
     * This method checks if Dashboard Update is enabled.
     *
     * @return true if update is enabled, false if disabled.
     */
    public static boolean isDashboardUpdateEnabled()
    {
        return DashboardParams.updateDashboardEnabled;
    }   //isDashboardUpdateEnabled

    /**
     * This method is called periodically to update various hardware/subsystem status of the robot to the dashboard
     * and trace log. In order to lower the potential impact these updates, this method will only update the dashboard
     * at DASHBOARD_UPDATE_INTERVAL.
     *
     * @param robot specifies the robot object.
     * @param lineNum specifies the first Dashboard line for printing status.
     * @return next available dashboard line.
     */
    public static int updateDashboard(Robot robot, int lineNum)
    {
        double currTime = TrcTimer.getCurrentTime();
        boolean slowLoop = nextDashboardUpdateTime == null || currTime >= nextDashboardUpdateTime;

        if (DashboardParams.updateDashboardEnabled)
        {
            if (slowLoop)
            {
                nextDashboardUpdateTime = currTime + RobotParams.Robot.DASHBOARD_UPDATE_INTERVAL;
            }

            if (RobotParams.Preferences.showSubsystems)
            {
                lineNum = TrcSubsystem.updateStatusAll(lineNum, slowLoop);
            }

            if (RobotParams.Preferences.showVisionStatus && robot.vision != null)
            {
                lineNum = robot.vision.updateStatus(lineNum, slowLoop);
            }
        }

        return lineNum;
    }   //updateDashboard

}   //class Dashboard
