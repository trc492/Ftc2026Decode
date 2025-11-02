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

import teamcode.autotasks.TaskAutoShoot;
import teamcode.subsystems.BaseDrive;
import teamcode.subsystems.Shooter;
import teamcode.subsystems.Spindexer;
import teamcode.vision.Vision;
import trclib.drivebase.TrcDriveBase;
import trclib.drivebase.TrcSwerveDriveBase;
import trclib.driverio.TrcGameController;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.sensor.TrcTriggerThresholdRange;
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
        public static TrcGameController.DriveMode driveMode = TrcGameController.DriveMode.ArcadeMode;
        public static TrcDriveBase.DriveOrientation driveOrientation  = TrcDriveBase.DriveOrientation.ROBOT;
        public static double driveSlowScale = 0.3;
        public static double driveNormalScale = 1.0;
        public static double turnSlowScale = 0.3;
        public static double turnNormalScale = 0.6;
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
        public static double shootMotor1Velocity = 2000.0;  // in RPM
        public static double tiltAngle = 26.0;              // in degrees
        public static TaskAutoShoot.TaskParams autoShootParams = TaskAutoShoot.autoShootParams;
        public static TrcMotor.PidParams shootMotor1Pid = Shooter.shootMotor1PidParams;
        public static TrcMotor.PidParams shootMotor2Pid = Shooter.shootMotor1PidParams;
        public static TrcMotor.PidParams panMotorPid = Shooter.panMotorPidParams;
        public static TrcMotor.PidParams tiltMotorPid = Shooter.tiltMotorPidParams;
        public static TrcServo.TuneParams launcherPos = Shooter.launcherParams;
    }   //class Subsystem_Shooter

    @Config
    public static class Subsystem_Spindexer
    {
        public static TrcMotor.PidParams motorPid = Spindexer.motorPidParams;
        public static TrcTriggerThresholdRange.TriggerParams entryTrigger = Spindexer.entryTriggerParams;
        public static TrcTriggerThresholdRange.TriggerParams exitTrigger = Spindexer.exitTriggerParams;
    }   //class Subsystem_Spindexer

}   //class Dashboard
