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

import teamcode.subsystems.BaseDrive;
import teamcode.subsystems.Shooter;
import teamcode.subsystems.Spindexer;
import teamcode.vision.Vision;
import trclib.drivebase.TrcDriveBase;
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
    public static class SubsystemDrivebase
    {
        public static TrcDriveBase.TuneParams robotDrive = BaseDrive.DecodeInfo.tuneParams;
    }   //class SubsystemDrivebase

    @Config
    public static class SubsystemVision
    {
        public static TrcOpenCvColorBlobPipeline.PipelineParams artifactVision = Vision.artifactPipelineParams;
        public static TrcOpenCvColorBlobPipeline.PipelineParams classifierVision = Vision.classifierPipelineParams;
    }   //class SubsystemVision

    @Config
    public static class SubsystemShooter
    {
        public static TrcMotor.TuneParams shootMotor1Pid = Shooter.shootMotor1PidParams;
        public static double shootMotor1Velocity = 5000.0;    // in RPM
        public static TrcMotor.TuneParams shootMotor2Pid = Shooter.shootMotor1PidParams;
        public static TrcMotor.TuneParams panMotorPid = Shooter.panMotorPidParams;
        public static TrcMotor.TuneParams tiltMotorPid = Shooter.tiltMotorPidParams;
        public static TrcServo.TuneParams launcherPos = Shooter.launcherParams;
    }   //class SubsystemShooter

    @Config
    public static class SubsystemSpindexer
    {
        public static TrcMotor.TuneParams motorPid = Spindexer.motorPidParams;
        public static TrcTriggerThresholdRange.TriggerParams entryTrigger = Spindexer.entryTriggerParams;
        public static TrcTriggerThresholdRange.TriggerParams exitTrigger = Spindexer.exitTriggerParams;
    }   //class SubsystemSpindexer

    private static Double nextDashboardUpdateTime =  null;

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

        if (slowLoop)
        {
            nextDashboardUpdateTime = currTime + RobotParams.Robot.DASHBOARD_UPDATE_INTERVAL;
        }

        if (RobotParams.Preferences.showVision && robot.vision != null)
        {
            lineNum = robot.vision.updateStatus(lineNum, slowLoop);
        }

        if (RobotParams.Preferences.showSubsystems)
        {
            lineNum = TrcSubsystem.updateStatusAll(lineNum, slowLoop);
        }

        return lineNum;
    }   //updateDashboard

}   //class Dashboard
