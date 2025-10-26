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

package teamcode.autotasks;

import androidx.annotation.NonNull;

import ftclib.vision.FtcLimelightVision;
import teamcode.Dashboard;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.indicators.LEDIndicator;
import teamcode.subsystems.Shooter;
import teamcode.vision.Vision;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.subsystem.TrcShootParamTable;
import trclib.timer.TrcTimer;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoShoot extends TrcAutoTask<TaskAutoShoot.State>
{
    private static final String moduleName = TaskAutoShoot.class.getSimpleName();

    public enum State
    {
        START,
        DO_VISION,
        AIM,
        SHOOT,
        SHOOT_NEXT,
        DONE
    }   //enum State

    public static class TaskParams
    {
        public FtcAuto.Alliance alliance = FtcAuto.Alliance.BLUE_ALLIANCE;
        public boolean useAprilTagVision = true;
        public boolean useClassifierVision = false;
        public int numArtifactsToShoot = 1;

        public TaskParams setAlliance(FtcAuto.Alliance alliance)
        {
            this.alliance = alliance;
            return this;
        }   //setAlliance

        public TaskParams setVision(boolean useAprilTagVision, boolean useClassifierVision)
        {
            this.useAprilTagVision = useAprilTagVision;
            this.useClassifierVision = useClassifierVision;
            return this;
        }   //setVision

        public TaskParams setNumArtifactsToShoot(int count)
        {
            this.numArtifactsToShoot = count;
            return this;
        }   //setNumArtifactsToShoot

        @NonNull
        public String toString()
        {
            return "(alliance=" + alliance +
                   ",useAprilTagVision=" + useAprilTagVision +
                   ",useClassifierVision=" + useClassifierVision +
                   ",numArtifactsToShoot=" + numArtifactsToShoot + ")";
        }   //toString
    }   //class TaskParams

    public static final TaskParams autoShootParams = new TaskParams();
    private final Robot robot;
    private final TrcEvent event;
    private final TrcEvent spindexerEvent;

    private Double visionExpiredTime = null;
    TrcPose2D targetPose = null;
    TrcShootParamTable.Params shootParams = null;
    Vision.ArtifactType[] motifSequence = null;
    int motifIndex = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoShoot(Robot robot)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.robot = robot;
        this.event = new TrcEvent(moduleName + ".event");
        this.spindexerEvent = new TrcEvent(moduleName + ".spindexerEvent");
    }   //TaskAutoShoot

    /**
     * This method starts the auto-assist operation.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     * @param alliance specifies the alliance color, can be null if useClassifierVision is false.
     * @param useAprilTagVision specifies true to use AprilTag Vision, false otherwise.
     * @param useClassifierVision specifies true to use Classifier Vision, false otherwise.
     * @param numArtifactsToShoot specifies the number of artifacts to shoot.
     */
    public void autoShoot(
        String owner, TrcEvent completionEvent, FtcAuto.Alliance alliance, boolean useAprilTagVision,
        boolean useClassifierVision, int numArtifactsToShoot)
    {
        autoShootParams
            .setAlliance(alliance)
            .setVision(useAprilTagVision, useClassifierVision)
            .setNumArtifactsToShoot(numArtifactsToShoot);
        tracer.traceInfo(
            moduleName,
            "autoShoot(owner=" + owner + ", event=" + completionEvent + ", taskParams=" + autoShootParams + ")");
        startAutoTask(owner, State.START, autoShootParams, completionEvent);
    }   //autoShoot

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called to acquire ownership of all subsystems involved in the auto task operation. This is
     * typically called before starting an auto task operation.
     *
     * @param owner specifies the owner to acquire the subsystem ownerships.
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership(String owner)
    {
        return owner == null ||
               robot.shooter.acquireExclusiveAccess(owner) &&
               robot.spindexer.acquireExclusiveAccess(owner);
    }   //acquireSubsystemsOwnership

    /**
     * This method is called to release ownership of all subsystems involved in the auto task operation. This is
     * typically called if the auto task operation is completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void releaseSubsystemsOwnership(String owner)
    {
        if (owner != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                moduleName,
                "Releasing subsystem ownership on behalf of " + owner +
                "\n\tshooter=" + ownershipMgr.getOwner(robot.shooter) +
                "\n\tspindexer=" + ownershipMgr.getOwner(robot.spindexer));
            robot.shooter.releaseExclusiveAccess(owner);
            robot.spindexer.releaseExclusiveAccess(owner);
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called to stop all the subsystems. This is typically called if the auto task operation is
     * completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void stopSubsystems(String owner)
    {
        tracer.traceInfo(moduleName, "Stopping subsystems.");
        robot.shooter.cancel(owner);
        robot.spindexer.cancel(owner);
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false if running the fast loop on the main robot thread.
     */
    @Override
    protected void runTaskState(
        String owner, Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode,
        boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                targetPose = null;
                if (!taskParams.useAprilTagVision)
                {
                    tracer.traceInfo(moduleName, "***** Not using AprilTag Vision, aim at AprilTag using odometry.");
                    int aprilTagIndex = taskParams.alliance == FtcAuto.Alliance.BLUE_ALLIANCE? 0: 4;
                    TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    targetPose = RobotParams.Game.APRILTAG_POSES[aprilTagIndex].subtractRelativePose(robotPose);
                    tracer.traceInfo(
                        moduleName, "robotPose=%s, aprilTagPose=%s, targetPose=%s",
                        robotPose, RobotParams.Game.APRILTAG_POSES[aprilTagIndex], targetPose);
                    sm.setState(State.AIM);
                }
                else if (robot.vision != null && robot.vision.isLimelightVisionEnabled() &&
                         (!taskParams.useClassifierVision || robot.vision.isClassifierVisionEnabled()))
                {
                    tracer.traceInfo(moduleName, "***** Using AprilTag Vision.");
                    visionExpiredTime = null;
                    sm.setState(State.DO_VISION);
                }
                else
                {
                    tracer.traceWarn(moduleName, "***** Using Vision but Vision is not enabled.");
                    sm.setState(State.DONE);
                }
                break;

            case DO_VISION:
                // Use vision to determine the appropriate AprilTag location.
                if (targetPose == null)
                {
                    TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> aprilTagInfo =
                        robot.vision.limelightVision.getBestDetectedTargetInfo(
                            FtcLimelightVision.ResultType.Fiducial,
                            taskParams.alliance == null? null:
                            taskParams.alliance == FtcAuto.Alliance.BLUE_ALLIANCE? new int[] {20}: new int[] {24},
                            null,
                            null);
                    if (aprilTagInfo != null)
                    {
                        int aprilTagId = (int) aprilTagInfo.detectedObj.objId;
                        targetPose = aprilTagInfo.detectedObj.getObjectPose();
                        tracer.traceInfo(
                            moduleName,
                            "***** Vision found AprilTag " + aprilTagId + ": aprilTagPose=" + targetPose);
                        if (robot.ledIndicator != null)
                        {
                            robot.ledIndicator.setStatusVisionPatternsOff();
                            robot.ledIndicator.setStatusPattern(
                                aprilTagId == 20? LEDIndicator.BLUE_APRILTAG: LEDIndicator.RED_APRILTAG, true);
                        }
                        // Determine shooter speed, pan and tilt angle according to detected AprilTag pose.
                        // Use vision distance to look up shooter parameters.
                        shootParams = Shooter.Params.shootParamTable.get(aprilTagInfo.detectedObj.targetDepth, false);
                        tracer.traceInfo(
                            moduleName,
                            "***** ShootParams: distance=" + aprilTagInfo.detectedObj.targetDepth +
                            ", params=" + shootParams);
                    }
                }

                if (taskParams.useClassifierVision && robot.obeliskMotif != null && motifSequence == null)
                {
                    motifSequence = robot.vision.getMotifSequence(taskParams.alliance, robot.obeliskMotif);
                }

                if (targetPose != null &&
                    (!taskParams.useClassifierVision || robot.obeliskMotif == null || motifSequence != null))
                {
                    motifIndex = 0;
                    sm.setState(State.AIM);
                }
                else if (visionExpiredTime == null)
                {
                    // Can't find AprilTag, set a timeout and try again.
                    visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out, moving on.
                    tracer.traceInfo(moduleName, "***** Vision timed out.");
                    if (robot.ledIndicator != null)
                    {
                        // Indicate we timed out and found nothing.
                        robot.ledIndicator.setStatusVisionPatternsOff();
                        robot.ledIndicator.setStatusPattern(LEDIndicator.NOT_FOUND, true);
                    }
                    sm.setState(State.DONE);
                }
                break;

            case AIM:
                // Determine what artifact type to shoot and check if Spindexer has it.
                Vision.ArtifactType motifArtifactType = motifSequence != null? motifSequence[motifIndex++]: null;
                Vision.ArtifactType artifactType;
                if (motifArtifactType != null)
                {
                    artifactType = motifArtifactType;
                    if (robot.spindexerSubsystem.getNumArtifacts(artifactType) == 0)
                    {
                        tracer.traceInfo(moduleName, "***** %s motif artifact not in Spindexer.", artifactType);
                        // Pick another artifact type to shoot. Ideally not the same type as the next motif sequence.
                        artifactType = motifIndex == motifSequence.length ? Vision.ArtifactType.Any :
                            motifSequence[motifIndex] == Vision.ArtifactType.Green ?
                                Vision.ArtifactType.Purple : Vision.ArtifactType.Green;
                        if (robot.spindexerSubsystem.getNumArtifacts(artifactType) == 0 &&
                            robot.spindexerSubsystem.getNumArtifacts(Vision.ArtifactType.Any) == 0)
                        {
                            tracer.traceInfo(moduleName, "***** Spindexer is empty, quit.");
                            artifactType = null;
                        }
                    }
                }
                else
                {
                    artifactType = Vision.ArtifactType.Any;
                }

                if (artifactType != null)
                {
                    tracer.traceInfo(
                        moduleName, "***** Shooting %s artifact (MotifArtifact=%s).", artifactType, motifArtifactType);
                    // Move Spindexer to the slot that has the correct artifact type.
                    spindexerEvent.clear();
                    sm.addEvent(spindexerEvent);
                    robot.spindexerSubsystem.moveToExitSlotWithArtifact(owner, artifactType, spindexerEvent);
                    // Spin the shooter flywheel up to speed and the turret pointing to the target.
                    event.clear();
                    sm.addEvent(event);
                    if (shootParams != null)
                    {
                        tracer.traceInfo(
                            moduleName, "***** Aiming: vel=%f RPM, tilt=%f, pan=%f, event=%s",
                            shootParams.shooter1Velocity, shootParams.tiltAngle, targetPose.angle, event);
                        robot.shooter.aimShooter(
                            owner, shootParams.shooter1Velocity/60.0, shootParams.shooter2Velocity/60.0,
                            shootParams.tiltAngle, targetPose.angle, event, 0.0, null, 0.0);
                    }
                    else
                    {
                        // We did not use vision, just shoot assuming operator manually aimed.
                        double shooterVel = Dashboard.Subsystem_Shooter.shootMotor1Velocity;
                        tracer.traceInfo(
                            moduleName, "***** ManualShoot: vel=%f RPM, event=%s", shooterVel, event);
                        // ShooterVel is in RPM, aimShooter wants RPS.
                        robot.shooter.aimShooter(
                            owner, shooterVel / 60.0, 0.0, null, null, event, 0.0, null, 0.0);
                    }
                    // Wait for Spindexer and Shooter to be ready before shooting.
                    sm.waitForEvents(State.SHOOT, false, true);
                }
                else
                {
                    // Nothing to shoot because Spindexer is empty, we are done.
                    sm.setState(State.DONE);
                }
                break;

            case SHOOT:
                tracer.traceInfo(moduleName, "***** Feed artifact to shooter.");
                robot.shooterSubsystem.shoot(owner, event);
                sm.waitForSingleEvent(event, State.SHOOT_NEXT);
                break;

            case SHOOT_NEXT:
                taskParams.numArtifactsToShoot--;
                sm.setState(taskParams.numArtifactsToShoot > 0? State.AIM: State.DONE);
                break;

            case DONE:
            default:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoShoot
