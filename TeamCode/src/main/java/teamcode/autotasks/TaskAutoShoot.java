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

import java.util.Arrays;

import ftclib.vision.FtcLimelightVision;
import teamcode.Dashboard;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.indicators.LEDIndicator;
import teamcode.subsystems.Shooter;
import teamcode.vision.Vision;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcDbgTrace;
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
        SETUP_VISION,
        DO_VISION,
        AIM,
        SHOOT,
        SHOOT_NEXT,
        NEXT_EXIT_SLOT,
        DONE
    }   //enum State

    public static class TaskParams
    {
        public FtcAuto.Alliance alliance = FtcAuto.Alliance.BLUE_ALLIANCE;
        public boolean inAuto = false;
        public boolean useAprilTagVision = true;
        public boolean doMotif = false;
        public boolean useClassifierVision = false;
        public boolean relocalize = false;
        public int numArtifactsToShoot = 3;
        public boolean moveToNextExitSlot = true;

        public TaskParams setAlliance(FtcAuto.Alliance alliance)
        {
            this.alliance = alliance;
            return this;
        }   //setAlliance

        public TaskParams setInAuto(boolean inAuto)
        {
            this.inAuto = inAuto;
            return this;
        }   //setInAuto

        public TaskParams setVision(boolean useAprilTagVision, boolean doMotif, boolean useClassifierVision)
        {
            this.useAprilTagVision = useAprilTagVision;
            this.doMotif = doMotif;
            this.useClassifierVision = useClassifierVision;
            return this;
        }   //setVision

        public TaskParams setRelocalizeEnabled(boolean enabled)
        {
            this.relocalize = enabled;
            return this;
        }   //setRelocalizeEnabled

        public TaskParams setNumArtifactsToShoot(int count, boolean moveToNextExitSlot)
        {
            this.numArtifactsToShoot = count;
            this.moveToNextExitSlot = moveToNextExitSlot;
            return this;
        }   //setNumArtifactsToShoot

        @NonNull
        public String toString()
        {
            return "(alliance=" + alliance +
                   ",inAuto=" + inAuto +
                   ",useAprilTagVision=" + useAprilTagVision +
                   ",doMotif=" + doMotif +
                   ",useClassifierVision=" + useClassifierVision +
                   ",relocalize=" + relocalize +
                   ",numArtifactsToShoot=" + numArtifactsToShoot +
                   ",moveToNextExitSlot=" + moveToNextExitSlot + ")";
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
    int[] autoTrackedAprilTagIds = null;
    FtcAuto.Alliance autoTrackedAlliance = null;


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
     * @param inAuto specifies true if running in Autonomous mode, false otherwise.
     * @param useAprilTagVision specifies true to use AprilTag Vision, false otherwise.
     * @param doMotif specifies true to shoot motif sequence.
     * @param useClassifierVision specifies true to use Classifier Vision, false otherwise, applicable only if doMotif
     *        is true.
     * @param relocalize specifies true to enable relocalization, false otherwise.
     * @param numArtifactsToShoot specifies the number of artifacts to shoot.
     * @param moveToNextExitSlot specifies true to move to next Exit slot after shooting is done.
     */
    public void autoShoot(
        String owner, TrcEvent completionEvent, FtcAuto.Alliance alliance, boolean inAuto, boolean useAprilTagVision,
        boolean doMotif, boolean useClassifierVision, boolean relocalize, int numArtifactsToShoot,
        boolean moveToNextExitSlot)
    {
        autoShootParams
            .setAlliance(alliance)
            .setInAuto(inAuto)
            .setVision(useAprilTagVision, doMotif, useClassifierVision)
            .setRelocalizeEnabled(relocalize)
            .setNumArtifactsToShoot(numArtifactsToShoot, moveToNextExitSlot);
        tracer.traceInfo(
            moduleName,
            "autoShoot(owner=" + owner + ", event=" + completionEvent + ", taskParams=" + autoShootParams + ")");
        autoTrackedAprilTagIds = robot.shooterSubsystem.getTrackedAprilTagIds();
        autoTrackedAlliance = robot.shooterSubsystem.getTrackedAlliance();
        // Turn off AprilTag tracking if it was ON.
        if (autoTrackedAprilTagIds != null || autoTrackedAlliance != null)
        {
            tracer.traceInfo(
                moduleName,
                "Goal Tracking is ON (Ids=%s, alliance=%s), turning it OFF.",
                Arrays.toString(autoTrackedAprilTagIds), autoTrackedAlliance);
            robot.shooterSubsystem.disableGoalTracking(null);
        }
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

        if (autoTrackedAprilTagIds != null)
        {
            tracer.traceInfo(
                moduleName,
                "AprilTag tracking was ON (Id=%s), turning it back ON.", Arrays.toString(autoTrackedAprilTagIds));
            robot.shooterSubsystem.enableGoalTracking(null, autoTrackedAprilTagIds);
            autoTrackedAprilTagIds = null;
        }

        if (autoTrackedAlliance != null)
        {
            tracer.traceInfo(
                moduleName,
                "Alliance tracking was ON (alliance=%s), turning it back ON.", autoTrackedAlliance);
            robot.shooterSubsystem.enableGoalTracking(null, autoTrackedAlliance);
            autoTrackedAlliance = null;
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
        if (robot.ledIndicator != null)
        {
            robot.ledIndicator.setStatusPatternState(LEDIndicator.RED_APRILTAG, false);
            robot.ledIndicator.setStatusPatternState(LEDIndicator.BLUE_APRILTAG, false);
            robot.ledIndicator.setStatusPatternState(LEDIndicator.SEARCHING_RED_APRILTAG, false);
            robot.ledIndicator.setStatusPatternState(LEDIndicator.SEARCHING_BLUE_APRILTAG, false);
        }
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
        Vision.ArtifactType artifactType = null;

        switch (state)
        {
            case START:
                motifIndex = 0;
//                if (robot.ledIndicator != null)
//                {
//                    robot.ledIndicator.setStatusPatternOn(LEDIndicator.SHOOT_IN_PROGRESS, false);
//                }
                //
                // Intentionally falling through.
                //
            case SETUP_VISION:
                targetPose = null;
                if (!taskParams.useAprilTagVision)
                {
                    tracer.traceInfo(moduleName, "***** Not using AprilTag Vision, aim at AprilTag using odometry.");
                    TrcPose2D robotPose = robot.robotBase.driveBase.getFieldPosition();
                    int aprilTagIndex = taskParams.alliance == FtcAuto.Alliance.BLUE_ALLIANCE? 0: 4;
                    targetPose = RobotParams.Game.APRILTAG_POSES[aprilTagIndex].relativeTo(robotPose);
                    tracer.traceInfo(
                        moduleName, "robotPose=%s, aprilTagPose=%s, targetPose=%s",
                        robotPose, RobotParams.Game.APRILTAG_POSES[aprilTagIndex], targetPose);
                    // Determine shooter speed, pan and tilt angle according to the AprilTag pose.
                    // Use vision distance to look up shooter parameters.
                    double shootDistance = TrcUtil.magnitude(targetPose.x, targetPose.y);
                    shootParams = Shooter.Params.shootParamTable.get(shootDistance, false);
                    tracer.traceInfo(
                        moduleName, "***** ShootParams: distance=" + shootDistance + ", params=" + shootParams);
                    sm.setState(State.AIM);
                }
                else if (robot.vision != null && robot.vision.isLimelightVisionEnabled() &&
                         (!taskParams.useClassifierVision || robot.vision.isClassifierVisionEnabled()))
                {
                    tracer.traceInfo(moduleName, "***** Using AprilTag Vision.");
                    visionExpiredTime = null;
                    if (robot.ledIndicator != null)
                    {
                        // Clear other LED states so this will show.
                        robot.ledIndicator.setStatusPatternOn(
                            taskParams.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                                LEDIndicator.SEARCHING_BLUE_APRILTAG: LEDIndicator.SEARCHING_RED_APRILTAG,
                            true);
                    }
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
                    if (taskParams.alliance == null)
                    {
                        TrcDbgTrace.globalTraceErr(moduleName, "Alliance is NULL!!!!!");
                        TrcDbgTrace.printThreadStack();
                    }
                    TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> aprilTagInfo =
                        robot.vision.limelightVision.getBestDetectedTargetInfo(
                            FtcLimelightVision.ResultType.Fiducial,
                            taskParams.alliance == null? RobotParams.Game.anyGoalAprilTags:
                            taskParams.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                                RobotParams.Game.blueGoalAprilTag: RobotParams.Game.redGoalAprilTag,
                            robot.robotBase.driveBase.getHeading(), null);
                    if (aprilTagInfo != null)
                    {
                        int aprilTagId = (int) aprilTagInfo.detectedObj.objId;

                        targetPose = aprilTagInfo.detectedObj.getObjectPose();
                        tracer.traceInfo(
                            moduleName, "***** Vision found AprilTag " + aprilTagId + ": aprilTagPose=" + targetPose);

                        if (taskParams.relocalize && aprilTagInfo.detectedObj.robotPose != null)
                        {
                            TrcPose2D originalRobotPose = robot.robotBase.driveBase.getFieldPosition();
                            TrcPose2D robotFieldPose =
                                robot.shooterSubsystem.adjustRobotFieldPosition(aprilTagInfo.detectedObj.robotPose);
                            robot.robotBase.driveBase.setFieldPosition(robotFieldPose, false);
                            tracer.traceInfo(
                                moduleName, "***** Relocalize: " + originalRobotPose + " -> " + robotFieldPose);
                        }

                        if (robot.ledIndicator != null)
                        {
                            // This is assuming vision is set to look for 20 or 24 only.
                            robot.ledIndicator.setStatusPatternOn(
                                aprilTagId == RobotParams.Game.blueGoalAprilTag[0]?
                                    LEDIndicator.BLUE_APRILTAG: LEDIndicator.RED_APRILTAG, true);
                        }

                        TrcPose2D adjustedTargetPose = targetPose.addRelativePose(
                            taskParams.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                                RobotParams.Game.BLUE_APRILTAG_TO_CORNER: RobotParams.Game.RED_APRILTAG_TO_CORNER);
                        tracer.traceInfo(moduleName, "adjustedTargetPose=%s", adjustedTargetPose);
                        targetPose.angle = adjustedTargetPose.angle;
                        // Adjusted target angle to absolute pan angle since Limelight is mounted on the turret.
                        targetPose.angle += robot.shooter.getPanAngle();
                        // Determine shooter speed, pan and tilt angle according to detected AprilTag pose.
                        // Use vision distance to look up shooter parameters.
                        shootParams = Shooter.Params.shootParamTable.get(aprilTagInfo.detectedObj.targetDepth, false);
                        tracer.traceInfo(
                            moduleName,
                            "***** ShootParams: distance=" + aprilTagInfo.detectedObj.targetDepth +
                            ", params=" + shootParams);
                    }
                }

                if (taskParams.doMotif && robot.obeliskMotif != null && motifSequence == null)
                {
                    // If we saw the obelisk and haven't determined the motif sequence, determine it now.
                    // Do this only once since DO_VISION gets called repeatedly.
                    motifSequence = robot.vision.getMotifSequence(
                        taskParams.alliance, robot.obeliskMotif, taskParams.useClassifierVision);
                    tracer.traceInfo(
                        moduleName, "***** MotifSequence=%s (index=%d)",
                        Arrays.toString(motifSequence), motifIndex);
                }

                if (targetPose != null && (robot.obeliskMotif == null || motifSequence != null))
                {
                    // Vision found the target AprilTag and either we don't see obelisk or we have determined
                    // the motif sequence, go ahead and shoot. If vision saw the obelisk but it may take time
                    // to determine the motif sequence if we are using Classifier Vision, then we wait for it.
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
                        robot.ledIndicator.setStatusPatternOn(LEDIndicator.NOT_FOUND, true);
                    }
                    sm.setState(State.DONE);
                }
                break;

            case AIM:
                // Determine what artifact type to shoot and check if Spindexer has it.
                sm.clearEventList();
                Vision.ArtifactType motifArtifactType = motifSequence != null? motifSequence[motifIndex++]: null;
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
                        if (artifactType == motifArtifactType)
                        {
                            // Don't pick the same color that we already know we don't have. In that case, just
                            // shoot any color.
                            artifactType = Vision.ArtifactType.Any;
                        }
                        if (robot.spindexerSubsystem.getNumArtifacts(artifactType) == 0 &&
                            robot.spindexerSubsystem.getNumArtifacts(Vision.ArtifactType.Any) == 0)
                        {
                            tracer.traceInfo(
                                moduleName, "***** Spindexer is empty, shoot any just in case sensor is wrong.");
                            artifactType = Vision.ArtifactType.Any;
                        }
                    }
                }
                else
                {
                    artifactType = Vision.ArtifactType.Any;
                }

                Vision.ArtifactType exitArtifactType = robot.spindexerSubsystem.getExitSlotArtifactType();
                tracer.traceInfo(
                    moduleName, "***** Shooting %s artifact (MotifArtifact=%s, exitArtifact=%s).",
                    artifactType, motifArtifactType, exitArtifactType);
                // Move Spindexer to the slot that has the correct artifact type.
                spindexerEvent.clear();
                sm.addEvent(spindexerEvent);
                if (!robot.spindexerSubsystem.moveToExitSlotWithArtifact(owner, artifactType, spindexerEvent) &&
                    exitArtifactType == null)
                {
                    // Spindexer did not move, probably because it is empty or it doesn't know what artifacts it has.
                    // Exit slot is not aligned, move to the next exit slot up so the launcher won't get stuck.
                    // Clear spindexer event again since moveToExitSlotWithArtifact would have signaled it.
                    spindexerEvent.clear();
                    robot.spindexerSubsystem.exitSlotUp(owner, spindexerEvent);
                }

                // Spin the shooter flywheel up to speed and the turret pointing to the target.
                event.clear();
                sm.addEvent(event);
                if (shootParams != null)
                {
                    tracer.traceInfo(
                        moduleName, "***** Aiming: vel=%f RPM, tilt=%f, pan=%f, event=%s",
                        shootParams.shooter1Velocity, shootParams.tiltAngle, targetPose.angle, event);
                    robot.shooter.setTiltAngle(shootParams.tiltAngle);
                    robot.shooter.aimShooter(
                        owner, shootParams.shooter1Velocity/60.0, shootParams.shooter2Velocity/60.0,
                        null, targetPose.angle, event, 0.0, null, 0.0);
                }
                else
                {
                    // We did not use vision, just shoot assuming operator manually aimed.
                    double shooterVel = Dashboard.Subsystem_Shooter.shootMotor1Velocity;
                    tracer.traceInfo(
                        moduleName, "***** ManualShoot: vel=%f RPM, event=%s", shooterVel, event);
                    // ShooterVel is in RPM, aimShooter wants RPS.
                    robot.shooter.aimShooter(
                        owner, shooterVel/60.0, 0.0, null, null, event, 0.0, null, 0.0);
                }
                // Wait for Spindexer and Shooter to be ready before shooting.
                sm.waitForEvents(State.SHOOT, false, true);
                break;

            case SHOOT:
                tracer.traceInfo(moduleName, "***** Feed artifact to shooter.");
                robot.shooterSubsystem.shoot(owner, event);
                sm.waitForSingleEvent(event, State.SHOOT_NEXT);
                break;

            case SHOOT_NEXT:
                taskParams.numArtifactsToShoot--;
                sm.setState(taskParams.numArtifactsToShoot > 0? State.SETUP_VISION: State.NEXT_EXIT_SLOT);
                break;

            case NEXT_EXIT_SLOT:
                if (taskParams.moveToNextExitSlot)
                {
                    tracer.traceInfo(moduleName, "***** Move to the next exit slot.");
                    robot.spindexerSubsystem.moveToExitSlotWithArtifact(owner, Vision.ArtifactType.Any, event);
                    sm.waitForSingleEvent(event, State.DONE);
                }
                else
                {
                    sm.setState(State.DONE);
                }
                break;

            case DONE:
            default:
                if (robot.ledIndicator != null)
                {
                    // We are done, turn off all LED patterns in case the "SEARCHING" and found AprilTag pattern is
                    // still ON.
                    robot.ledIndicator.resetStatusPatterns();
                }
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoShoot
