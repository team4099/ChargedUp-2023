/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
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

package com.team4099.lib.utils.sim;

import com.team4099.lib.vision.LogUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class SimVisionSystem {

    private final String tableName;

    private final Map<String, PhotonCameraSim> camSimMap = new HashMap<>();
    private final Map<PhotonCameraSim, Transform3d> camTrfMap = new HashMap<>();

    private final TimeInterpolatableBuffer<Pose3d> robotPoseBuffer = TimeInterpolatableBuffer.createBuffer(1.5);

    private Map<String, Set<SimVisionTarget>> targetSets = new HashMap<>();

    private final Field2d dbgField;

    /**
     * Create a simulated vision system involving a camera(s) and coprocessor(s) mounted on a mobile robot
     * running PhotonVision, detecting one or more targets scattered around the field.
     */
    public SimVisionSystem(String visionSystemName) {
        dbgField = new Field2d();
        tableName = "vision-"+visionSystemName;
        SmartDashboard.putData(tableName + "/Sim Field", dbgField);
    }

    /**
     * Get one of the simulated cameras.
     */
    public PhotonCameraSim getCameraSim(String name) {
        return camSimMap.get(name);
    }
    /**
     * Get all of the simulated cameras.
     */
    public Collection<PhotonCameraSim> getCameraSims() {
        return camSimMap.values();
    }
    /**
     * Get a simulated camera's position relative to the robot.
     */
    public Transform3d getRobotToCamera(String name) {
        return camTrfMap.get(camSimMap.get(name));
    }
    /**
     * Get a simulated camera's position relative to the robot.
     */
    public Transform3d getRobotToCamera(PhotonCameraSim cameraSim) {
        return camTrfMap.get(cameraSim);
    }
    /**
     * Adds a simulated camera to this vision system with a specified robot-to-camera transformation.
     * The vision targets registered with this vision system simulation will be observed by the simulated
     * {@link PhotonCamera}.
     * 
     * @param cameraSim The camera simulation
     * @param robotToCamera The transform from the robot pose to the camera pose
     */
    public void addCamera(PhotonCameraSim cameraSim, Transform3d robotToCamera) {
        var existing = camSimMap.putIfAbsent(cameraSim.getCamera().name, cameraSim);
        if(existing == null) {
            SmartDashboard.putData(
                tableName+"/"+cameraSim.getCamera().name+"/Sim Corners",
                cameraSim.getDebugCorners()
            );
        }
        camTrfMap.put(cameraSim, robotToCamera);
    }
    /**
     * Remove all simulated cameras from this vision system.
     */
    public void clearCameras() {
        camSimMap.clear();
        camTrfMap.clear();
    }
    /**
     * Remove a simulated camera from this vision system.
     * 
     * @return If the camera was present and removed
     */
    public boolean removeCamera(PhotonCameraSim cameraSim) {
        boolean success = camSimMap.remove(cameraSim.getCamera().name) != null;
        camTrfMap.remove(cameraSim);
        return success;
    }
    /**
     * Adjust a camera's position relative to the robot. Use this if your camera is on a gimbal or
     * turret or some other mobile platform.
     *
     * @param cameraSim The simulated camera to change the relative position of
     * @param robotToCamera New transform from the robot to the camera
     */
    public void adjustCamera(PhotonCameraSim cameraSim, Transform3d robotToCamera) {
        camTrfMap.put(cameraSim, robotToCamera);
    }
    
    public Set<SimVisionTarget> getVisionTargets() {
        var all = new HashSet<SimVisionTarget>();
        for(var entry : targetSets.entrySet()) {
            all.addAll(entry.getValue());
        }
        return all;
    }
    public Set<SimVisionTarget> getVisionTargets(String type) {
        return targetSets.get(type);
    }
    /**
     * Adds targets on the field which your vision system is designed to detect. The
     * {@link PhotonCamera}s simulated from this system will report the location of the camera
     * relative to the subset of these targets which are visible from the given camera position.
     * 
     * <p>By default these are added under the type "targets".
     *
     * @param targets Targets to add to the simulated field
     */
    public void addVisionTargets(SimVisionTarget... targets) {
        addVisionTargets("targets", targets);
    }
    /**
     * Adds targets on the field which your vision system is designed to detect. The
     * {@link PhotonCamera}s simulated from this system will report the location of the camera
     * relative to the subset of these targets which are visible from the given camera position.
     * 
     * <p>The AprilTags from this layout will be added as vision targets under the type "apriltags".
     * The poses added preserve the tag layout's current alliance origin.
     *
     * @param tagLayout The field tag layout to get Apriltag poses and IDs from 
     */
    public void addVisionTargets(AprilTagFieldLayout tagLayout) {
        for(AprilTag tag : tagLayout.getTags()){ 
            addVisionTargets("apriltags",
            new SimVisionTarget(
                tagLayout.getTagPose(tag.ID).get(), // preserve alliance rotation
                Units.inchesToMeters(6),
                tag.ID
            ));
        }
    }
    /**
     * Adds targets on the field which your vision system is designed to detect. The
     * {@link PhotonCamera}s simulated from this system will report the location of the camera
     * relative to the subset of these targets which are visible from the given camera position. 
     * 
     * @param type Type of target (e.g. "cargo").
     * @param targets Targets to add to the simulated field
     */
    public void addVisionTargets(String type, SimVisionTarget... targets) {
        if(targetSets.get(type) == null) targetSets.put(type, new HashSet<>());
        for(var tgt : targets) {
            targetSets.get(type).add(tgt);
        }
    }
    public void clearVisionTargets() {
        targetSets.clear();
    }
    public Set<SimVisionTarget> removeVisionTargets(String type) {
        return targetSets.remove(type);
    }
    public Set<SimVisionTarget> removeVisionTargets(SimVisionTarget... targets) {
        var removeList = List.of(targets);
        var removedSet = new HashSet<SimVisionTarget>();
        for(var entry : targetSets.entrySet()) {
            entry.getValue().removeIf(t -> {
                if(removeList.contains(t)) {
                    removedSet.add(t);
                    return true;
                }
                else return false;
            });
        }
        return removedSet;
    }

    public void clearRobotPoses() {
        robotPoseBuffer.clear();
        camSimMap.forEach((name, cam) -> cam.clearCameraPoses());
    }
    /**
     * Get the robot pose in meters saved by the vision system secondsAgo.
     * @param secondsAgo Seconds to look into the past
     */
    public Pose3d getRobotPose(double secondsAgo) {
        return robotPoseBuffer.getSample(Timer.getFPGATimestamp() - secondsAgo).orElse(new Pose3d());
    }

    public Field2d getDebugField() {
        return dbgField;
    }

    /**
     * Periodic update. Ensure this is called repeatedly-- camera performance is used to
     * automatically determine if a new frame should be submitted.
     * @param robotPoseMeters The current robot pose in meters
     */
    public void update(Pose2d robotPoseMeters) {
        update(new Pose3d(robotPoseMeters));
    }
    /**
     * Periodic update. Ensure this is called repeatedly-- camera performance is used to
     * automatically determine if a new frame should be submitted.
     * @param robotPoseMeters The current robot pose in meters
     */
    public void update(Pose3d robotPoseMeters) {
        var targetTypes = targetSets.entrySet();
        // update vision targets on field
        targetTypes.forEach(entry -> dbgField.getObject(entry.getKey()).setPoses(
            entry.getValue().stream().map(t -> t.getPose().toPose2d()).collect(Collectors.toList())
        ));

        if(robotPoseMeters == null) return;

        // save "real" robot poses over time
        double now = Timer.getFPGATimestamp();
        robotPoseBuffer.addSample(now, robotPoseMeters);
        dbgField.setRobotPose(robotPoseMeters.toPose2d());
        SmartDashboard.putNumberArray(
            tableName+"/RobotPose3d",
            LogUtil.toPoseArray3d(robotPoseMeters)
        );

        var allTargets = new ArrayList<SimVisionTarget>();
        targetTypes.forEach((entry) -> allTargets.addAll(entry.getValue()));
        var visibleTargets = new ArrayList<Pose3d>();
        var cameraPose2ds = new ArrayList<Pose2d>();
        // process each camera
        for(var camSim : camSimMap.values()) {            
            // check if this camera is ready to process and get latency
            var optionalLatency = camSim.getShouldProcess();
            if(optionalLatency.isEmpty()) continue;
            double latencyMillis = optionalLatency.get();
            // save "real" camera poses over time
            camSim.updateCameraPose(robotPoseMeters.transformBy(getRobotToCamera(camSim)));
            // display camera latency milliseconds ago
            Pose3d lateCameraPose = camSim.getCameraPose(latencyMillis / 1000.0);
            cameraPose2ds.add(lateCameraPose.toPose2d());
            SmartDashboard.putNumberArray(
                tableName+"/"+camSim.getCamera().name+"/LatePose3d",
                LogUtil.toPoseArray3d(lateCameraPose)
            );

            // update camera's visible targets
            var camResult = camSim.process(latencyMillis, lateCameraPose, allTargets);
            // display results
            for(var target : camResult.getTargets()) {
                visibleTargets.add(
                    lateCameraPose.transformBy(target.getBestCameraToTarget())
                );
            }
        }
        if(visibleTargets.size() != 0) {
            SmartDashboard.putNumberArray(tableName+"/EstTargetPoses3d", LogUtil.toPoseArray3d(visibleTargets));
            dbgField.getObject("visibleTargets").setPoses(
                visibleTargets.stream().map(p -> p.toPose2d()).collect(Collectors.toList())
            );
        }
        if(cameraPose2ds.size() != 0) dbgField.getObject("cameras").setPoses(cameraPose2ds);
    }
}
