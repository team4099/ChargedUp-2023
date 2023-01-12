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

import java.util.List;

import com.team4099.lib.utils.estimation.TargetModel;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

public class SimVisionTarget {

    private Pose3d pose;
    private TargetModel model;
    
    public final int id;

    /**
     * Describes a vision target located somewhere on the field that your SimVisionSystem can detect.
     *
     * @param pose Pose3d of the tag in field-relative coordinates
     * @param model TargetModel which describes the shape of the target
     */
    public SimVisionTarget(Pose3d pose, TargetModel model) {
        this.pose = pose;
        this.model = model;
        this.id = -1;
    }
    /**
     * Describes an orientation-agnostic vision target (like a sphere) located somewhere on the field
     * that your SimVisionSystem can detect.
     *
     * @param pose Pose3d of the target in field-relative coordinates
     * @param diamMeters Size(diameter) of the outer bounding box of the target in meters.
     */
    public SimVisionTarget(Pose3d pose, double diamMeters) {
        this(pose, diamMeters, diamMeters, -1);
    }
    /**
     * Describes a planar vision target located somewhere on the field that your SimVisionSystem can detect.
     *
     * @param pose Pose3d of the target in field-relative coordinates
     * @param widthMeters Width of the outer bounding box of the target in meters.
     * @param heightMeters Height of the outer bounding box of the target in meters.
     */
    public SimVisionTarget(Pose3d pose, double widthMeters, double heightMeters) {
        this(pose, widthMeters, heightMeters, -1);
    }
    /**
     * Describes a fiducial tag located somewhere on the field that your SimVisionSystem can detect.
     *
     * @param pose Pose3d of the tag in field-relative coordinates
     * @param lengthMeters Width/height of the outer bounding box of the tag(black square) in meters.
     * @param id The ID of this fiducial tag
     */
    public SimVisionTarget(Pose3d pose, double lengthMeters, int id) {
        this(pose, lengthMeters, lengthMeters, id);
    }
    /**
     * Describes a fiducial tag located somewhere on the field that your SimVisionSystem can detect.
     *
     * @param pose Pose3d of the tag in field-relative coordinates
     * @param widthMeters Width of the outer bounding box of the tag(black square) in meters.
     * @param heightMeters Height of the outer bounding box of the tag(black square) in meters.
     * @param id The ID of this fiducial tag
     */
    public SimVisionTarget(Pose3d pose, double widthMeters, double heightMeters, int id) {
        this.pose = pose;
        this.model = TargetModel.ofPlanarRect(widthMeters, heightMeters);
        this.id = id;
    }

    public void setPose(Pose3d pose) {
        this.pose = pose;
    }
    public void setModel(TargetModel model) {
        this.model = model;
    }

    public Pose3d getPose() {
        return pose;
    }
    public TargetModel getModel(){
        return model;
    }
    public List<Translation3d> getPlanarFieldCorners() {
        return model.getFieldCorners(pose);
    }

    @Override
    public boolean equals(Object obj) {
        if(this == obj) return true;
        if(obj instanceof SimVisionTarget) {
            var o = (SimVisionTarget)obj;
            return pose.equals(o.pose) &&
                    model.equals(o.model);
        }
        return false;
    }
}
