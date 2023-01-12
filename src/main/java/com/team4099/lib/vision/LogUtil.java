package com.team4099.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.List;

public final class LogUtil {
    public static double[] toPoseArray2d(Pose2d pose) {
        return new double[]{
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        };
    }
    public static double[] toPoseArray2d(List<Pose2d> poses) {
        double[] result = new double[3 * poses.size()];
        for(int i = 0; i < poses.size(); i++) {
            double[] vals = toPoseArray2d(poses.get(i));
            for(int j = 0; j < vals.length; j++) {
                result[3*i + j] = vals[j];
            }
        }
        return result;
    }
    public static double[] toPoseArray3d(Pose3d pose) {
        var rot = pose.getRotation();
        return new double[] {
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            rot.getQuaternion().getW(),
            rot.getQuaternion().getX(),
            rot.getQuaternion().getY(),
            rot.getQuaternion().getZ()
        };
    }
    public static double[] toPoseArray3d(List<Pose3d> poses) {
        double[] result = new double[7 * poses.size()];
        for(int i = 0; i < poses.size(); i++) {
            double[] vals = toPoseArray3d(poses.get(i));
            for(int j = 0; j < vals.length; j++) {
                result[7*i + j] = vals[j];
            }
        }
        return result;
    }
}
