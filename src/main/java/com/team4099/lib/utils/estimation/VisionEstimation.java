package com.team4099.lib.utils.estimation;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.team4099.lib.vision.MathUtils;
import com.team4099.lib.vision.RotTrlTransform3d;
import com.team4099.lib.utils.util.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionEstimation {

    public static final TargetModel kTagModel = TargetModel.ofPlanarRect(
        Units.inchesToMeters(6),
        Units.inchesToMeters(6)
    );

    /**
     * Uses trigonometry and the known height of the AprilTags on the field to estimate the
     * translations of the detected AprilTags relative to the robot. The returned poses of
     * the AprilTags will have zero rotation.
     * 
     * @param detectedTags The detected tag in the camera-- specifically, its fiducial ID,
     *     yaw, and pitch. Pitch is assumed to be positive up.
     * @return The estimated AprilTags with ID and pose(without rotation) corresponding to the 2d
     *     detected tags from the camera, relative to the robot.
     */
    public static List<AprilTag> estimateTagsTrig(
            Transform3d robotToCamera, List<PhotonTrackedTarget> detectedTags, AprilTagFieldLayout knownTags) {
        if(detectedTags == null || knownTags == null || robotToCamera == null) return List.of();
        var tags = detectedTags.stream().map(t -> {
            var knownTagPose = knownTags.getTagPose(t.getFiducialId());
            if(knownTagPose.isEmpty()) return null;
            var camToTagTrl = PhotonUtils.estimateCamToTargetTrl(
                robotToCamera,
                knownTagPose.get().getZ(),
                Rotation2d.fromDegrees(t.getYaw()), Rotation2d.fromDegrees(-t.getPitch())
            );

            return new AprilTag(
                t.getFiducialId(),
                new Pose3d(camToTagTrl, new Rotation3d()).relativeTo(
                    new Pose3d().plus(robotToCamera.inverse())
                )
            );  
        }).filter(t -> t != null).collect(Collectors.toList());
        
        return tags;
    }

    /**
     * Performs solvePNP using 3d-2d point correspondences to estimate the field-to-camera transformation.
     * If only one tag is visible, the result may have an alternate solution.
     * 
     * <p><b>Note:</b> The returned transformation is from the field origin to the camera pose!
     * 
     * @param prop The camera properties
     * @param corners The visible tag corners in the 2d image
     * @param knownTags The known tag field poses corresponding to the visible tag IDs
     * @return The transformation that maps the field origin to the camera pose
     */
    public static PNPResults estimateCamPosePNP(
            CameraProperties prop, List<TargetCorner> corners, List<AprilTag> knownTags) {
        if(knownTags == null || corners == null ||
                corners.size() != knownTags.size()*4 || knownTags.size() == 0) {
            return new PNPResults();
        }
        // single-tag pnp
        if(corners.size() == 4) {
            var camToTag = OpenCVHelp.solveTagPNP(prop, kTagModel.cornerOffsets, corners);
            var bestPose = knownTags.get(0).pose.transformBy(camToTag.best.inverse());
            var altPose = new Pose3d();
            if(camToTag.ambiguity != 0) altPose = knownTags.get(0).pose.transformBy(camToTag.alt.inverse());
            var o = new Pose3d();
            return new PNPResults(
                new Transform3d(o, bestPose),
                new Transform3d(o, altPose),
                camToTag.ambiguity, camToTag.bestReprojErr, camToTag.altReprojErr
            );
        }
        // multi-tag pnp
        else {
            var objectTrls = new ArrayList<Translation3d>();
            for(var tag : knownTags) objectTrls.addAll(kTagModel.getFieldCorners(tag.pose));
            var camToOrigin = OpenCVHelp.solveTagsPNP(prop, objectTrls, corners);
            // var camToOrigin = OpenCVHelp.solveTagsPNPRansac(prop, objectTrls, corners);
            return new PNPResults(
                camToOrigin.best.inverse(),
                camToOrigin.alt.inverse(),
                camToOrigin.ambiguity, camToOrigin.bestReprojErr, camToOrigin.altReprojErr);
        }
    }

    /**
     * Finds the rigid transform that best maps the list of measuredTags onto the list of
     * knownTags.
     * 
     * <p>The lists must have the same size, and the tags at the same index of both lists
     * must correspond to the same fiducial ID. If useCorners is true, the 4 corners of
     * the tag are used in addition to the center point. Based on this, there must be more
     * than 2 points, and the points must not all be collinear.
     * 
     * <p>If a solution cannot be found, the returned transformation is empty and the RMSE is -1.
     * 
     * @param measuredTags The "measured" set of tags to map onto the "known" tags
     * @param knownTags The "known" set of tags to be mapped onto
     * @param useCorners If the corners of the tags should be used in addition to the center point.
     *     This multiplies the total points by 5, but may be less accurate.
     * @return The estimated transform(rotation-translation) and associated RMSE. If the
     *     RMSE of this is -1, no solution could be found.
     * @see #estimateRigidTransform(List, List)
     */
    public static SVDResults estimateTransformLS(
            List<AprilTag> measuredTags, List<AprilTag> knownTags, boolean useCorners) {
        if(measuredTags == null || knownTags == null ||
                measuredTags.size() < 1 || measuredTags.size() != knownTags.size()) {
            return new SVDResults();
        }
        var measuredTrls = new ArrayList<Translation3d>();
        var knownTrls = new ArrayList<Translation3d>();
        for(int i = 0; i < measuredTags.size(); i++) {
            var mTag = measuredTags.get(i);
            var kTag = knownTags.get(i);
            if(useCorners) {
                measuredTrls.addAll(kTagModel.getFieldCorners(mTag.pose));
                knownTrls.addAll(kTagModel.getFieldCorners(kTag.pose));
            }
            var mTrl = mTag.pose.getTranslation();
            var up = new Translation3d(0, 0, 0.5);
            measuredTrls.add(mTrl);
            measuredTrls.add(mTrl.plus(up));
            var kTrl = kTag.pose.getTranslation();
            knownTrls.add(kTrl);
            knownTrls.add(kTrl.plus(up));
        }
        
        // return new SVDResults(OpenCVHelp.estimateRigidTransform(measuredTrls, knownTrls), 0);
        return estimateRigidTransform(measuredTrls, knownTrls);
    }
    /**
     * Finds the rigid transform that best maps the list of translations A onto the list of
     * translations B.
     * 
     * <p>The lists must have the same size, and the translations at the same
     * index of both lists must correspond to the same point. The lists must have more than
     * 2 points, and the points must not all be collinear.
     * 
     * <p>If a solution cannot be found, the returned transformation is empty and the RMSE is -1.
     * 
     * @param trlsA The "measured" set of translations to map onto B
     * @param trlsB The "known" set of translations to be mapped onto
     * @return The estimated transform(rotation-translation) and associated RMSE. If the
     *     RMSE of this is -1, no solution could be found.
     */
    public static SVDResults estimateRigidTransform(
            List<Translation3d> trlsA, List<Translation3d> trlsB) {
        if(trlsA == null || trlsB == null ||
                trlsA.size() != trlsB.size() ||
                trlsA.size() < 3
                ) {
            return new SVDResults();
        }
        // convert lists to matrices
        var matA = MathUtils.translationsToMatrix(trlsA);
        // System.out.println("matA: "+matA);
        var matB = MathUtils.translationsToMatrix(trlsB);
        // System.out.println("matB: "+matB);

        // check if points are collinear
        // System.out.println("matA collinear: "+MathUtils.isCollinear(matA));
        // System.out.println("matB collinear: "+MathUtils.isCollinear(matB));
        if(MathUtils.isCollinear(matA) || MathUtils.isCollinear(matB)) return new SVDResults();

        // find the centers of our lists of translations
        var centroidTrlA = MathUtils.calcAvg(
            trlsA.stream()
                .map(t -> new Pose3d(t, new Rotation3d()))
                .collect(Collectors.toList()).toArray(new Pose3d[0])
        ).getTranslation();
        var centroidMatA = MathUtils.translationsToMatrix(List.of(centroidTrlA));
        var centroidTrlB = MathUtils.calcAvg(
            trlsB.stream()
                .map(t -> new Pose3d(t, new Rotation3d()))
                .collect(Collectors.toList()).toArray(new Pose3d[0])
        ).getTranslation();
        var centroidMatB = MathUtils.translationsToMatrix(List.of(centroidTrlB));
        
        // re-center our set of translations with their centroids as the origin
        var relMatA = matA.copy();
        MathUtils.columnsMinusTrl(relMatA, centroidTrlA);
        var relMatB = matB.copy();
        MathUtils.columnsMinusTrl(relMatB, centroidTrlB);

        // covariance matrix of the re-centered translations
        var matH = relMatA.times(relMatB.transpose());
        // Singular Value Decomposition
        var svd = matH.getStorage().svd();
        // find the rotation matrix R from SVD
        var matR = new Matrix<N3, N3>(svd.getV().mult(svd.getU().transpose()));
        // check for "reflection" case
        if(matR.det() < 0) {
            // mult 3rd column of V by -1
            var newVCol = svd.getV().extractVector(false, 2).scale(-1);
            svd.getV().setColumn(2, 0, newVCol.getDDRM().getData());
            matR = new Matrix<N3, N3>(svd.getV().mult(svd.getU().transpose()));
        }
        // matR = MathUtils.orthogonalizeRotationMatrix(matR);
        matR = matR.div(matR.det());
        // find the translation matrix after rotating
        var matTrl = centroidMatB.minus(matR.times(centroidMatA));
        double[] trlData = matTrl.getData();
        
        //System.out.println("matR det: "+matR.det());
        var rot = new Rotation3d(matR);
        // Our estimated transform must first apply rotation, and then translation
        var trf = new RotTrlTransform3d(
            rot,
            new Translation3d(trlData[0], trlData[1], trlData[2])
        );
        // measure the error of this estimated transform
        double rmse = 0;
        var estTrlsA = trf.applyTrls(trlsA);
        for(int i = 0; i < estTrlsA.size(); i++) {
            rmse += trlsB.get(i).minus(estTrlsA.get(i)).getNorm();
        }
        rmse /= estTrlsA.size();

        return new SVDResults(trf, rmse);
    }

    /**
     * The best estimated transformation to the target, and possibly an alternate transformation
     * depending on the solvePNP method. If an alternate solution is present, the ambiguity value
     * represents the ratio of reprojection error in the best solution to the alternate (best / alternate).
     */
    public static class PNPResults {
        public final Transform3d best;
        public final double bestReprojErr;
        
        /**
         * Alternate, ambiguous solution from solvepnp. This may be empty
         * if no alternate solution is found.
         */
        public final Transform3d alt;
        /** If no alternate solution is found, this is 0 */
        public final double altReprojErr;
        
        /** If no alternate solution is found, this is 0 */
        public final double ambiguity;

        public PNPResults() {
            this(new Transform3d(), new Transform3d(), 0, 0, 0);
        }
        public PNPResults(
                Transform3d best, Transform3d alt,
                double ambiguity, double bestReprojErr, double altReprojErr) {
            this.best = best;
            this.alt = alt;
            this.ambiguity = ambiguity;
            this.bestReprojErr = bestReprojErr;
            this.altReprojErr = altReprojErr;
        }
    }
    /**
     * The best estimated transformation (Rotation-translation composition) that maps a set of
     * translations onto another with point correspondences, and its RMSE.
     */
    public static class SVDResults {
        public final RotTrlTransform3d trf;
        /** If the result is invalid, this value is -1 */
        public final double rmse;

        public SVDResults() {
            this(new RotTrlTransform3d(), -1);
        }
        public SVDResults(RotTrlTransform3d trf, double rmse) {
            this.trf = trf;
            this.rmse = rmse;
        }
    }
}
