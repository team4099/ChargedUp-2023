package com.team4099.lib.utils.estimation

import com.team4099.lib.utils.estimation.TargetModel.Companion.ofPlanarRect
import com.team4099.lib.utils.util.PhotonUtils.estimateCamToTargetTrl
import com.team4099.lib.utils.estimation.OpenCVHelp.solveTagPNP
import com.team4099.lib.utils.estimation.OpenCVHelp.solveTagsPNP
import com.team4099.lib.vision.MathUtils.translationsToMatrix
import com.team4099.lib.vision.MathUtils.isCollinear
import com.team4099.lib.vision.MathUtils.calcAvg
import com.team4099.lib.vision.MathUtils.columnsMinusTrl
import com.team4099.lib.utils.estimation.TargetModel
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.Optional
import edu.wpi.first.math.geometry.Rotation2d
import java.util.stream.Collectors
import com.team4099.lib.utils.estimation.CameraProperties
import org.photonvision.targeting.TargetCorner
import com.team4099.lib.utils.estimation.VisionEstimation.PNPResults
import com.team4099.lib.utils.estimation.OpenCVHelp
import com.team4099.lib.utils.estimation.VisionEstimation
import com.team4099.lib.utils.estimation.VisionEstimation.SVDResults
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.Num
import com.team4099.lib.vision.MathUtils
import org.ejml.simple.SimpleSVD
import org.ejml.simple.SimpleMatrix
import com.team4099.lib.vision.RotTrlTransform3d
import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import java.util.ArrayList
import kotlin.jvm.JvmOverloads
import kotlin.streams.toList

object VisionEstimation {
  val kTagModel = ofPlanarRect(
    Units.inchesToMeters(6.0),
    Units.inchesToMeters(6.0)
  )

  /**
   * Uses trigonometry and the known height of the AprilTags on the field to estimate the
   * translations of the detected AprilTags relative to the robot. The returned poses of
   * the AprilTags will have zero rotation.
   *
   * @param detectedTags The detected tag in the camera-- specifically, its fiducial ID,
   * yaw, and pitch. Pitch is assumed to be positive up.
   * @return The estimated AprilTags with ID and pose(without rotation) corresponding to the 2d
   * detected tags from the camera, relative to the robot.
   */
  fun estimateTagsTrig(
    robotToCamera: Transform3d?,
    detectedTags: List<PhotonTrackedTarget>?,
    knownTags: AprilTagFieldLayout?
  ): List<AprilTag> {
    return if (detectedTags == null || knownTags == null || robotToCamera == null) listOf<AprilTag>() else detectedTags.stream()
      .map { t: PhotonTrackedTarget ->
        val knownTagPose = knownTags.getTagPose(t.fiducialId)
        if (knownTagPose.isEmpty) return@map null
        val camToTagTrl = estimateCamToTargetTrl(
          robotToCamera,
          knownTagPose.get().z,
          Rotation2d.fromDegrees(t.yaw), Rotation2d.fromDegrees(-t.pitch)
        )
        AprilTag(
          t.fiducialId,
          Pose3d(camToTagTrl, Rotation3d()).relativeTo(
            Pose3d().plus(robotToCamera.inverse())
          )
        )
      }.filter { t: AprilTag? -> t != null }.toList().map { it!! }
  }

  /**
   * Performs solvePNP using 3d-2d point correspondences to estimate the field-to-camera transformation.
   * If only one tag is visible, the result may have an alternate solution.
   *
   *
   * **Note:** The returned transformation is from the field origin to the camera pose!
   *
   * @param prop The camera properties
   * @param corners The visible tag corners in the 2d image
   * @param knownTags The known tag field poses corresponding to the visible tag IDs
   * @return The transformation that maps the field origin to the camera pose
   */
  fun estimateCamPosePNP(
    prop: CameraProperties?, corners: List<TargetCorner?>?, knownTags: List<AprilTag>?
  ): PNPResults {
    if (knownTags == null || corners == null || corners.size != knownTags.size * 4 || knownTags.size == 0) {
      return PNPResults()
    }
    // single-tag pnp
    return if (corners.size == 4) {
      val camToTag = solveTagPNP(prop!!, kTagModel.cornerOffsets,
        corners.map{ it!! }
      )
      val bestPose = knownTags[0].pose.transformBy(camToTag.best.inverse())
      var altPose: Pose3d? = Pose3d()
      if (camToTag.ambiguity != 0.0) altPose = knownTags[0].pose.transformBy(camToTag.alt.inverse())
      val o = Pose3d()
      PNPResults(
        Transform3d(o, bestPose),
        Transform3d(o, altPose),
        camToTag.ambiguity, camToTag.bestReprojErr, camToTag.altReprojErr
      )
    } else {
      val objectTrls = ArrayList<Translation3d>()
      for (tag in knownTags) objectTrls.addAll(kTagModel.getFieldCorners(tag.pose))
      val camToOrigin = solveTagsPNP(prop!!, objectTrls, corners.map{ it!!})
      // var camToOrigin = OpenCVHelp.solveTagsPNPRansac(prop, objectTrls, corners);
      PNPResults(
        camToOrigin.best.inverse(),
        camToOrigin.alt.inverse(),
        camToOrigin.ambiguity, camToOrigin.bestReprojErr, camToOrigin.altReprojErr
      )
    }
  }

  /**
   * Finds the rigid transform that best maps the list of measuredTags onto the list of
   * knownTags.
   *
   *
   * The lists must have the same size, and the tags at the same index of both lists
   * must correspond to the same fiducial ID. If useCorners is true, the 4 corners of
   * the tag are used in addition to the center point. Based on this, there must be more
   * than 2 points, and the points must not all be collinear.
   *
   *
   * If a solution cannot be found, the returned transformation is empty and the RMSE is -1.
   *
   * @param measuredTags The "measured" set of tags to map onto the "known" tags
   * @param knownTags The "known" set of tags to be mapped onto
   * @param useCorners If the corners of the tags should be used in addition to the center point.
   * This multiplies the total points by 5, but may be less accurate.
   * @return The estimated transform(rotation-translation) and associated RMSE. If the
   * RMSE of this is -1, no solution could be found.
   * @see .estimateRigidTransform
   */
//    fun estimateTransformLS(
//        measuredTags: List<AprilTag>?, knownTags: List<AprilTag>?, useCorners: Boolean
//    ): SVDResults {
//        if (measuredTags == null || knownTags == null || measuredTags.size < 1 || measuredTags.size != knownTags.size) {
//            return SVDResults()
//        }
//        val measuredTrls = ArrayList<Translation3d?>()
//        val knownTrls = ArrayList<Translation3d>()
//        for (i in measuredTags.indices) {
//            val mTag = measuredTags[i]
//            val kTag = knownTags[i]
//            if (useCorners) {
//                measuredTrls.addAll(kTagModel.getFieldCorners(mTag.pose))
//                knownTrls.addAll(kTagModel.getFieldCorners(kTag.pose))
//            }
//            val mTrl = mTag.pose.translation
//            val up = Translation3d(0.0, 0.0, 0.5)
//            measuredTrls.add(mTrl)
//            measuredTrls.add(mTrl.plus(up))
//            val kTrl = kTag.pose.translation
//            knownTrls.add(kTrl)
//            knownTrls.add(kTrl.plus(up))
//        }
//
//        // return new SVDResults(OpenCVHelp.estimateRigidTransform(measuredTrls, knownTrls), 0);
//        return estimateRigidTransform(measuredTrls, knownTrls)
//    }

  /**
   * Finds the rigid transform that best maps the list of translations A onto the list of
   * translations B.
   *
   *
   * The lists must have the same size, and the translations at the same
   * index of both lists must correspond to the same point. The lists must have more than
   * 2 points, and the points must not all be collinear.
   *
   *
   * If a solution cannot be found, the returned transformation is empty and the RMSE is -1.
   *
   * @param trlsA The "measured" set of translations to map onto B
   * @param trlsB The "known" set of translations to be mapped onto
   * @return The estimated transform(rotation-translation) and associated RMSE. If the
   * RMSE of this is -1, no solution could be found.
   */
//    fun estimateRigidTransform(
//        trlsA: List<Translation3d?>?, trlsB: List<Translation3d>?
//    ): SVDResults {
//        if (trlsA == null || trlsB == null || trlsA.size != trlsB.size || trlsA.size < 3) {
//            return SVDResults()
//        }
//        // convert lists to matrices
//        val matA: Matrix<N3, Num?> = translationsToMatrix(trlsA)
//        // System.out.println("matA: "+matA);
//        val matB: Matrix<N3, Num?> = translationsToMatrix(trlsB)
//        // System.out.println("matB: "+matB);
//
//        // check if points are collinear
//        // System.out.println("matA collinear: "+MathUtils.isCollinear(matA));
//        // System.out.println("matB collinear: "+MathUtils.isCollinear(matB));
//        if (isCollinear(matA) || isCollinear(matB)) return SVDResults()
//
//        // find the centers of our lists of translations
//        val centroidTrlA: List<Translation3d> = calcAvg(
//            *trlsA.stream()
//                .map { t: Translation3d? -> Pose3d(t, Rotation3d()) }
//        ).translation
//        val centroidMatA: Matrix<N3?, Num> = translationsToMatrix(centroidTrlA)
//        val centroidTrlB = calcAvg(
//            *trlsB.stream()
//                .map { t: Translation3d? -> Pose3d(t, Rotation3d()) }
//                .collect(Collectors.toList()).toTypedArray()
//        ).translation
//        val centroidMatB: Matrix<N3?, Num> = translationsToMatrix(listOf(centroidTrlB))
//
//        // re-center our set of translations with their centroids as the origin
//        val relMatA = matA.copy()
//        columnsMinusTrl(relMatA, centroidTrlA)
//        val relMatB = matB.copy()
//        columnsMinusTrl(relMatB, centroidTrlB)
//
//        // covariance matrix of the re-centered translations
//        val matH = relMatA.times(relMatB.transpose())
//        // Singular Value Decomposition
//        val svd = matH.storage.svd()
//        // find the rotation matrix R from SVD
//        var matR = Matrix<N3?, N3?>(svd.v.mult(svd.u.transpose()))
//        // check for "reflection" case
//        if (matR.det() < 0) {
//            // mult 3rd column of V by -1
//            val newVCol = svd.v.extractVector(false, 2).scale(-1.0)
//            svd.v.setColumn(2, 0, *newVCol.ddrm.getData())
//            matR = Matrix(svd.v.mult(svd.u.transpose()))
//        }
//        // matR = MathUtils.orthogonalizeRotationMatrix(matR);
//        matR = matR.div(matR.det())
//        // find the translation matrix after rotating
//        val matTrl = centroidMatB.minus(matR.times(centroidMatA))
//        val trlData = matTrl.data
//
//        //System.out.println("matR det: "+matR.det());
//        val rot = Rotation3d(matR)
//        // Our estimated transform must first apply rotation, and then translation
//        val trf = RotTrlTransform3d(
//            rot,
//            Translation3d(trlData[0], trlData[1], trlData[2])
//        )
//        // measure the error of this estimated transform
//        var rmse = 0.0
//        val estTrlsA = trf.applyTrls(trlsA)
//        for (i in estTrlsA.indices) {
//            rmse += trlsB[i].minus(estTrlsA[i]).norm
//        }
//        rmse /= estTrlsA.size.toDouble()
//        return SVDResults(trf, rmse)
//    }

  /**
   * The best estimated transformation to the target, and possibly an alternate transformation
   * depending on the solvePNP method. If an alternate solution is present, the ambiguity value
   * represents the ratio of reprojection error in the best solution to the alternate (best / alternate).
   */
  class PNPResults @JvmOverloads constructor(
    val best: Transform3d = Transform3d(),
    /**
     * Alternate, ambiguous solution from solvepnp. This may be empty
     * if no alternate solution is found.
     */
    val alt: Transform3d = Transform3d(),
    /** If no alternate solution is found, this is 0  */
    val ambiguity: Double = 0.0, val bestReprojErr: Double = 0.0,
    /** If no alternate solution is found, this is 0  */
    val altReprojErr: Double = 0.0
  )

  /**
   * The best estimated transformation (Rotation-translation composition) that maps a set of
   * translations onto another with point correspondences, and its RMSE.
   */
  class SVDResults @JvmOverloads constructor(
    val trf: RotTrlTransform3d = RotTrlTransform3d(),
    /** If the result is invalid, this value is -1  */
    val rmse: Double = -1.0
  )
}
