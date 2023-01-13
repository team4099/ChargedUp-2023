package com.team4099.lib.utils.estimation

import com.team4099.lib.vision.RotTrlTransform3d.Companion.makeRelativeTo
import org.ejml.simple.SimpleMatrix
import org.opencv.core.MatOfDouble
import org.opencv.core.Mat
import org.opencv.core.CvType
import edu.wpi.first.math.Num
import org.opencv.core.MatOfPoint3f
import org.opencv.core.Point3
import com.team4099.lib.utils.estimation.OpenCVHelp
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.Nat
import org.photonvision.targeting.TargetCorner
import org.opencv.core.MatOfPoint2f
import java.util.function.IntFunction
import com.team4099.lib.utils.estimation.CameraProperties
import com.team4099.lib.vision.RotTrlTransform3d
import org.opencv.calib3d.Calib3d
import java.util.Arrays
import org.opencv.core.RotatedRect
import org.opencv.imgproc.Imgproc
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfInt
import com.team4099.lib.utils.estimation.VisionEstimation.PNPResults
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.cscore.CameraServerCvJNI
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.CoordinateSystem
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import org.opencv.core.Point
import java.lang.Exception
import java.lang.RuntimeException
import java.util.ArrayList

object OpenCVHelp {
  fun matrixToMat(matrix: SimpleMatrix): MatOfDouble {
    val mat = Mat(matrix.numRows(), matrix.numCols(), CvType.CV_64F)
    mat.put(0, 0, *matrix.ddrm.getData())
    val wrappedMat = MatOfDouble()
    mat.convertTo(wrappedMat, CvType.CV_64F)
    mat.release()
    return wrappedMat
  }

  fun matToMatrix(mat: Mat): Matrix<Num, Num> {
    val data = DoubleArray(mat.total().toInt() * mat.channels())
    val doubleMat = Mat(mat.rows(), mat.cols(), CvType.CV_64F)
    mat.convertTo(doubleMat, CvType.CV_64F)
    doubleMat[0, 0, data]
    return Matrix(SimpleMatrix(mat.rows(), mat.cols(), true, data))
  }

  /**
   * Creates a new [MatOfPoint3f] with these 3d translations.
   * The opencv tvec is a vector with three elements representing {x, y, z} in the
   * EDN coordinate system.
   */
  fun translationToTvec(vararg translations: Translation3d?): MatOfPoint3f {
    val points = arrayOfNulls<Point3>(translations.size)
    for (i in 0 until translations.size) {
      val trl = CoordinateSystem.convert(
        translations[i],
        CoordinateSystem.NWU(),
        CoordinateSystem.EDN()
      )
      points[i] = Point3(trl.x, trl.y, trl.z)
    }
    return MatOfPoint3f(*points)
  }

  /**
   * Returns a new 3d translation from this [Mat].
   * The opencv tvec is a vector with three elements representing {x, y, z} in the
   * EDN coordinate system.
   */
  fun tvecToTranslation(tvecInput: Mat): Translation3d {
    val data = FloatArray(3)
    val wrapped = Mat(tvecInput.rows(), tvecInput.cols(), CvType.CV_32F)
    tvecInput.convertTo(wrapped, CvType.CV_32F)
    wrapped[0, 0, data]
    wrapped.release()
    return CoordinateSystem.convert(
      Translation3d(
        data[0].toDouble(), data[1].toDouble(), data[2].toDouble()
      ),
      CoordinateSystem.EDN(),
      CoordinateSystem.NWU()
    )
  }

  /**
   * Creates a new [MatOfPoint3f] with this 3d rotation.
   * The opencv rvec Mat is a vector with three elements representing the axis
   * scaled by the angle in the EDN coordinate system.
   * (angle = norm, and axis = rvec / norm)
   */
  fun rotationToRvec(rotation: Rotation3d): MatOfPoint3f {
    var rotation = rotation
    rotation = rotationNWUtoEDN(rotation)
    return MatOfPoint3f(Point3(rotation.quaternion.toRotationVector().data))
  }

  /**
   * Returns a 3d rotation from this [Mat].
   * The opencv rvec Mat is a vector with three elements representing the axis
   * scaled by the angle in the EDN coordinate system.
   * (angle = norm, and axis = rvec / norm)
   */
  fun rvecToRotation(rvecInput: Mat): Rotation3d {
    val data = FloatArray(3)
    val wrapped = Mat(rvecInput.rows(), rvecInput.cols(), CvType.CV_32F)
    rvecInput.convertTo(wrapped, CvType.CV_32F)
    wrapped[0, 0, data]
    wrapped.release()
    val axis = Vector(Nat.N3())
    axis[0, 0] = data[0].toDouble()
    axis[1, 0] = data[1].toDouble()
    axis[2, 0] = data[2].toDouble()
    return rotationEDNtoNWU(Rotation3d(axis.div(axis.norm()), axis.norm()))
  }

  fun targetCornersToMat(corners: List<TargetCorner>): MatOfPoint2f {
    return targetCornersToMat(*corners.toTypedArray())
  }

  fun targetCornersToMat(vararg corners: TargetCorner): MatOfPoint2f {
    val points = arrayOfNulls<Point>(corners.size)
    for (i in 0 until corners.size) {
      points[i] = Point(corners[i].x, corners[i].y)
    }
    return MatOfPoint2f(*points)
  }

  fun matToTargetCorners(matInput: MatOfPoint2f): Array<TargetCorner?> {
    val corners = arrayOfNulls<TargetCorner>(matInput.total().toInt())
    val data = FloatArray(matInput.total().toInt() * matInput.channels())
    matInput[0, 0, data]
    for (i in corners.indices) {
      corners[i] = TargetCorner(data[0 + 2 * i].toDouble(), data[1 + 2 * i].toDouble())
    }
    return corners
  }

  /**
   * Reorders the list, optionally indexing backwards and wrapping around to the
   * last element after the first, and shifting all indices in the direction of indexing.
   *
   *
   * e.g.
   *
   * ({1,2,3}, false, 1) -> {2,3,1}
   *
   * ({1,2,3}, true, 0) -> {1,3,2}
   *
   * ({1,2,3}, true, 1) -> {3,2,1}
   * @param <T>
   * @param elements
   * @param backwards
   * @param shift
   * @return
  </T> */
  fun <T> reorderCircular(elements: List<T>, backwards: Boolean, shift: Int): List<T> {
    val size = elements.size
    val dir = if (backwards) -1 else 1
    val reordered = ArrayList(elements)
    for (i in 0 until size) {
      var index = (i * dir + shift * dir) % size
      if (index < 0) index = size + index
      reordered[i] = elements[index]
    }
    return reordered
  }

  /**
   * Convert a rotation from EDN to NWU. For example, if you have a rotation X,Y,Z {1, 0, 0}
   * in EDN, this would be XYZ {0, -1, 0} in NWU.
   */
  private fun rotationEDNtoNWU(rot: Rotation3d): Rotation3d {
    return CoordinateSystem.convert(
      Rotation3d(),
      CoordinateSystem.NWU(),
      CoordinateSystem.EDN()
    ).plus(
      CoordinateSystem.convert(
        rot,
        CoordinateSystem.EDN(),
        CoordinateSystem.NWU()
      )
    )
  }

  /**
   * Convert a rotation from EDN to NWU. For example, if you have a rotation X,Y,Z {1, 0, 0}
   * in EDN, this would be XYZ {0, -1, 0} in NWU.
   */
  private fun rotationNWUtoEDN(rot: Rotation3d): Rotation3d {
    return CoordinateSystem.convert(
      Rotation3d(),
      CoordinateSystem.EDN(),
      CoordinateSystem.NWU()
    ).plus(
      CoordinateSystem.convert(
        rot,
        CoordinateSystem.NWU(),
        CoordinateSystem.EDN()
      )
    )
  }

  /**
   * Project object points from the 3d world into the 2d camera image.
   * The camera properties(intrinsics, distortion) determine the results of
   * this projection.
   *
   * @param camPose The current camera pose in the 3d world
   * @param camProp The properties of this camera
   * @param objectTranslations The 3d points to be projected
   * @return The 2d points in pixels which correspond to the image of the 3d points on the camera
   */
  fun projectPoints(
    camPose: Pose3d?, camProp: CameraProperties,
    objectTranslations: List<Translation3d>
  ): List<TargetCorner> {
    // translate to opencv classes
    val objectPoints = translationToTvec(*objectTranslations.toTypedArray())
    // opencv rvec/tvec describe a change in basis from world to camera
    val basisChange = makeRelativeTo(camPose!!)
    val rvec = rotationToRvec(basisChange.rotation)
    val tvec = translationToTvec(basisChange.translation)
    val cameraMatrix = matrixToMat(camProp.intrinsics.storage)
    val distCoeffs = matrixToMat(camProp.getDistCoeffs().storage)
    val imagePoints = MatOfPoint2f()
    // project to 2d
    Calib3d.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints)

    // turn 2d point Mat into TargetCorners
    val corners = matToTargetCorners(imagePoints)

    // release our Mats from native memory
    objectPoints.release()
    rvec.release()
    tvec.release()
    cameraMatrix.release()
    distCoeffs.release()
    imagePoints.release()
    return corners.map { it!! }
  }

  /**
   * Undistort 2d image points using a given camera's intrinsics and distortion.
   *
   * will naturally be distorted, so this operation is important if the image points
   * need to be directly used (e.g. 2d yaw/pitch).
   * @param camProp The properties of this camera
   * @param corners The distorted image points
   * @return The undistorted image points
   */
  fun undistortPoints(camProp: CameraProperties, corners: List<TargetCorner>): List<TargetCorner> {
    val points_in = targetCornersToMat(corners)
    val points_out = MatOfPoint2f()
    val cameraMatrix = matrixToMat(camProp.intrinsics.storage)
    val distCoeffs = matrixToMat(camProp.getDistCoeffs().storage)
    Calib3d.undistortImagePoints(points_in, points_out, cameraMatrix, distCoeffs)
    val corners_out = matToTargetCorners(points_out)
    points_in.release()
    points_out.release()
    cameraMatrix.release()
    distCoeffs.release()
    return corners_out.map {it!!}
  }

  /**
   * Get the rectangle with minimum area which bounds this contour. This is useful for finding the center of a bounded
   * contour or the size of the bounding box.
   *
   * @param corners The corners defining this contour
   * @return Rectangle bounding the contour created by the given corners
   */
  fun getMinAreaRect(corners: List<TargetCorner>): RotatedRect {
    val corn = targetCornersToMat(corners)
    val rect = Imgproc.minAreaRect(corn)
    corn.release()
    return rect
  }

  /**
   * Get the area in pixels of this target's contour. It's important to note that this may
   * be different from the area of the bounding rectangle around the contour.
   *
   * @param corners The corners defining this contour
   * @return Area in pixels (units of corner x/y)
   */
  fun getContourAreaPx(corners: List<TargetCorner>): Double {
    val temp = targetCornersToMat(corners)
    val corn = MatOfPoint(*temp.toArray())
    temp.release()

    // outputHull gives us indices (of corn) that make a convex hull contour
    val outputHull = MatOfInt()
    Imgproc.convexHull(corn, outputHull)
    val indices = outputHull.toArray()
    outputHull.release()
    val tempPoints = corn.toArray()
    val points = tempPoints.clone()
    for (i in indices.indices) {
      points[i] = tempPoints[indices[i]]
    }
    corn.fromArray(*points)
    // calculate area of the (convex hull) contour
    val area = Imgproc.contourArea(corn)
    corn.release()
    return area
  }

  /**
   * Finds the transformation(s) that map the camera's pose to the target pose.
   * The camera's pose relative to the target is determined by the supplied
   * 3d points of the target's model and their associated 2d points imaged by the camera.
   *
   *
   * For planar targets, there may be an alternate solution which is plausible given
   * the 2d image points. This has an associated "ambiguity" which describes the
   * ratio of reprojection error between the "best" and "alternate" solution.
   *
   * @param camProp The properties of this camera
   * @param modelTrls The translations of the object corners. These should have the object
   * pose as their origin. These must come in a specific, pose-relative order (in NWU):
   *
   *  *  Point 0: [0, -squareLength / 2,  squareLength / 2]
   *  *  Point 1: [0,  squareLength / 2,  squareLength / 2]
   *  *  Point 2: [0,  squareLength / 2, -squareLength / 2]
   *  *  Point 3: [0, -squareLength / 2, -squareLength / 2]
   *
   * @param imageCorners The projection of these 3d object points into the 2d camera image.
   * The order should match the given object point translations.
   * @return The resulting **transformation(s)** that map the camera pose to the target pose
   * and the ambiguity if alternate solutions are also available.
   */
  @JvmStatic
  fun solveTagPNP(
    camProp: CameraProperties, modelTrls: List<Translation3d?>, imageCorners: List<TargetCorner>
  ): PNPResults {
    // IPPE_SQUARE expects our corners in a specific order
    var modelTrls = modelTrls
    var imageCorners = imageCorners
    modelTrls = reorderCircular(modelTrls, false, 2)
    imageCorners = reorderCircular(imageCorners, false, 2)
    // translate to opencv classes
    val objectPoints = translationToTvec(*modelTrls.toTypedArray())
    val imagePoints = targetCornersToMat(imageCorners)
    val cameraMatrix = matrixToMat(camProp.intrinsics.storage)
    val distCoeffs = matrixToMat(camProp.getDistCoeffs().storage)
    val rvecs = ArrayList<Mat>()
    val tvecs = ArrayList<Mat>()
    val rvec = Mat.zeros(3, 1, CvType.CV_32F)
    val tvec = Mat.zeros(3, 1, CvType.CV_32F)
    val reprojectionError = Mat()
    // calc rvecs/tvecs and associated reprojection error from image points
    Calib3d.solvePnPGeneric(
      objectPoints, imagePoints,
      cameraMatrix, distCoeffs,
      rvecs, tvecs,
      false, Calib3d.SOLVEPNP_IPPE_SQUARE,
      rvec, tvec,
      reprojectionError
    )
    val errors = FloatArray(2)
    reprojectionError[0, 0, errors]
    // convert to wpilib coordinates
    val best = Transform3d(
      tvecToTranslation(tvecs[0]),
      rvecToRotation(rvecs[0])
    )
    var alt = Transform3d()
    if (tvecs.size > 1) {
      alt = Transform3d(
        tvecToTranslation(tvecs[1]),
        rvecToRotation(rvecs[1])
      )
    }

    // release our Mats from native memory
    objectPoints.release()
    imagePoints.release()
    cameraMatrix.release()
    distCoeffs.release()
    for (v in rvecs) v.release()
    for (v in tvecs) v.release()
    rvec.release()
    tvec.release()
    reprojectionError.release()
    return PNPResults(
      best, alt, (errors[0] / errors[1]).toDouble(), errors[0].toDouble(), errors[1]
        .toDouble()
    )
  }

  /**
   * Finds the transformation that maps the camera's pose to the field origin.
   * The camera's pose relative to the targets is determined by the supplied 3d points of
   * the target's corners on the field and their associated 2d points imaged by the camera.
   *
   *
   * For planar targets, there may be an alternate solution which is plausible given
   * the 2d image points. This has an associated "ambiguity" which describes the
   * ratio of reprojection error between the "best" and "alternate" solution.
   *
   * @param camProp The properties of this camera
   * @param objectTrls The translations of the object corners, relative to the field.
   * @param imageCorners The projection of these 3d object points into the 2d camera image.
   * The order should match the given object point translations.
   * @return The resulting transformation(s) that map the camera pose to the target pose
   * and the ambiguity if alternate solutions are also available.
   */
  @JvmStatic
  fun solveTagsPNP(
    camProp: CameraProperties, objectTrls: List<Translation3d>, imageCorners: List<TargetCorner>
  ): PNPResults {
    // translate to opencv classes
    val objectPoints = translationToTvec(*objectTrls.toTypedArray())
    val imagePoints = targetCornersToMat(imageCorners)
    val cameraMatrix = matrixToMat(camProp.intrinsics.storage)
    val distCoeffs = matrixToMat(camProp.getDistCoeffs().storage)
    val rvecs = ArrayList<Mat>()
    val tvecs = ArrayList<Mat>()
    val rvec = Mat.zeros(3, 1, CvType.CV_32F)
    val tvec = Mat.zeros(3, 1, CvType.CV_32F)
    val reprojectionError = Mat()
    // calc rvec/tvec from image points
    Calib3d.solvePnPGeneric(
      objectPoints, imagePoints,
      cameraMatrix, distCoeffs,
      rvecs, tvecs,
      false, Calib3d.SOLVEPNP_SQPNP,
      rvec, tvec, reprojectionError
    )
    val errors = FloatArray(2)
    reprojectionError[0, 0, errors]
    // convert to wpilib coordinates
    val best = Transform3d(
      tvecToTranslation(tvecs[0]),
      rvecToRotation(rvecs[0])
    )
    val alt = Transform3d()

    // release our Mats from native memory
    objectPoints.release()
    imagePoints.release()
    cameraMatrix.release()
    distCoeffs.release()
    for (v in rvecs) v.release()
    for (v in tvecs) v.release()
    rvec.release()
    tvec.release()
    reprojectionError.release()
    return PNPResults(best, alt, 0.0, errors[0].toDouble(), 0.0)
  }

  fun solveTagsPNPRansac(
    camProp: CameraProperties, objectTrls: List<Translation3d>, imageCorners: List<TargetCorner>
  ): PNPResults {
    // translate to opencv classes
    val objectPoints = translationToTvec(*objectTrls.toTypedArray())
    val imagePoints = targetCornersToMat(imageCorners)
    val cameraMatrix = matrixToMat(camProp.intrinsics.storage)
    val distCoeffs = matrixToMat(camProp.getDistCoeffs().storage)
    val rvec = Mat.zeros(3, 1, CvType.CV_32F)
    val tvec = Mat.zeros(3, 1, CvType.CV_32F)
    val inliers = Mat()
    // calc rvec/tvec from image points
    Calib3d.solvePnPRansac(
      objectPoints, imagePoints,
      cameraMatrix, distCoeffs,
      rvec, tvec,
      false, 10000,
      1f, 0.99, inliers,
      Calib3d.USAC_MAGSAC
    )
    // System.out.println("--Inliers:");
    // System.out.println(inliers.dump());
    val best = Transform3d(
      tvecToTranslation(tvec),
      rvecToRotation(rvec)
    )
    val alt = Transform3d()

    // release our Mats from native memory
    objectPoints.release()
    imagePoints.release()
    cameraMatrix.release()
    distCoeffs.release()
    rvec.release()
    tvec.release()
    inliers.release()
    return PNPResults(best, alt, 0.0, 0.0, 0.0)
  }

  fun estimateRigidTransform(
    measuredTrls: List<Translation3d>, knownTrls: List<Translation3d>
  ): RotTrlTransform3d {
    val measuredPts = translationToTvec(*measuredTrls.toTypedArray())
    val knownPts = translationToTvec(*knownTrls.toTypedArray())
    val outputMat = Calib3d.estimateAffine3D(measuredPts, knownPts, null, true)
    val outputMatrix = matToMatrix(outputMat)

    // rotation matrix
    var rot = Rotation3d(outputMatrix.block(3, 3, 0, 0))
    rot = rotationEDNtoNWU(rot)
    // translation vector
    val trlData = outputMatrix.block<N3, N1>(3, 1, 0, 3).data
    var trl: Translation3d? = Translation3d(trlData[0], trlData[1], trlData[2])
    trl = CoordinateSystem.convert(trl, CoordinateSystem.EDN(), CoordinateSystem.NWU())
    val trf = RotTrlTransform3d(
      rot,
      trl
    )

    // release our Mats from native memory
    measuredPts.release()
    knownPts.release()
    outputMat.release()
    return trf
  }

  init {
    try {
      CameraServerCvJNI.forceLoad()
    } catch (e: Exception) {
      throw RuntimeException("Failed to load native libraries!", e)
    }
  }
}
