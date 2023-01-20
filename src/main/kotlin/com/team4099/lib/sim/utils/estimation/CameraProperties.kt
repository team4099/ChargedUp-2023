package com.team4099.lib.sim.utils.estimation

import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N5
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Nat
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Rotation3d
import java.util.stream.Collectors
import java.util.*

/**
 * Calibration and performance values for this camera.
 *
 *
 * The resolution will affect the accuracy of projected(3d->2d) target corners and
 * similarly the severity of image noise on estimation(2d->3d).
 *
 *
 * The camera intrinsics and distortion coefficients describe the results of calibration,
 * and how to map between 3d field points and 2d image points.
 *
 *
 * The performance values (framerate/exposure time, latency) determine how often results
 * should be updated and with how much latency in simulation. High exposure time causes motion
 * blur which can inhibit target detection while moving. Note that latency estimation does not
 * account for network latency and the latency reported will always be perfect.
 */
class CameraProperties {
  private val rand = Random()

  // calibration
  var resWidth = 0
    private set
  var resHeight = 0
    private set
  private var camIntrinsics: Matrix<N3, N3>? = null
  private var distCoeffs: Matrix<N5?, N1>? = null
  private var avgErrorPx = 0.0
  private var errorStdDevPx = 0.0

  // performance
  var frameSpeedMs = 0.0
    private set

  /**
   * @param exposureTimeMs The amount of time the "shutter" is open for one frame.
   * Affects motion blur. **Frame speed(from FPS) is limited to this!**
   */
  var exposureTimeMs = 0.0

  /**
   * @param avgLatencyMs The average latency (image capture -> data) in milliseconds
   * a frame should have
   */
  var avgLatencyMs = 0.0

  /**
   * @param latencyStdDevMs The standard deviation in milliseconds of the latency
   */
  var latencyStdDevMs = 0.0
  fun setRandomSeed(seed: Long) {
    rand.setSeed(seed)
  }

  fun setCalibration(resWidth: Int, resHeight: Int, fovDiag: Rotation2d) {
    val fovWidth = Rotation2d(
      fovDiag.radians * (resWidth / Math.sqrt((resWidth * resWidth + resHeight * resHeight).toDouble()))
    )
    val fovHeight = Rotation2d(
      fovDiag.radians * (resHeight / Math.sqrt((resWidth * resWidth + resHeight * resHeight).toDouble()))
    )
    // assume no distortion
    val distCoeff = VecBuilder.fill(0.0, 0.0, 0.0, 0.0, 0.0)
    // assume centered principal point (pixels)
    val cx = resWidth / 2.0
    val cy = resHeight / 2.0
    // use given fov to determine focal point (pixels)
    val fx = cx / Math.tan(fovWidth.radians / 2.0)
    val fy = cy / Math.tan(fovHeight.radians / 2.0)
    // create camera intrinsics matrix
    val camIntrinsics = Matrix.mat(Nat.N3(), Nat.N3()).fill(
      fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0
    )
    setCalibration(resWidth, resHeight, camIntrinsics, distCoeff)
  }

  fun setCalibration(
    resWidth: Int, resHeight: Int,
    camIntrinsics: Matrix<N3, N3>?, distCoeffs: Matrix<N5?, N1>?
  ) {
    this.resWidth = resWidth
    this.resHeight = resHeight
    this.camIntrinsics = camIntrinsics
    this.distCoeffs = distCoeffs
  }

  fun setCameraIntrinsics(camIntrinsics: Matrix<N3, N3>?) {
    this.camIntrinsics = camIntrinsics
  }

  fun setDistortionCoeffs(distCoeffs: Matrix<N5?, N1>?) {
    this.distCoeffs = distCoeffs
  }

  fun setCalibError(avgErrorPx: Double, errorStdDevPx: Double) {
    this.avgErrorPx = avgErrorPx
    this.errorStdDevPx = errorStdDevPx
  }

  val resArea: Int
    get() = resWidth * resHeight
  val intrinsics: Matrix<N3, N3>
    get() = camIntrinsics!!.copy()

  fun getDistCoeffs(): Vector<N5?> {
    return Vector(distCoeffs)
  }

  /**
   * @param fps The average frames per second the camera should process at
   */
  var fPS: Double
    get() = 1000.0 / frameSpeedMs
    set(fps) {
      frameSpeedMs = 1000.0 / fps
    }

  fun copy(): CameraProperties {
    val newProp = CameraProperties()
    newProp.setCalibration(resWidth, resHeight, camIntrinsics, distCoeffs)
    newProp.setCalibError(avgErrorPx, errorStdDevPx)
    newProp.fPS = fPS
    newProp.exposureTimeMs = exposureTimeMs
    newProp.avgLatencyMs = avgLatencyMs
    newProp.latencyStdDevMs = latencyStdDevMs
    return CameraProperties()
  }

  /**
   * Undistorts a detected target's corner points, and returns a new PhotonTrackedTarget
   * with updated corners, yaw, pitch, skew, and area. Best/alt pose and ambiguity are unchanged.
   */
  fun undistort2dTarget(target: PhotonTrackedTarget): PhotonTrackedTarget {
    val undistortedCorners = undistort(target.corners)
    // find the 2d yaw/pitch
    val boundingCenterRot = getPixelRot(undistortedCorners)
    // find contour area
    val areaPercent = getContourAreaPercent(undistortedCorners)
    return PhotonTrackedTarget(
      Math.toDegrees(boundingCenterRot.z),
      -Math.toDegrees(boundingCenterRot.y),
      areaPercent,
      Math.toDegrees(boundingCenterRot.x),
      target.fiducialId,
      target.bestCameraToTarget,
      target.alternateCameraToTarget,
      target.poseAmbiguity,
      undistortedCorners
    )
  }

  fun undistort(points: List<TargetCorner?>?): List<TargetCorner> {
    return OpenCVHelp.undistortPoints(this, points!!.map {it!!})
  }

  /**
   * The percentage(0 - 100) of this camera's resolution the contour takes up in pixels
   * of the image.
   * @param corners Corners of the contour
   */
  fun getContourAreaPercent(corners: List<TargetCorner>?): Double {
    return OpenCVHelp.getContourAreaPx(corners!!) / resArea * 100
  }

  /**
   * The yaw from the principal point of this camera to the pixel x value.
   * Positive values left.
   */
  fun getPixelYaw(pixelX: Double): Rotation2d {
    val fx = camIntrinsics!![0, 0]
    // account for principal point not being centered
    val cx = camIntrinsics!![0, 2]
    val xOffset = cx - pixelX
    return Rotation2d(
      fx,
      xOffset
    )
  }

  /**
   * The pitch from the principal point of this camera to the pixel y value.
   * Pitch is positive down.
   */
  fun getPixelPitch(pixelY: Double): Rotation2d {
    val fy = camIntrinsics!![1, 1]
    // account for principal point not being centered
    val cy = camIntrinsics!![1, 2]
    val yOffset = cy - pixelY
    return Rotation2d(
      fy,
      -yOffset
    )
  }

  /**
   * Undistorts these image points, and then finds the yaw and pitch to the center
   * of the rectangle bounding them. Yaw is positive left, and pitch is positive down.
   */
  fun getUndistortedPixelRot(points: List<TargetCorner?>?): Rotation3d {
    return getPixelRot(undistort(points))
  }

  /**
   * Finds the yaw and pitch to the center of the rectangle bounding the given
   * image points. Yaw is positive left, and pitch is positive down.
   */
  fun getPixelRot(points: List<TargetCorner>?): Rotation3d {
    if (points == null || points.size == 0) return Rotation3d()
    val rect = OpenCVHelp.getMinAreaRect(points)
    return Rotation3d(
      rect.angle,
      getPixelPitch(rect.center.y).radians,
      getPixelYaw(rect.center.x).radians
    )
  }

  // sum of FOV left and right principal point
  val horizFOV: Rotation2d
    get() {
      // sum of FOV left and right principal point
      val left = getPixelYaw(0.0)
      val right = getPixelYaw(resWidth.toDouble())
      return left.minus(right)
    }

  // sum of FOV above and below principal point
  val vertFOV: Rotation2d
    get() {
      // sum of FOV above and below principal point
      val above = getPixelPitch(0.0)
      val below = getPixelPitch(resHeight.toDouble())
      return below.minus(above)
    }
  val diagFOV: Rotation2d
    get() = Rotation2d(Math.hypot(horizFOV.radians, vertFOV.radians))

  /** Width:height  */
  val aspectRatio: Double
    get() = resWidth.toDouble() / resHeight

  /**
   * Returns these pixel points as fractions of a 1x1 square image.
   * This means the camera's aspect ratio and resolution will be used, and the
   * points' x and y may not reach all portions(e.g. a wide aspect ratio means
   * some of the top and bottom of the square image is unreachable).
   *
   * @param points Pixel points on this camera's image
   * @return Points mapped to an image of 1x1 resolution
   */
  fun getPixelFraction(points: List<TargetCorner>): List<TargetCorner> {
    val resLarge = if (aspectRatio > 1) resWidth.toDouble() else resHeight.toDouble()
    return points.stream()
      .map { p: TargetCorner ->
        TargetCorner(
          (p.x + (resLarge - resWidth) / 2.0) / resLarge,
          (p.y + (resLarge - resHeight) / 2.0) / resLarge
        )
      }
      .collect(Collectors.toList())
  }

  /**
   * @return Estimate of new points based on this camera's noise.
   */
  fun estPixelNoise(points: List<TargetCorner>): List<TargetCorner> {
    return if (avgErrorPx == 0.0 && errorStdDevPx == 0.0) points else points.stream()
      .map { p: TargetCorner ->
        // error pixels in random direction
        val error = Random().nextGaussian() * errorStdDevPx + avgErrorPx
        val errorAngle = -Math.PI + (Math.PI + Math.PI) * rand.nextDouble()
        TargetCorner(
          p.x + error * Math.cos(errorAngle),
          p.y + error * Math.sin(errorAngle)
        )
      }
      .collect(Collectors.toList())
  }

  /**
   * @return Noisy estimation of a frame's processing latency in milliseconds
   */
  fun estLatencyMs(): Double {
    return Math.max(rand.nextGaussian() * latencyStdDevMs + avgLatencyMs, 0.0)
  }

  /**
   * @return Estimate how long until the next frame should be processed in milliseconds
   */
  fun estMsUntilNextFrame(): Double {
    // exceptional processing latency blocks the next frame
    return frameSpeedMs + Math.max(0.0, estLatencyMs() - frameSpeedMs)
  }

  companion object {
    // pre-calibrated example cameras
    /** 960x720 resolution, 90 degree FOV, "perfect" lagless camera  */
    val PERFECT_90DEG = CameraProperties()
    val PI4_PICAM2_480p = CameraProperties()
    val LL2_480p = CameraProperties()
    val LL2_960_720p = CameraProperties()
    val LL2_1280_720p = CameraProperties()

    init {
      PI4_PICAM2_480p.setCalibration(
        640, 480,
        Matrix.mat(Nat.N3(), Nat.N3()).fill( // intrinsic
          497.8204072694636, 0.0, 314.53659309737424,
          0.0, 481.6955284883231, 231.95042993880858,
          0.0, 0.0, 1.0
        ),
        VecBuilder.fill( // distort
          0.16990717177326176,
          -0.47305087536583684,
          -0.002992219989630736,
          -0.0016840919550094836,
          0.36623021008942863
        )
      )
      PI4_PICAM2_480p.setCalibError(0.25, 0.07)
      PI4_PICAM2_480p.fPS = 17.0
      PI4_PICAM2_480p.avgLatencyMs = 35.0
      PI4_PICAM2_480p.latencyStdDevMs = 8.0
      LL2_480p.setCalibration(
        640, 480,
        Matrix.mat(Nat.N3(), Nat.N3()).fill( // intrinsic
          511.22843367007755, 0.0, 323.62049380211096,
          0.0, 514.5452336723849, 261.8827920543568,
          0.0, 0.0, 1.0
        ),
        VecBuilder.fill( // distort
          0.1917469998873756,
          -0.5142936883324216,
          0.012461562046896614,
          0.0014084973492408186,
          0.35160648971214437
        )
      )
      LL2_480p.setCalibError(0.25, 0.05)
      LL2_480p.fPS = 15.0
      LL2_480p.avgLatencyMs = 35.0
      LL2_480p.latencyStdDevMs = 8.0
      LL2_960_720p.setCalibration(
        960, 720,
        Matrix.mat(Nat.N3(), Nat.N3()).fill( // intrinsic
          769.6873145148892, 0.0, 486.1096609458122,
          0.0, 773.8164483705323, 384.66071662358354,
          0.0, 0.0, 1.0
        ),
        VecBuilder.fill( // distort
          0.189462064814501,
          -0.49903003669627627,
          0.007468423590519429,
          0.002496885298683693,
          0.3443122090208624
        )
      )
      LL2_960_720p.setCalibError(0.35, 0.10)
      LL2_960_720p.fPS = 10.0
      LL2_960_720p.avgLatencyMs = 50.0
      LL2_960_720p.latencyStdDevMs = 15.0
      LL2_1280_720p.setCalibration(
        1280, 720,
        Matrix.mat(Nat.N3(), Nat.N3()).fill( // intrinsic
          1011.3749416937393, 0.0, 645.4955139388737,
          0.0, 1008.5391755084075, 508.32877656020196,
          0.0, 0.0, 1.0
        ),
        VecBuilder.fill( // distort
          0.13730101577061535,
          -0.2904345656989261,
          8.32475714507539E-4,
          -3.694397782014239E-4,
          0.09487962227027584
        )
      )
      LL2_1280_720p.setCalibError(0.37, 0.06)
      LL2_1280_720p.fPS = 7.0
      LL2_1280_720p.avgLatencyMs = 60.0
      LL2_1280_720p.latencyStdDevMs = 20.0
    }
  }

  /**
   * Default constructor which is the same as [.PERFECT_90DEG]
   */
  init {
    setCalibration(960, 720, Rotation2d.fromDegrees(90.0))
  }
}
