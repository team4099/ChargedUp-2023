package com.team4099.lib.sim.utils.util

import org.opencv.core.Mat
import java.util.HashMap
import edu.wpi.first.cscore.CvSource
import com.team4099.lib.sim.utils.estimation.CameraProperties
import org.opencv.imgcodecs.Imgcodecs
import kotlin.jvm.JvmOverloads
import org.opencv.core.MatOfPoint2f
import org.opencv.imgproc.Imgproc
import org.opencv.core.Scalar
import org.opencv.core.CvType
import java.util.Arrays
import org.opencv.core.MatOfPoint
import edu.wpi.first.cscore.CameraServerCvJNI
import org.opencv.core.Core
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Size
import java.lang.Exception
import java.lang.RuntimeException
import java.util.List

object VideoSimUtil {
  const val kTagImagesPath = "./src/main/kotlin/com/team4099/lib/utils/sim/apriltag-images/"
  const val kTag16h5ImageName = "tag16_05_00000"
  const val kNumTags16h5 = 30

  // All 16h5 tag images
  private val kTag16h5Images: MutableMap<Int, Mat> = HashMap()

  // Points corresponding to marker(black square) corners of 8x8 16h5 tag images
  val kTag16h5MarkerPts: Array<Point>

  /**
   * Updates the properties of this CvSource video stream with the given camera properties.
   */
  @JvmStatic
  fun updateVideoProp(video: CvSource, prop: CameraProperties) {
    video.setResolution(prop.resWidth, prop.resHeight)
    video.setFPS(prop.fPS.toInt())
  }

  /**
   * Gets the points representing the corners of this image. Because image pixels are
   * accessed through a Mat, the point (0,0) actually represents the center of the top-left
   * pixel and not the actual top-left corner.
   * @param size Size of image
   */
  fun getImageCorners(size: Size): Array<Point> {
    return arrayOf(
      Point(-0.5, -0.5),
      Point(size.width - 0.5, -0.5),
      Point(size.width - 0.5, size.height - 0.5),
      Point(-0.5, size.height - 0.5)
    )
  }

  /**
   * Gets the 8x8 (grayscale) image of a specific 16h5 AprilTag.
   *
   * @param id The fiducial id of the desired tag
   */
  fun get16h5TagImage(id: Int): Mat {
    var name = kTag16h5ImageName
    val idString = id.toString()
    name = name.substring(0, name.length - idString.length) + idString
    return Imgcodecs.imread(kTagImagesPath + name + ".png", Imgcodecs.IMREAD_GRAYSCALE)
  }
  /**
   * Gets the points representing the marker(black square) corners.
   * @param scale The scale of the tag image (8*scale x 8*scale image)
   */
  /**
   * Gets the points representing the marker(black square) corners.
   */
  @JvmOverloads
  fun get16h5MarkerPts(scale: Int = 1): Array<Point> {
    val roi16h5 = Rect(Point(1.0, 1.0), Size(6.0, 6.0))
    roi16h5.x *= scale
    roi16h5.y *= scale
    roi16h5.width *= scale
    roi16h5.height *= scale
    val pts = getImageCorners(roi16h5.size())
    for (i in pts.indices) {
      val pt = pts[i]
      pts[i] = Point(roi16h5.tl().x + pt.x, roi16h5.tl().y + pt.y)
    }
    return pts
  }

  /**
   * Warps the image of a specific 16h5 AprilTag onto the destination image at the given points.
   *
   * @param tagId The id of the specific tag to warp onto the destination image
   * @param dstPoints Points(4) in destination image where the tag marker(black square) corners
   * should be warped onto.
   * @param destination The destination image to place the warped tag image onto.
   * @param antialiasing If antialiasing should be performed by automatically
   * supersampling/interpolating the warped image. This should be used if better stream quality
   * is desired or target detection is being done on the stream, but can hurt performance.
   */
  @JvmStatic
  fun warp16h5TagImage(
    tagId: Int, dstPoints: MatOfPoint2f, destination: Mat, antialiasing: Boolean
  ) {
    val tagImage = kTag16h5Images[tagId] ?: return
    val tagPoints = MatOfPoint2f(*kTag16h5MarkerPts)
    // points of tag image corners
    val tagImageCorners = MatOfPoint2f(*getImageCorners(tagImage.size()))
    // the rectangle describing the rectangle-of-interest(ROI)
    var boundingRect = Imgproc.boundingRect(dstPoints)
    // find the perspective transform from the tag image to the warped destination points
    var perspecTrf = Imgproc.getPerspectiveTransform(tagPoints, dstPoints)
    // check extreme image corners after transform to check if we need to expand bounding rect
    val extremeCorners = MatOfPoint2f()
    Core.perspectiveTransform(tagImageCorners, extremeCorners, perspecTrf)
    // dilate ROI to fit full tag
    boundingRect = Imgproc.boundingRect(extremeCorners)

    // adjust interpolation strategy based on size of warped tag compared to tag image
    val warpedContourArea = Imgproc.contourArea(extremeCorners)
    val warpedTagUpscale = Math.sqrt(warpedContourArea) / Math.sqrt(tagImage.size().area())
    var warpStrategy = Imgproc.INTER_NEAREST
    // automatically determine the best supersampling of warped image and scale of tag image
    /*
    (warpPerspective does not properly resample, so this is used to avoid aliasing in the
    warped image. Supersampling is used when the warped tag is small, but is very slow
    when the warped tag is large-- scaling the tag image up and using linear interpolation
    instead can be performant while still effectively antialiasing. Some combination of these
    two can be used in between those extremes.)

    TODO: Simplify magic numbers to one or two variables, or use a more proper approach?
    */
    var supersampling = 6
    supersampling = Math.ceil(supersampling / warpedTagUpscale).toInt()
    supersampling = Math.max(Math.min(supersampling, 8), 1)
    val scaledTagImage = Mat()
    if (warpedTagUpscale > 2.0) {
      warpStrategy = Imgproc.INTER_LINEAR
      var scaleFactor = (warpedTagUpscale / 3.0).toInt() + 2
      scaleFactor = Math.max(Math.min(scaleFactor, 40), 1)
      scaleFactor *= supersampling
      Imgproc.resize(
        tagImage, scaledTagImage,
        Size(), scaleFactor.toDouble(), scaleFactor.toDouble(),
        Imgproc.INTER_NEAREST
      )
      tagPoints.fromArray(*get16h5MarkerPts(scaleFactor))
    } else tagImage.assignTo(scaledTagImage)

    // constrain the bounding rect inside of the destination image
    boundingRect.x -= 1
    boundingRect.y -= 1
    boundingRect.width += 2
    boundingRect.height += 2
    if (boundingRect.x < 0) {
      boundingRect.width += boundingRect.x
      boundingRect.x = 0
    }
    if (boundingRect.y < 0) {
      boundingRect.height += boundingRect.y
      boundingRect.y = 0
    }
    boundingRect.width = Math.min(destination.width() - boundingRect.x, boundingRect.width)
    boundingRect.height = Math.min(destination.height() - boundingRect.y, boundingRect.height)
    if (boundingRect.width <= 0 || boundingRect.height <= 0) return

    // upscale if supersampling
    val scaledDstPts = Mat()
    if (supersampling > 1) {
      Core.multiply(dstPoints, Scalar(supersampling.toDouble(), supersampling.toDouble()), scaledDstPts)
      boundingRect.x *= supersampling
      boundingRect.y *= supersampling
      boundingRect.width *= supersampling
      boundingRect.height *= supersampling
    } else dstPoints.assignTo(scaledDstPts)

    // update transform relative to expanded, scaled bounding rect
    Core.subtract(scaledDstPts, Scalar(boundingRect.tl().x, boundingRect.tl().y), scaledDstPts)
    perspecTrf = Imgproc.getPerspectiveTransform(tagPoints, scaledDstPts)

    // warp (scaled) tag image onto (scaled) ROI image representing the portion of
    // the destination image encapsulated by boundingRect
    val tempROI = Mat()
    Imgproc.warpPerspective(
      scaledTagImage, tempROI,
      perspecTrf,
      boundingRect.size(),
      warpStrategy
    )

    // downscale ROI with interpolation if supersampling
    if (supersampling > 1) {
      boundingRect.x /= supersampling
      boundingRect.y /= supersampling
      boundingRect.width /= supersampling
      boundingRect.height /= supersampling
      Imgproc.resize(tempROI, tempROI, boundingRect.size(), 0.0, 0.0, Imgproc.INTER_AREA)
    }

    // we want to copy ONLY the transformed tag to the result image, not the entire bounding rect
    // using a mask only copies the source pixels which have an associated non-zero value in the mask
    val tempMask = Mat.zeros(tempROI.size(), CvType.CV_8UC1)
    Core.subtract(extremeCorners, Scalar(boundingRect.tl().x, boundingRect.tl().y), extremeCorners)
    val tempCenter = Point()
    tempCenter.x = Arrays.stream(extremeCorners.toArray()).mapToDouble { p: Point -> p.x }
      .average().asDouble
    tempCenter.y = Arrays.stream(extremeCorners.toArray()).mapToDouble { p: Point -> p.y }
      .average().asDouble
    // dilate tag corners
    Arrays.stream(extremeCorners.toArray()).forEach { p: Point ->
      var xdiff = p.x - tempCenter.x
      var ydiff = p.y - tempCenter.y
      xdiff += 1 * Math.signum(xdiff)
      ydiff += 1 * Math.signum(ydiff)
      Point(tempCenter.x + xdiff, tempCenter.y + ydiff)
    }
    // (make inside of tag completely white in mask)
    Imgproc.fillConvexPoly(
      tempMask,
      MatOfPoint(*extremeCorners.toArray()),
      Scalar(255.0)
    )

    // copy transformed tag onto result image
    tempROI.copyTo(destination.submat(boundingRect), tempMask)
  }

  /**
   * Draws a contour around the given points and text of the id onto the destination image.
   *
   * @param id Fiducial ID number to draw
   * @param dstPoints Points representing the four corners of the tag marker(black square) in
   * the destination image.
   * @param destination The destination image to draw onto. The image should be in the BGR
   * color space.
   */
  @JvmStatic
  fun drawTagDetection(id: Int, dstPoints: MatOfPoint2f, destination: Mat) {
    val dstPointsd = MatOfPoint(*dstPoints.toArray())
    val scaleX = destination.width() / 640.0
    val scaleY = destination.height() / 480.0
    val minScale = Math.min(scaleX, scaleY)
    val thickness = (1 * minScale).toInt()
    // for(var pt : dstPoints.toArray()) {
    //     Imgproc.circle(destination, pt, 4, new Scalar(255), 1, Imgproc.LINE_AA);
    // }
    // Imgproc.rectangle(destination, extremeRect, new Scalar(255), 1, Imgproc.LINE_AA);
    // Imgproc.rectangle(destination, Imgproc.boundingRect(dstPoints), new Scalar(255), 1, Imgproc.LINE_AA);
    Imgproc.polylines(
      destination,
      List.of(dstPointsd),
      true,
      Scalar(0.0, 0.0, 255.0),
      thickness,
      Imgproc.LINE_AA
    )
    val textPt = Imgproc.boundingRect(dstPoints).tl()
    textPt.x -= 10.0 * scaleX
    textPt.y -= 12.0 * scaleY
    Imgproc.putText(
      destination, id.toString(),
      textPt, Imgproc.FONT_HERSHEY_PLAIN, 1.5 * minScale, Scalar(0.0, 0.0, 255.0),
      thickness, Imgproc.LINE_AA
    )
  }

  init {
    try {
      CameraServerCvJNI.forceLoad()
    } catch (e: Exception) {
      throw RuntimeException("Failed to load native libraries!", e)
    }

    // create Mats of 8x8 apriltag images
    for (i in 0 until kNumTags16h5) {
      val tagImage = get16h5TagImage(i)
      kTag16h5Images[i] = tagImage
    }
    kTag16h5MarkerPts = get16h5MarkerPts()
  }
}
