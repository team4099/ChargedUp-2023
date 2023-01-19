package com.team4099.lib.sim.vision

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Rotation2d
import java.util.Arrays
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.Num
import edu.wpi.first.math.Nat
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d

object MathUtils {
  fun calcAvg(vararg values: Pose3d): Pose3d {
    if (values.size == 0) return Pose3d()
    val trlXVals = DoubleArray(values.size)
    val trlYVals = DoubleArray(values.size)
    val trlZVals = DoubleArray(values.size)
    val rotXCosVals = DoubleArray(values.size)
    val rotXSinVals = DoubleArray(values.size)
    val rotYCosVals = DoubleArray(values.size)
    val rotYSinVals = DoubleArray(values.size)
    val rotZCosVals = DoubleArray(values.size)
    val rotZSinVals = DoubleArray(values.size)
    for (i in 0 until values.size) {
      val v = values[i]
      trlXVals[i] = v.x
      trlYVals[i] = v.y
      trlZVals[i] = v.z
      rotXCosVals[i] = Math.cos(v.rotation.x)
      rotXSinVals[i] = Math.sin(v.rotation.x)
      rotYCosVals[i] = Math.cos(v.rotation.y)
      rotYSinVals[i] = Math.sin(v.rotation.y)
      rotZCosVals[i] = Math.cos(v.rotation.z)
      rotZSinVals[i] = Math.sin(v.rotation.z)
    }
    return Pose3d(
      calcAvg(*trlXVals),
      calcAvg(*trlYVals),
      calcAvg(*trlZVals),
      Rotation3d(
        Rotation2d(calcAvg(*rotXCosVals), calcAvg(*rotXSinVals)).radians,
        Rotation2d(calcAvg(*rotYCosVals), calcAvg(*rotYSinVals)).radians,
        Rotation2d(calcAvg(*rotZCosVals), calcAvg(*rotZSinVals)).radians
      )
    )
  }

  fun calcAvg(vararg values: Double): Double {
    return if (values.size == 0) 0.0 else Arrays.stream(values).average().asDouble
  }

  fun calcStdDev(avgPose: Pose3d, vararg values: Pose3d): Pose3d {
    if (values.size == 0) return Pose3d()
    val trlXVals = DoubleArray(values.size)
    val trlYVals = DoubleArray(values.size)
    val trlZVals = DoubleArray(values.size)
    var rotXDev = 0.0
    var rotYDev = 0.0
    var rotZDev = 0.0
    for (i in 0 until values.size) {
      val v = values[i]
      trlXVals[i] = v.x
      trlYVals[i] = v.y
      trlZVals[i] = v.z
      rotXDev += Math.pow(
        MathUtil.angleModulus(v.rotation.x - avgPose.rotation.x), 2.0
      )
      rotYDev += Math.pow(
        MathUtil.angleModulus(v.rotation.y - avgPose.rotation.y), 2.0
      )
      rotZDev += Math.pow(
        MathUtil.angleModulus(v.rotation.z - avgPose.rotation.z), 2.0
      )
    }
    rotXDev /= values.size.toDouble()
    rotXDev = Math.sqrt(rotXDev)
    rotYDev /= values.size.toDouble()
    rotYDev = Math.sqrt(rotYDev)
    rotZDev /= values.size.toDouble()
    rotZDev = Math.sqrt(rotZDev)
    return Pose3d(
      calcStdDev(avgPose.x, *trlXVals),
      calcStdDev(avgPose.y, *trlYVals),
      calcStdDev(avgPose.z, *trlZVals),
      Rotation3d(rotXDev, rotYDev, rotZDev)
    )
  }

  fun calcStdDev(mean: Double, vararg values: Double): Double {
    if (values.size == 0) return 0.0
    var variance = 0.0
    for (v in values) {
      variance += Math.pow(v - mean, 2.0)
    }
    variance /= values.size.toDouble()
    return Math.sqrt(variance)
  }

  /** Converts this list of translations to a 3xN matrix.  */
  @JvmStatic
  fun translationsToMatrix(trls: List<Translation3d>): Matrix<N3, Num> {
    val C: Nat<Num> = Nat { trls.size }
    val matrix = Matrix(Nat.N3(), C)
    for (i in trls.indices) {
      val trl = trls[i]
      val col = Matrix.mat(Nat.N3(), Nat.N1()).fill(trl.x, trl.y, trl.z)
      matrix.setColumn(i, col)
    }
    return matrix
  }

  /** Subtracts this translation from every column of this 3xN matrix.  */
  @JvmStatic
  fun columnsMinusTrl(matrix: Matrix<N3, Num?>, trl: Translation3d) {
    val col = Matrix.mat(Nat.N3(), Nat.N1()).fill(trl.x, trl.y, trl.z)
    for (i in 0 until matrix.numCols) {
      matrix.setColumn(i, matrix.extractColumnVector(i).minus(col))
    }
  }

  /** Checks if the columns of this matrix, as points, are collinear  */
  @JvmStatic
  fun <R : Num?, C : Num?> isCollinear(matrix: Matrix<R, C>?): Boolean {
    if (matrix == null) return false
    if (matrix.numRows < 2 || matrix.numCols < 3) return true
    val vecA = Vector(matrix.extractColumnVector(0))
    val vecAB = Vector(matrix.extractColumnVector(1).minus(vecA))
    for (i in 2 until matrix.numCols) {
      val vecAC = Vector(matrix.extractColumnVector(i).minus(vecA))
      val `val` = Math.abs(vecAB.dot(vecAC) / (vecAB.norm() * vecAC.norm()))
      if (Math.abs(`val` - 1) > 5e-5) return false
    }
    return true
  }

  /**
   * Finds the bounded center (average of minimum and maximum) of these values.
   */
  fun boundedCenter(vararg values: Double): Double {
    var minX = 0.0
    var maxX = 0.0
    for (i in 0 until values.size) {
      val x = values[i]
      if (i == 0) {
        minX = x
        maxX = x
      } else {
        minX = Math.min(minX, x)
        maxX = Math.max(maxX, x)
      }
    }
    return (minX + maxX) / 2.0
  }

  fun within(value: Int, from: Int, to: Int): Boolean {
    return from <= value && value <= to
  }

  fun within(value: Double, from: Double, to: Double): Boolean {
    return from <= value && value <= to
  }

  fun clamp(value: Int, from: Int, to: Int): Int {
    return Math.max(from, Math.min(value, to))
  }

  fun clamp(value: Double, from: Double, to: Double): Double {
    return Math.max(from, Math.min(value, to))
  }

  /**
   * Linear percentage from start to end
   * @return Percentage (NOT CLAMPED)
   */
  fun percentTo(value: Double, start: Double, end: Double): Double {
    return (value - start) / (end - start)
  }

  /**
   * Linearly interpolates from start to end
   * @return Resulting value (NOT CLAMPED)
   */
  fun lerp(percent: Double, start: Double, end: Double): Double {
    return start + (end - start) * percent
  }

  /**
   * Linearly maps value in [startFrom, startTo] to [endFrom, endTo]
   * @return Resulting value (NOT CLAMPED)
   */
  fun map(value: Double, inStart: Double, inEnd: Double, outStart: Double, outEnd: Double): Double {
    return lerp(percentTo(value, inStart, inEnd), outStart, outEnd)
  }
}
