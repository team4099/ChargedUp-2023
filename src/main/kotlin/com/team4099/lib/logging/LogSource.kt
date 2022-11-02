package com.team4099.lib.logging

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget

data class LogSource<T>(
  val tab: String,
  val name: String,
  val supplier: () -> T,
  val shuffleboardWidget: SimpleWidget?,
  val followSupplier: Boolean
)
