package org.shockwave.subsystem.swerve

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkBase
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import java.util.*
import java.util.concurrent.ArrayBlockingQueue
import java.util.function.DoubleSupplier

object SparkOdometryThread {
  private val sparks = arrayListOf<SparkBase>()
  private val sparkSignals = arrayListOf<DoubleSupplier>()
  private val genericSignals = arrayListOf<DoubleSupplier>()
  private val sparkQueues = arrayListOf<Queue<Double>>()
  private val genericQueues = arrayListOf<Queue<Double>>()
  private val timestampQueues = arrayListOf<Queue<Double>>()

  private val notifier = Notifier { this.run() }

  init {
    notifier.setName("OdometryThread")
  }

  fun start() {
    if (timestampQueues.isNotEmpty()) {
      notifier.startPeriodic(1.0 / SwerveConstants.ODOMETRY_FREQUENCY)
    }
  }

  /** Registers a Spark signal to be read from the thread.  */
  fun registerSignal(spark: SparkBase, signal: DoubleSupplier): Queue<Double> {
    val queue = ArrayBlockingQueue<Double>(20)
    SwerveSubsystem.ODOMETRY_LOCK.lock()
    try {
      sparks.add(spark)
      sparkSignals.add(signal)
      sparkQueues.add(queue)
    } finally {
      SwerveSubsystem.ODOMETRY_LOCK.unlock()
    }
    return queue
  }

  /** Registers a generic signal to be read from the thread.  */
  fun registerSignal(signal: DoubleSupplier): Queue<Double> {
    val queue = ArrayBlockingQueue<Double>(20)
    SwerveSubsystem.ODOMETRY_LOCK.lock()
    try {
      genericSignals.add(signal)
      genericQueues.add(queue)
    } finally {
      SwerveSubsystem.ODOMETRY_LOCK.unlock()
    }
    return queue
  }

  /** Returns a new queue that returns timestamp values for each sample.  */
  fun makeTimestampQueue(): Queue<Double> {
    val queue = ArrayBlockingQueue<Double>(20)
    SwerveSubsystem.ODOMETRY_LOCK.lock()
    try {
      timestampQueues.add(queue)
    } finally {
      SwerveSubsystem.ODOMETRY_LOCK.unlock()
    }
    return queue
  }

  private fun run() {
    // Save new data to queues
    SwerveSubsystem.ODOMETRY_LOCK.lock()
    try {
      // Get sample timestamp
      val timestamp = RobotController.getFPGATime() / 1e6

      // Read Spark values, mark invalid in case of error
      val sparkValues = DoubleArray(sparkSignals.size)
      var isValid = true
      for (i in sparkSignals.indices) {
        sparkValues[i] = sparkSignals[i].asDouble
        if (sparks[i].lastError != REVLibError.kOk) {
          isValid = false
        }
      }

      // If valid, add values to queues
      if (isValid) {
        for (i in sparkSignals.indices) {
          sparkQueues[i].offer(sparkValues[i])
        }
        for (i in genericSignals.indices) {
          genericQueues[i].offer(genericSignals[i].asDouble)
        }
        for (i in timestampQueues.indices) {
          timestampQueues[i].offer(timestamp)
        }
      }
    } finally {
      SwerveSubsystem.ODOMETRY_LOCK.unlock()
    }
  }
}