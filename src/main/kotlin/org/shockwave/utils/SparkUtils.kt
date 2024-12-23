package org.shockwave.utils

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkBase
import java.util.function.Consumer
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import java.util.function.Function

var SPARK_STICKY_FAULT: Boolean = false

fun SparkBase.ifOk(supplier: DoubleSupplier, consumer: DoubleConsumer) {
  val value = supplier.asDouble
  if (lastError == REVLibError.kOk) {
    consumer.accept(value)
  } else {
    SPARK_STICKY_FAULT = true
  }
}

fun SparkBase.ifOk(suppliers: Array<DoubleSupplier>, consumer: Consumer<DoubleArray>) {
  val values = DoubleArray(suppliers.size)
  for (i in suppliers.indices) {
    values[i] = suppliers[i].asDouble
    if (lastError != REVLibError.kOk) {
      SPARK_STICKY_FAULT = true
      return
    }
  }
  consumer.accept(values)
}

fun SparkBase.tryUntilOk(maxAttempts: Int, command: Function<SparkBase, REVLibError>) {
  for (i in 0..<maxAttempts) {
    val error = command.apply(this)
    if (error == REVLibError.kOk) {
      break
    } else {
      SPARK_STICKY_FAULT = true
    }
  }
}
