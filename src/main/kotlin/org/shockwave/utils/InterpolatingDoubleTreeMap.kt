package org.shockwave.utils

import java.util.*

class InterpolatingDoubleTreeMap(data: Map<Double, Double>) {
  private val map = TreeMap<Double, Double>()

  init {
    map.putAll(data)
  }

  fun predict(key: Double): Double {
    if (map.containsKey(key)) return map[key]!!

    val lower = map.floorKey(key)
    val upper = map.ceilingKey(key)

    if (lower == null) return map.firstEntry().value
    if (upper == null) return map.lastEntry().value

    val lowerValue = map[lower]!!
    val upperValue = map[upper]!!
    return lowerValue + (upperValue - lowerValue) * (key - lower) / (upper - lower)
  }
}