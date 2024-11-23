package org.shockwave.utils

data class PIDFGains(val p: Double, val i: Double, val d: Double, val ff: Double) {
  constructor(p: Double, i: Double, d: Double) : this(p, i, d, 0.0)
}