package org.shockwave.utils

class TunablePIDF(prefix: String, default: PIDFGains) {
  private val isManualMode = TunableBoolean("$prefix(1) ManualMode", false)
  private val manualValue = TunableNumber("$prefix(2) ManualValue", 0.0)
  private val p = TunableNumber("$prefix(3) P", default.p)
  private val i = TunableNumber("$prefix(4) I", default.i)
  private val d = TunableNumber("$prefix(5) D", default.d)
  private val ff = TunableNumber("$prefix(6) FF", default.ff)

  fun periodic(pidfConfigurator: (PIDFGains) -> Unit, manualValueSetter: (Double) -> Unit) {
    TunableNumber.ifChanged(hashCode(), { values ->
      pidfConfigurator(PIDFGains(values[0], values[1], values[2], values[3]))
    }, p, i, d, ff)

    if (isManualMode.get()) {
      TunableNumber.ifChanged(2 * hashCode(), {
        manualValueSetter(it[0])
      }, manualValue)
    }
  }

  fun isManualMode() = isManualMode.get()
}