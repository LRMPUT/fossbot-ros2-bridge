"""Battery charge estimation. Pure Python so it can be tested without ROS."""

# Resting Li-ion cell voltage -> state of charge. Approximate, but far closer
# than a straight line between full and empty.
LIION_CURVE = ((3.30, 0.00), (3.55, 0.10), (3.65, 0.20), (3.70, 0.30),
               (3.75, 0.40), (3.80, 0.50), (3.85, 0.60), (3.92, 0.70),
               (4.00, 0.80), (4.10, 0.90), (4.20, 1.00))


def liion_fraction(cell_volts):
    """0..1 charge of one Li-ion cell from its voltage."""
    curve = LIION_CURVE
    if cell_volts <= curve[0][0]:
        return 0.0
    for (v0, f0), (v1, f1) in zip(curve, curve[1:]):
        if cell_volts <= v1:
            return f0 + (f1 - f0) * (cell_volts - v0) / (v1 - v0)
    return 1.0


def rail_fraction(volts, empty_v, full_v):
    """0..1 from a regulated supply rail, linear between empty and full.

    Coarse by nature: a regulated rail stays near nominal for most of the
    discharge and only sags once the battery can no longer hold it.
    """
    if full_v <= empty_v:
        raise ValueError("full_v must be above empty_v")
    return max(0.0, min(1.0, (volts - empty_v) / (full_v - empty_v)))


class Smoother:
    """Exponential smoothing with a time constant, robust to uneven sampling.

    Motor current sags the voltage; without this a gauge swings with every
    acceleration.
    """

    def __init__(self, time_constant_s):
        self.tau = max(time_constant_s, 1e-3)
        self.value = None
        self.last_t = None

    def update(self, x, now):
        if self.value is None:
            self.value = x
        else:
            alpha = min(1.0, max(0.0, now - self.last_t) / self.tau)
            self.value += alpha * (x - self.value)
        self.last_t = now
        return self.value
