"""Walk sequence: a repeating timeline of base-frame velocity commands, ((seconds, vx, vy, wz), ...)."""


def command_at(t: float, sequence) -> tuple[float, float, float]:
    """Return the [vx, vy, wz] command active at time t; the sequence repeats forever.

    A final step of math.inf seconds (WalkPlan.abort) holds forever: t % inf == t.
    """
    t %= sum(segment[0] for segment in sequence)
    for seconds, vx, vy, wz in sequence:
        if t < seconds:
            return vx, vy, wz
        t -= seconds
    return sequence[-1][1:]
