"""State of charge from pack voltage, for batteries the Roomba can't gauge itself."""

# Resting voltage of one Li-ion (NMC) cell -> state of charge, typical of 18650 power-tool cells.
# Good to roughly +-10 % with the reading smoothed and corrected for sag under load.
LI_ION_CURVE = (
    (3.00, 0.00), (3.45, 0.05), (3.60, 0.10), (3.67, 0.20), (3.71, 0.30), (3.75, 0.40),
    (3.79, 0.50), (3.85, 0.60), (3.92, 0.70), (4.00, 0.80), (4.10, 0.90), (4.20, 1.00),
)


def li_ion_fraction(cell_voltage):
    """State of charge (0-1) of a Li-ion cell at this resting voltage, linearly interpolated."""
    if cell_voltage != cell_voltage:
        return float('nan')
    if cell_voltage <= LI_ION_CURVE[0][0]:
        return 0.0
    for (v0, f0), (v1, f1) in zip(LI_ION_CURVE, LI_ION_CURVE[1:]):
        if cell_voltage <= v1:
            return f0 + (f1 - f0) * (cell_voltage - v0) / (v1 - v0)
    return 1.0
