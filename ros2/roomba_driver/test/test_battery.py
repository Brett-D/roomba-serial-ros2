import pytest

from roomba_driver.battery import li_ion_fraction


def test_limits():
    assert li_ion_fraction(2.5) == 0.0
    assert li_ion_fraction(3.0) == 0.0
    assert li_ion_fraction(4.2) == pytest.approx(1.0)
    assert li_ion_fraction(4.35) == 1.0


def test_interpolates_and_never_decreases():
    assert li_ion_fraction(3.79) == pytest.approx(0.5)
    assert li_ion_fraction(3.77) == pytest.approx(0.45)
    values = [li_ion_fraction(2.9 + i * 0.005) for i in range(300)]
    assert all(b >= a for a, b in zip(values, values[1:]))


def test_five_cell_pack():
    # An 18 V (5S) power-tool pack read 20.26 V at rest shortly after charging.
    assert 0.8 < li_ion_fraction(20.26 / 5) < 0.9
    assert li_ion_fraction(15.0 / 5) == 0.0


def test_nan():
    assert li_ion_fraction(float('nan')) != li_ion_fraction(float('nan'))
