import unittest
from inspection_logic import accumulate_exposure, alarm_gas


class ExposureTests(unittest.TestCase):
    def test_integrates_trapezoid(self):
        self.assertAlmostEqual(accumulate_exposure(1.,1.2,10.,30.,5.),9.)

    def test_paused_simulation_does_not_accumulate(self):
        self.assertEqual(accumulate_exposure(2.,2.,80.,80.,4.),4.)

    def test_reset_clock_resets_dose(self):
        self.assertEqual(accumulate_exposure(20.,.1,80.,80.,40.),0.)

    def test_missing_interval_is_not_invented(self):
        self.assertEqual(accumulate_exposure(2.,6.,80.,80.,4.),4.)

    def test_concentration_and_dose_independently_trigger(self):
        cfg={'gas_alarm_ppm':50.,'exposure_alarm_ppm_s':200.}
        self.assertFalse(alarm_gas(49.,199.,cfg))
        self.assertTrue(alarm_gas(50.,0.,cfg))
        self.assertTrue(alarm_gas(0.,200.,cfg))


if __name__=='__main__':unittest.main()
