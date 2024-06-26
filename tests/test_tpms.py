# tests/test_tpms.py
import unittest
from tpms import TPMS


class TestTPMS(unittest.TestCase):
    def test_init_default(self):
        tpms = TPMS()
        self.assertIsNotNone(tpms)
        self.assertEqual(tpms.config["temperature_unit"], "Celsius")
        self.assertEqual(tpms.config["pressure_unit"], "kPa")

    def test_init_custom(self):
        tpms = TPMS(temp_unit="Fahrenheit", pressure_unit="psi")
        self.assertEqual(tpms.config["temperature_unit"], "Fahrenheit")
        self.assertEqual(tpms.config["pressure_unit"], "psi")

    def test_decode_tire_status(self):
        tpms = TPMS()
        frame = bytearray([85, 170, 8, 0, 100, 60, 0])
        event = tpms.decode_tire_status(frame)
        self.assertEqual(event["pressure"], 344)
        self.assertEqual(event["temperature"], 10)


if __name__ == "__main__":
    unittest.main()
