import unittest
import numpy as np
from PoliceCar import PoliceCar

test_car = PoliceCar((10,10))

class TestPoliceCar(unittest.TestCase):    
    def test_kinematics(self):
        initial_state = (15, 10, 0, np.pi/12)
        x, y, theta, zeta = test_car.new_kinematics(initial_state, 0.5, 10, np.pi/6)
        self.assertAlmostEqual(x, 19.438479010997447, places=3)
        self.assertAlmostEqual(y, 12.302152051654083, places=3)
        self.assertAlmostEqual(theta, 0.478, places=3)
        self.assertAlmostEqual(zeta, np.pi/6, places=3)

if __name__ == '__main__':
    unittest.main()