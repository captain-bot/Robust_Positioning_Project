import numpy as np
import math

premult_ikfast = np.array([[0.7071, 0.7071, 0, -0.1727],
                           [-0.7071, 0.7071, 0, -0.1379],
                           [0, 0, 1.0, -0.1186],
                           [0, 0, 0, 1]])

ee_pose = np.array([[0.3916, 0.0628, 0.9180, 1.0820],
                    [0.1374, -0.9905, 0.0091, 0.3150],
                    [0.9098, 0.1225, -0.3964, 0.1890],
                    [0, 0, 0, 1.0000]])

result = np.dot(premult_ikfast, ee_pose)
print(result)
print(np.sqrt(math.pow(0.3740, 2)+math.pow(-0.1797, 2)+math.pow(0.9098, 2)))
