import math
import numpy as np

from typing import List, Tuple

def rotation_matrix_to_rpy(R: np.ndarray) -> Tuple:
    """
    Calculate Roll, Pitch, and Yaw angles from a 3x3 rotation matrix.
    Assumes the rotation matrix follows the ZYX rotation order.

    Parameters:
        R (numpy array): 3x3 rotation matrix.

    Returns:
        tuple: Roll, Pitch, and Yaw angles in radians.
    """

    # Ensure the matrix is square and 3x3
    if R.shape != (3, 3):
        raise ValueError("The shape of the rotation matrix must be 3x3.")

    # Calculate Roll
    roll = math.atan2(R[2, 1], R[2, 2])

    # Calculate Pitch
    pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1]**2 + R[2, 2]**2))

    # Calculate Yaw
    yaw = math.atan2(R[1, 0], R[0, 0])

    return roll, pitch, yaw

def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to a 3x3 rotation matrix.

    Args:
        q (numpy array): Quaternion in the form [x, y, z, w].

    Returns:
        numpy array: 3x3 rotation matrix.
    """

    # Extract quaternion components for better readability
    x, y, z, w = q
    
    # Initialize 3x3 rotation matrix
    R = np.zeros((3, 3))
    
    # Populate the matrix according to the conversion formula
    R[0, 0] = 1 - 2 * (y ** 2) - 2 * (z ** 2)
    R[0, 1] = 2 * x * y - 2 * w * z
    R[0, 2] = 2 * x * z + 2 * w * y
    
    R[1, 0] = 2 * x * y + 2 * w * z
    R[1, 1] = 1 - 2 * (x ** 2) - 2 * (z ** 2)
    R[1, 2] = 2 * y * z - 2 * w * x
    
    R[2, 0] = 2 * x * z - 2 * w * y
    R[2, 1] = 2 * y * z + 2 * w * x
    R[2, 2] = 1 - 2 * (x ** 2) - 2 * (y ** 2)
    
    return R

def rpy_to_rotation_matrix(rpy: List) -> List[List]:
    """
    Convert Roll, Pitch, and Yaw angles to a 3x3 rotation matrix.
    Assumes the rotation matrix follows the ZYX rotation order.

    Parameters:
        rpy (list): Roll, Pitch, and Yaw angles in radians.

    Returns:
        list[list]: 3x3 rotation matrix.
    """

    # Ensure the input is a list
    if not isinstance(rpy, list):
        raise TypeError("The input must be a list.")

    # Ensure the list has 3 elements
    if len(rpy) != 3:
        raise ValueError("The list must have 3 elements.")

    # Extract Roll, Pitch, and Yaw angles
    roll, pitch, yaw = rpy

    # Calculate the rotation matrix
    R = np.array([
        [math.cos(yaw) * math.cos(pitch),
         math.cos(yaw) * math.sin(pitch) * math.sin(roll) - math.sin(yaw) * math.cos(roll),
         math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll)],
        [math.sin(yaw) * math.cos(pitch),
         math.sin(yaw) * math.sin(pitch) * math.sin(roll) + math.cos(yaw) * math.cos(roll),
         math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll)],
        [-math.sin(pitch),
         math.cos(pitch) * math.sin(roll),
         math.cos(pitch) * math.cos(roll)]
    ])

    return R

if __name__ == '__main__':
    # tests
    r, p, y = (0.1, 0.2, 0.3)
    print(f"rpy: {r, p, y}")
    R = rpy_to_rotation_matrix([r, p, y])
    print(f"R: {R}")