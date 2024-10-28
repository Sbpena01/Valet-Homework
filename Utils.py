import numpy as np
def clamp(n, min, max): 
    """Clamps a value between two values. Useful when enforcing a max velocity or steering angle.

    Args:
        n (float): Value to be clamped
        min (float): Minimum value
        max (float): Maximum value

    Returns:
        float: Clamped value
    """
    if n < min: 
        return min
    elif n > max: 
        return max
    else: 
        return n 

def calculateUnitVector(angle: float):
    """Calculates the x and y components of an angle. Used in visualization tool

    Args:
        angle (float): Angle to get the unit vector for

    Returns:
        tuple: The x and y components of the unit vector
    """
    return (np.cos(angle), -np.sin(angle))

def normalize(vec):
    """Normalizes a vector

    Args:
        vec (np.array): Vector to be normalized

    Raises:
        ValueError: If the vector is 0, then do not attempt to normalize

    Returns:
        np.array: The normalized vector
    """
    norm = np.linalg.norm(vec)
    if norm == 0:
        raise ValueError("Cannot normalize a zero vector.")
    ans = vec / norm
    return np.array([ans[0], ans[1]])

def calculateAngle(vec1, vec2):
    """Calculates the angle between two vectors

    Args:
        vec1 (np.array): First vector in the angle
        vec2 (np.array): Second vector in the angle

    Returns:
        float: Angle between both vectors
    """
    norm_vec1 = normalize(vec1)
    norm_vec2 = normalize(vec2)

    num = norm_vec1.dot(norm_vec2)
    denom = np.linalg.norm(norm_vec1) * np.linalg.norm(norm_vec2)

    angle = np.arccos(np.clip(num / denom, -1.0, 1.0))
    
    cross_product = norm_vec1[0] * norm_vec2[1] - norm_vec1[1] * norm_vec2[0]
    if cross_product < 0:
        return -angle
    else:
        return angle
