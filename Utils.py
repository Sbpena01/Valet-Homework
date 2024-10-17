import numpy as np
def clamp(n, min, max): 
    if n < min: 
        return min
    elif n > max: 
        return max
    else: 
        return n 

def calculateUnitVector(angle: float):
    return (np.cos(angle), -np.sin(angle))

def normalize(vec):
    norm = np.linalg.norm(vec)
    if norm == 0:
        raise ValueError("Cannot normalize a zero vector.")
    ans = vec / norm
    return np.array([ans[0], ans[1]])

def calculateAngle(vec1, vec2):
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
