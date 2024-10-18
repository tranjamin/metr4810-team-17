from math import pi

def wrapToPi(angle: float) -> float:
    '''
    Wraps an angle to the range -pi/2, pi/2

    Parameters:
        angle (float): the angle, in radians
    Returns:
        (float): the wrapped angle, in radians
    '''
    return (angle + pi) % (2 * pi) - pi
