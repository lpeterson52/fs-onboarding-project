from collections.abc import Callable

def create_drag_function(drag_coefficient: float,
                         reference_area: float,
                         density: float = 1.225) -> Callable[[float], float]:
    """Returns a function from velocity to drag.
    
    Args:
        drag_coefficient: Coefficient of drag in equation.
        density: Air density [kg/m^3].
        reference_area: Reference area of object [m^2].
    
    Returns:
        A lambda that takes in velocity and returns drag.
    """
    return lambda velocity: drag_coefficient * density * reference_area * ((velocity ** 2) / 2)
    