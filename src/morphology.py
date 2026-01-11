class Morphology:
    def __init__(self,num_legs :int, leg_length :float, body_radius :str, symmetry :bool=True):
        self.num_legs = num_legs
        self.leg_length = leg_length
        self.body_shape = body_radius
        self.symmetry = symmetry
    """
    Represents the geometric and structural design of a robot.
    This is a parameterized design object, not a CAD model.
    """

    def parameter_vector(self):
        """Return parameters as a vector for optimization"""
        return [
            self.num_legs,
            self.leg_length,
            self.body_radius,
            int(self.symmetry)]