from morphology import Morphology

class Robot:
    """
    Represents Robot as an embodied system : morphology + future control+ future fabrication constraints
    """

    def __init__(self, morphology: Morphology):
        self.morphology = morphology

    def describe(self):
        """
        Returns human-readable description of robot design.
        """
        return{
            "num_legs": self.morphology.num_legs,
            "leg_length": self.morphology.leg_length,
            "body_radius": self.morphology.body_radius,
            "symmetry": self.morphology.symmetry

        }
    

if __name__ == "__main__":
    morph = Morphology(
        num_legs= 4,
        leg_length= 0.3,
        body_radius= 0.2,
        symmetry= True
    )

    robot = Robot(morphology=morph)
    print(robot.describe())