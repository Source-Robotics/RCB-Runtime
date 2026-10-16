"""DH model definition for the SKAR robot arm (Source Robotics)."""

import numpy as np
import utility.quiet_imports  # noqa: F401,E402  # first: imports roboticstoolbox without NumPy's 1.x stderr block
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import time


class SKAR(DHRobot):
    """
    Class that models a Source Robotics PAR6 manipulator with dynamic gripper switching.

    Defined joint configurations:
    - qz: Zero joint angle configuration
    - qr: Arm horizontal along x-axis

    .. note::
        - SI units are used.
        - Gripper can be changed after initialization.

    .. codeauthor:: Petar Crnjak
    """

    # Default gripper is flange
    def __init__(self, symbolic=False, gripper_name="Flange"):
        if symbolic:
            import spatialmath.base.symbolic as sym
            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi
            zero = 0.0

        deg = pi / 180
        inch = 0.0254

        # Robot length values (meters)
        a0 = 100 / 1000
        a1 = 110 / 1000
        a2 = 110 / 1000
        a3 = 110 / 1000
        a4 = 20 / 1000
 

        # DH parameters in radians
        alpha_DH = [0,0,0,0]

        # Mass data (kg)
        mass = [1993.14 / 1000, 1332.24 / 1000, 685.11 / 1000, 579.56 / 1000]
        mbase = 0.410 # Mass of the base (in kg)

        # Center of mass (mm)
        center_of_mass_mm = np.array([
            [-17.91, 3.24, 17.97],
            [-102.85, 11.08, -15.88],
            [33.92, -3.74, -22.19],
            [-0.09, -65.57, -17.13],
        ])

        # Center of mass (meters)
        center_of_mass = center_of_mass_mm / 1000

        # Actuator inertia kg⋅m2
        Jm = [0.0000102, 0.0000102, 0.0000057, 0.0000057]

        # actuator viscous friction coefficient (measured at the motor)
        # 	N·m·s/rad
        Bc = [1e-4, 1e-4, 1e-4, 1e-4]

        # actuator Coulomb friction coefficient for
        # direction [-,+] (measured at the motor) [Nm]
        Tc = [
            [0.02, -0.02],
            [0.02, -0.02],
            [0.02, -0.02],
            [0.02, -0.02],
        ]

        # Gear ratios
        GearRatio = [6,6,6,6]

        # Joint limits #TODO
        qlim = [
            [-160 * deg, 160 * deg],
            [-160 * deg, 160 * deg],
            [-160 * deg, 160 * deg],
            [-160 * deg, 160 * deg],
        ]

        # inertia tensor of link with respect to
        # center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
        # Inertia tensor (g·mm²)
        In_gmm2 = np.array([
            [12745988.34, 13519754.43, 2549480.10, 56669.03, -269843.18, -2478897.85],
            [3221953.71, 6447953.78, 3824316.87, -90782.24, 256719.53, -298551.95],
            [610221.64, 856820.16, 776469.55, -25285.64, -56989.24, 61225.38],
            [1170465.61, 358425.03, 1040586.89, 1081.45, -287452.28, 124.38],

        ])
        # Inertia tensor (kg·m²)
        In = In_gmm2 * 1e-9

        # Define core robot links (first 5 joints)
        L = [
            PrismaticDH(
                theta=a0, a=0, alpha=alpha_DH[0], I=In[0], r=center_of_mass[0], m=mass[0],
                Jm=Jm[0], G=GearRatio[0], B=Bc[0], Tc=Tc[0], qlim=qlim[0]
            ),
            RevoluteDH(
                d=0, a=a1, alpha=alpha_DH[1], I=In[1], r=center_of_mass[1], m=mass[1],
                Jm=Jm[1], G=GearRatio[1], B=Bc[1], Tc=Tc[1], qlim=qlim[1]
            ),
            RevoluteDH(
                d=0, a=a2, alpha=alpha_DH[2], I=In[2], r=center_of_mass[2], m=mass[2],
                Jm=Jm[2], G=GearRatio[2], B=Bc[2], Tc=Tc[2], qlim=qlim[2]
            )
        ]

        # Initialize grippers
        self._grippers = self._define_grippers(symbolic=symbolic)

        # Add initial gripper / flange
        initial_gripper = self._grippers.get(gripper_name, self._grippers["Flange"])
        L.extend(initial_gripper)

        # Initialize the robot
        super().__init__(
            L,
            name="SKAR",
            manufacturer="Source Robotics",
            keywords=("dynamics", "symbolic", "mesh"),
            symbolic=symbolic
        )

        # Configurations
        self.qr = np.array([0, pi, -pi/2, 0]) # robot arm horizontal (max reach and motor load)
        self.qs = np.array([0, -pi/2, pi, 0]) # robot arm standby pose
        self.qz = np.zeros(4) # zero joint angle configuration
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qs", self.qz)
        self.addconfiguration("qz", self.qz)

    def _define_grippers(self, symbolic=False):
        from math import pi
        deg = pi / 180

        grippers = {}

        # SSG48 gripper
        grippers["SSG48"] = [
            RevoluteDH(
                d=0,
                a=0,
                alpha=0,
                m=414.84 / 1000,
                r=[0.01 / 1000, -0.12 / 1000, -37.21 / 1000],
                I=[326186.68 * 1e-9, 225190.25 * 1e-9, 285406.41 * 1e-9, -391.70 * 1e-9, -1208.93 * 1e-9, -299.82 * 1e-9],
                Jm=0.0000015,
                G=10,
                B=1e-4,
                Tc=[0.02, -0.02],
                qlim=[-160 * deg, 160 * deg]
            )
        ]

        # Robot flange
        grippers["Flange"] = [
            RevoluteDH(
                d= -0.1, #TODO: check this value
                a=0.1,
                alpha=0,
                m=0.0555,
                r=[0, 0, 0.0],
                I=[0, 0, 0, 0, 0, 0],
                Jm=0.0000015,
                G=10,
                B=1e-4,
                Tc=[0.02, -0.02],
                qlim=[-160 * deg, 160 * deg]
            )
        ]

        # MHZ2-10D pneumatic gripper
        # TODO edit dimensions
        grippers["MHZ2-10D"] = [
            RevoluteDH(
                d=-62.8 / 1000,
                a=-45.25 / 1000,
                alpha=0,
                m=0.195,
                r=[0, 0, 0.03], #TODO
                I=[0,0,0,0,0,0], #TODO
                Jm=0.0000015,
                G=10,
                B=1e-4,
                Tc=[0.02, -0.02],
                qlim=[-160 * deg, 160 * deg]
            )
        ]

        return grippers

def print_robot_dynamics(robot):
    for i, link in enumerate(robot.links):
        print(f"\n--- Link {i+1} ---")
        print(f"Mass (kg): {link.m}")
        print(f"Center of Mass (r, in meters): {link.r}")
        print(f"Inertia Tensor (kg·m²): {link.I}")
        print(f"Motor Inertia (Jm): {link.Jm}")
        print(f"Gear Ratio (G): {link.G}")
        print(f"Viscous Friction (B): {link.B}")
        print(f"Coulomb Friction (Tc): {link.Tc}")
        print(f"Joint Limits (qlim): {link.qlim}")
        print(f"DH params: a={link.a}, d={link.d}, alpha={link.alpha}")

if __name__ == "__main__":
    # Initialize robot with default gripper
    par6 = SKAR(symbolic=False, gripper_name="Flange")
    print("Initial configuration (Flange):")
    print(par6.dynamics_list())
    print(par6)
    print("flange")
    rne_test = par6.rne(q=par6.qs,qd=[0,0,0,0], qdd=[0,0,0,0])
    print(rne_test)

    """
    par6 = SKAR(symbolic=False, gripper_name="SSG48")
    print("\nAfter switching to SSG48:")
    print(par6.dynamics_list())
    print(par6)
    print("ssg48")
    rne_test = par6.rne(q=par6.qs,qd=[0,0,0,0], qdd=[0,0,0,0])
    print(rne_test)
    robotplot = par6.plot(par6.qs)
    """
    # T = I * Kt 
    # I = T / Kt 2.57 Nm /20 Nm/A = 0.1285 Nm/A 
    # 0.1285 Nm/A / 0.26 Nm/A = 0.493 Nm/A
    # 2.72081574e+00 / 18.09 = 0.1504 Nm/A
    # 0.1504 Nm/A / 0.179 Nm/A = 0.839 Nm/A

    


# Payload has an error where it replaces the mass of the last link
# NOTE Payload will be added to the COM of the end effector / last link
# NOTE Setting payload(0) will remove the mass from end effector / last link
"""
robot.payload(m = (payload + m1), p = [-lc1, 0, 0]) 
print(f"gravload is",robot.gravload([0]))
robot.payload(m = m1, p = [-lc1, 0, 0])
print(f"gravload is",robot.gravload([0]))
"""