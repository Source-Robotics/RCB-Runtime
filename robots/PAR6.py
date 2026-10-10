"""DH model definition for the PAR6 6-axis robot arm (Source Robotics)."""

import os

import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
import time


def _software_joint_limits():
    """[[lo, hi], ...] per joint, from this robot's XML (the one source of joint limits)."""
    import sys
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if root not in sys.path:            # `python3 robots/PAR6.py` run directly
        sys.path.insert(0, root)
    from config.xml_parser import load_software_joint_limits
    lo, hi = load_software_joint_limits(os.path.join(os.path.dirname(__file__), "PAR6.xml"))
    if np.any(np.isnan(lo)) or np.any(np.isnan(hi)):
        raise ValueError("robots/PAR6.xml must declare software_joint_limit_negative/positive "
                         "for every joint")
    return [[float(a), float(b)] for a, b in zip(lo, hi)]


class PAR6(DHRobot):
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
        a1 = 110.50 / 1000
        a2 = 23.42 / 1000
        a3 = 180 / 1000
        a4 = 43.5 / 1000
        a5 = 177 / 1000
        a6 = 117.45 / 1000
        a7 = 0 / 1000

        # DH parameters in radians
        alpha_DH = [-pi / 2, pi, pi/2, -pi/2, pi/2, pi]

        # Mass data (kg)
        mass = [1993.14 / 1000, 1332.24 / 1000, 685.11 / 1000, 579.56 / 1000, 523.81 / 1000, 0]
        mbase = 0.410 # Mass of the base (in kg)

        # Center of mass (mm)
        center_of_mass_mm = np.array([
            [-17.91, 3.24, 17.97],
            [-102.85, 11.08, -15.88],
            [33.92, -3.74, -22.19],
            [-0.09, -65.57, -17.13],
            [0.00, -0.24, 3.55],
            [0, 0, 0],  # Flange COM is zero
        ])

        # Center of mass (meters)
        center_of_mass = center_of_mass_mm / 1000

        # Actuator inertia kg⋅m2
        Jm = [0.0000102, 0.0000102, 0.0000057, 0.0000057, 0.0000057, 0.0000015]

        # actuator viscous friction coefficient (measured at the motor)
        # 	N·m·s/rad
        Bc = [1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4]

        # actuator Coulomb friction coefficient for
        # direction [-,+] (measured at the motor) [Nm]
        Tc = [
            [0.02, -0.02],
            [0.02, -0.02],
            [0.02, -0.02],
            [0.02, -0.02],
            [0.02, -0.02],
            [0.02, -0.02],
        ]

        # Gear ratios
        # In this case gear ratio is negative which means that motor controllers sense
        # of positive rotation is the oposite to the kinematic convention for the robot.
        # J2 was 20 here while robots/PAR6.xml says 25 (found 2026-10-01; only the rotor term Jm*G^2 of the
        # inertia uses it — gravity does not). test_joint_limit_config.py now checks |G| against the XML.
        GearRatio = [6.4, 25, -20*(38/42), -4, -4, -10]
        
        # Joint limits = the SOFTWARE limits in robots/PAR6.xml, read from the file rather than
        # copied here, so the planner and IK can never disagree with RTI and JOG. They used to be
        # a hand-typed copy of those values (2026-09-30).
        qlim = _software_joint_limits()

        # inertia tensor of link with respect to
        # center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
        # Inertia tensor (g·mm²)
        In_gmm2 = np.array([
            [12745988.34, 13519754.43, 2549480.10, 56669.03, -269843.18, -2478897.85],
            [3221953.71, 6447953.78, 3824316.87, -90782.24, 256719.53, -298551.95],
            [610221.64, 856820.16, 776469.55, -25285.64, -56989.24, 61225.38],
            [1170465.61, 358425.03, 1040586.89, 1081.45, -287452.28, 124.38],
            [413791.07, 409560.68, 182591.11, 26.13, -945.93, -35.57],
            [0,0,0,0,0,0], # Flange inertia is zero
        ])
        # Inertia tensor (kg·m²)
        In = In_gmm2 * 1e-9

        # Define core robot links (first 5 joints)
        L = [
            RevoluteDH(
                d=a1, a=a2, alpha=alpha_DH[0], I=In[0], r=center_of_mass[0], m=mass[0],
                Jm=Jm[0], G=GearRatio[0], B=Bc[0], Tc=Tc[0], qlim=qlim[0]
            ),
            RevoluteDH(
                d=0, a=a3, alpha=alpha_DH[1], I=In[1], r=center_of_mass[1], m=mass[1],
                Jm=Jm[1], G=GearRatio[1], B=Bc[1], Tc=Tc[1], qlim=qlim[1]
            ),
            RevoluteDH(
                d=0, a=-a4, alpha=alpha_DH[2], I=In[2], r=center_of_mass[2], m=mass[2],
                Jm=Jm[2], G=GearRatio[2], B=Bc[2], Tc=Tc[2], qlim=qlim[2]
            ),
            RevoluteDH(
                d=-a5, a=0, alpha=alpha_DH[3], I=In[3], r=center_of_mass[3], m=mass[3],
                Jm=Jm[3], G=GearRatio[3], B=Bc[3], Tc=Tc[3], qlim=qlim[3]
            ),
            RevoluteDH(
                d=0, a=0, alpha=alpha_DH[4], I=In[4], r=center_of_mass[4], m=mass[4],
                Jm=Jm[4], G=GearRatio[4], B=Bc[4], Tc=Tc[4], qlim=qlim[4]
            )
        ]

        # Initialize grippers
        self._grippers = self._define_grippers(symbolic=symbolic)

        # Add initial gripper / flange. Gripper XML supplies the tool-link geometry.
        initial_gripper = self._grippers.get(gripper_name, self._grippers["Flange"])
        L.extend(initial_gripper)
        # J6's limit is tool-dependent in principle (a gripper cable limits wrist rotation), and
        # every gripper XML carries a <qlim_deg>. But those values are unverified — +/-160 deg in
        # every tool file, centred on 0 while the arm's own J6 range is centred near 180 — and only
        # the planner/IK ever read them, while RTI and JOG used the robot XML. Until real per-tool
        # values exist, J6 uses the robot XML range for every tool, the same as RTI and JOG
        # (2026-09-30; TODO/safety_stack_research.md).
        L[5].qlim = qlim[5]

        # Initialize the robot
        super().__init__(
            L,
            name="PAR6",
            manufacturer="Source Robotics",
            keywords=("dynamics", "symbolic", "mesh"),
            symbolic=symbolic
        )

        # Configurations
        self.qr = np.array([0, pi, -pi/2, 0, 0, 0]) # robot arm horizontal (max reach and motor load)
        self.qs = np.array([0, -pi/2, pi, 0, 0, pi]) # robot arm standby pose
        self.qz = np.zeros(6) # zero joint angle configuration
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qs", self.qz)
        self.addconfiguration("qz", self.qz)

    def _define_grippers(self, symbolic=False):
        import os
        import glob
        import xml.etree.ElementTree as ET
        from math import pi

        deg = pi / 180
        grippers = {}
        gripper_dir = os.path.join(os.path.dirname(__file__), "..", "grippers")

        for xml_path in sorted(glob.glob(os.path.join(gripper_dir, "*.xml"))):
            try:
                root = ET.parse(xml_path).getroot()
            except ET.ParseError:
                continue
            kin = root.find("kinematics")
            if kin is None:
                continue
            name = root.get("name")
            if not name:
                continue
            d     = float(kin.findtext("d"))
            a     = float(kin.findtext("a", default="0"))
            alpha = float(kin.findtext("alpha", default=str(pi)))
            m     = float(kin.findtext("mass"))
            r     = [float(x) for x in kin.findtext("com").split()]
            I     = [float(x) for x in kin.findtext("inertia").split()]
            Jm    = float(kin.findtext("Jm", default="1.5e-6"))
            G     = float(kin.findtext("G",  default="10"))
            B     = float(kin.findtext("B",  default="1e-4"))
            Tc    = [float(x) for x in kin.findtext("Tc", default="0.02 -0.02").split()]
            qlim  = [float(x) * deg for x in kin.findtext("qlim_deg").split()]
            grippers[name] = [
                RevoluteDH(d=d, a=a, alpha=alpha, m=m, r=r, I=I,
                           Jm=Jm, G=G, B=B, Tc=Tc, qlim=qlim)
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
    par6 = PAR6(symbolic=False, gripper_name="Flange")
    print("Initial configuration (Flange):")
    print(par6.dynamics_list())
    print(par6)
    print("flange")
    rne_test = par6.rne(q=par6.qs,qd=[0,0,0,0,0,0], qdd=[0,0,0,0,0,0])
    print(rne_test)

    par6 = PAR6(symbolic=False, gripper_name="SSG48")
    print("\nAfter switching to SSG48:")
    print(par6.dynamics_list())
    print(par6)
    print("ssg48")
    rne_test = par6.rne(q=par6.qs,qd=[0,0,0,0,0,0], qdd=[0,0,0,0,0,0])
    print(rne_test)
    robotplot = par6.plot(par6.qs)

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