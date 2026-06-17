"""Shared gravity model backend (KDL with Pinocchio fallback)."""

from typing import List, Optional, Sequence, Tuple


class RobotModelBase:
    def compute(
        self,
        q_rad: Sequence[float],
        v_rad_s: Optional[Sequence[float]] = None,
        a_rad_s2: Optional[Sequence[float]] = None,
    ) -> List[float]:
        raise NotImplementedError


class RobotModelKDL(RobotModelBase):
    def __init__(self, urdf_path: str, root_link: str, tip_link: str, joint_count: int):
        try:
            from urdf_parser_py.urdf import URDF  # type: ignore
            import PyKDL as kdl  # type: ignore
            from kdl_parser_py.urdf import treeFromUrdfModel  # type: ignore
        except Exception as exc:
            raise ImportError(
                "PyKDL + urdf_parser_py + kdl_parser_py are required for KDL gravity."
            ) from exc

        urdf_model = URDF.from_xml_file(urdf_path)
        ok, tree = treeFromUrdfModel(urdf_model)
        if not ok:
            raise RuntimeError(f"Failed to build KDL tree from URDF: {urdf_path}")
        chain = tree.getChain(root_link, tip_link)
        if chain.getNrOfJoints() < joint_count:
            raise RuntimeError("KDL chain has fewer joints than expected")

        self._chain = chain
        self._q = kdl.JntArray(chain.getNrOfJoints())
        self._g = kdl.JntArray(chain.getNrOfJoints())
        self._dyn = kdl.ChainDynParam(chain, kdl.Vector(0.0, 0.0, -9.81))
        self._joint_count = joint_count

    def compute(
        self,
        q_rad: Sequence[float],
        v_rad_s: Optional[Sequence[float]] = None,
        a_rad_s2: Optional[Sequence[float]] = None,
    ) -> List[float]:
        if v_rad_s is not None or a_rad_s2 is not None:
            raise RuntimeError("KDL backend supports gravity only (v/a must be None).")
        if len(q_rad) < self._joint_count:
            raise RuntimeError("Insufficient joint positions provided to gravity solver")
        for i in range(self._chain.getNrOfJoints()):
            self._q[i] = q_rad[i] if i < len(q_rad) else 0.0
        if self._dyn.JntToGravity(self._q, self._g) < 0:
            raise RuntimeError("KDL JntToGravity failed")
        return [float(self._g[i]) for i in range(self._joint_count)]


class RobotModelPinocchio(RobotModelBase):
    def __init__(self, urdf_path: str, joint_names: Sequence[str]):
        try:
            import pinocchio as pin  # type: ignore
            import numpy as np  # type: ignore
        except Exception as exc:
            raise ImportError("Pinocchio is required for gravity fallback") from exc

        self._pin = pin
        self._np = np
        self._model = pin.buildModelFromUrdf(urdf_path)
        self._data = self._model.createData()
        self._joint_names = list(joint_names)

    def compute(
        self,
        q_rad: Sequence[float],
        v_rad_s: Optional[Sequence[float]] = None,
        a_rad_s2: Optional[Sequence[float]] = None,
    ) -> List[float]:
        q = self._np.zeros(self._model.nq)
        v = self._np.zeros(self._model.nv)
        a = self._np.zeros(self._model.nv)
        for idx, name in enumerate(self._joint_names):
            jid = self._model.getJointId(name)
            if jid == 0:
                raise RuntimeError(f"Joint '{name}' not found in URDF model.")
            q_idx = self._model.joints[jid].idx_q
            v_idx = self._model.joints[jid].idx_v
            q[q_idx] = q_rad[idx]
            if v_rad_s is not None:
                v[v_idx] = v_rad_s[idx]
            if a_rad_s2 is not None:
                a[v_idx] = a_rad_s2[idx]

        tau = self._pin.rnea(self._model, self._data, q, v, a)
        out: List[float] = []
        for name in self._joint_names:
            jid = self._model.getJointId(name)
            v_idx = self._model.joints[jid].idx_v
            out.append(float(tau[v_idx]))
        return out


def build_robot_model(
    urdf_path: str,
    root_link: str,
    tip_link: str,
    joint_names: Sequence[str],
    backend: str = "auto",
) -> Tuple[RobotModelBase, str]:
    joint_count = len(joint_names)
    backend = backend.lower().strip()

    if backend == "kdl":
        model = RobotModelKDL(urdf_path, root_link, tip_link, joint_count)
        return model, "KDL"
    if backend == "pinocchio":
        model = RobotModelPinocchio(urdf_path, joint_names)
        return model, "Pinocchio"
    if backend != "auto":
        raise ValueError(f"Unknown backend: {backend}")

    try:
        model = RobotModelKDL(urdf_path, root_link, tip_link, joint_count)
        return model, "KDL"
    except Exception as exc:
        kdl_err = str(exc)

    model = RobotModelPinocchio(urdf_path, joint_names)
    return model, f"Pinocchio (KDL unavailable: {kdl_err})"
