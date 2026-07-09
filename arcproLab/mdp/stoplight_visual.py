"""Per-env stoplight prims: the camera-visible leg of the V2I intersection.

One stoplight per env clone, built during RoadManager._initialize_gates()
(the stage is already in hand there), placed beside the env's first
discovered laneGate. Three lamp spheres (red/amber/green, top to bottom)
with UsdPreviewSurface emissive materials; the active phase's lamp is
visible, the others invisible. Visibility writes happen only on phase
TRANSITIONS, so per-step cost is zero.

Vision mandate: obs slots 8-9 are masked to force the ResNet to learn from
pixels. This prim is what makes signal phase learnable from pixels at all,
enabling the vision+V2I redundancy study (degrade the radio, watch the
policy fall back to the light it can see).

Pure-visual gprims: no collision APIs are applied, so the vehicle cannot
crash into the pole (intentional for phase A). Lateral offset from the
gate is a GUI-verify tunable -- adjust LATERAL_OFFSET if the pole lands
on the road.

pxr imports live inside methods (house style: RoadManager._initialize_gates
does the same with omni), so this module py_compiles and its sync logic
unit-tests without Isaac.
"""

import re


class StoplightVisual:
    POLE_HEIGHT = 2.5
    LAMP_RADIUS = 0.12
    LATERAL_OFFSET = (1.0, 0.0)  # (x, y) offset from gate position
    COLORS = {
        "red": (1.0, 0.05, 0.05),
        "amber": (1.0, 0.6, 0.05),
        "green": (0.05, 1.0, 0.1),
    }
    LAMP_ORDER = ("red", "amber", "green")  # top to bottom
    PHASE_TO_LAMP = {0: "green", 1: "amber", 2: "red"}

    def __init__(self):
        self._lamps_by_env: dict = {}  # env_idx -> {name: UsdGeom.Imageable}
        self._last_phase: dict = {}

    # ------------------------------ build --------------------------------

    @staticmethod
    def _env_index(prim_path: str) -> int:
        m = re.search(r"env_(\d+)", prim_path)
        return int(m.group(1)) if m else 0

    @staticmethod
    def _env_root(prim_path: str) -> str:
        m = re.match(r"(.*env_\d+)", prim_path)
        return m.group(1) if m else "/World"

    def build(self, stage, gate_records):
        """gate_records: list of (prim_path, (x, y, z)) from gate discovery.

        Builds one stoplight per env (beside that env's first gate).
        """
        for path, pos in gate_records:
            env_idx = self._env_index(path)
            if env_idx in self._lamps_by_env:
                continue  # phase A: one stoplight per env
            base = self._env_root(path) + "/Stoplight"
            self._build_one(stage, base, pos, env_idx)
        if self._lamps_by_env:
            print(f"[StoplightVisual] Built {len(self._lamps_by_env)} stoplights.")

    def _build_one(self, stage, base: str, pos, env_idx: int):
        from pxr import UsdGeom, Gf

        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        ox, oy = self.LATERAL_OFFSET

        xform = UsdGeom.Xform.Define(stage, base)
        xform.AddTranslateOp().Set(Gf.Vec3d(x + ox, y + oy, z))

        pole = UsdGeom.Cylinder.Define(stage, base + "/Pole")
        pole.CreateRadiusAttr(0.05)
        pole.CreateHeightAttr(self.POLE_HEIGHT)
        pole.CreateAxisAttr("Z")
        pole.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, self.POLE_HEIGHT / 2))
        pole.CreateDisplayColorAttr([Gf.Vec3f(0.15, 0.15, 0.15)])

        housing = UsdGeom.Cube.Define(stage, base + "/Housing")
        housing.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, self.POLE_HEIGHT + 0.45))
        housing.AddScaleOp().Set(Gf.Vec3f(0.15, 0.12, 0.45))
        housing.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.1, 0.1)])

        lamps = {}
        for i, name in enumerate(self.LAMP_ORDER):
            lamp = UsdGeom.Sphere.Define(stage, f"{base}/Lamp_{name}")
            lamp.CreateRadiusAttr(self.LAMP_RADIUS)
            z_off = self.POLE_HEIGHT + 0.75 - i * 0.3
            # front face of the housing (-y), toward oncoming traffic
            lamp.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.16, z_off))
            self._bind_emissive(stage, lamp.GetPrim(), f"{base}/Mat_{name}", self.COLORS[name])
            lamps[name] = UsdGeom.Imageable(lamp.GetPrim())

        self._lamps_by_env[env_idx] = lamps
        self._last_phase[env_idx] = None

    @staticmethod
    def _bind_emissive(stage, prim, mat_path: str, color):
        from pxr import UsdShade, Sdf, Gf

        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    # ------------------------------ sync ---------------------------------

    def sync(self, phase_per_env):
        """phase_per_env: (num_envs,) tensor of displayed phase for the
        agent's approach group (0 green, 1 yellow, 2 red). Touches USD only
        for envs whose phase changed since the last call.
        """
        for env_idx, lamps in self._lamps_by_env.items():
            if env_idx >= phase_per_env.shape[0]:
                continue
            phase = int(phase_per_env[env_idx].item())
            if phase == self._last_phase.get(env_idx):
                continue
            self._last_phase[env_idx] = phase
            active = self.PHASE_TO_LAMP.get(phase, "red")
            for name, imageable in lamps.items():
                if name == active:
                    imageable.MakeVisible()
                else:
                    imageable.MakeInvisible()
