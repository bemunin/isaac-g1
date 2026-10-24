from collections.abc import Iterable

import carb
from pxr import Usd

# warehouse.usda marks these props `active = false`; ObjectManager.show puts them back.
# Names are children of full_warehouse.
_FULL_WAREHOUSE = "/Environment/full_warehouse"
_CLUTTER = (
    "SM_CardBoxA_02_2",
    "SM_CardBoxA_3",
    "SM_CardBoxD_04_6",
    "SM_CardBoxD_03_9",
    "SM_CardBoxD_4",
    "SM_CratePlasticNote_B_03_18",
    "SM_CratePlasticNote_B_5",
    "SM_PushcartA_02_22",
    "SM_BarelPlastic_C_01_25",
    "SM_BarelPlastic_C_2",
    "SM_CardBoxB_01_29",
    "SM_CardBoxB_02_32",
    "SM_BottlePlasticD_01_35",
    "SM_BottlePlasticD_2",
    "SM_BottlePlasticD_3",
    "SM_BottlePlasticD_4",
    "SM_BucketPlastic_B_41",
    "SM_CratePlastic_E_03_44",
    "SM_CratePlastic_D_01_47",
    "SM_PaperNote_Small_15",
    "SM_PaperNote_Small_06_54",
    "SM_PaperNote_Small_20",
    "SM_PaperNote_Small_16",
    "SM_PaperNote_Small_13",
    "SM_PaperNote_Small_17",
    "SM_PaperNote_Small_18",
    "SM_PaperNote_Small_09_72",
    "SM_PaperNote_Small_08_75",
    "SM_CardBoxB_3",
    "S_TrafficCone_2",
    "S_TrafficCone2",
    "S_TrafficCone3",
    "S_TrafficCone4",
    "S_TrafficCone5",
    "S_TrafficCone6",
    "S_WetFloorSign_2",
    "S_WetFloorSign2",
)


class ObjectManager:
    """Shows and hides the warehouse's obstacle props by kind; edits go to the stage's session layer."""

    OBSTACLES: dict[str, tuple[str, ...]] = {
        "clutter": tuple(f"{_FULL_WAREHOUSE}/{name}" for name in _CLUTTER),
        "forklift": (f"{_FULL_WAREHOUSE}/forklift",),
    }

    def __init__(self, stage: Usd.Stage):
        self._stage = stage

    def show(self, kinds: Iterable[str]) -> None:
        """Activate the props of each kind."""
        self._set_active(kinds, True)

    def hide(self, kinds: Iterable[str]) -> None:
        """Deactivate the props of each kind."""
        self._set_active(kinds, False)

    def visible(self) -> list[str]:
        """Kinds whose prims are all on the stage and active."""
        return [
            kind
            for kind, paths in self.OBSTACLES.items()
            if all((prim := self._stage.GetPrimAtPath(path)) and prim.IsActive() for path in paths)
        ]

    def _set_active(self, kinds: Iterable[str], active: bool) -> None:
        """Set the active flag of each kind's prims in the session layer; missing prims are logged and skipped."""
        kinds = list(kinds)
        unknown = sorted(set(kinds) - self.OBSTACLES.keys())
        if unknown:
            raise ValueError(f"unknown obstacles {unknown}, choose from {sorted(self.OBSTACLES)}")
        missing = []
        with Usd.EditContext(self._stage, self._stage.GetSessionLayer()):
            for path in (path for kind in kinds for path in self.OBSTACLES[kind]):
                if prim := self._stage.GetPrimAtPath(path):
                    prim.SetActive(active)
                else:
                    missing.append(path)
        if missing:
            carb.log_warn(f"ObjectManager: not on the stage, skipped: {missing}")
