from dataclasses import dataclass, field
from typing import List, Dict, Any
import pandas as pd

@dataclass
class MapData:
    lanes: List[Dict[str, Any]] = field(default_factory=list)
    buildings: List[Dict[str, Any]] = field(default_factory=list)
    guardrails: List[Dict[str, Any]] = field(default_factory=list)
    signs: List[Dict[str, Any]] = field(default_factory=list)
    traffic_lights: List[Dict[str, Any]] = field(default_factory=list)

    def ToDataFrames(self) -> Dict[str, pd.DataFrame]:
        lanes_df = pd.DataFrame(self.lanes, columns=["lane_id", "x", "y", "z", "lane_type"])
        buildings_df = pd.DataFrame(self.buildings, columns=["id", "x", "y", "h"])
        guardrails_df = pd.DataFrame(self.guardrails, columns=["cx", "cy", "length", "yaw_deg"])
        signs_df = pd.DataFrame(self.signs, columns=["x", "y", "z", "yaw", "sign_class"])
        traffic_lights_df = pd.DataFrame(self.traffic_lights, columns=["x", "y", "z", "state"])
        return {
            "lanes": lanes_df,
            "buildings": buildings_df,
            "guardrails": guardrails_df,
            "signs": signs_df,
            "traffic_lights": traffic_lights_df,
        }
