from typing import Dict, Any
import matplotlib.pyplot as plt

def PlotMap(axes: plt.Axes, map_data: Dict[str, Any]) -> None:
    axes.clear()
    axes.set_aspect("equal")
    axes.grid(True, alpha=0.3)
    axes.set_xlabel("x")
    axes.set_ylabel("y")

    # lanes
    lanes_data = map_data.get("lanes", [])
    if lanes_data:
        lanes_grouped_by_id = {}
        for row in lanes_data:
            lane_id_value = row["lane_id"]
            lanes_grouped_by_id.setdefault(lane_id_value, []).append(row)
        for lane_id_value, lane_points in lanes_grouped_by_id.items():
            lane_points_x = [point["x"] for point in lane_points]
            lane_points_y = [point["y"] for point in lane_points]
            axes.plot(
                lane_points_x,
                lane_points_y,
                marker="o",
                linewidth=2,
                markersize=3,
                alpha=0.9,
                label=f"lane {lane_id_value}",
            )

    # buildings
    buildings_data = map_data.get("buildings", [])
    if buildings_data:
        building_positions_x = [building["x"] for building in buildings_data]
        building_positions_y = [building["y"] for building in buildings_data]
        axes.scatter(building_positions_x, building_positions_y, marker="s", s=40, alpha=0.9, label="buildings")

    # guardrails
    guardrails_data = map_data.get("guardrails", [])
    if guardrails_data:
        guardrail_centers_x = [guardrail["cx"] for guardrail in guardrails_data]
        guardrail_centers_y = [guardrail["cy"] for guardrail in guardrails_data]
        axes.scatter(guardrail_centers_x, guardrail_centers_y, marker="^", s=50, alpha=0.9, label="guardrails")

    # signs
    signs_data = map_data.get("signs", [])
    if signs_data:
        sign_positions_x = [sign["x"] for sign in signs_data]
        sign_positions_y = [sign["y"] for sign in signs_data]
        axes.scatter(sign_positions_x, sign_positions_y, marker="*", s=60, alpha=0.9, label="signs")

    # traffic_lights
    traffic_lights_data = map_data.get("traffic_lights", [])
    if traffic_lights_data:
        traffic_light_positions_x = [light["x"] for light in traffic_lights_data]
        traffic_light_positions_y = [light["y"] for light in traffic_lights_data]
        axes.scatter(traffic_light_positions_x, traffic_light_positions_y, marker="x", s=60, alpha=0.9, label="traffic_lights")

    axes.legend(loc="best", fontsize=8)
    axes.set_title("Map Viewer — Click 'Edit' to enter editing mode")
    axes.figure.canvas.draw_idle()