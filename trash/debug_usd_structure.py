from pxr import Usd, UsdGeom
import os

def main():
    # Use absolute path for reliability
    current_dir = os.path.dirname(os.path.abspath(__file__))
    stage_path = os.path.join(current_dir, "../../openStreetUSD/no_graph_sim_clean_1x.usda")
    print(f"Opening stage: {stage_path}")
    
    stage = Usd.Stage.Open(stage_path)
    
    if not stage:
        print("FAILED to open stage!")
        return

    count = 0
    print("\n--- Auditing White/Yellow Prims in Intersection areas ---")
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        path_lower = path.lower()
        if "yellow" in path_lower or "white" in path_lower:
            # Look for indicators of stop lines, crosswalks, or intersections
            if any(k in path_lower for k in ["stop", "inter", "cross", "horizontal"]):
                print(f"[SPECIAL] {path}")
            else:
                # Just print a few regular ones to see the naming convention
                if count < 10:
                    print(f"[REGULAR] {path}")
            count += 1
    
    print(f"\nTotal markers found: {count}")

if __name__ == "__main__":
    main()
