import os

def remove_terrain_tiles(file_path):
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        return

    with open(file_path, 'r') as f:
        lines = f.readlines()

    new_lines = []
    skip_depth = 0
    in_terrain_tile = False
    
    for line in lines:
        stripped = line.strip()
        
        # Detect start of a terrain tile Mesh
        if 'def Mesh "terrain_tile_' in line or 'def Xform "terrain_tile_' in line:
            in_terrain_tile = True
            skip_depth = 1
            continue
            
        if in_terrain_tile:
            # Track brace depth to find the end of the prim block
            if '{' in line:
                skip_depth += line.count('{')
            if '}' in line:
                skip_depth -= line.count('}')
            
            if skip_depth <= 0:
                in_terrain_tile = False
            continue
            
        new_lines.append(line)

    with open(file_path, 'w') as f:
        f.writelines(new_lines)
    print(f"Removed terrain tiles from {file_path}")

if __name__ == "__main__":
    usd_file = "openStreetUSD/no_graph_sim_clean_1x.usda"
    remove_terrain_tiles(usd_file)
