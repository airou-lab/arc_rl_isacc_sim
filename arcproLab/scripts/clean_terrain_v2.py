import os
import re

def remove_terrain_tiles_regex(file_path):
    with open(file_path, 'r') as f:
        content = f.read()

    # Match def (Mesh|Xform) "terrain_tile_X" { ... } blocks
    # This is a bit risky with regex but usda is relatively structured
    # We use a non-greedy approach for the first set of braces
    pattern = r'def (Mesh|Xform) "terrain_tile_\d+" \([^\)]*\)\s*\{[^\}]*\}'
    
    # Actually, USDA blocks can be much larger and nested.
    # Let's use a robust line-by-line parser that counts braces properly.
    lines = content.splitlines()
    new_lines = []
    skip_mode = False
    brace_count = 0
    
    for line in lines:
        if ('def Mesh "terrain_tile_' in line or 'def Xform "terrain_tile_' in line) and not skip_mode:
            skip_mode = True
            brace_count = 0
            if '{' in line:
                brace_count += line.count('{')
            if '}' in line:
                brace_count -= line.count('}')
            continue
            
        if skip_mode:
            brace_count += line.count('{')
            brace_count -= line.count('}')
            if brace_count <= 0:
                skip_mode = False
            continue
            
        new_lines.append(line)

    with open(file_path, 'w') as f:
        f.write("\n".join(new_lines))
    print(f"Cleaned terrain from {file_path}")

if __name__ == "__main__":
    remove_terrain_tiles_regex("openStreetUSD/no_graph_sim_clean_1x.usda")
