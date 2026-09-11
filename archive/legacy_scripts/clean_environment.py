import os

def clean_usd_environment(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()

    # Prims to remove (patterns)
    patterns_to_remove = [
        'def Xform "terrain_',
        'def Xform "ref_terrain_',
        'def Xform "pavement_',
        'def Xform "ref_pavement_',
        'def Mesh "terrain_',
        'def Mesh "ref_terrain_'
    ]

    new_lines = []
    skip_mode = False
    brace_count = 0
    removed_count = 0
    
    for line in lines:
        stripped = line.strip()
        
        # Start skipping if we hit a pattern
        if any(p in line for p in patterns_to_remove) and not skip_mode:
            skip_mode = True
            brace_count = 0
            removed_count += 1
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
        f.writelines(new_lines)
    print(f"Cleaned {removed_count} environmental prim groups from {file_path}")

if __name__ == "__main__":
    clean_usd_environment("openStreetUSD/no_graph_sim_clean_1x.usda")
