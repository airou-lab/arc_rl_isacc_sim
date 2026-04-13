    sim: SimulationCfg = SimulationCfg(
        dt=0.002, # 500Hz for high-fidelity small scale physics
        render_interval=25, # Maintain 20Hz visual (500 / 25)
        device="cuda:0",
        physx=sim_utils.PhysxCfg(
            solver_type=1, # TGS
            max_position_iteration_count=16, # Increased from 8
            max_velocity_iteration_count=4, 
            bounce_threshold_velocity=0.5, 
            enable_ccd=True, 
            enable_stabilization=True,
            gpu_max_rigid_contact_count=2**20, # 1M contacts
            gpu_max_rigid_patch_count=2**17,
            gpu_heap_capacity=2**26, # 64MB heap
            gpu_found_lost_pairs_capacity=2**20,
        ),
    )
