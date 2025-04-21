You can find the implementation in ./mpc/scripts/mpc.py
# MPC Modified Pure Pursuit STRATEGY
Our strategy balances aggressive racing with safety considerations, using a multi-layered approach to speed control, steering angle control, racing stage monitoring, and obstacle avoidance while maintaining competitive performance.

The main priority is to ensure the high performance speed and smooth steering angle controls to set the advantage in the race while avoiding the obstacles(We try to catch up the opponent to set advantage), and then keep following the "to-overtake" opponent safely and overtake only when the passing condition is mature otherwise taking a follow strategy which still ensures our advantages in the competition.<br>
If we are passively being followed up by the opponent, our agile and sharp steering angles when passing the obstacles naturally perform defensive mode, as we did cause disruption to the following car which tries to overtake and lead it to sudden confusion in gap finding and crash into the wall.

# Core Algorithm Overview
We use a modified Pure Pursuit, which I call it Modified Pursuit Control (MPC) with enhanced features including:
1. Disparity extender for safety and obstacles avoiding
2. Dynamic speed control
3. Opportunistic passing system
4. Acceleration zones
5. Gap-finding approach for path selection
6. Emergent stop in very urgent situation when being very close to the opponent car ahead

# Speed Management
Base Speed Profiles<br>
- max_speed = 3.0      # Standard maximum speed<br>
- max_speed_2 = 4.0    # Straight-line maximum speed<br>
- min_speed = 1.6      # Minimum operating speed<br>

We added several strategic features to better adjust speed in different conditions
1. Speed Adjustment Factors (We set 1.3 times the speed when identified as Acceleration Zone and Passing Mode)
2. Angle-based Speed Control (We tend to optimize the performance when turning at a U turn)
3. Very sharp turns (>0.3 rad): 1.0 m/s
4. Sharp turns (>0.2 rad): 2.0 m/s
5. Moderate turns (>0.1 rad): 3.0 m/s
6. Slight turns (>0.05 rad): Base speed
7. Straight sections: 4.0 m/s
8. Acceleration Zones<br>
  - Triggers when forward clearance >= 1.5m<br>
  - Speed boost: 20%-30% above base speed (We used 30% in few actual rounds)<br>
  - Only activates when not in passing mode<br>
9. Passing mode which is described in details below<br>


# Passing Strategy
1. Passing Conditions<br>
  - Minimum gap width required: 0.45m<br>
  - Maximum angle for initiation: π/4 (45 degrees)<br>
  - Speed boost during pass: 20%<br>
  - Cooldown between attempts: 0.01s<br>

2. Passing Zones Analysis<br>
  - Checks both left (π/6 to π/3) and right (-π/6 to -π/3) sectors<br>
  - Selects side with larger gap<br>
  - Maintains higher speed through pass<br>

# Safety Considerations
1. Disparity Detection<br>
   -> Threshold: 0.08m<br>
   -> Extends disparities based on:<br>
      - Car width (0.35m)<br>
      - Safety margin (0.15m)<br>
      - Current distance to obstacle<br>
   -> Distance Constraints<br>
      - Minimum detection range: 0.1m<br>
      - Maximum detection range: 3.0m<br>
        
2. Emergency stop if front distance < 0.1m
3. Check the Acceleration Zone based on a long clear distance ahead 1.5m to safely accelerate

# Path Planning and Optimization
1. Gap Finding<br>
   - Identifies gaps in front 180° view<br>
   - Filters for gaps wider than car width<br>
   
2. Targets center of widest gap
3. Falls back to deepest point if no valid gaps
4. Steering Control<br>
   - Wheelbase: 0.48m<br>
   - Lookahead distance: 1.0m<br>
   - Maximum steering angle: 1.0 rad<br>
   
6. Uses pure pursuit geometry for smooth transitions

# Race-Specific Optimizations
1. Track Characteristics Response<br>
- Wide sections: Maximizes acceleration zones<br>
- Narrow sections: Prioritizes stability over speed<br>
- Corners: Progressive speed reduction based on angle<br>

2. Risk Management<br>
- Conservative in narrow sections<br>
- Aggressive in clear straight sections<br>
- Opportunistic passing when clear gaps present<br>
- Immediate speed reduction on detecting obstacles<br>

# Performance Monitoring
1.Execution time logging for scan callback <br>
2. Passing mode status logging <br>
3. Acceleration zone entry logging 

# Potential Improvements
1. Consider implementing dynamic lookahead distance
2. Add track learning/mapping capabilities
3. Implement predictive trajectory planning
4. Add dynamic obstacle prediction
5. Consider adding racing line optimization

