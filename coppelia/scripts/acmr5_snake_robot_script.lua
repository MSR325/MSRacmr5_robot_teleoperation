function sysCall_init()
    sim = require('sim')
    simROS2 = require('simROS2')
    
    -- Initialize the snake robot system
    model = sim.getObject('..')
    
    moduleCount = 8  -- 8 modules (from first to last)
    mass = 1.1
    str_val = -20
    maxHAngle = 45
    
    modules = {}
    
    -- Proximity sensor setup
    prox = sim.getObject('../body/proxSensor')
    obstacles = sim.createCollection(0)
    sim.addItemToCollection(obstacles, sim.handle_all, -1, 0)
    sim.addItemToCollection(obstacles, sim.handle_tree, model, 1)
    sim.setObjectInt32Parameter(prox, sim.proxintparam_entity_to_detect, obstacles)
    
    -- Initialize module references
    local currentModule = model
    for i = 0, moduleCount - 1 do
        local m = {}
        m.body = sim.getObject('../body', {proxy = currentModule})
        m.vJoint = sim.getObject('../vJoint_' .. i, {proxy = currentModule})
        m.hJoint = sim.getObject('../hJoint_' .. i, {proxy = currentModule})
        m.bodyRespondable = sim.getObject('../_bodyRespondable', {proxy = currentModule})
        modules[i] = m
        
        -- Advance to next bodyRespondable
        currentModule = m.bodyRespondable
    end
    
    -- Initialize joint command storage
    vertical_joint_commands = {}
    horizontal_joint_commands = {}
    
    for i = 0, moduleCount - 1 do
        vertical_joint_commands[i] = 0.0
        horizontal_joint_commands[i] = 0.0
    end
    
    -- ROS2 setup
    local success, error = pcall(function()
        v_joint_subs = {}
        h_joint_subs = {}
        v_joint_pubs = {}
        h_joint_pubs = {}
        
        for i = 0, moduleCount - 1 do
            -- Create subscribers for joint commands
            v_joint_subs[i] = simROS2.createSubscription(
                '/snake_robot/v_joint_' .. i .. '/command',
                'std_msgs/msg/Float64',
                'v_joint_callback_' .. i
            )
            h_joint_subs[i] = simROS2.createSubscription(
                '/snake_robot/h_joint_' .. i .. '/command',
                'std_msgs/msg/Float64',
                'h_joint_callback_' .. i
            )
            
            -- Create publishers for joint states
            v_joint_pubs[i] = simROS2.createPublisher(
                '/snake_robot/v_joint_' .. i .. '/state',
                'std_msgs/msg/Float64'
            )
            h_joint_pubs[i] = simROS2.createPublisher(
                '/snake_robot/h_joint_' .. i .. '/state',
                'std_msgs/msg/Float64'
            )
        end
        
        -- Additional publishers
        proximity_pub = simROS2.createPublisher('/snake_robot/proximity_detected', 'std_msgs/msg/Bool')
        pose_pub = simROS2.createPublisher('/snake_robot/pose', 'geometry_msgs/msg/Pose')
        
        print("ROS2 configured for ACMR5 snake_robot")
    end)
    
    if not success then
        print("ROS2 not available: " .. tostring(error))
        -- Initialize empty variables if ROS2 fails
        v_joint_subs = {}
        h_joint_subs = {}
        v_joint_pubs = {}
        h_joint_pubs = {}
        proximity_pub = nil
        pose_pub = nil
    end
    
    -- Joint configuration
    configure_joints()
    
    proxCounter = 0
    lastPublishTime = 0.0
end

-- Create individual callback functions for each joint
for i = 0, 7 do  -- Assuming moduleCount = 8
    _G['v_joint_callback_' .. i] = function(msg)
        if vertical_joint_commands then
            vertical_joint_commands[i] = msg.data
        end
    end
    
    _G['h_joint_callback_' .. i] = function(msg)
        if horizontal_joint_commands then
            horizontal_joint_commands[i] = msg.data
        end
    end
end

function configure_joints()
    -- Configure joint properties
    for i = 0, moduleCount - 1 do
        local vj = modules[i].vJoint
        local hj = modules[i].hJoint
        
        if vj ~= -1 then
            sim.setJointMode(vj, sim.jointmode_kinematic, 0)
            sim.setJointInterval(vj, false, {-math.pi/4, math.pi/4})
            sim.setJointPosition(vj, 0)
            sim.setJointTargetPosition(vj, 0)
        end
        
        if hj ~= -1 then
            sim.setJointMode(hj, sim.jointmode_kinematic, 0)
            sim.setJointInterval(hj, false, {-math.pi/4, math.pi/4})
            sim.setJointPosition(hj, 0)
            sim.setJointTargetPosition(hj, 0)
        end
    end
end

function apply_buoyancy_and_drag()
    -- Apply buoyancy and drag forces to modules
    for i = 0, #modules do
        if modules[i] then
            local p = sim.getObjectPosition(modules[i].body)
            local cm = (0.05 - p[3]) / 0.05  -- p[3] is z-coordinate in Lua (1-indexed)
            if cm > 1.05 then
                cm = 1.05
            end
            if cm < 0 then
                cm = 0
            end
            
            -- Apply buoyancy force
            sim.addForceAndTorque(modules[i].body, {0, 0, 9.81 * cm})
            
            -- Apply drag
            local linV, angV = sim.getVelocity(modules[i].body)
            local m = sim.getObjectMatrix(modules[i].body)
            m[4], m[8], m[12] = 0, 0, 0  -- Clear translation components (1-indexed)
            local mi = sim.getMatrixInverse(m)
            linV = sim.multiplyVector(mi, linV)
            linV[1] = 0  -- Set x-component to 0 (1-indexed)
            linV = sim.multiplyVector(m, linV)
            local f = {linV[1] * mass * str_val * cm,
                      linV[2] * mass * str_val * cm,
                      linV[3] * mass * str_val * cm}
            sim.addForceAndTorque(modules[i].body, f)
        end
    end
end

function check_proximity()
    -- Check proximity sensor and publish status
    local detected = (sim.readProximitySensor(prox) == 1)
    if detected then
        proxCounter = 4  -- proxTiming * 4, assuming proxTiming = 1
    end
    
    -- Publish proximity status
    if proximity_pub then
        pcall(function()
            simROS2.publish(proximity_pub, {data = detected})
        end)
    end
    
    return detected
end

function publish_joint_states()
    -- Publish current joint states
    if not v_joint_pubs or not h_joint_pubs then
        return
    end
        
    pcall(function()
        for i = 0, moduleCount - 1 do
            if modules[i] then
                -- Publish vertical joint state
                local v_pos = sim.getJointPosition(modules[i].vJoint)
                simROS2.publish(v_joint_pubs[i], {data = v_pos})
                
                -- Publish horizontal joint state  
                local h_pos = sim.getJointPosition(modules[i].hJoint)
                simROS2.publish(h_joint_pubs[i], {data = h_pos})
            end
        end
    end)
end

function publish_robot_pose()
    -- Publish robot pose
    if not pose_pub then
        return
    end
        
    pcall(function()
        local pos = sim.getObjectPosition(model)
        local orient = sim.getObjectQuaternion(model)
        
        local pose_msg = {
            position = {x = pos[1], y = pos[2], z = pos[3]},
            orientation = {x = orient[1], y = orient[2], z = orient[3], w = orient[4]}
        }
        
        simROS2.publish(pose_pub, pose_msg)
    end)
end

function apply_joint_commands()
    -- Apply received joint commands to the robot
    local max_angle_rad = maxHAngle * math.pi / 180
    
    for i = 0, moduleCount - 1 do
        if modules[i] and vertical_joint_commands[i] and horizontal_joint_commands[i] then
            local v_cmd = math.max(-max_angle_rad, math.min(max_angle_rad, vertical_joint_commands[i]))
            local h_cmd = math.max(-max_angle_rad, math.min(max_angle_rad, horizontal_joint_commands[i]))
            
            pcall(function()
                if modules[i].vJoint ~= -1 then
                    sim.setJointPosition(modules[i].vJoint, v_cmd)
                end
                if modules[i].hJoint ~= -1 then
                    sim.setJointPosition(modules[i].hJoint, h_cmd)
                end
            end)
        end
    end
end

function sysCall_actuation()
    -- Main actuation loop called at each simulation step
    apply_buoyancy_and_drag()
    check_proximity()
    apply_joint_commands()
    
    if proxCounter > 0 then
        proxCounter = proxCounter - 1
    end
    
    -- Publish states at 10Hz
    local currentTime = sim.getSimulationTime()
    if currentTime - lastPublishTime > 0.1 then
        publish_joint_states()
        publish_robot_pose()
        lastPublishTime = currentTime
    end
end

function sysCall_cleanup()
    -- Cleanup function called when simulation ends
    pcall(function()
        -- Shutdown subscribers
        if v_joint_subs then
            for i, sub in pairs(v_joint_subs) do
                simROS2.shutdownSubscription(sub)
            end
        end
        if h_joint_subs then
            for i, sub in pairs(h_joint_subs) do
                simROS2.shutdownSubscription(sub)
            end
        end
            
        -- Shutdown publishers
        if v_joint_pubs then
            for i, pub in pairs(v_joint_pubs) do
                simROS2.shutdownPublisher(pub)
            end
        end
        if h_joint_pubs then
            for i, pub in pairs(h_joint_pubs) do
                simROS2.shutdownPublisher(pub)
            end
        end
            
        if proximity_pub then
            simROS2.shutdownPublisher(proximity_pub)
        end
        if pose_pub then
            simROS2.shutdownPublisher(pose_pub)
        end
    end)
    
    -- Reset joints to zero position
    pcall(function()
        for i = 0, moduleCount - 1 do
            if modules[i] then
                local v_joint = modules[i].vJoint
                local h_joint = modules[i].hJoint
                
                if v_joint ~= -1 then
                    sim.setJointTargetPosition(v_joint, 0)
                end
                if h_joint ~= -1 then
                    sim.setJointTargetPosition(h_joint, 0)
                end
            end
        end
    end)
end