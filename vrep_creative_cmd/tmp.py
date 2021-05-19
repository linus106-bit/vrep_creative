if (sim_call_type==sim.syscb_init) then
    link = {
        sim.getObjectHandle('frame'),
        sim.getObjectHandle('move_long'),
        sim.getObjectHandle('move_short'),
        sim.getObjectHandle('group_sauce_1'),
        sim.getObjectHandle('group_sauce_2'),
        sim.getObjectHandle('group_sauce_3')
    }
    joint = {
        sim.getObjectHandle('move_long_joint'),
        sim.getObjectHandle('move_short_joint'),
        sim.getObjectHandle('stick_l_joint'),
        sim.getObjectHandle('stick_m_joint'),
        sim.getObjectHandle('cover_joint')
    }
    tip = {
        sim.getObjectHandle('Tip'),
    }    
    connector = {    
        sim.getObjectHandle('connector_1'),
        sim.getObjectHandle('connector_2'),
        sim.getObjectHandle('connector_3'),
    }
    sauce = {
        sim.getObjectHandle('group_sauce_1'),
        sim.getObjectHandle('group_sauce_2'),
        sim.getObjectHandle('group_sauce_3'),
    }
    dummy_pos = { -- Position used for returning the container
        sim.getObjectHandle('group_sauce_1_pos'),
        sim.getObjectHandle('group_sauce_2_pos'),
        sim.getObjectHandle('group_sauce_3_pos'),
        sim.getObjectHandle('free_sauce')
    }
    sensor = {
        sim.getObjectHandle('Ultrasonic_1')
    }
    sub_velocity = simROS.subscribe('/cmd_vel','geometry_msgs/Twist','velocity_callback')
    pub_time = simROS.advertise('/clock', 'rosgraph_msgs/Clock')
    pos_pub = simROS.advertise('sauce_pos','geometry_msgs/Point')
    -- sauce_number = simROS.subscribe('sauce_num','std_msgs/Int8MultiArray','sauce_callback')
    max_radps = 9.83 -- Maximum Radian per second
    max_mps = 1.5 -- Maximum Meter per second
    
    -- Init sauce_msg
    sauce_index = 0 -- Index of group_sauce
    sauce_weight = 0 -- Weight of sauce
    burner_index = 0 -- Index of burner
    
    -- Init of Object
    sim.setObjectInt32Parameter(sauce[1],sim.shapeintparam_static,1)
    sim.setObjectInt32Parameter(sauce[2],sim.shapeintparam_static,1)
    sim.setObjectInt32Parameter(sauce[3],sim.shapeintparam_static,1)
    sim.setLinkDummy(tip[1],-1)
    sauce_init_pos1 = sim.getObjectPosition(link[4],-1)
    sauce_init_pos2 = sim.getObjectPosition(link[5],-1)
    sauce_init_pos3 = sim.getObjectPosition(link[6],-1)
    go_back = false
    finished = false
    cover_close = false
end

function getPosition(obj) -- obj: object, relTO: related to
    objPos=sim.getObjectPosition(obj,-1)
    simROS.publish(pos_pub,{x=objPos[1],y=objPos[2],z=objPos[3]})
end


function velocity_callback(msg)
    move_l = (msg.linear.x) --move long related to x axis
    move_s = (msg.linear.y) --move short related to y axis
    if (move_l > max_mps) then -- Velocity Constraint (MAX & MIN)
        move_l = max_mps
    elseif (move_l < -max_mps) then
        move_l = -max_mps
    end
    if (move_s > max_mps) then
        move_s = max_mps
    elseif (move_s < -max_mps) then
        move_s = -max_mps
    end
    sim.setJointTargetVelocity(joint[1], move_l)
    sim.setJointTargetVelocity(joint[2], move_s)
end

function sauce_num_callback(msg)
    sauce_index = msg.data[1]
    burner_index = msg.data[3]
    if sauce_index == 1 then
        sauce_init_pos = sim.getObjectPosition(dummy_pos[1],-1)
    elseif sauce_index == 2 then
        sauce_init_pos = sim.getObjectPosition(dummy_pos[2],-1)
    elseif sauce_index == 3 then
        sauce_init_pos = sim.getObjectPosition(dummy_pos[3],-1)
    end
    target_x = msg.data[4]
    target_y = msg.data[5]

end



function grab_sauce(sauce_index)
	local t=sim.getSimulationTime()
    sim.setObjectInt32Parameter(joint[5],sim.jointintparam_ctrl_enabled,1)
    sim.setJointTargetPosition(joint[5],90)
    result_5,value_5=sim.getObjectFloatParameter(joint[5],2012)
    -- Grab Group Sauce
    if value_5 <= 0.001 and value_5 > 0 then
        sim.setJointTargetVelocity(joint[3], 0.02)
        sim.setJointTargetVelocity(joint[4], 0.02)
    end
    if t > 15 then
        sim.setLinkDummy(tip[1],connector[sauce_index])
        sim.setObjectInt32Parameter(sauce[sauce_index],sim.shapeintparam_static,0)
        sim.setJointTargetVelocity(joint[3], -0.02)
        sim.setJointTargetVelocity(joint[4], -0.02)
    end
    
    -- Move to the Burner Position
    if t > 23 then
        sim.setObjectInt32Parameter(joint[1],sim.jointintparam_ctrl_enabled,1)
        sim.setObjectInt32Parameter(joint[2],sim.jointintparam_ctrl_enabled,1)
        sim.setJointTargetPosition(joint[1], target_x - sauce_init_pos[1])
        sim.setJointTargetPosition(joint[2], target_y - sauce_init_pos[2])
        -- 0.27201998233795166
        result1,value1=sim.getObjectFloatParameter(joint[1],2012)
        result2,value2=sim.getObjectFloatParameter(joint[2],2012)
        if value1 == 0 then
            r,d=sim.readProximitySensor(sensor[1])
            sim.setJointTargetVelocity(joint[3], 0.05)
            sim.setJointTargetVelocity(joint[4], 0.05)
            if d == nil then
                print('Not Detected')
            elseif d <=0.1 then
                print('Desired Height')
                sim.setJointTargetVelocity(joint[3], 0)
                sim.setJointTargetVelocity(joint[4], 0)
            else
                print(d)
            end
        end
    end
    -- After emiting the sauce, bringup the container
    if t > 38 then
        sim.setJointTargetVelocity(joint[3], -0.1)
        sim.setJointTargetVelocity(joint[4], -0.1)
        result4,value4=sim.getObjectFloatParameter(joint[3],2012)
        if value4 > 0.0001 and value4 <= 0.0005 then
            go_back = true
        end
    end
    
    -- Return to the Refrig
    if go_back == true then
        sim.setJointTargetPosition(joint[1], -(target_x - sauce_init_pos[1]))
        sim.setJointTargetPosition(joint[2], -(target_y - sauce_init_pos[2]))
        result5,value5=sim.getObjectFloatParameter(joint[1],2012)
        result6,value6=sim.getObjectFloatParameter(joint[2],2012)
        if finished == false then
            if value5 == 0 then
                    sim.setJointTargetVelocity(joint[3], 0.1)
                    sim.setJointTargetVelocity(joint[4], 0.1)
                    result7,value7=sim.getObjectFloatParameter(joint[3],2012)
                if value7 == 0 then
                    finished = true
                end
            end
        end
        if finished == true then
            sim.resetDynamicObject(sauce[sauce_index])
            sim.setObjectInt32Parameter(sauce[sauce_index],sim.shapeintparam_static,1)
            sim.setLinkDummy(tip[1],-1)
            sim.setJointTargetVelocity(joint[3], -0.1)
            sim.setJointTargetVelocity(joint[4], -0.1)
            if value7 == 0 then
                -- print('Close Cover')
                cover_close = true
            end
        end
        if cover_close == true then
            sim.setObjectInt32Parameter(joint[5],sim.jointintparam_ctrl_enabled,0)        
            sim.setJointTargetVelocity(joint[5],-1)
        end
    end
end


if (sim_call_type==sim.syscb_actuation) then
    --getPosition(link[3])
    sub_sauce_num = simROS.subscribe('/group_num','std_msgs/Float32MultiArray','sauce_num_callback')
    grab_sauce(1)
end