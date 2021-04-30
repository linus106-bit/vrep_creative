-- At V-REP, Add dummy & Non-threded child script

if (sim_call_type==sim.syscb_init) then
    link = {
        sim.getObjectHandle('odom'),
        sim.getObjectHandle('base_footprint'),
        sim.getObjectHandle('fastHokuyo_ref')
    }
    joint = {
        sim.getObjectHandle('rollingJoint_fl'),
        sim.getObjectHandle('rollingJoint_fr'),
        sim.getObjectHandle('rollingJoint_rl'),
        sim.getObjectHandle('rollingJoint_rr')
    }
    sub_velocity = simROS.subscribe('/cmd_vel','geometry_msgs/Twist','velocity_callback')
    pub_odom = simROS.advertise('/odom', 'nav_msgs/Odometry')
    pub_time = simROS.advertise('/clock', 'rosgraph_msgs/Clock')
    max_radps = 9.83
end

function getTransformStamped(obj,n,relTo,relN)
    --t = simROS.getTime()
    t = sim.getSimulationTime()
    p=sim.getObjectPosition(obj,relTo)
    o=sim.getObjectQuaternion(obj,relTo)
    return {
        header={
            stamp=t,
            frame_id=relN
        },
        child_frame_id=n,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end

function getOdometry(obj,relTo)
    --t = simROS.getTime()
    t = sim.getSimulationTime()
    p = sim.getObjectPosition(obj, relTo)
    o = sim.getObjectQuaternion(obj, relTo)
    v, w =sim.getObjectVelocity(obj)
    return {
        header={seq=odom_seq, stamp=t, frame_id="odom"},
        child_frame_id='base_footprint',
        pose={
            pose={
                position={x=p[1]*0.9,y=p[2]*0.9,z=p[3]*0.9},
                orientation={x=o[1],y=o[2],z=o[3],w=o[4]}
            },
            covariance={0.00001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001}
        },
        twist={
            twist={
                linear={x=v[1],y=v[2],z=v[3]},
                angular={x=w[1],y=w[2],z=w[3]}
            },
            covariance={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
        }
    }
end

function velocity_callback(msg)
    -- This is the sub_velocity callback function
    -- sim.addStatusbarMessage('sub_velocity receiver: ' .. msg.linear.x)
    fl = (msg.linear.x - msg.linear.y - 0.5165*msg.angular.z)/0.076
    fr = (msg.linear.x + msg.linear.y + 0.5165*msg.angular.z)/0.076
    rl = (msg.linear.x + msg.linear.y - 0.5165*msg.angular.z)/0.076
    rr = (msg.linear.x - msg.linear.y + 0.5165*msg.angular.z)/0.076
    if (fl > max_radps) then
        fl = max_radps
    elseif (fl < -max_radps) then
        fl = -max_radps
    end
    if (fr > max_radps) then
        fr = max_radps
    elseif (fr < -max_radps) then
        fr = -max_radps
    end
    if (rl > max_radps) then
        rl = max_radps
    elseif (rl < -max_radps) then
        rl = -max_radps
    end
    if (rr > max_radps) then
        rr = max_radps
    elseif (rr < -max_radps) then
        rr = -max_radps
    end
    sim.setJointTargetVelocity(joint[1], fl)
    sim.setJointTargetVelocity(joint[2], fr)
    sim.setJointTargetVelocity(joint[3], rl)
    sim.setJointTargetVelocity(joint[4], rr)
end

if (sim_call_type==sim.syscb_sensing) then
    transforms={
        -- getTransformStamped(link[1],'odom',-1,'world'),
        getTransformStamped(link[2],'base_footprint',link[1],'odom'),
        --getTransformStamped(link[2],'base_link',link[2],'base_footprint'),
        --getTransformStamped(link[3],'base_scan',link[2],'base_link'),
    }
    --simROS.sendTransforms(transforms)
    odom = getOdometry(link[2],link[1])
    simROS.publish(pub_odom, odom)
    simROS.publish(pub_time, {clock=sim.getSimulationTime()})
end
