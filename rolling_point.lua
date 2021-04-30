function sysCall_init() 
    rolling=sim.getObjectHandle('rollingJoint_fl')
    slipping=sim.getObjectHandle('slippingJoint_fl')
    wheel=sim.getObjectHandle('wheel_respondable_fl')
end
-- Following script resets the second joint on the omni-wheel (important to achieve the desired omni-wheel effect)


function sysCall_cleanup() 
 
end 

function sysCall_actuation() 
    sim.resetDynamicObject(wheel)
    sim.setObjectPosition(slipping,rolling,{0,0,0})
    sim.setObjectOrientation(slipping,rolling,{math.pi/4,0,0})
    sim.setObjectPosition(wheel,rolling,{0,0,0})
    sim.setObjectOrientation(wheel,rolling,{0,0,0})
end 
