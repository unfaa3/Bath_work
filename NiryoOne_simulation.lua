-- This is a threaded script.

enableIk=function(enable)
    if enable then
        sim.setObjectMatrix(ikTarget,-1,sim.getObjectMatrix(ikTip,-1))
        for i=1,#jointHandles,1 do
            sim.setJointMode(jointHandles[i],sim.jointmode_ik,1)
        end

        sim.setExplicitHandling(ikGroupHandle,0)
    else
        sim.setExplicitHandling(ikGroupHandle,1)
        for i=1,#jointHandles,1 do
            sim.setJointMode(jointHandles[i],sim.jointmode_force,0)
        end
    end
end

function sysCall_threadmain()
    -- Initialize some values:
    jointHandles={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('NiryoOneJoint'..i)
    end
    
    connection=sim.getObjectHandle('NiryoOne_connection')
    gripper=sim.getObjectChild(connection,0)
    gripperName="NiryoNoGripper"
    if gripper~=-1 then
        gripperName=sim.getObjectName(gripper)
    end
    
    ikGroupHandle=sim.getIkGroupHandle('NiryoOne')
    ikTip=sim.getObjectHandle('NiryoOne_tip')
    ikTarget=sim.getObjectHandle('NiryoOne_target')
    point1=sim.getObjectHandle("NiryoOne_goal")
    point2=sim.getObjectHandle("Wall_goal")
    point3=sim.getObjectHandle("cuboid")

    -- Set-up some of the RML vectors:
    vel=20
    accel=40
    jerk=80
    currentVel={0,0,0,0,0,0,0}
    currentAccel={0,0,0,0,0,0,0}
    maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    targetVel={0,0,0,0,0,0}

    ikMaxVel={0.4,0.4,0.4,1.8}
    ikMaxAccel={0.8,0.8,0.8,0.9}
    ikMaxJerk={0.6,0.6,0.6,0.8}

    initialConfig={0,0,0,0,0,0}
    startConfig={-70.1*math.pi/180,18.85*math.pi/180,93.18*math.pi/180,68.02*math.pi/180,109.9*math.pi/180,90*math.pi/180}

    enableIk(false)  

    
        targetPos1={0,0,0,0,0*math.pi/180,0*math.pi/180}
        sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos1,targetVel)
        sim.wait(12)
        enableIk(true)

        pos=sim.getObjectPosition(point1,-1)
        quat=sim.getObjectQuaternion(point1,-1)
        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1],pos[2],pos[3]},quat,nil)
        sim.wait(3)
        enableIk(false)
        
        sim.setJointTargetPosition(jointHandles[5],50*math.pi/180,nil)
        sim.wait(3)
        sim.setJointTargetPosition(jointHandles[6],0*math.pi/180,nil)
        sim.wait(3)
        sim.setIntegerSignal(gripperName..'_close',1)
        sim.wait(10)
        
        
        targetPos2={0,0,0,0,0,0}
        sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos2,targetVel)
        targetPos3={-75*math.pi/180,0*math.pi/180,-60*math.pi/180,0*math.pi/180,-30*math.pi/180,0*math.pi/180}
        sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos3,targetVel)
        sim.wait(3)
        targetPos4={-75*math.pi/180,-30*math.pi/180,0*math.pi/180,0*math.pi/180,-50*math.pi/180,10*math.pi/180}
------------------------------------------------------------------------
        sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos4,targetVel)
        targetPos5={-75*math.pi/180,-30*math.pi/180,0*math.pi/180,0*math.pi/180,-65*math.pi/180,10*math.pi/180}
        sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos5,targetVel)
        sim.wait(20)
        sim.clearIntegerSignal(gripperName..'_close')
        sim.wait(200)
        targetPos1={0,0,0,0,0*math.pi/180,0*math.pi/180}
        sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos1,targetVel)
        sim.wait(2)
        enableIk(true)

        pos=sim.getObjectPosition(point3,-1)
        quat=sim.getObjectQuaternion(point3,-1)
        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1],pos[2],pos[3]},quat,nil)
        sim.wait(3)
        enableIk(false)
        
        sim.setJointTargetPosition(jointHandles[5],50*math.pi/180,nil)
        sim.wait(3)
        sim.setJointTargetPosition(jointHandles[6],0*math.pi/180,nil)
        sim.wait(3)
        sim.setIntegerSignal(gripperName..'_close',1)
        sim.wait(10)
        targetPos1={170*math.pi/180,0*math.pi/180,0*math.pi/180,0,0*math.pi/180,0*math.pi/180}
        sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos1,targetVel)
        sim.wait(3)
        targetPos1={170*math.pi/180,-70*math.pi/180,50*math.pi/180,0,10*math.pi/180,0*math.pi/180}
        sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos1,targetVel)
        sim.wait(3)
        sim.clearIntegerSignal(gripperName..'_close')
        sim.wait(5)

end
