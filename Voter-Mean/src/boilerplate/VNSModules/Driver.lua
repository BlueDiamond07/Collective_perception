-- Driver --------------------------------------
------------------------------------------------------
-- In this file, one of two possible motion vectors are chosen for the current time step: 
-- Option 1) move forward, as specified by RandomWalker.lua,
-- Option 2) respond to obstacle (either turn right, turn left), as specified in BoxAvoider.lua
------------------------------------------------------
-- Note: the motion vector chosen here can be overriden by wall avoidance, as specified in boilerplate_loop_functions.cpp
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")

local Driver = {VNSMODULECLASS = true}
Driver.__index = Driver
local tmpVector = {}
local counter = -1
local circulateFlag = 1
function Driver:new()
	local instance = {}
	setmetatable(instance, self)
	instance.lastReceivedSpeed = {
		locV3 = Vec3:create(),
		dirV3 = Vec3:create(),
	}
	return instance
end

function Driver:deleteParent(vns)
	self.lastReceivedSpeed = {
		locV3 = Vec3:create(),
		dirV3 = Vec3:create(),
	}
end

function Driver:run(vns, paraT)
		counter = counter + 1
		local sum = 0
		if vns.robotType == "vehicle" then
			for i=1, 12 do		-- to check if any proximilty sensor of the ground robot senses an obstacle or not
				sum = sum + paraT.proxTR[i]
			end
			sum = sum/12
		end

------------------------------------------------------
-- This section not used
------------------------------------------------------
		-- listen to drive from parent
		local chillRate = 0.1
		self.lastReceivedSpeed.locV3 = self.lastReceivedSpeed.locV3 * chillRate
		self.lastReceivedSpeed.dirV3 = self.lastReceivedSpeed.dirV3 * chillRate
		for _, msgM in pairs(vns.Msg.getAM(vns.parentS, "drive")) do
			-- a drive message data is:
			--	{	yourLocV3, yourDirQ,
			--		transV3, rotateV3
			--	}
			
			msgM.dataT.transV3 = vns.Msg.recoverV3(msgM.dataT.transV3)
			msgM.dataT.rotateV3 = vns.Msg.recoverV3(msgM.dataT.rotateV3)
			msgM.dataT.yourLocV3 = vns.Msg.recoverV3(msgM.dataT.yourLocV3)
			msgM.dataT.yourDirQ = vns.Msg.recoverQ(msgM.dataT.yourDirQ)

			local transV3 = Linar.mySpeedToYou(
				msgM.dataT.transV3,
				msgM.dataT.yourDirQ
			)
			local rotateV3 = Linar.mySpeedToYou(
				msgM.dataT.rotateV3,
				msgM.dataT.yourDirQ
			)

			self.lastReceivedSpeed.locV3 = transV3
			self.lastReceivedSpeed.dirV3 = rotateV3 
		end

		local transV3 = self.lastReceivedSpeed.locV3
		local rotateV3 = self.lastReceivedSpeed.dirV3

		-- add emergency speed
		if vns.emergencySpeed ~= nil then
			local scalar = 1
			transV3 = transV3 + vns.emergencySpeed.locV3 * scalar
			rotateV3 = rotateV3 + vns.emergencySpeed.dirV3 * scalar
		end
------------------------------------------------------

		------------------------------------------------------
		-- Define default motion beahvior as : moving forward
		------------------------------------------------------
		if vns.randomWalkerSpeed ~= nil then -- if the robot should move forward
			local scalar = 1
			transV3 = transV3 + vns.randomWalkerSpeed.locV3 * scalar -- set the vector to indicate moving forward
			rotateV3 = rotateV3 + vns.randomWalkerSpeed.dirV3 * scalar
		end
		------------------------------------------------------
		-- If an obstacle is detected, then override the default motion behavior (i.e., forward)
		------------------------------------------------------
		if vns.robotType == "vehicle" and sum~=0 then 
			local scalar = 1.2					
			if vns.boxAvoiderSpeed ~= nil then
				transV3 = vns.boxAvoiderSpeed.locV3
			else
				transV3 = transV3
			end
		end
		------------------------------------------------------

		if vns.idS == vns.brainS and vns.corner == 1 then  -- not used (this "if" condition is never true)
			print("test", vns.brainS)
		else
			vns.move(transV3, rotateV3, paraT.stepCounter, paraT.energyN) -- implementation detail that sends the motion beahvior to the robot (this function is responsible for movement of the groudn robots and is called by ground robots)
		end
	
------------------------------------------------------
-- This section not used
------------------------------------------------------
	-- useless as there is no children for the UAV or the ground robots
	-- send drive to children
	for _, robotVns in pairs(vns.childrenTVns) do
		if robotVns.rallyPoint == nil then 
			robotVns.rallyPoint = {
				locV3 = Vec3:create(),
				dirQ = Quaternion:create(),
			}
		end

		-- calc speed
		local totalTransV3, totalRotateV3

		-- add rallypointspeed
		local rallypointScalar = 2
		local dV3 = robotVns.rallyPoint.locV3 - robotVns.locV3
		local d = dV3:len()
		local rallypointTransV3 = rallypointScalar / d * dV3:nor()

		local rotateQ = robotVns.rallyPoint.dirQ * robotVns.dirQ:inv()
		local ang = rotateQ:getAng()
		if ang > math.pi then ang = ang - math.pi * 2 end
		local rallypointRotateV3 = rotateQ:getAxis() * ang

		if d < 4 then rallypointTransV3 = Vec3:create() end
		if rallypointRotateV3:len() < math.pi/12 then rallypointRotateV3 = Vec3:create() end

		totalTransV3 = rallypointTransV3
		totalRotateV3 = rallypointRotateV3

		local timestep = 1 / 50
		-- add parent speed
		local parentScalar = 0
		totalTransV3 = totalTransV3 + (transV3+rotateV3*robotVns.locV3) * timestep * parentScalar
		totalRotateV3 = totalRotateV3 + rotateV3 * timestep * parentScalar

		-- add obstacle avoidence
		local avoiderScalar = 15
		if robotVns.avoiderSpeed ~= nil then
		totalTransV3 = totalTransV3 + robotVns.avoiderSpeed.locV3 * avoiderScalar
		totalRotateV3 = totalRotateV3 + robotVns.avoiderSpeed.dirV3 * avoiderScalar

			-- clear avoiderspeed
		robotVns.avoiderSpeed.locV3 = Vec3:create()
		end
		
	end

end
------------------------------------------------------

function Driver:move(transV3, rotateV3)
	print("VNS.Modules.Driver.move needs to be implemented")
end

return Driver

