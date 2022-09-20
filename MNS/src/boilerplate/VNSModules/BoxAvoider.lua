-- External Avoider --------------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")

local BoxAvoider = {VNSMODULECLASS = true}
BoxAvoider.__index = BoxAvoider
timer = 0
timerFlag = 0
randomNum = 0
randomNumCount = 0
function BoxAvoider:new()
	local instance = {}
	setmetatable(instance, self)
	return instance
end

function BoxAvoider:run(vns, paraT)
	local flagAvoid = 0
	if vns.boxAvoiderSpeed == nil then
		vns.boxAvoiderSpeed = {
			locV3 = Vec3:create(),
			dirV3 = Vec3:create(),
		}
	end
	-- if a ground robot receives "normal" message, it sets its normal variable to 1; this variable is used for obstacle avoidance when the MNS is shifting to left or right
	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "normal")) do
		vns.normal = 1
	end
	-- to avoid robot-robot collisions during obstacle avoidance (when the MNS is moving forward or backward) a traffic rule strategy is applied by a parent UAV
	-- based on the rule, when two ground robots get closer than ~3.7 cm to each other (edge-to-edge distance), depending on the movement direction of the MNS, if the robot that is more "behind" senses an object (e.g., the other robot, an obstacle) it stops moving temporarily (but might change its direction based on some conditions; lines 192 to 208 of Driver.lua), and the other one avoids it
	-- to this end, a "avoid" message is sent to the robot that is located behind (this message is used by Driver.lua) and a "avoid2" message is sent to the other one
	-- when a ground robot receives "avoid2" message, it sets flagAvoid to 1
	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "avoid2")) do
		flagAvoid = 1
	end
	-- the following timer is used only from step 300 to 320 when only proximity sensor 10 from the proximity sensor set of the front half of the robot (i.e., proximity sensors 1, 2, 3, 4, 12, 11, 10) senses an obstacle
	-- this is useful in case that two robots senses each other while they are side by side of each other (i.e., their x coordinate is the same). 
	-- by using this timer the one that senses the other peer from its proximity sensor 10 stays staionary for 10 seconds while the other avoiding it, and in this way, when MNS starts moving, the x coordinate of the two ground robots that might still sense each other will not be the same; so through sending "avoid" and "avoid2" messages to them (which is done by UAVs), they will successfully avoid each other and keep moving toward their target points
	if timer > 0 then    
		timer = timer - 1
		if timer == 0 then
			timerFlag = 0
		end
	end
	-- reduce the timer of the chosen turn direction (useable for cases in which only proximity sensor 1 senses an obstacle)
	if randomNumCount > 0 then
		randomNumCount = randomNumCount - 1
	end 
	-- when the MNS is shifting to left or right (i.e., vns.normal==1) or when it is moving forward or backward but no robot-robot collision is observed by the UAVs (i.e., flagAvoid == 0 and  paraT.stepCounter >= 320), the following set of static rules for obstacle avoidance is used by the ground robots
	if vns.normal == 1 or (flagAvoid == 0 and  paraT.stepCounter >= 320) then
		if paraT.proxTR[12]~=0 then		
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, 0.08, 0)    -- left wheel's speed: -2.8, and right wheel's speed: 5.2
			-- the vns.flagTurn (i.e., the following flag) might rarely be used by structure_vehicle.lua in emergency cases (by "move" function which is the function that is called after applying all the speeds and is responsible for the movement of the robot)
			-- When flagTurn is 0, the ground robot does not sense any obstacle by its set of front half proximity sensors (i.e., proximity sensors 1, 2, 3, 4, 12, 11, 10), and the parent UAV sends negative values to the ground robots as its left wheel and right wheel speeds, the ground robot quickly turns left on spot
			-- When flagTurn is 1, the ground robot does not sense any obstacle by its set of front half proximity sensors (i.e., proximity sensors 1, 2, 3, 4, 12, 11, 10), and the parent UAV sends negative values to the ground robots as its left wheel and right wheel speeds, the ground robot quickly turns right on spot
			vns.flagTurn = 1    
		elseif paraT.proxTR[2]~=0 then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, -0.08, 0)    -- left wheel's speed: 5.2, and right wheel's speed: -2.8
			vns.flagTurn = 0
		elseif paraT.proxTR[3]~=0 then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.1, -0.08, 0)    -- left wheel's speed: 5.5, and right wheel's speed: -2.5
			vns.flagTurn = 0
		elseif paraT.proxTR[4]~=0 then    -- this rule simulates a half-circle trajectory from right side of an obstacle	
			if vns.countFlag4 == 0 or vns.countFlag4 == nil then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.5, -0.02, 0)    -- left wheel's speed: 8.1, and right wheel's speed: 6.9
			else 		
				vns.countFlag4 = vns.countFlag4 - 1
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.5, 0.1, 0)    -- left wheel's speed: 4.5, and right wheel's speed: 10
			end
			vns.flagTurn = 0
		elseif paraT.proxTR[11]~=0 then	
			if paraT.proxTR[11]<0.99 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.1, 0.08, 0)    -- left wheel's speed: -2.5, and right wheel's speed: 5.5
			else
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0, 1, 0)    -- left wheel's speed: -10, and right wheel's speed: 10
			end
			vns.flagTurn = 1
		elseif paraT.proxTR[10]~=0 then    -- this rule simulates a half-circle trajectory from left side of an obstacle	
				
			if vns.countFlag10 == 0 or vns.countFlag10 == nil then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.5, 0.02, 0)    -- left wheel's speed: 6.9, and right wheel's speed: 8.1
			else 			
				vns.countFlag10 = vns.countFlag10 - 1	
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.5, -0.1, 0)    -- left wheel's speed: 10, and right wheel's speed: 4.5	
			end
			vns.flagTurn = 1
		elseif paraT.proxTR[1] ~= 0 then
			if randomNumCount == 0 then
				randomNum = math.random()   -- a random real number between 0 and 1. This number is used to select the direction of turn (50% right and 50% left) when only proximity sensor 1 senses an obstacle. 
				randomNumCount = 20   -- randomNumCount (a counter) is set to 20: for 20 steps the direction of turn is not changed as long as only proximity sensor 1 senses an obstacle 
			end
			if randomNum > 0.5 then   -- select the speed and direction of turn (50% right and 50% left)
				vns.boxAvoiderSpeed.locV3 = Vec3:create(1, -0.8, 0)    -- left wheel's speed: 10, and right wheel's speed: 3
			else
				vns.boxAvoiderSpeed.locV3 = Vec3:create(1, 0.8, 0)    -- left wheel's speed: 3, and right wheel's speed: 10
			end
		end
	-- when a ground robot receives "avoid2" message, the following set of static rules for obstacle avoidance is used by the ground robot
	elseif flagAvoid == 1 and paraT.stepCounter>320 then
		if paraT.proxTR[4]~=0 then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(1, -0.8, 0)    -- left wheel's speed: 10, and right wheel's speed: 3	
		elseif paraT.proxTR[10]~=0 then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(1, 0.8, 0)    -- left wheel's speed: 3, and right wheel's speed: 10
		elseif paraT.proxTR[12]~=0 then		
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, 0.08, 0)    -- left wheel's speed: -2.8, and right wheel's speed: 5.2
			vns.flagTurn = 1
		elseif paraT.proxTR[2]~=0 then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, -0.08, 0)    -- left wheel's speed: 5.2, and right wheel's speed: -2.8
			vns.flagTurn = 0
		elseif paraT.proxTR[3]~=0 then	
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.1, -0.08, 0)    -- left wheel's speed: 5.5, and right wheel's speed: -2.5
			vns.flagTurn = 0
		elseif paraT.proxTR[11]~=0 then	
			if paraT.proxTR[11]<0.99 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.1, 0.08, 0)    -- left wheel's speed: -2.5, and right wheel's speed: 5.5
			else
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0, 1, 0)    -- left wheel's speed: -10, and right wheel's speed: 10
			end
			vns.flagTurn = 1
		end
	-- the following set of static rules are used for obstacle avoidance during the formation of the MNS (i.e., the first 320 steps)	
	elseif  paraT.stepCounter<320 then
		if paraT.proxTR[12]~=0 then		
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, 0.08, 0)    -- left wheel's speed: -2.8, and right wheel's speed: 5.2    
			vns.flagTurn = 1
		elseif paraT.proxTR[2]~=0 then			
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, -0.08, 0)    -- left wheel's speed: 5.2, and right wheel's speed: -2.8    
			vns.flagTurn = 0
		elseif paraT.proxTR[1]~=0 then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, -0.08, 0)    -- left wheel's speed: 5.2, and right wheel's speed: -2.8    
			vns.flagTurn = 0
		elseif paraT.proxTR[3]~=0 then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.1, -0.08, 0)    -- left wheel's speed: 5.5, and right wheel's speed: -2.5    
			vns.flagTurn = 0
		elseif paraT.proxTR[4]~=0 then    -- this rule simulates a half-circle trajectory from right side of an obstacle			
			if vns.countFlag4 == 0 or vns.countFlag4 == nil then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(1, -0.2, 0)    -- left wheel's speed: 10, and right wheel's speed: 10    
			else 				
				vns.countFlag4 = vns.countFlag4 - 1
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.5, 0.1, 0)    -- left wheel's speed: 4.5, and right wheel's speed: 10   
			end
			vns.flagTurn = 0
		elseif paraT.proxTR[11]~=0 then
			if paraT.proxTR[11]<0.99 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.1, 0.08, 0)    -- left wheel's speed: -2.5, and right wheel's speed: 5.5
			else
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0, 1, 0)    -- left wheel's speed: -10, and right wheel's speed: 10    
			end
			vns.flagTurn = 1
		elseif paraT.proxTR[10]~=0 and paraT.stepCounter > 300 then
			if timerFlag == 0 then
				timer = 20    -- set timer to 20
				timerFlag = 1
			end
			if timer >= 10 then    -- if timer is equal or greater than 10, do not move or rotate for 10 steps
				if vns.countFlag10 == 0 or vns.countFlag10 == nil then
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0, 0.0, 0)    -- stay stationary    
				else 		
					vns.countFlag10 = vns.countFlag10 - 1	
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0, 0, 0)    -- stay stationary    
				end
			else    -- otherwise: this rule simulates a half-circle trajectory from left side of an obstacle	
				if vns.countFlag10 == 0 or vns.countFlag10 == nil then
					vns.boxAvoiderSpeed.locV3 = Vec3:create(1, 0.2, 0)    -- left wheel's speed: 10, and right wheel's speed: 10    
				else 		
					vns.countFlag10 = vns.countFlag10 - 1	
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0.5, -0.1, 0)    -- left wheel's speed: 10, and right wheel's speed: 4.5    		
				end
			end
			vns.flagTurn = 1
		end
	end

end



return BoxAvoider

