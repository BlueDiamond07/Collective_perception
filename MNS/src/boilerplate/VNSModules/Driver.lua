-- Driver --------------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")

local Driver = {VNSMODULECLASS = true}
Driver.__index = Driver
local tmpVector = {}    -- not used
local prx10 = {}
local prx4 = {}
local counter = -1
Const_shift_counter =  332    -- the number of steps spent in the shift process
shiftCounter =  Const_shift_counter    -- used to control the zig-zag motion: the MNS shifts for Const_shift_counter steps each time it detects a border
makeDelay = 0    -- used to control the zig-zag motion: a counter used to make and keep border detection inactive for a certain period
l_timer = 0    -- a timer used for managing the motion of a ground robot with respect to its target point
r_timer = 0    -- a timer used for managing the motion of a ground robot with respect to its target point
waitActive = 0    -- used to control the zig-zag motion
waitCounter = 0    -- used to control the zig-zag motion
Const_delay = 400    -- the number of steps used to introduce a delay for border detection or the next activation of the waitCounter; it should be set to a value bigger than Const_shift_counter
local Border = 0    -- this variable is used for detecting the border; it is set to 1 when the MNS is shifting (otherwise, it is 0)
flag_NewRound = 0    -- this flag is set to 1 at the step in which the MNS finishes one complete sweep of the arena
local file = io.open ("border_value.csv","w" )    -- the Border variable defined above is recored in this file at each step and is read by the boilerplate_loop_functions.cpp to calculate the opinions
file:write(Border)
file:close()
local file0 = io.open ("end_round.csv","w" )    -- a csv file to record the current value of the flag_NewRound; this file is read by the boilerplate_loop_functions.cpp to reset the LEDs at the end of a sweep round
file0:write(flag_NewRound)
file0:close()
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
		if vns.idS == vns.brainS then  -- at the beginning of each step the brain records the current value of the flag_NewRound flag in the end_round.csv and reset the flag
			local file0 = io.open ("end_round.csv","w" )
			file0:write(flag_NewRound)
			file0:close()

			flag_NewRound = 0
		end

		-- at each step the following variables are set to 0 (lines 57 to 63):
		if vns.idS == vns.brainS then
			Border_Flag = 0    -- this flag is used by the brain dealing with the Border variable used to detect the border		
		end
		adjust_R = 0	-- this flag is used to adjust the position of the robot with respect to its target point
		adjust_L = 0	-- this flag is used to adjust the position of the robot with respect to its target point
		vns.normal = 0	-- this flag is used for obstacle avoidance when the MNS is shifting to left or right
		flagAvoid = 0	-- this falg is used to control robot-robot collisions when they are avoiding obstacles
		if waitActive > 0 then
			waitActive = waitActive- 1
		end
		if waitCounter > 0 then
			waitCounter = waitCounter - 1
		end
		if l_timer > 0 then
			l_timer = l_timer -1
		end
		if r_timer > 0 then
			r_timer = r_timer -1
		end
		if vns.robotType ~= "vehicle" then
			if makeDelay > 0 then
				makeDelay = makeDelay -2
			end	
		end
		
		local randomflag
		local flag = 0
		counter = counter + 1
		local sum = 0
		if vns.robotType == "vehicle" then	
			for i=1, 12 do
				if i<=4 or i>=10 then    -- to check if the front half of the ground robot senses an obstacle or not
					sum = sum + paraT.proxTR[i]
				end
			end
			-- in lines 96 to 127 the right and left proximity sensors' data reading (proximity sensor number 10 and 4, respectively) are recorded in two separate arrays
			-- based on the data readings of the mentioned two sensors, a counter with length of 10 steps is defined: if for 10 steps, each of these two sensors does not sense any obstacle, a timer of length 10 is set
		        -- these timers are used by the BoxAvoider module (i.e., BoxAvoider.lua): the mentiond module simulates a half-circle trajectory (from left side or right side of the obstacle) for the corresponding ground robot to pass the obtacle during this time
			-- the priority (lines 122 to 127) is with proximity sensor 4 which leads to a half-circle trajectory from right side of the obstacle
			if paraT.proxTR[10] ~= 0 then
				prx10[counter] = 1
			elseif paraT.proxTR[10] == 0 then
				prx10[counter] = 0
			end	
			if paraT.proxTR[4] ~= 0 then
				prx4[counter] = 1
			elseif paraT.proxTR[4] == 0 then
				prx4[counter] = 0
			end
			iCount10 = 0
			if (paraT.proxTR[10]>0 and #prx10>10) then
				for i=1, 10 do 
					if prx10[#prx10-i] == 0 then 
						iCount10 = iCount10 + 1
					end
				end
			end
			iCount4 = 0
			if (paraT.proxTR[4]>0 and #prx4>10) then
				for i=1, 10 do 
					if prx4[#prx4-i] == 0 then 
						iCount4 = iCount4 + 1
					end
				end
			end
			if iCount4 == 10 then
				vns.countFlag4 = 10	
			elseif iCount10 == 10 then
				vns.countFlag10 = 10
				
			end	
		end
		-- listen to drive from parent
		for _, msgM in pairs(vns.Msg.getAM(vns.parentS, "reverseMove")) do    -- the other UAVs by receiving this message understand the direction of movement (forward/backward); it helps them to manage collisions between robots during obstacle avoidance in the same way that the brain does
			vns.reverseMove = msgM.dataT
		end
		local chillRate = 0.1
		self.lastReceivedSpeed.locV3 = self.lastReceivedSpeed.locV3 * chillRate
		self.lastReceivedSpeed.dirV3 = self.lastReceivedSpeed.dirV3 * chillRate
		for _, msgM in pairs(vns.Msg.getAM(vns.parentS, "drive")) do    -- the movement instructions are received through this message (this message is sent by each parent at each step)
	
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

		-- add random speed; this speed makes the brain move forward (if vns.reverseMove is equal to 0) or backward (if vns.reverseMove is equal to 1) 
		if vns.randomWalkerSpeed ~= nil then
			local scalar = 1
			transV3 = transV3 + vns.randomWalkerSpeed.locV3 * scalar
			rotateV3 = rotateV3 + vns.randomWalkerSpeed.dirV3 * scalar
			randomflag = 1
		end
		-- if a ground robot receives adjust_left message, it sets adjust_L to 1
		for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "adjust_left")) do
			adjust_L = 1
		end
		-- if a ground robot receives adjust_right message, it sets adjust_R to 1
		for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "adjust_right")) do
			adjust_R = 1
		end
		-- if a ground robot receives normal message, it sets its normal variable to 1; as mentioned before, this flag is used for obstacle avoidance when the MNS is shifting to left or right
		for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "normal")) do
			vns.normal = 1
		end
		-- if a ground robot receives avoid message, it sets flagAvoid to 1; as mentioned before, this falg is used to control robot-robot collisions when they are avoiding obstacles
		-- when this flag is 1, the robot stops moveing if it senses an object (e.g., the other robot, an obstacle) and just changes its direction depending on some conditions; in this way, the other robot can easily avoid the stationary robot
		for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "avoid")) do
			flagAvoid = 1
		end
	        -- lines 187 to 202 allows the robot to frequently adjust its y coordiate with respect to its y coordiate of the target point in the formation to stay approximately in the middle of its lane in the formation
	        -- the first 320 steps is used to form the MNS; The MNS starts moving at step 321 and the outputs are recorded onward
		-- if the l_timer is 0, none of proximity sensors of the front half of the ground robot senses an obstacle, the time step is greater than 320, and adjust_L is 1, the robot gradually moves forward slightly toward its left
		-- after one step, the l_timer is set to 7, and for the 7 following steps the robot does not perfrom this motion
		if l_timer == 0 and vns.robotType == "vehicle" and sum == 0 and paraT.stepCounter > 320 and adjust_L == 1 then

					vns.boxAvoiderSpeed = {
						locV3 = Vec3:create(1.2,1.2,0),	-- left wheel's speed: 3, and right wheel's speed: 10				
					}
					l_timer = 7    -- the l_timer is set to 7
					transV3 = vns.boxAvoiderSpeed.locV3
		-- if the r_timer is 0, none of proximity sensors of the front half of the ground robot senses an obstacle, the time step is greater than 320, and adjust_R is 1, the robot gradually moves forward slightly toward its right
		-- after one step, the r_timer is set to 6, and for the 6 following steps the robot does not perfrom this motion
		elseif r_timer == 0 and vns.robotType == "vehicle" and sum == 0 and paraT.stepCounter > 320 and adjust_R == 1 then

					vns.boxAvoiderSpeed = {
						locV3 = Vec3:create(1.2,-1.2,0), -- left wheel's speed: 10, and right wheel's speed: 3		
					}
					r_timer = 6    -- the r_timer is set to 6
					transV3 = vns.boxAvoiderSpeed.locV3
		elseif vns.robotType == "vehicle" and sum~=0 then	-- if the robot senses an object (e.g., the other robot, an obstacle) 
			local scalar = 1.2
				-- as mentioned before, when a ground robot receives avoid message, it sets flagAvoid variable to 1 (see line 180)
				if vns.normal == 0 and paraT.stepCounter > 320 and flagAvoid == 1 then -- the robot stops moving and just changes its direction depending on some conditions
					if paraT.proxTR[12]~=0  then
						vns.boxAvoiderSpeed = {
							locV3 = Vec3:create(0,1,0),    -- rotate to left on spot (left wheel's speed: -10, and right wheel's speed: 10)		
						}
						transV3 = vns.boxAvoiderSpeed.locV3
					elseif paraT.proxTR[2]~=0  then
						vns.boxAvoiderSpeed = {
							locV3 = Vec3:create(0,-1,0),   -- rotate to right on spot (left wheel's speed: 10, and right wheel's speed: -10)		
						}
						transV3 = vns.boxAvoiderSpeed.locV3
					else
						vns.boxAvoiderSpeed = {
							locV3 = Vec3:create(0,0,0),	-- no movement
						}
						transV3 = vns.boxAvoiderSpeed.locV3
					end
									
				elseif vns.boxAvoiderSpeed ~= nil then

					transV3 = vns.boxAvoiderSpeed.locV3 -- this speed is calculated by the boxAvoider module (i.e., boxAvoider.lua) for avoiding obstacles
				else
					transV3 = transV3
				end

		end
		-- lines 237 to 346 simulate a boustrophedon path for the MNS:
		-- the brain UAV stops for 50 steps as soon as detecting a border in its front or back (i.e., after sweeping a lane) to let the ground robots that are behind reach the group
		-- then, the brain for Const_shift_counter time steps shifts to either left or right depending on the direction of its shift movement (i.e., it shifts to left if its LeftOrRight variable is 1, and shifts to right if its LeftOrRight variable is -1)
		-- afterwards, the brain reverses the direction of its movement: from forward (i.e., vns.reverseMove == 0) to backward (i.e., vns.reverseMove == 1) or from backward to forward
		-- each time the brain sweeps the entire environment, it reverses both its movement and shift direction at the end, and continues sweeping the environment
		if vns.escape == 1 then
			if shiftCounter>0 then
				Border_Flag = 1
				for _, robotVns3 in pairs(paraT.vehiclesTR) do
					vns.Msg.send(robotVns3.idS, "normal")    -- this message is used by module (i.e., boxAvoider.lua) to let the ground robots avoid obstacles when the MNS is shifting to left or right
				end
				if vns.LeftOrRight == 1 then    -- shift to left
					vns.speed(0.0, 0.075, 0.0)
				elseif vns.LeftOrRight == -1 then    -- shift to right
					vns.speed(0.0, -0.075, 0.0)
				end
				shiftCounter = shiftCounter - 1
				if shiftCounter == 0 then
					vns.escape = 0
				end
			end
		elseif vns.idS == vns.brainS and paraT.cornerN == 3 and makeDelay == 0  then    -- detecting a white LED which is placed in a corner of the arena to make the MNS aware of reaching the corner and helps it to realize when it is done with one complete sweep of the arena
			if waitActive == 0 then
				waitCounter = 50    -- a counter/timer which is set to 50 and during which the brain stays stationary to let the ground robots that are behind reach the group
				waitActive = Const_delay    -- a counter used to control the activation of the waitCounter (decreased by 1 at each step)
			end
			if waitCounter == 0 then
				vns.termination = vns.termination + 1    -- helps the brain to realize whether it has finished one complete sweep of the arena or not; each time vns.termination % 2 becomes 1, the entire arena has been completely swept
				makeDelay = Const_delay
				file2 = io.open ("termination.csv","w" )
				file2:write(vns.termination)
				file2:close()
				if (vns.termination % 2 == 0) then
					vns.LeftOrRight = -1 *vns.LeftOrRight   -- reversing the shift direction after sweeping the first lane in the new sweep round
					vns.escape = 1
				end
				if (vns.termination % 2 == 1) then  -- at the end of each sweep round (i.e., one complete sweep of the arena), the flag_NewRound flag is set to 1
					flag_NewRound = 1
				end

				if shiftCounter>0 then
					shiftCounter = 0

					if vns.reverseMove == 0 then
						vns.reverseMove = 1   -- reversing the movement direction

					else
						vns.reverseMove = 0   -- reversing the movement direction

					end
					shiftCounter =  Const_shift_counter

					
				end
			elseif shiftCounter ==  Const_shift_counter then	-- no movement for 50 steps (as waitCounter variable is set to 50, this condition is true for 50 steps)
				vns.speed(0.0, 0.0, 0.0)    
			end

		elseif vns.idS == vns.brainS and paraT.cornerN == 1 and makeDelay == 0 then     -- detecting a yellow LED in the front

			if waitActive == 0 then
				waitCounter = 50
				waitActive = Const_delay
			end
			if waitCounter == 0 then
				if shiftCounter>0 then
					Border_Flag = 1    -- the MNS is shifting; when this flag is 1, it means the MNS is shifting; this information is stored (the Border variable) and used by the boilerplate_loop_functions.cpp to calculate the opinions
					for _, robotVns3 in pairs(paraT.vehiclesTR) do
						vns.Msg.send(robotVns3.idS, "normal")    -- this message is used by module (i.e., boxAvoider.lua) to let the ground robots avoid obstacles when the MNS is shifting to left or right
					end
					if vns.LeftOrRight == 1 then
						vns.speed(0.0, 0.075, 0.0)
					elseif vns.LeftOrRight == -1 then
						vns.speed(0.0, -0.075, 0.0)
					end
					shiftCounter = shiftCounter - 1
				else
					
					vns.reverseMove = 1
					shiftCounter =  Const_shift_counter
					makeDelay = Const_delay
					
				end
			elseif shiftCounter ==  Const_shift_counter then	-- no movement for 50 steps (as waitCounter variable is set to 50, this condition is true for 50 steps)
				vns.speed(0.0, 0.0, 0.0)
			end
		elseif vns.idS == vns.brainS and paraT.cornerN == 2 and makeDelay == 0 then     -- detecting a yellow LED in the back
			if waitActive == 0 then
				waitCounter = 50
				waitActive = Const_delay
			end
			if waitCounter == 0 then
				if shiftCounter>0 then
					Border_Flag = 1    -- the MNS is shifting; when this flag is 1, it means the MNS is shifting; this information is stored (the Border variable) and used by the boilerplate_loop_functions.cpp to calculate the opinions
					for _, robotVns3 in pairs(paraT.vehiclesTR) do
						vns.Msg.send(robotVns3.idS, "normal")
					end
					if vns.LeftOrRight == 1 then
						vns.speed(0.0, 0.075, 0.0)
					elseif vns.LeftOrRight == -1 then
						vns.speed(0.0, -0.075, 0.0)
					end
					shiftCounter = shiftCounter - 1
				else
					vns.reverseMove = 0
					shiftCounter =  Const_shift_counter
					makeDelay = Const_delay
					
				end
			elseif shiftCounter ==  Const_shift_counter then	-- no movement for 50 steps (as waitCounter variable is set to 50, this condition is true for 50 steps)
				vns.speed(0.0, 0.0, 0.0)
			end
		elseif flag == 0 then    -- the following function is called by the brain in order to move forward or backward (depending on the movement direction); it is also called by non-brain UAVs and ground robots in order to move
			vns.move(transV3, rotateV3, paraT.stepCounter, paraT.energyN, randomflag)	
		end
	-- send drive to children
	for _, robotVns in pairs(vns.childrenTVns) do    -- for each children
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
		
		local dV3Temp = robotVns.rallyPoint.locV3 - robotVns.locV3

		if robotVns.robotType == "vehicle" and math.abs(robotVns.rallyPoint.locV3.y - robotVns.locV3.y) > 1  then    -- define a temporary  x coordinate for the target point that is closer to the ground robot compared to the x coordinate of the actual target point: to help the robot get back to the approximately middle of its lane faster if it is not there
			if reverseMove == 0 then 
				dV3Temp.x = dV3Temp.x-0.5
			else
				dV3Temp.x = dV3Temp.x+0.5
			end 
			
			local dTemp = dV3Temp:len()
			rallypointTransV3 =  rallypointScalar *dTemp * dV3Temp:nor()
		elseif robotVns.robotType == "vehicle" and math.abs(robotVns.rallyPoint.locV3.y - robotVns.locV3.y) <= 1  then    -- use the actual rallypoint used in the default MNS project	
			rallypointTransV3 = rallypointScalar / d * dV3:nor()
		else    -- for UAVs:  use the actual rallypoint used in the default MNS project			
			rallypointTransV3 = rallypointScalar / d * dV3:nor()
		end


		local rotateQ = robotVns.rallyPoint.dirQ * robotVns.dirQ:inv()
		local ang = rotateQ:getAng()
		if ang > math.pi then ang = ang - math.pi * 2 end
		local rallypointRotateV3 = rotateQ:getAxis() * ang
		
		-- the ground robots are not asked to move when their distance from the target point is less than 2 (in the view sight of the parent UAV) 	
		if robotVns.robotType == "vehicle" and d < 2 then rallypointTransV3 = Vec3:create() end
		-- the non-brain UAVs are not asked to move when their distance from the target point is less than 1 (in the view sight of the parent UAV) 
		if robotVns.robotType == "quadcopter" and d < 1 then rallypointTransV3 = Vec3:create() end
		-- the ground robots are not asked to change their orientation when their orientation with respect to the target point is less than math.pi/12 (in the sight view of the parent UAV)
		if robotVns.robotType == "vehicle" and rallypointRotateV3:len() < math.pi/12 then rallypointRotateV3 = Vec3:create() end
		-- the non-brain UAVs are not asked to change their orientation when their orientation with respect to the target point is less than math.pi/24 (in the sight view of the parent UAV)
		if robotVns.robotType == "quadcopter" and rallypointRotateV3:len() < math.pi/24 then rallypointRotateV3 = Vec3:create() end
		
		-- if the distance between a ground robot and its target point is greater than 5 (in the view sight of the parent UAV), the parent UAV adjusts the y coordinate of the robot based on the current brain's movement direction and the difference between y coordinate of the robot and that of its corresponding target point
		-- this is done through sending "adjust_left" and "adjust_right" messages to the corresponding ground robot
		-- by adjusting the y coordinate of a ground robot, it gets back to the approximately middle of its lane in the formation
		if robotVns.robotType == "vehicle" and d > 5 then 
			if ((robotVns.rallyPoint.locV3.y - robotVns.locV3.y > 1 and vns.reverseMove == 0) or (robotVns.rallyPoint.locV3.y - robotVns.locV3.y < -1 and vns.reverseMove == 1)) then
				vns.Msg.send(robotVns.idS, "adjust_left")			
			elseif ((robotVns.rallyPoint.locV3.y - robotVns.locV3.y < -1 and vns.reverseMove == 0) or (robotVns.rallyPoint.locV3.y - robotVns.locV3.y > 1 and vns.reverseMove == 1)) then
				vns.Msg.send(robotVns.idS, "adjust_right")					
			end
 		end

		totalTransV3 = rallypointTransV3
		totalRotateV3 = rallypointRotateV3

		local timestep = 1 / 50
		-- add parent speed; used in the default MNS version
		local parentScalar = 0
		totalTransV3 = totalTransV3 + (transV3+rotateV3*robotVns.locV3) * timestep * parentScalar
		totalRotateV3 = totalRotateV3 + rotateV3 * timestep * parentScalar

		
		-- the following "for" loop is used for robot-robot collision avoidance (while they are avoiding obstacles) which simulates a traffic rule strategy by a parent UAV
		-- based on the rule, when two ground robots get closer than ~3.7 cm to each other (edge-to-edge distance), depending on the movement direction of the brain, if the robot that is more "behind" senses an object (e.g., the other robot, an obstacle), it stops moving temporarily (but might change its direction based on some conditions; lines 206 to 222), and the other one avoids it
		-- to this end, a "avoid" message is sent to the robot that is located behind and a "avoid2" message is sent to the other one (avoid2 is used by the boxAvoider module)
		for _, robotVns2 in pairs(paraT.vehiclesTR) do    -- for all the ground robots that can be seen
			if robotVns.idS ~= robotVns2.idS then  
				relativeV = robotVns.locV3 - robotVns2.locV3
				local disTo = math.sqrt(relativeV.x * relativeV.x +
									relativeV.y * relativeV.y )
				
				if disTo < 12 then    -- this is equivalent to a ~3.7 cm distance in the simulation in practice
					if (vns.reverseMove == 0) then
						if vns.idS == vns.brainS then
							if robotVns2.locV3.x > robotVns.locV3.x then
								vns.Msg.send(robotVns.idS, "avoid")
								vns.Msg.send(robotVns2.idS, "avoid2")
							end
						elseif vns.idS ~= vns.brainS then
							if robotVns2.locV3.x > robotVns.locV3.x then
								vns.Msg.send(robotVns.idS, "avoid")
								vns.Msg.send(robotVns2.idS, "avoid2")
							end
							
						end
					elseif (vns.reverseMove == 1) then
						if vns.idS == vns.brainS then
							if robotVns2.locV3.x < robotVns.locV3.x then
								vns.Msg.send(robotVns.idS, "avoid")
								vns.Msg.send(robotVns2.idS, "avoid2")
							end
						elseif vns.idS ~= vns.brainS then
							if robotVns2.locV3.x < robotVns.locV3.x then
								vns.Msg.send(robotVns.idS, "avoid")
								vns.Msg.send(robotVns2.idS, "avoid2")
							end

						end 
					end
					
				end
				
			end
		end

		vns.Msg.send(robotVns.idS, "drive",
			{	yourLocV3 = robotVns.locV3,
				yourDirQ = robotVns.dirQ,

				transV3 = totalTransV3:nor(),
				rotateV3 = totalRotateV3:nor(),
			}
		)
		if vns.idS == vns.brainS then    -- the movement direction of the brain is sent to the other UAVs and ground robots; useful for the non-brain UAVs for managing the robot-robot collisions
			vns.Msg.send(robotVns.idS, "reverseMove", vns.reverseMove)
		end
		if vns.idS == vns.brainS then    -- according to the  Border_Flag, the brain sets and records the Border variable in the border_value.csv which is used by the boilerplate_loop_functions.cpp to calculate the opinions 	
			if Border_Flag == 1 then
				Border = 1
				local file = io.open ("border_value.csv","w" )
				file:write(Border)
				file:close()
			else
				Border = 0
				local file = io.open ("border_value.csv","w" )
				file:write(Border)
				file:close()
			end
		end			

		
	end

end

function Driver:move(transV3, rotateV3)
	
end

return Driver
