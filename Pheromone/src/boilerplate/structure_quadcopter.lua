------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";math/?.lua"
package.path = package.path .. ";VNSModules/?.lua"
package.path = package.path .. ";testing/?.lua"

require("TableTools")

local IF = {} -- Robot Interface
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")

local VNS = require("VNS")

VNS.EnableModules = {
	VNS.Modules.VehicleConnector,
	VNS.Modules.QuadcopterConnector,
	VNS.Modules.LostCounter,
	VNS.Modules.Shifter,
	VNS.Modules.RandomWalker,
	VNS.Modules.Driver,
}

local vns

------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()
	stepCounter = 0   -- time step

	local dis = 20
	pr = {}
	local structure = {
		locV3 = Vec3:create(),
		dirQ = Quaternion:create(),
		children = {
		
			
		},
	}

	vns = VNS:new{idS = IF.myIDS(), robotType = "quadcopter"}
	vns.modules[4]:setGene(vns, structure)

	reset()
end

-------------------------------------------------------------------
function reset()
	vns:reset()
end

-------------------------------------------------------------------
function step()
	print("----------" .. IF.myIDS() .. "------------------")
	stepCounter = stepCounter+1
	Nb_robots = 8   -- number of robots: used in line 61  
	if stepCounter%200 == 1 then   -- at each 200 steps select a random number between 30 to 75 for each of the ground robots
		for i = 1,Nb_robots do    -- a loop that goes through each of the ground robots
			pr[i] = math.random(30, 75)    -- select a random number between 30 to 75 to simulate a turn between 30 degrees to approximately 90 degrees
				-- IMPORTANT -- the robot turns according to a loop function of 10 degree turns where it checks if it has turned enough or not, and there can be delays in this check
				--------------- the delay can cause the robot to overshoot the target angle by up to 2 full 10-degree loop cycles
				--------------- this means that the delay can cause the robot to overshoot by up to 20 degrees
				--------------- the delays are not always the same - sometimes there is no delay
				--------------- in short, the possible delays can cause the robot to overshoot the target angle by 0-20 degrees
		end
	end

	vns:run{vehiclesTR = getVehicleTR(), 
	        boxesTR = getBoxesTR(),
	        predatorsTR = getPredatorsTR(), cornerN = detectCorner(), stepCounter = stepCounter}
	if stepCounter == 1 then   -- initialize vectors and variables
		circulate_counter = {}   -- this vector stores the time step at which a ground robot senses a pheromone sourse for the first time in the duration that it can show reaction to pheromone
		forceCounter = {}   -- this vector stores a counter for a ground robot so that it is forced to moved forward  
		forceFlag = {}   -- when this flag is 0 for a ground robot, it can show reaction to a pheromone source
		for i, v in pairs(vns.childrenTVns) do -- initialize vectors and variables for all the ground robots
			circulate_counter[v.idS] = nil
			forceCounter[v.idS] = 0
			forceFlag[v.idS] = 0
		end 
	end
	if stepCounter > 1 then
		local loopCount = 1    -- a counter to address each of the ground robots in the loop, one by one
		for i2, v2 in pairs(getVehicleTR()) do    -- for each ground robot do:
			
			if forceFlag[v2.idS] == 1 then    -- if forceFlag is 1, the ground robot moves forward and does not show any reaction to a pheromone source
				circulate_counter[v2.idS] = nil
				if (forceCounter[v2.idS] ~= 0) then    -- if the forceFlag is greater than 0 do:
					forceCounter[v2.idS] = forceCounter[v2.idS] - 1    -- decrease forceFlag by one at each time step
				else
					forceFlag[v2.idS] = 0
				end

			else
				if (circulate_counter[v2.idS]~=nil and (stepCounter - circulate_counter[v2.idS]==50)) then   -- check if the difference between the current time step and the time that the robot has sensed a pheromone source for the first time in the duration that the forceFlag is not 1 is equal to 50 or not
					forceFlag[v2.idS] = 1    -- if the mentioned difference is 50, set forceFlag to 1
					circulate_counter[v2.idS] = nil    -- if the mentioned difference is 50, set circulate_counter to null
					forceCounter[v2.idS] = 200    -- if the mentioned difference is 50, set forceCounter to 200; in this way the robot does not show reaction for 200 steps
				end
			end
			
			for i1, v1 in pairs(getPredatorsTR()) do -- for each block do:
				relBoxToRobotV = {x = v1.locV3.x - v2.locV3.x, y = v1.locV3.y - v2.locV3.y,}   -- a vector from the robot's center to the block's center 
				disBoxToRobot = math.sqrt(relBoxToRobotV.x * relBoxToRobotV.x + relBoxToRobotV.y * relBoxToRobotV.y )   -- distance between cenhter of the robot and center of the block
				-- in argos simulator based on the height of the quadcopter from the ground in the experiment setup (.argos file), disBoxToRobot <= 12 is equivalent to disBoxToRobot <= ~40 cm in practice
				if (disBoxToRobot <= 12 and v1.colorS == "blue") then    -- when the distance between the robot and the block is less than or equal to ~40 cm and the block is active do:
				-- IMPORTANT -- disBoxToRobot is measured according to the image seen by the camera
				--------------- for instance, distance of 12 in the camera (based on the camera height in the arena and the lens angle of the camera) might indicate ~40 cm on the ground
						
					local a = {x = v2.front.x - v2.locV3.x, y = v2.front.y - v2.locV3.y,}   -- a vector from the robot's center to it's front (i.e., robot's heading) 
					local ans = nil
					ans = math.acos(((a.x*relBoxToRobotV.x)+(a.y*relBoxToRobotV.y)) / (math.sqrt((a.x * a.x + a.y * a.y)) * disBoxToRobot))    -- caclulate the angle between the robot's heading and the vector from robot's center to the block's center
									
					if forceFlag[v2.idS] == 0 then   -- if this flag is 0, the robot can show reaction to the active block

						if (math.abs(math.deg(ans)) < pr[loopCount]) then    -- this reaction is based on the random number that is selected at each 200 steps and the angle calculated in line 114
						
							if (circulate_counter[v2.idS] == nil) then    -- if this is the first time that the robot senses an active block during the duration that its forceFlag is 0
								circulate_counter[v2.idS] = stepCounter    -- remember this time step
							end
							vns.Msg.send(v2.idS, "circulate")    -- this message should be sent to the ground robot to make it rotate; this process has one step delay as the ground robot receives the command in the next step

						end
						
					end

				end
			end
			loopCount = loopCount + 1
		end

	end


end

-------------------------------------------------------------------
function destroy()
end

------------------------------------------------------------------------
--   Customize Functions
------------------------------------------------------------------------
function detectCorner()    -- this function is not used
	local array = {}
	local cornerflag = 0
	for i, detectionT in ipairs(IF.getLEDsT()) do 
		array[i]={x = detectionT.center.x, y = detectionT.center.y}
		if ((detectionT.color.blue == 255) and detectionT.center.x>240 and detectionT.center.x<=390) then
			
			cornerflag = 1
		end

		if cornerflag == 1 and detectionT.color.blue == 255 then
			for i=1, #array do
				for j=i+1, #array do
					if (array[i].x > 490 and array[i].x > 490) and math.abs(array[i].x-array[j].x) <= 1 then
						 cornerflag = 0		
					end
				end
			end
		end	
	end
	
	return cornerflag
end

function getVehicleTR()    -- a function that gets all the ground robots and returns them
	local vehicleTR = {}
	for i, tagDetectionT in pairs(IF.getTagsT()) do
		-- a tag detection is a complicated table
		-- containing center, corners, payloads
		
		-- get robot info
		local locV3, dirQ, idS, front = getVehicleInfo(tagDetectionT)
			-- locV3 for vector3 {x,y,z}
			-- loc (0,0) in the middle, x+ right, y+ up , 
			-- dir a quaternion, x+ as 0

		vehicleTR[idS] = {locV3 = locV3, dirQ = dirQ, idS = idS, front = front}
	end
	return vehicleTR
end

function getVehicleInfo(tagT)
	-- a tag is a complicated table with center, corner, payload
	local degQ, front = calcVehicleDir(tagT.corners)
		-- a direction is a Quaternion
		-- with 0 as the x+ axis of the quadcopter
	front.x = front.x - 320
	front.y = -(((tagT.corners[3].y + tagT.corners[4].y) / 2)-240) 	
	local x = tagT.center.x - 320
	local y = tagT.center.y - 240
	y = -y 				-- make it left handed coordination system
	local z = 0
	local locV3 = Vec3:create(x,y,z)
	local idS = tagT.payload

	return locV3, degQ, idS, front
end

function calcVehicleDir(corners)
	-- a direction is a number from 0 to 360, 
	-- with 0 as the x+ axis of the quadcopter
	local front = {}
	front.x = (corners[3].x + corners[4].x) / 2
	front.y = -(corners[3].y + corners[4].y) / 2
	local back = {}
	back.x = (corners[1].x + corners[2].x) / 2
	back.y = -(corners[1].y + corners[2].y) / 2
	local deg = calcDir(back, front)
	return deg, front
end

function calcDir(center, target)
	-- from center{x,y} to target{x,y} in left hand
	-- return a Quaternion
	local x = target.x - center.x
	local y = target.y - center.y
	local rad = math.atan(y / x)
	if x < 0 then
		rad = rad + math.pi
	end
	if rad < 0 then
		rad = rad + math.pi*2
	end
	return Quaternion:create(0,0,1, rad)
end

function getBoxesTR()
	local boxesTR = {}   -- vt for vector, which a table = {x,y}
	for i, ledDetectionT in ipairs(IF.getLEDsT()) do	
		-- a detection is a table
		-- {center = {x,y}, color = {blue, green, red}}
		local locV3, dirQ, colorS = getBoxInfo(ledDetectionT)
		boxesTR[i] = {locV3 = locV3, dirQ = dirQ, colorS = colorS}
	end	
	return boxesTR
end

function getBoxInfo(detection)
	local x = detection.center.x - 320
	local y = detection.center.y - 240
	y = -y
	local z = 0
	local locV3 = Vec3:create(x,y,z)
	local dirQ = Quaternion:create()
	local colorS = "red"
	return locV3, dirQ, colorS
end

function getPredatorsTR()    -- a function that gets all the blocks and returns them
	local predatorsTR = {}   -- vt for vector, which a table = {x,y}
	local j = 0
	for i, ledDetectionT in ipairs(IF.getLEDsT()) do	
		if ledDetectionT.color.blue > 150 or ledDetectionT.color.green > 150 or ledDetectionT.color.red > 150 then -- else continue
		j = j + 1
		-- a detection is a table
		-- {center = {x,y}, color = {blue, green, red}}
		local locV3, dirQ, colorS = getPredatorInfo(ledDetectionT)
		predatorsTR[j] = {locV3 = locV3, dirQ = dirQ, colorS = colorS}
	end	end
	return predatorsTR
end

function getPredatorInfo(detection)    
	local x = detection.center.x - 320
	local y = detection.center.y - 240
	y = -y
	local z = 0
	local locV3 = Vec3:create(x,y,z)
	local dirQ = Quaternion:create()
	local colorS -- = "blue" 
	if detection.color.blue > 150 and detection.color.red == 0 then
		colorS = "blue"
	elseif detection.color.green > 150 then
		colorS = "green"
	elseif detection.color.red > 150 and detection.color.blue == 0 then
		colorS = "red"
	end
	return locV3, dirQ, colorS
end

------------------------------------------------------------------------
--   VNS Callback Functions
------------------------------------------------------------------------
VNS.Msg.myIDS = function()
	return IF.myIDS()
end

VNS.Msg.Packet.sendData = function(dataAN)
	robot.radios["radio_0"].tx_data(dataAN)	
end

VNS.Msg.Packet.receiveDataAAN = function()
	return robot.radios["radio_0"].rx_data
end

VNS.move = function(transV3, rotateV3, time, energy)
	local speedscale = 0.15
	local turnscale = 1
	local x = transV3.x * speedscale
	local y = transV3.y * speedscale
	local w = rotateV3:len() * turnscale
	if rotateV3.z < 0 then w = -w end
	IF.setVelocity(x, y, w)

end

VNS.speed = function(x, y, w)
	IF.setVelocity(x, y, w)
end
------------------------------------------------------------------------
--  Robot Interface 
------------------------------------------------------------------------
function IF.myIDS()
	return robot.id
end

function IF.getTagsT()
	return robot.cameras.fixed_camera.tag_detector
end

function IF.getLEDsT()
	return robot.cameras.fixed_camera.led_detector
end

function IF.setVelocity(x, y, w)
	--quadcopter heading is the x+ axis
	local thRad = robot.joints.axis2_body.encoder
	local xWorld = x * math.cos(thRad) - y * math.sin(thRad)
	local yWorld = x * math.sin(thRad) + y * math.cos(thRad)
	robot.joints.axis0_axis1.set_target(xWorld)
	robot.joints.axis1_axis2.set_target(yWorld)
	robot.joints.axis2_body.set_target(w)
end

