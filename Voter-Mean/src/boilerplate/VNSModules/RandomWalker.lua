-- Random Walker--------------------------------
------------------------------------------------------
-- Note: in the decentralized approaches (pheromone and voter-average), the vectors are used to determine whether the robot turns right at a constant velocity, turns left at a constant velocity, or moves forward at a constant velocity
-- for the details of how x and y are used to calculate the motion of the robot, please refer to structure_vehicle.lua
------------------------------------------------------
-- In this file, x is always set to 1 and y is always set ot 0, which means that the robot always moves forward.
------------------------------------------------------
-- Note: The choice of whether to follow the vector from this file or one from a different file is made in driver.lua
------------------------------------------------------

local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")

local RandomWalker = {VNSMODULECLASS = true}
RandomWalker.__index = RandomWalker

function RandomWalker:new()
	local instance = {}
	setmetatable(instance, self)
	return instance
end

function RandomWalker:run(vns, paraT) 		-- not used
	if vns.parentS ~= nil then 		-- not used
		vns.randomWalkerSpeed = nil 	-- not used
		return 				-- not used
	end

	local x = math.random() 
	local y = math.random() - 0.5
	local z = 0
	if vns.robotType == "quadcopter" then	-- implementation detail that does not impact robot behavior 
		x = 0				-- implementation detail that does not impact robot behavior
		y = 0				-- implementation detail that does not impact robot behavior
	elseif vns.robotType == "vehicle" then
		x = 1
		y = 0
	end
	local transV3 = Vec3:create(x,y,z):nor() * 0.5

	x = 0
	y = 0 
	z = math.random() - 0.5
	if vns.robotType == "quadcopter" then 	-- implementation detail that does not impact robot behavior
		z = 0				-- implementation detail that does not impact robot behavior
	end

	local rotateV3 = Vec3:create(x,y,z):nor() * 0.5

	vns.randomWalkerSpeed = {
		locV3 = transV3,
		dirV3 = rotateV3,
	}
end

return RandomWalker
