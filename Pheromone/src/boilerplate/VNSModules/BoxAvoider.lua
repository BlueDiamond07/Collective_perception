-- External Avoider --------------------------------------
------------------------------------------------------
-- Defines whether the robot turns to the left or to the right in 
-- the current time step. If an obstacle is sensed to the right of 
-- a robot’s heading, it turns left, and vice versa. 
-- Note that each proximity sensor's view is a line, and the sensing
-- range is 3cm from the sensor (i.e., from the edge of the robot).
------------------------------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")

local BoxAvoider = {VNSMODULECLASS = true}
BoxAvoider.__index = BoxAvoider

function BoxAvoider:new()
	local instance = {}
	setmetatable(instance, self)
	return instance
end

function BoxAvoider:run(vns, paraT)
	if vns.boxAvoiderSpeed == nil then
		vns.boxAvoiderSpeed = {
			locV3 = Vec3:create(),
			dirV3 = Vec3:create(),
		}
	end
	
	------------------------------------------------------
	-- Choose whehter the robot turns right or left. (Only the signs of the values are used here. The precise values are used only in the MNS setup.)
	------------------------------------------------------
	if paraT.proxTR[12]~=0 then -- for the proximity sensor 30 degrees to the right of the heading
		if paraT.proxTR[12]~=0 then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, 0.08, 0) -- the robot turns left
		else
			vns.boxAvoiderSpeed.locV3 = Vec3:create(-1, 0, 0) -- not used
		end
		
	elseif paraT.proxTR[2]~=0 then -- for the proximity sensor 30 degrees to the left of the heading
		if paraT.proxTR[2]~=0  then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, -0.08, 0) -- the robot turns right
		else
			vns.boxAvoiderSpeed.locV3 = Vec3:create(-1, 0, 0)  -- not used
		end
		vns.flagTurn = 0
	elseif paraT.proxTR[1]~=0 then -- for the proximity sensor in front of the heading
		if paraT.proxTR[1]~=0  then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, -0.08, 0) -- the robot turns right
		else
			vns.boxAvoiderSpeed.locV3 = Vec3:create(-1, 0, 0)  -- not used
		end
		
	elseif paraT.proxTR[3]~=0 then -- for the proximity sensor 60 degrees to the left of the heading
		if paraT.proxTR[3]~=0  then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.1, -0.08, 0) -- the robot turns right
		else
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0, -1, 0)  -- not used
		end
		
		
	elseif paraT.proxTR[11]~=0 then -- for the proximity sensor 60 degrees to the right of the heading
		if paraT.proxTR[11]<0.99 then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0.1, 0.08, 0) -- the robot turns left
		else
			vns.boxAvoiderSpeed.locV3 = Vec3:create(0, 1, 0) -- the robot turns left
		end

	end


end



return BoxAvoider

