------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";math/?.lua"
package.path = package.path .. ";VNSModules/?.lua"
package.path = package.path .. ";testing/?.lua"

require("TableTools")

local IF = {} -- Robot Interface
local VNS = require("VNS")

VNS.EnableModules = {
	VNS.Modules.RandomWalker,
	VNS.Modules.BoxAvoider,
	VNS.Modules.Driver,
	
}

local vns

------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()
	vns = VNS:new{idS = IF.myIDS(), robotType = "vehicle"}
	reset()
	stepCounter = 0
end

-------------------------------------------------------------------
function reset()
	IF.setTag(IF.myIDS())
	vns:reset()
end

-------------------------------------------------------------------
function step()
	print("----------" .. IF.myIDS() .. "------------------")
	stepCounter = stepCounter+1
	vns:run{proxTR=IF.getProximityTableTR(),stepCounter = stepCounter, energyN = IF.getenergy()}

end

-------------------------------------------------------------------
function destroy()
end

------------------------------------------------------------------------
--   Customize Functions
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

-- the ground robot moves by calling this function
-- the speed of the ground robot is always 7.46 cm/s when it is moving forward
-- the speed of the left wheel of the ground robot and the speed of the right wheel of it are always +7.46 cm/s and -7.46 cm/s when it is turing right
-- the speed of the left wheel of the ground robot and the speed of the right wheel of it are always -7.46 cm/s and +7.46 cm/s when it is turing left	
VNS.move = function(transV3, rotateV3,stepCounter, energy)

		
	------------------------------------------------------------------------
	-- Handling of all motion vectors in the decentralized approaches 	
	------------------------------------------------------------------------
	local left = transV3.x -- take x of transV3 (a 3D vector used to indicate motion control)  
	local right = transV3.x -- take x of transV3 (a 3D vector used to indicate motion control)  

	local turnRate = 1
	local speedscale = 15

	local smalllimit = 0.3 -- used to set a minimum magnitude of the vector
	-- ensure the vector has a large enough magnitude for the robot
	if 0 <= transV3.x and transV3.x < smalllimit then transV3.x = smalllimit end
	if 0 > transV3.x and transV3.x >-smalllimit then transV3.x =-smalllimit end
		
	-- define left and right
	left  = left  - transV3.y/transV3.x * turnRate -- set as: x - (y/x)
	right = right + transV3.y/transV3.x * turnRate -- set as: x + (y/x)
		
		-- multiply both right and left by the same value
		left = left * speedscale
		right = right * speedscale
		
	-- to ensure the values are never 0 (but this does not occur in this project)
	if left == 0 and right == 0 then	-- not used
		left = 7.46			-- not used
		right = 7.46			-- not used
	end					-- not used

	-- set the speeds for the left and right motor
	-- possible outcomes are: 
		-- left wheel pos, right wheel neg (i.e., robot turns right at a constant radial velocity of 7.46 cm/s)
		-- left wheel neg, right wheel pos (i.e., robot turns left at a constant radial velocity of 7.46 cm/s)
		-- left wheel pos, right wheel pos (i.e., robot move forward at a constant linear velocity of 7.46 cm/s)
		
	if left > 0 then -- if (x - (y/x)) is positve, then set the left wheel to turn forward at the constant radial velocity (7.46 cm/s)
		left = 7.46 
	end
	if right > 0 then -- if (x + (y/x)) is positve, then set the right wheel to turn forward at the constant radial velocity (7.46 cm/s)
		right = 7.46
	end
	if left < -0 then -- if (x - (y/x)) is negative, then set the left wheel to turn backward at the constant radial velocity (7.46 cm/s)
		left = -7.46
	end
	if right < -0 then -- if (x + (y/x)) is negative, then set the right wheel to turn backward at the constant radial velocity (7.46 cm/s)
		right = -7.46
	end
		
	------------------------------------------------------------------------
		
		
	IF.setVelocity(left, right)

end


function IF.setenergy(m)    -- this function is not used
	energy = math.abs(m)
	local file = io.open ("energy.csv","w" )
	file:write(energy)
	file:close()
end

function IF.getenergy()    -- this function is not used
	local file = io.open ("energy.csv","r" )
	io.input(file)
	local energy = io.read()
	return energy
end

VNS.setSpeed = function(x, y)
	IF.setVelocity(x, -y)
end
------------------------------------------------------------------------
--  Robot Interface 
------------------------------------------------------------------------
function IF.getProximityTableTR()
	return robot.proximity
end

function IF.getProximityN(x)
	return robot.proximity[x]
end

function IF.myIDS()
	return robot.id
end

function IF.setTag(str)
	robot.tags.set_all_payloads(str)
end

function IF.setVelocity(x, y)
	robot.joints.base_wheel_left.set_target(x)
	robot.joints.base_wheel_right.set_target(-y)
end
