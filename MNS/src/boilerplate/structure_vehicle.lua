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
	VNS.Modules.ParentWaitorDeny,
	VNS.Modules.LostCounter,
	VNS.Modules.Shifter,
	VNS.Modules.RandomWalker,
	VNS.Modules.BoxAvoider,
	VNS.Modules.Driver,
}

local vns

------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()
	vns = VNS:new{id = IF.myIDS(), robotType = "vehicle"}
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
	--print("----------" .. IF.myIDS() .. "------------------")
	stepCounter = stepCounter+1
	vns:run{proxTR=IF.getProximityTableTR(),stepCounter = stepCounter, energyN = IF.getenergy(), myid = IF.myIDS()}
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

VNS.move = function(transV3, rotateV3,stepCounter, energy, randomflag)
	local left = transV3.x
	local right = transV3.x

	local turnRate = 1
	local speedscale = 15
	if randomflag ~= nil then
		speedscale = 20
	end
	local smalllimit = 0.3
	if 0 <= transV3.x and transV3.x < smalllimit then transV3.x = smalllimit end
	if 0 > transV3.x and transV3.x >-smalllimit then transV3.x =-smalllimit end
	left  = left  - transV3.y/transV3.x * turnRate
	right = right + transV3.y/transV3.x * turnRate
	-- we do not let the ground robots move backward while the MNS is moveing
	-- to this end, when both left wheel and right wheel speeds sent by a parent UAV are negative, the ground robot quickly rotates to adjust its heading with respect to the heading of the parent UAV
	-- after performing this action, the robot is not received negative speeds for both wheels concurrently
	-- this action is performed in initialization phase with higher probability, but later when the MNS moves, it might be rarely performed (i.e., with a very low probability) to avoid a robot of avoiding a block that it has already avoided	
	if right < 0 and left < 0 then
		if vns.flagTurn == 0 then
			IF.setVelocity(-40, 40)    -- rotate left
		elseif vns.flagTurn == 1 then
			IF.setVelocity(40, -40)    -- rotate right
		end
	else	
		left = left * speedscale
		right = right * speedscale
		-- if a wheel speed is higher than 10, it is set to 10
		-- if a wheel speed is less than -10, it is set to -10 	
		-- maximum speed of the ground robot is 10 cm/s
		if left > 10 then
			left = 10
		end
		if right > 10 then
			right = 10
		end
		if left < -10 then
			left = -10
		end
		if right < -10 then
			right = -10
		end

		IF.setVelocity(left, right)
	end
	
end

VNS.OB_move = function(left, right)
	IF.setVelocity(left, right)
end

function IF.setenergy(m)    -- not used
	energy = math.abs(m)
	local file = io.open ("energy.csv","w" )
	file:write(energy)
	file:close()
end

function IF.getenergy()    -- not used
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
