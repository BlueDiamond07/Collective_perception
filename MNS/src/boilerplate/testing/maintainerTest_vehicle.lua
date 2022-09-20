------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";../?.lua"
package.path = package.path .. ";../math/?.lua"
package.path = package.path .. ";../VNSModules/?.lua"
--require("debugger")
require("TableTools")

local IF = {} -- Robot Interface
local VNS = require("VNS")

VNS.EnableModules = {
	VNS.Modules.ParentWaitorDeny,
	VNS.Modules.LostCounter,

	--VNS.Modules.Assigner,
	VNS.Modules.Maintainer,
	--VNS.Modules.ShiftUpper,
	--VNS.Modules.Shifter,

	--VNS.Modules.RandomWalker,
	VNS.Modules.Driver,
}

local vns

------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()
	vns = VNS:new{id = IF.myIDS()}
	reset()
end

-------------------------------------------------------------------
function reset()
	IF.setTag(IF.myIDS())
	vns:reset()
end

-------------------------------------------------------------------
function step()
	print("----------" .. IF.myIDS() .. "------------------")

	vns:run()

	print("parent = ", vns.parentS)
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

VNS.Modules.Driver.move = function(transV3, rotateV3) --legacy mode
	VNS.move(transV3, rotateV3)
end

VNS.move = function(transV3, rotateV3)
	local left = transV3.x
	local right = transV3.x

	local turnRate = 1
	local speedscale = 25

	local smalllimit = 0.3
	if 0 <= transV3.x and transV3.x < smalllimit then transV3.x = smalllimit end
	if 0 > transV3.x and transV3.x >-smalllimit then transV3.x =-smalllimit end
	left  = left  - transV3.y/transV3.x * turnRate
	right = right + transV3.y/transV3.x * turnRate

	left = left * speedscale
	right = right * speedscale
	IF.setVelocity(left, right)
end

------------------------------------------------------------------------
--  Robot Interface 
------------------------------------------------------------------------
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
