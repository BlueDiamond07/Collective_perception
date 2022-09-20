------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";../?.lua"
package.path = package.path .. ";../math/?.lua"
--require("debugger")
require("TableTools")

local IF = {} -- Robot Interface
local VNS = require("VNS")

VNS.runModules = {
	VNS.modules.parentwaitor,
	--VNS.modules.driver,
}

local vns

------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()
	IF.setTag(IF.myIDS())
	reset()
end

-------------------------------------------------------------------
function reset()
	vns = VNS:new{id = IF.myIDS()}
end

-------------------------------------------------------------------
function step()
	print("----------" .. IF.myIDS() .. "------------------")
	--showTable(vns)

	vns:run()
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

------------------------------------------------------------------------
--  Robot Interface 
------------------------------------------------------------------------
function IF.myIDS()
	return robot.id
end

function IF.setTag(str)
	robot.tags.set_all_payloads(str)
end
