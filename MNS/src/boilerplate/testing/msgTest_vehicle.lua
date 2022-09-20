------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";../?.lua"
package.path = package.path .. ";../math/?.lua"
--require("debugger")
Message = require("Message")

------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()
	reset()
end

-------------------------------------------------------------------
function reset()
end

-------------------------------------------------------------------
function step()
	Message.send("quadcopter0", "fly", {1, 2, robot.id})
	Message.send("quadcopter0", "run", {1, 2, robot.id})
end

-------------------------------------------------------------------
function destroy()
end

------------------------------------------------------------------------
--   Customize Functions
------------------------------------------------------------------------
Message.myIDS = function()
	return robot.id
end

Message.Packet.sendData = function(dataAN)
	robot.radios["radio_0"].tx_data(dataAN)	
end
Message.Packet.receiveDataAAN = function()
	return robot.radios["radio_0"].rx_data
end
