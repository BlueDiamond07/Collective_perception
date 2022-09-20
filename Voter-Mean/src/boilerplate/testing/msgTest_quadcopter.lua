------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";../?.lua"
package.path = package.path .. ";../math/?.lua"
--require("debugger")
Message = require("Message")

function showTable(table, number)
	if number == nil then number = 0 end
	if type(table) ~= "table" then return nil end

	for i, v in pairs(table) do
		local str = ""
		for j = 1, number do
			str = str .. "\t"
		end

		str = str .. tostring(i) .. "\t"

		if type(v) == "table" then
			print(str)
			showTable(v, number + 1)
		else
			str = str .. tostring(v)
			print(str)
		end
	end
end
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
	Message.prestep()
	showTable(Message.getAM("ALLMSG", "ALLMSG"))
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
