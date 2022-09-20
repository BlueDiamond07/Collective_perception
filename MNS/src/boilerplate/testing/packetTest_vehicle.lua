------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";../?.lua"
--require("debugger")
local Packet = require("Packet")

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

local sendTable
------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()
	reset()
end

-------------------------------------------------------------------
function step()
	print("im vehicle-------------------")
	for iN, vT in ipairs(Packet.getTablesAT()) do
		print(iN)
		showTable(vT)
	end
	print("-------------------")

	Packet.sendTable(sendTable)
end

-------------------------------------------------------------------
function reset()
	sendTable = {
		"a", "b", "c", "d", 1, -2, -3.45, 4e2
	}
	sendTable["id"] = robot.id
	sendTable["aaa"] = "aaaa"
	sendTable["bbb"] = 1234.56
	sendTable["table"] = {
		"aa", "bb", "cc",
	}

	sendTable["table"]["tableagain"] = {
		lalala = "hehehe"
	}
end

-------------------------------------------------------------------
function destroy()
end

------------------------------------------------------------------------
--   Customize Functions
------------------------------------------------------------------------
Packet.sendData = function(dataAN)
	robot.radios["radio_0"].tx_data(dataAN)	
end
Packet.receiveDataAAN = function()
	return robot.radios["radio_0"].rx_data
end
