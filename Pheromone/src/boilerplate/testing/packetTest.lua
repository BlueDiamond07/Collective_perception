package.path = package.path .. ";../?.lua"
local PacketTool = require("Packet")
--require("../debugger")

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

print("test")
local sendTable = {
	"a", "b", "c", "d", 1, -2, -3.45, 4e2
}
sendTable["aaa"] = "aaaa"
sendTable["bbb"] = 1234.56
sendTable["table"] = {
	"aa", "bb", "cc",
}

sendTable["table"]["tableagain"] = {
	lalala = "hehehe"
}


local sendStr = PacketTool.tableToStr(sendTable)
print("sendStr = ", sendStr)
local data = PacketTool.strToBytesAN(sendStr)


local getStr = PacketTool.bytesToStrS(data)
print("getStr = ", getStr)
local getTable = PacketTool.strToTable(getStr)

showTable(getTable, 0)

