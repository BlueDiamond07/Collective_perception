local Packet = {}
-----------------------------------------------------------
function Packet.sendData(data)
	print("Packet.sendData() needs to be implement")
end

function Packet.receiveDataAAN()
	print("Packet.receiveDataAAN() needs to be implement here")
end

-----------------------------------------------------------
function Packet.sendTable(table)
	local str = Packet.tableToStr(table)
	local dataAN = Packet.strToBytesAN(str)
	Packet.sendData(dataAN)
end

function Packet.getTablesAT()
	local datasAAN = Packet.receiveDataAAN()
	if type(datasAAN) ~= "table" then return {} end

	local tablesAT = {}
	for iN, dataAN in ipairs(datasAAN) do
		local str = Packet.bytesToStrS(dataAN)
		local table = Packet.strToTable(str)
		tablesAT[iN] = table
	end
	return tablesAT
end

-----------------------------------------------------------
function Packet.strToBytesAN(str)
	local bytesAN = {}
	for i = 1, string.len(str) do
		bytesAN[i] = string.byte(str, i)
	end
	return bytesAN
end

function Packet.bytesToStrS(bytesAN)
	local str = ""
	for i, v in ipairs(bytesAN) do
		--str = str .. string.char(bytesAN[i])
		str = str .. string.char(v)
	end
	return str
end
-----------------------------------------------------------
function Packet.tableToStr(table)
	local str = "B" -- table begin
	for i, v in pairs(table) do
		if type(i) == "number" then
			str = str .. " n " .. tostring(i)
		elseif type(i) == "string" then
			str = str .. " s " .. tostring(i)
		-- for now index can't be table
		--elseif type(i) == "table" then					
		--	str = str .. " t " .. Packet.tableToStr(i)
		end

		if type(v) == "number" then
			str = str .. " n " .. tostring(v)
		elseif type(v) == "string" then
			str = str .. " s " .. tostring(v)
		elseif type(v) == "table" then
			str = str .. " t " .. Packet.tableToStr(v)
		end
	end
	str = str .. " E" -- table begin
	return str
end

function Packet.strToTable(str)
	local arr = Packet.strToArrS(str)
	local table = Packet.arrSToTable(arr, 1)
	return table
end

function Packet.strToArrS(str)
	local i = 1
	local arr = {}
	for vS in string.gmatch(str, "%S+") do -- get each divided by space
		arr[i] = vS
		i = i + 1
	end
	return arr
end

function Packet.arrSToTable(arr, i) 
	-- and array is an array of string in a table format, start parsing from i'th element
	if arr[i] ~= "B" then return nil end

	local table = {}
	local len = 1	-- len is the length of this table from "B" to "E"
	i = i + 1

	while arr[i] ~= "E" do
		local index, value
		if     arr[i] == "n" then index = tonumber(arr[i+1])
		elseif arr[i] == "s" then index = arr[i+1]
		-- for now index can't be table
		--elseif arr[i] == "t" then 
		--	local l
		--	index, l = Packet.arrToTable(arr, i+1)
		--	i = i + l - 1
		end
		i = i + 2

		local l = 1
		if     arr[i] == "n" then value = tonumber(arr[i+1])
		elseif arr[i] == "s" then value = arr[i+1]
		elseif arr[i] == "t" then
			value, l = Packet.arrSToTable(arr, i+1)
		end
		i = i + 1 + l
		len = len + 3 + l
		table[index] = value
	end
	len = len + 1

	return table, len
end

-----------------------------------------------------------

return Packet
