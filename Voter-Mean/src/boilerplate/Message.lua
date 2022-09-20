local Message = {}
Message.Packet = require("Packet")
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")

Message.list = {}


function Message.prestep()
	Message.list = {}
end

function Message.arrange()
	for iN, msgM in ipairs(Message.Packet.getTablesAT()) do
		if msgM.toS == Message.myIDS() or msgM.toS == "broadcast" then
			local i = #Message.list + 1
			Message.list[i] = msgM
			if Message.list[msgM.cmdS] == nil then
				Message.list[msgM.cmdS] = {}
			end
			local i = #Message.list[msgM.cmdS] + 1
			Message.list[msgM.cmdS][i] = msgM
		end
	end
	Message.list.ListArranged = true
end

function Message.myIDS()
	print("Message.myIDS() needs to be implement")
end

function Message.send(toIDS, cmdS, dataT)
	Message.Packet.sendTable{
		toS = toIDS,
		fromS = Message.myIDS(),
		cmdS = cmdS,
		dataT = dataT,
	}
end


function Message.getAM(fromS, cmdS)
	if Message.list.ListArranged == nil then
		Message.arrange()
	end

	local listAM = {}
	local i = 0
	local searchList

	if cmdS == "ALLMSG" then searchList = Message.list
		                else searchList = Message.list[cmdS] or {} end
	for iN, msgM in ipairs(searchList) do
		if msgM.toS == Message.myIDS() or msgM.toS == "broadcast" then
		if fromS == "ALLMSG" or fromS == msgM.fromS then
		if cmdS == "ALLMSG" or cmdS == msgM.cmdS then
			i = i + 1
			listAM[i] = msgM
		end end end
	end

	return listAM
end

function Message.recoverTable(table)
	if type(table) ~= "table" then return table end
	for i, v in pairs(table) do
		table[i] = Message.recoverTable(v)
		table[i] = Message.recoverV3(table[i])
		table[i] = Message.recoverQ(table[i])
	end
	return table
end

function Message.recoverV3(v3T)
	if type(v3T) ~= "table" then return v3T end
	if type(v3T.x) == "number" and 
	   type(v3T.y) == "number" and 
	   type(v3T.z) == "number" then
		return Vec3:create(v3T.x, v3T.y, v3T.z)
	else
		return v3T
	end
end

function Message.recoverQ(qT)
	if type(qT) ~= "table" then return qT end
	qT.v = Message.recoverV3(qT.v)
	if type(qT.v) == "table" and 
	   type(qT.v.x) == "number" and
	   type(qT.v.y) == "number" and
	   type(qT.v.z) == "number" and
	   type(qT.w) == "number" then
		return Quaternion:createFromHardValue(qT.v.x, qT.v.y, qT.v.z, qT.w)
	else
		return qT
	end
end

return Message
