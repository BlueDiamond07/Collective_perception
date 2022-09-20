-- Assigner --------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")

local Assigner = {VNSMODULECLASS = true}
Assigner.__index = Assigner

function Assigner:new()
	local instance = {}
	setmetatable(instance, self)

	instance.parentLocV3 = Vec3:create()
	instance.childrenAssignTS = {}

	return instance
end

function Assigner:run(vns)
	-------------
	--  recruit from myAssignParent --> ack
	--  							--> bye
	--
	
	for _, msgM in ipairs(vns.Msg.getAM(vns.myAssignParent, "recruit")) do
		vns.Msg.send(msgM.fromS, "ack")
		if vns.myAssignParent ~= vns.parentS then
			vns.Msg.send(vns.parentS, "assign_bye") end
		vns.parentS = msgM.fromS
		break
	end

	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "assign_bye")) do
		if vns.childrenTVns[msgM.fromS] ~= nil then
			vns:deleteChild(msgM.fromS)
		end
	end

	for _, msgM in ipairs(vns.Msg.getAM(vns.parentS, "assign")) do
		vns.myAssignParent = msgM.dataT.assignToS
	end

	-- update parentLoc by drive message
	if vns.parentS ~= nil then
		for _,msgM in ipairs(vns.Msg.getAM(vns.parentS, "drive")) do
			local yourLocV3 = vns.Msg.recoverV3(msgM.dataT.yourLocV3)
			local yourDirQ = vns.Msg.recoverQ(msgM.dataT.yourDirQ)
			self.parentLocV3 = Linar.myVecToYou(Vec3:create(), yourLocV3, yourDirQ)
			break
		end
	else
		self.parentLocV3 = Vec3:create()
	end

	-- allcate children
	if self.childrenAssignTS == nil then return end

	for idS, childVns in pairs(vns.childrenTVns) do
		if self.childrenAssignTS[idS] ~= nil then
			local assignToS = self.childrenAssignTS[idS]
			if vns.childrenTVns[assignToS] ~= nil then
				childVns.rallyPoint = {
					locV3 = vns.childrenTVns[assignToS].locV3,
					--dirQ = vns.childrenTVns[assignToS].dirQ,
					dirQ = Quaternion:create()
				}
			elseif vns.parentS == assignToS then
				childVns.rallyPoint = {
					locV3 = self.parentLocV3,
					dirQ = Quaternion:create()
				}
			end
		end
	end
end

function Assigner:assign(_childidS, _assignToIds, vns)
	if self.childrenAssignTS == nil then
		self.childrenAssignTS = {} end
	self.childrenAssignTS[_childidS] = _assignToIds
	vns.Msg.send(_childidS, "assign", {assignToS = _assignToIds})

	--update rally point immediately
	local assignToS = _assignToIds
	local childVns = vns.childrenTVns[_childidS]
	if childVns == nil then return end
	if vns.childrenTVns[assignToS] ~= nil then
		childVns.rallyPoint = {
			locV3 = vns.childrenTVns[assignToS].locV3,
			--dirQ = vns.childrenTVns[assignToS].dirQ,
			dirQ = Quaternion:create()
		}
	elseif vns.parentS == assignToS then
		childVns.rallyPoint = {
			locV3 = self.parentLocV3,
			dirQ = Quaternion:create()
		}
	end
end

function Assigner:unsign(_childidS, vns)
	if self.childrenAssignTS ~= nil then
		self.childrenAssignTS[_childidS] = nil
	end
	vns.Msg.send(_childidS, "assign", {assignToS = nil})

	local childVns = vns.childrenTVns[_childidS]
	childVns.rallyPoint = {
		locV3 = Vec3:create(),
		dirQ = Quaternion:create(),
	}
end

function Assigner:deleteChild(idS)
	self.childrenAssignTS[idS] = nil
end

function Assigner:deleteParent(vns)
	self.parentLocV3 = Vec3:create()
	vns.myAssignParent = nil
end

function Assigner:reset(vns)
	self.childrenAssignTS = {}
	vns.myAssignParent = nil
end

return Assigner
