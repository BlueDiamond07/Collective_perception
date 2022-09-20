-- Shifter: up and down -------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")
local Maintainer = require("Maintainer")

local Shifter = {VNSMODULECLASS = true}
setmetatable(Shifter, Maintainer)
Shifter.__index = Shifter

local Counter = {robotTypes = {"vehicle", "quadcopter"}}

function Shifter:new()
	local instance = Maintainer:new()
	setmetatable(instance, self)

	instance.need = {}
	
	instance.myLastSentNeed = {}
	
	instance.getBranch = false
	
	return instance
end

function Shifter:reset(vns)
	Maintainer.reset(self, vns)
	self.need = {}
	self.myLastSentNeed = {}
end

function Shifter:deleteChild(idS)
	Maintainer.deleteChild(self, idS)
	self.need[idS] = nil
	self.myLastSentNeed[idS] = nil
end

function Shifter:deleteParent(vns)
	Maintainer.deleteParent(self, vns)
	self.need[vns.parentS] = nil
	self.myLastSentNeed[vns.parentS] = nil
end

function Shifter:setStructure(vns, structure)
	Maintainer.setStructure(self, vns, structure)
	self.getBranch = true
end

function Shifter:run(vns)
	Maintainer.run(self, vns)

	-- receive need
	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "need")) do
		if msgM.fromS == vns.parentS or 
		   vns.childrenTVns[msgM.fromS] ~= nil then
			self.need[msgM.fromS] = msgM.dataT
		end
	end

	local need = {}
	if self.structure ~= nil and
	   self.structure.children ~= nil then
		for idS, childVns in pairs(vns.childrenTVns) do
			need[idS] = Counter:add(self.need[idS], nil)
			-- count already assigned robots
			for idSmore, childVnsmore in pairs(vns.childrenTVns) do
				if self.childrenAssignTS[idSmore] == idS then
					-- idSmore is a more robot assigned to idS
					if need[idS][childVnsmore.robotType] > 0 then
						need[idS][childVnsmore.robotType] =
						need[idS][childVnsmore.robotType] - 1
					else
						self:unsign(idSmore, vns)
					end
				end
			end

			-- assign more robots
			for idSmore, childVnsmore in pairs(vns.childrenTVns) do
				if need[idS][childVnsmore.robotType] > 0 then
					if self.allocated[idSmore] == nil and 
					   self.childrenAssignTS[idSmore] == nil then
						self:assign(idSmore, idS, vns)
						need[idS][childVnsmore.robotType] =
						need[idS][childVnsmore.robotType] - 1
					end
				end
			end
		end 
	end

	-- count unallocated branches
	local unAllocatedBranchTN = Counter:new()
	if self.structure ~= nil and
	   self.structure.children ~= nil then
		for _, branchT in ipairs(self.structure.children) do
			if branchT.actorS == nil then
				if unAllocatedBranchTN[branchT.robotType] == nil then
					unAllocatedBranchTN[branchT.robotType] = 0 end
				unAllocatedBranchTN[branchT.robotType] = 
					unAllocatedBranchTN[branchT.robotType] + 1
			end
		end
	end

	-- count more robots
	local moreRobotsTN = Counter:new()
	for idS, childVns in pairs(vns.childrenTVns) do
		if self.allocated[idS] == nil and 
		   self.childrenAssignTS[idS] == nil then
			if moreRobotsTN[childVns.robotType] == nil then
				moreRobotsTN[childVns.robotType] = 0 end
			moreRobotsTN[childVns.robotType] = 
				moreRobotsTN[childVns.robotType] - 1
		end
	end


	-- add children needs
	local totalNeedTN = Counter:new()
	for idS, childVns in pairs(vns.childrenTVns) do
		if need[idS] ~= nil then
			totalNeedTN = Counter:add(totalNeedTN, need[idS])
		end
	end

	if need[vns.parentS] ~= nil then
		totalNeedTN = Counter:add(totalNeedTN,
		                          need[vns.parentS])
	end

	totalNeedTN = Counter:add(totalNeedTN, unAllocatedBranchTN)
	totalNeedTN = Counter:add(totalNeedTN, moreRobotsTN)


	-- send need to parent
	if vns.parentS == nil then
		self.getBranch = false
	else
		local needForParent = Counter:add(totalNeedTN, self.need[vns.parentS])
		for idS, childVns in pairs(vns.childrenTVns) do
			if self.allocated[idS] == nil and 
		   	   self.childrenAssignTS[idS] == vns.parentS then
				needForParent[childVns.robotType] = 
					needForParent[childVns.robotType] - 1
			end
		end

		if self.getBranch == true and
	   	   Counter:equel(needForParent, 
		                 self.myLastSentNeed[vns.parentS]) 
		   == false then
			vns.Msg.send(vns.parentS, "need", needForParent)
			self.myLastSentNeed[vns.parentS] = needForParent
		end

	end

	-- assign more to parent
	for idS, childVns in pairs(vns.childrenTVns) do
		if self.allocated[idS] == nil and
		   self.childrenAssignTS[idS] == nil then
			self:assign(idS, vns.parentS, vns)
		end
	end
end

--------------------------------------------------
function Counter:new()
	local counter = {}
	for i, v in ipairs(self.robotTypes) do
		counter[v] = 0
	end
	return counter
end

function Counter:add(aTN, bTN)
	if aTN == nil then aTN = self:new() end
	if bTN == nil then bTN = self:new() end
	cTN = self:new()
	for i, v in ipairs(self.robotTypes) do
		cTN[v] = aTN[v] + bTN[v]
	end
	return cTN
end

function Counter:sub(aTN, bTN)
	if aTN == nil then aTN = self:new() end
	if bTN == nil then bTN = self:new() end
	cTN = self:new()
	for i, v in ipairs(self.robotTypes) do
		cTN[v] = aTN[v] - bTN[v]
	end
	return cTN
end

function Counter:equel(aTN, bTN)
	if aTN == nil then aTN = self:new() end
	if bTN == nil then bTN = self:new() end
	for i, v in ipairs(self.robotTypes) do
		if aTN[v] ~= bTN[v] then return false end
	end
	return true
end

return Shifter
