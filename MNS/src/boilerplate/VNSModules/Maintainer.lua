-- VNS Tree Maintainer -------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")
local Assigner = require("Assigner")

local Maintainer = {VNSMODULECLASS = true}
setmetatable(Maintainer, Assigner)
Maintainer.__index = Maintainer

function Maintainer:new()
	local instance = Assigner:new()
	setmetatable(instance, self)

	instance.allocated = {}
	instance.lastSendBrain = {}

	return instance
end

function Maintainer:setGene(vns, structure)
	self:setStructure(vns, structure)
	self.gene = structure
end

function Maintainer:setStructure(vns, structure)
	self.structure = structure

	-- re-allocate/re-assign every child
	self:reset(vns)

end

function Maintainer:reset(vns)
	Assigner.reset(self, vns)
	self.allocated = {}
	self.lastSendBrain = {}
	if self.structure ~= nil and
	   self.structure.children ~= nil then
		for _, branchT in ipairs(self.structure.children) do
			branchT.actorS = nil
		end
	end
end

function Maintainer:deleteChild(idS)
	Assigner.deleteChild(self, idS)
	self.allocated[idS] = nil
	self.lastSendBrain[idS] = nil
end

function Maintainer:deleteParent(vns)
	Assigner.deleteParent(self, vns)
	vns.brainS = vns.idS
	self:setStructure(vns, self.gene)
end

function Maintainer:run(vns)
	Assigner.run(self, vns)

	-- about brain
	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "loop_bye")) do
		if vns.childrenTVns[msgM.fromS] ~= nil then
			self:deleteChild(msgM.fromS)
		end
	end

	for _, msgM in ipairs(vns.Msg.getAM(vns.parentS, "brain")) do
		if msgM.dataT.brainS == vns.idS then
			-- some loop is happenning
			vns.Msg.send(vns.parentS, "loop_bye")
			vns:deleteParent()
			vns.brainS = vns.idS
			break
		end

		if vns.brainS ~= msgM.dataT.brainS then
			vns.brainS = msgM.dataT.brainS
		end
	end

	for idS, childVns in pairs(vns.childrenTVns) do
		if self.lastSendBrain[idS] ~= vns.brainS then
			vns.Msg.send(idS, "brain", {brainS = vns.brainS})
			self.lastSendBrain[idS] = vns.brainS
		end
	end

	-- about branch
	for _, msgM in ipairs(vns.Msg.getAM(vns.parentS, "branch")) do
		self:setStructure(vns, vns.Msg.recoverTable(msgM.dataT.structure))
	end

	-- allocate branch -- TODO: hungarian 
	if self.structure ~= nil and
	   self.structure.children ~= nil then
		for _, branchT in ipairs(self.structure.children) do
			-- some branch is lost
			if branchT.actorS ~= nil and
			   vns.childrenTVns[branchT.actorS] == nil then
					branchT.actorS = nil
			end

			-- some branch is not fulfilled
			if branchT.actorS == nil then
				-- try find the nearest child
				local nearestId = nil
				local nearestLen = 9999999
				for idS, childVns in pairs(vns.childrenTVns) do
					if self.allocated[idS] == nil and 
					   childVns.robotType == branchT.robotType then
						local len = (childVns.locV3 - branchT.locV3):len()
						if len < nearestLen then
							nearestLen = len
							nearestId = idS
						end
					end
				end

				if nearestId ~= nil then
					if self.childrenAssignTS[nearestId] ~= nil then
						self:unsign(nearestId, vns)
					end
					branchT.actorS = nearestId 
					self.allocated[nearestId] = true
					vns.childrenTVns[nearestId].rallyPoint = {
						locV3 = branchT.locV3,
						dirQ = branchT.dirQ,
					}
					vns.Msg.send(nearestId, "branch", {structure = branchT})
				end
			end
		end
	end

	-- more children set rallypoint to 0
	for idS, childVns in pairs(vns.childrenTVns) do
		if self.allocated[idS] == nil and
		   self.childrenAssignTS[idS] == nil then
			childVns.rallyPoint = {
				locV3 = Vec3:create(),
				dirQ = Quaternion:create(),
			}
		end
	end
end

return Maintainer
