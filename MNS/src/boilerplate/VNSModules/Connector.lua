-- connector -----------------------------------------
------------------------------------------------------
local Connector = {VNSMODULECLASS = true}
Connector.__index = Connector

function Connector:new()
	local instance = {}
	setmetatable(instance, self)

	instance.countTN = {}
	instance.waitingTVns = {}

	return instance
end

function Connector:reset()
	self.countTN = {}
	self.waitingTVns = {}
end

function Connector:step(vns, robotListR)
	self:update(robotListR, vns)
	self:lostCount(robotListR)
	-- recruit new
	for idS, robotR in pairs(robotListR) do
		if vns.childrenTVns[idS] == nil and 
		   self.waitingTVns[idS] == nil and 
		   vns.parentS ~= idS and
		   vns.brainS ~= idS then
			self:recruit(robotR, vns, self.robotType)
		end
	end

	-- check ack
	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "ack")) do
		if self.waitingTVns[msgM.fromS] ~= nil then
			vns.childrenTVns[msgM.fromS] = self.waitingTVns[msgM.fromS]
			self.waitingTVns[msgM.fromS] = nil
			self.countTN[msgM.fromS] = nil
		end
	end
end

function Connector:recruit(robotR, vns, robotType)
	vns.Msg.send(robotR.idS, "recruit", {number = math.random()}) 
		--TODO: give vns status in the future
	self.countTN[robotR.idS] = 0
	self.waitingTVns[robotR.idS] = getmetatable(vns):new{
		idS = robotR.idS,
		locV3 = robotR.locV3,
		dirQ = robotR.dirQ,
		robotType = robotType
	}
end

function Connector:update(robotListR, vns)
	if type(robotListR) ~= "table" then return end

	-- update waiting list
	for idS, robotR in pairs(robotListR) do
		if self.waitingTVns[idS] ~= nil then
			self.waitingTVns[idS].locV3 = robotR.locV3
			self.waitingTVns[idS].dirQ = robotR.dirQ
		end
	end

	-- update vns children list
	for idS, robotR in pairs(robotListR) do
		if vns.childrenTVns[idS] ~= nil then
			vns.childrenTVns[idS].locV3 = robotR.locV3
			vns.childrenTVns[idS].dirQ = robotR.dirQ
			vns.childrenTVns[idS].updated = true
		end
	end
end

function Connector:lostCount(robotListR)
	-- lost count
	for idS, _ in pairs(self.waitingTVns) do
		self.countTN[idS] = self.countTN[idS] + 1
		if self.countTN[idS] == 3 then
			self.countTN[idS] = nil
			self.waitingTVns[idS] = nil
		end
	end
end

return Connector
