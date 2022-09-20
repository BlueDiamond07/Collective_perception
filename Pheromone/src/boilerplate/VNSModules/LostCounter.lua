-- lost counter --------------------------------------
------------------------------------------------------
local LostCounter = {VNSMODULECLASS = true}
LostCounter.__index = LostCounter

function LostCounter:new()
	local instance = {}
	setmetatable(instance, self)

	instance.countTN = {}	-- father class data, not used if inherited
	instance.parentcount = 0

	return instance
end

function LostCounter:reset()
	self.countTN = {}
	self.parentcount = 0
end

function LostCounter:run(vns)
	-- lose parent
	if vns.parentS ~= nil then
		local flag = false
		for _, msgM in ipairs(vns.Msg.getAM(vns.parentS, "ALLMSG")) do
			flag = true break end
		if flag == false then
			self.parentcount = self.parentcount + 1
		else
			self.parentcount = 0
		end

		if self.parentcount == 3 then
			vns.Msg.send(vns.parentS, "lost_bye")
			vns:deleteParent()
		end

		-- get lost by parent
		for _, msgM in ipairs(vns.Msg.getAM(vns.parentS, "lost_dismiss")) do
			vns:deleteParent()
			break
		end
	end

	-- lose children
	for idS, childVns in pairs(vns.childrenTVns) do
		if childVns.updated == true then
			self.countTN[idS] = 0
		else
			if self.countTN[idS] == nil then self.countTN[idS] = 0 end
			self.countTN[idS] = self.countTN[idS] + 1
		end
		childVns.updated = false

		if self.countTN[idS] == 3 then
			vns.Msg.send(idS, "lost_dismiss")
			vns:deleteChild(idS)
		end

		-- get lost by child
		for _, msgM in ipairs(vns.Msg.getAM(idS, "lost_bye")) do
			vns:deleteChild(idS)
			break
		end
	end
end

return LostCounter
