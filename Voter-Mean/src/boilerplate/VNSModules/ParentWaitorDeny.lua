local ParentWaitorDeny = {VNSMODULECLASS = true}
ParentWaitorDeny.__index = ParentWaitorDeny

function ParentWaitorDeny:new()
	local instance = {}
	setmetatable(instance, self)
	return instance
end

function ParentWaitorDeny:run(vns, paraT)
	if vns.parentS == nil then
		for _, msgM in pairs(vns.Msg.getAM("ALLMSG", "recruit")) do
			vns.parentS = msgM.fromS
			vns.Msg.send(msgM.fromS, "ack")
			break
		end
	end

	-- deny all recruit from not my parent and not myAssignTarget
	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "recruit")) do
		if msgM.fromS ~= vns.parentS and msgM.fromS ~= vns.myAssignParent then --check assigner
			vns.Msg.send(msgM.fromS, "deny", {myParentS = vns.parentS})
		end
	end
end

return ParentWaitorDeny
