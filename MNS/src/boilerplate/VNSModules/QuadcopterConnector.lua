-- Quadcopter Connector --------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")

local Connector = require("Connector")
local QuadcopterConnector = {VNSMODULECLASS = true}
setmetatable(QuadcopterConnector, Connector)
QuadcopterConnector.__index = QuadcopterConnector

totalNum=0
jj = 1

local tmpVector = {}
local Ave = {}
function QuadcopterConnector:new()
	local instance = Connector:new()
	setmetatable(instance, self)
	instance.robotType = "quadcopter"
	instance.corner = 0
	
	return instance
end

function QuadcopterConnector:run(vns, paraT)
	------------------
	--	if No Parent
	--		recruit --> ack
	--		deny    --> report
	--	have parent
	--		recruit from others --> deny
	--		recruit from parent --> nothing
	--		always  report
	------------------
	vns.corner = 0
	local tempAve = 0
	local av = 0
	print("stepCounter:", paraT.stepCounter)
	-- if I dont't have a parent, ack the first recruit TODO: ack the best recruit
	if vns.parentS == nil then
		for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "recruit")) do
			vns.parentS = msgM.fromS
			vns.Msg.send(msgM.fromS, "ack")
			break
		end
		if paraT.cornerN ~= 0  then
			vns.corner = 1
		end
	end

	-- deny all recruit from not my parent
	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "recruit")) do
		if msgM.fromS ~= vns.parentS and msgM.fromS ~= vns.myAssignParent then --check assigner
			vns.Msg.send(msgM.fromS, "deny", {myParentS = vns.parentS})
		end
	end

	-- report my sight to parent
	if vns.parentS ~= nil then
		vns.Msg.send(vns.parentS, "reportForDuty", {mySight = paraT.vehiclesTR})
	end

	-- if I don't have a parent, or deny come from myAssign Parent report back to deny source
	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "deny")) do
		if vns.parentS == nil or vns.myAssignParent == msgM.dataT.myParentS then
			vns.Msg.send(msgM.dataT.myParentS, "reportForDuty", {mySight = paraT.vehiclesTR})
		end
	end

	-- generate quads list
	quadsTR = {}
	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "reportForDuty")) do
		quadsTR[msgM.fromS]= self:calcQuadR(vns, msgM.fromS, paraT.vehiclesTR, msgM.dataT.mySight)
	end

	self:step(vns, quadsTR)
	
	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "cornerInfo")) do
		if msgM.dataT.cornerInfo ~= 0 then
			if vns.parentS ~= nil then
				vns.Msg.send(vns.parentS, "cornerInfo", {cornerInfo = msgM.dataT.cornerInfo})
			end
			vns.corner = 1
			print("vns.corner:", vns.corner)
		end
	end
end

function QuadcopterConnector:calcQuadR(vns, idS, myVehiclesTR, yourVehiclesTR)
	local quadR
	for _, robotR in pairs(yourVehiclesTR) do
		if myVehiclesTR[robotR.idS] ~= nil then
			myRobotR = myVehiclesTR[robotR.idS]
			robotR.locV3 = vns.Msg.recoverV3(robotR.locV3)
			robotR.dirQ = vns.Msg.recoverQ(robotR.dirQ)

			quadR = {
				idS = idS,
				locV3 = Linar.yourLocBySameObj(
					myRobotR.locV3, myRobotR.dirQ,
					robotR.locV3, robotR.dirQ),
				dirQ = Linar.yourDirBySameObj(
					myRobotR.dirQ, robotR.dirQ),
			}
			break
		end
	end
	return quadR
end

return QuadcopterConnector



