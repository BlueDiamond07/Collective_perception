-- vehicle Connector --------------------------------
------------------------------------------------------
local Connector = require("Connector")
local VehicleConnector = {VNSMODULECLASS = true}
setmetatable(VehicleConnector, Connector)
VehicleConnector.__index = VehicleConnector

function VehicleConnector:new()
	local instance = Connector:new()
	setmetatable(instance, self)
	instance.robotType = "vehicle"
	return instance
end

function VehicleConnector:run(vns, paraT)
	self:step(vns, paraT.vehiclesTR)
end

return VehicleConnector
